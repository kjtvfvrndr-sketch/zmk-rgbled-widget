/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Split central: sends the layer colour to each peripheral's LED
 * (RGBLED_WIDGET_LAYER_PUSH) as the lyr_sync behavior, with the colour and its
 * blanking time. The latest colour always wins; colours that were overtaken
 * before they went out are never sent.
 *
 * Over Bluetooth, at most one write is in flight per peripheral. The colour
 * goes to ZMK's own run-behavior characteristic on the peripheral -- exactly
 * the payload zmk_split_central_invoke_behavior() would send, so the
 * peripheral side is unchanged -- but with bt_gatt_write_without_response_cb(),
 * whose callback comes when the peripheral's link layer has acknowledged the
 * write. ZMK's own path writes without one: the sender never learns whether a
 * write got through.
 *
 * That matters because a write holds a TX context, an ATT TX record and a
 * controller buffer -- shared with this half's reports to the computer, 5 TX
 * contexts by default -- until it is acknowledged. Normally that is the next
 * connection event the peripheral attends, well under a second. A peripheral
 * that has gone away and is not yet declared lost acknowledges nothing: with
 * no limit, an auto-mouse layer toggling with every touch of the ball takes
 * them all within seconds, and the trackball stalls. So the next write goes
 * out only when the previous one is acknowledged, and only the latest colour
 * waits meanwhile. A change goes out at once when nothing is in flight.
 *
 * A write can also stall in the host's ATT queue on a link that is fine: when
 * a write from the system work queue finds no free TX context, Zephyr puts it
 * back and only the next completed write on that link sends it on. A write
 * the link layer had lost would have dropped the link within the supervision
 * timeout; one still unacknowledged after that, on a link still up, is stuck
 * in that queue. It is sent again then, which pushes the queue on.
 *
 * Over a wired split there are no shared buffers to protect: the colour goes
 * through zmk_split_central_invoke_behavior(), as ZMK sends any behavior.
 *
 * Threads: over Bluetooth, discovery and writes run on a work queue of this
 * file's own, where blocking on a Bluetooth buffer is allowed and holds up
 * nothing else. The Bluetooth callbacks update atomics and wake it; a
 * discovery callback may also continue the discovery, as Zephyr allows. Over
 * a wired split the sender runs on the system work queue, as ZMK's own
 * commands do.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <string.h>

#include <zmk_rgbled_widget/widget.h>

LOG_MODULE_DECLARE(rgbled_widget, CONFIG_RGBLED_WIDGET_LOG_LEVEL);

/* The behavior node the peripheral runs; see dts/behaviors/rgbled_widget.dtsi. */
#define LAYER_SYNC_BEHAVIOR "lyr_sync"

/* A discovery or a write the stack refused is tried again after this,
 * doubling with every refusal in a row up to MAX_RETRY_MS. */
#define RETRY_MS     1000
#define MAX_RETRY_MS 30000

/* ---- what the peripherals should show ---- */

static struct k_spinlock wanted_lock;
static uint32_t wanted_rgb;
static uint32_t wanted_ms;
static bool wanted_set;

static bool get_wanted(uint32_t *rgb, uint32_t *blank_ms) {
    k_spinlock_key_t key = k_spin_lock(&wanted_lock);
    const bool have = wanted_set;

    *rgb = wanted_rgb;
    *blank_ms = wanted_ms;
    k_spin_unlock(&wanted_lock, key);
    return have;
}

/* ---- the sender's queue ---- */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
/* A queue of its own: a write may wait for a Bluetooth buffer, and that holds
 * up nothing else here. Its deepest path -- a discovery or a write blocking
 * for a buffer -- measures about 1.2 KB with logging and 0.5 KB without, plus
 * the queue's own frames and an interrupt, about 0.2 KB. */
BUILD_ASSERT(CONFIG_RGBLED_WIDGET_LAYER_PUSH_STACK_SIZE >= (IS_ENABLED(CONFIG_LOG) ? 1536 : 1024),
             "RGBLED_WIDGET_LAYER_PUSH_STACK_SIZE: at least 1536 with logging, 1024 without");
static K_THREAD_STACK_DEFINE(layer_push_stack, CONFIG_RGBLED_WIDGET_LAYER_PUSH_STACK_SIZE);
static struct k_work_q layer_push_q;
#define SENDER_QUEUE (&layer_push_q)
#else
/* Wired: ZMK's own commands go out from the system work queue, and its UART
 * ring buffer takes no lock -- so the colour goes from there too. Nothing in
 * that path waits. */
#define SENDER_QUEUE (&k_sys_work_q)
#endif

static void send_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(send_work, send_work_cb);

/* Run the sender in `delay_ms`, or sooner if it is due sooner already. A
 * wake-up that is due later is brought forward: whichever is earliest wins. */
static void wake_sender(uint32_t delay_ms) {
    if (delay_ms == 0U || !k_work_delayable_is_pending(&send_work) ||
        k_ticks_to_ms_ceil32(k_work_delayable_remaining_get(&send_work)) > delay_ms) {
        (void)k_work_reschedule_for_queue(SENDER_QUEUE, &send_work, K_MSEC(delay_ms));
    }
}

void rgbled_widget_push_layer_color_to_peripherals(uint32_t rgb, uint32_t blank_ms) {
    k_spinlock_key_t key = k_spin_lock(&wanted_lock);
    wanted_rgb = rgb;
    wanted_ms = blank_ms;
    wanted_set = true;
    k_spin_unlock(&wanted_lock, key);

    wake_sender(0);
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)

static int layer_push_init(void) {
    const struct k_work_queue_config cfg = {.name = "rgbled_layer_push"};

    k_work_queue_start(&layer_push_q, layer_push_stack, K_THREAD_STACK_SIZEOF(layer_push_stack),
                       CONFIG_RGBLED_WIDGET_LAYER_PUSH_THREAD_PRIORITY, &cfg);
    return 0;
}

/* Before anything can push a colour or a link can come up. */
SYS_INIT(layer_push_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <zmk/split/bluetooth/service.h>
#include <zmk/split/bluetooth/uuid.h>

BUILD_ASSERT(sizeof(LAYER_SYNC_BEHAVIOR) <= ZMK_SPLIT_RUN_BEHAVIOR_DEV_LEN,
             "lyr_sync does not fit ZMK's behavior name field");
/* The acknowledgement carries the slot in the low byte of its cookie. */
BUILD_ASSERT(CONFIG_BT_MAX_CONN <= 256, "slot index does not fit the write cookie");

/* Discovery waits this long after the link is secured, so that ZMK's own --
 * the one that makes the peripheral's keys work -- goes first. */
#define DISCOVERY_DELAY_MS 2000
/* A discovery that ends without the characteristic on a link that is up --
 * Zephyr ends one that way when it has no free request slot for its next
 * step -- is started again, this many times in all. */
#define DISCOVERY_TRIES 3
/* A write unacknowledged this long past the link's supervision timeout, on a
 * link still up, is stuck in the host's queue. */
#define STUCK_MARGIN_MS 1000

static const struct bt_uuid_128 split_service_uuid =
    BT_UUID_INIT_128(ZMK_SPLIT_BT_SERVICE_UUID);
static const struct bt_uuid_128 run_behavior_uuid =
    BT_UUID_INIT_128(ZMK_SPLIT_BT_CHAR_RUN_BEHAVIOR_UUID);

/* ---- one slot per connection, indexed by bt_conn_index() ---- */

enum peripheral_flag {
    /* A discovery of the run-behavior characteristic is under way or done. */
    FLAG_DISCOVERY_STARTED,
};

struct peripheral_slot {
    atomic_t flags;
    /* Value handle of ZMK's run-behavior characteristic; 0 until found. */
    atomic_t run_behavior_handle;
    /* k_uptime_get_32() when the link was secured: discovery waits
     * DISCOVERY_DELAY_MS from there. */
    atomic_t secured_at;
    /* Bumped on every disconnect. What was sent, and a discovery under way,
     * count only for the link they were made on. */
    atomic_t link_generation;
    /* Discoveries started on this link. */
    atomic_t discovery_tries;
    /* The next retry delay; RETRY_MS after a success. */
    atomic_t retry_ms;
    /* The write in flight, by its sequence number; 0 when none. Cleared by
     * its own acknowledgement only, or by the disconnect. */
    atomic_t in_flight_seq;
    /* The rest is the sender's own. */
    uint32_t in_flight_since;
    uint32_t next_seq;
    uint32_t discovery_generation;
    uint32_t sent_generation;
    bool sent_valid;
    uint32_t sent_rgb;
    uint32_t sent_ms;
    struct bt_gatt_discover_params discover;
};

/* Indexed by bt_conn_index(): a link's slot among Zephyr's connections, which
 * the logs call the link slot -- not the peripheral's number in the split. */
static struct peripheral_slot slots[CONFIG_BT_MAX_CONN] = {
    [0 ... CONFIG_BT_MAX_CONN - 1] = {.retry_ms = ATOMIC_INIT(RETRY_MS)},
};

static uint8_t slot_index(const struct peripheral_slot *slot) { return (uint8_t)(slot - slots); }

/* A split peripheral: an LE link on which this half is central. */
static bool is_peripheral_link(struct bt_conn *conn) {
    struct bt_conn_info info;

    return !bt_conn_get_info(conn, &info) && info.type == BT_CONN_TYPE_LE &&
           info.role == BT_CONN_ROLE_CENTRAL;
}

static bool is_up(struct bt_conn *conn) {
    struct bt_conn_info info;

    return !bt_conn_get_info(conn, &info) && info.state == BT_CONN_STATE_CONNECTED;
}

/* The delay before the next retry, in ms, doubling it for the one after. */
static uint32_t next_retry(struct peripheral_slot *slot) {
    const uint32_t delay = (uint32_t)atomic_get(&slot->retry_ms);

    atomic_set(&slot->retry_ms, (atomic_val_t)MIN(2U * delay, (uint32_t)MAX_RETRY_MS));
    return delay;
}

/* ---- discovery ---- */

/* The discovery ended without the characteristic. The link dropping under it
 * ends it that way too; that is no failure. */
static void discovery_ended_empty(struct bt_conn *conn, struct peripheral_slot *slot,
                                  const char *what) {
    if (!is_up(conn)) {
        return;
    }
    if (atomic_get(&slot->discovery_tries) < DISCOVERY_TRIES) {
        LOG_DBG("Layer colour push: no %s on link slot %u yet, looking again", what,
                slot_index(slot));
        atomic_clear_bit(&slot->flags, FLAG_DISCOVERY_STARTED);
        wake_sender(next_retry(slot));
        return;
    }
    LOG_WRN("Layer colour push: link slot %u has no %s", slot_index(slot), what);
}

static uint8_t on_run_behavior_found(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     struct bt_gatt_discover_params *params) {
    struct peripheral_slot *slot = CONTAINER_OF(params, struct peripheral_slot, discover);

    if (slot->discovery_generation != (uint32_t)atomic_get(&slot->link_generation)) {
        return BT_GATT_ITER_STOP;
    }
    if (!attr) {
        discovery_ended_empty(conn, slot, "ZMK run-behavior characteristic");
        return BT_GATT_ITER_STOP;
    }

    const uint16_t handle = ((const struct bt_gatt_chrc *)attr->user_data)->value_handle;

    atomic_set(&slot->run_behavior_handle, handle);
    atomic_set(&slot->retry_ms, RETRY_MS);
    LOG_INF("Layer colour push: link slot %u ready (handle 0x%04x)", slot_index(slot), handle);
    wake_sender(0);
    return BT_GATT_ITER_STOP;
}

static uint8_t on_split_service_found(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      struct bt_gatt_discover_params *params) {
    struct peripheral_slot *slot = CONTAINER_OF(params, struct peripheral_slot, discover);

    if (slot->discovery_generation != (uint32_t)atomic_get(&slot->link_generation)) {
        return BT_GATT_ITER_STOP;
    }
    if (!attr) {
        discovery_ended_empty(conn, slot, "ZMK split service");
        return BT_GATT_ITER_STOP;
    }

    /* Reusing the parameters from inside their own callback is allowed: the
     * procedure is over once this returns STOP. */
    params->uuid = &run_behavior_uuid.uuid;
    params->func = on_run_behavior_found;
    params->start_handle = attr->handle + 1;
    params->end_handle = ((const struct bt_gatt_service_val *)attr->user_data)->end_handle;
    params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(conn, params);

    if (err && is_up(conn)) {
        LOG_WRN("Layer colour push: characteristic discovery failed (%d), retrying", err);
        atomic_clear_bit(&slot->flags, FLAG_DISCOVERY_STARTED);
        wake_sender(next_retry(slot));
    }
    return BT_GATT_ITER_STOP;
}

static void start_discovery(struct bt_conn *conn, struct peripheral_slot *slot) {
    slot->discovery_generation = (uint32_t)atomic_get(&slot->link_generation);
    atomic_inc(&slot->discovery_tries);
    slot->discover.uuid = &split_service_uuid.uuid;
    slot->discover.func = on_split_service_found;
    slot->discover.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    slot->discover.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    slot->discover.type = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(conn, &slot->discover);

    if (err) {
        atomic_clear_bit(&slot->flags, FLAG_DISCOVERY_STARTED);
        /* A refusal does not count as a try: nothing was looked at. */
        atomic_dec(&slot->discovery_tries);
        if (is_up(conn)) {
            LOG_WRN("Layer colour push: service discovery failed (%d), retrying", err);
            wake_sender(next_retry(slot));
        }
    }
}

/* ---- sending ---- */

/* The peripheral acknowledged write `seq`: the next one may go. Never called
 * for a write the link dropped; the disconnect clears the slot then. */
static void on_write_acknowledged(struct bt_conn *conn, void *cookie) {
    ARG_UNUSED(conn);

    const uintptr_t value = (uintptr_t)cookie;
    struct peripheral_slot *slot = &slots[value & 0xFF];
    const uint32_t seq = (uint32_t)(value >> 8);

    /* An earlier write that was stuck and sent again acknowledges too; only
     * the latest one ends the wait. */
    (void)atomic_cas(&slot->in_flight_seq, (atomic_val_t)seq, 0);
    wake_sender(0);
}

/* How long a write may go unacknowledged on a link that is up before it
 * counts as stuck in the host's queue. */
static uint32_t stuck_after_ms(struct bt_conn *conn) {
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info)) {
        return 32000U + STUCK_MARGIN_MS; /* the longest timeout Bluetooth allows */
    }
    return info.le.timeout * 10U + STUCK_MARGIN_MS;
}

static void send_to(struct bt_conn *conn, uint32_t rgb, uint32_t blank_ms) {
    struct peripheral_slot *slot = &slots[bt_conn_index(conn)];
    /* Read before anything else: a disconnect from here on changes it, and
     * what this run sends then does not count for the next link. */
    const uint32_t generation = (uint32_t)atomic_get(&slot->link_generation);
    const uint16_t handle = (uint16_t)atomic_get(&slot->run_behavior_handle);
    const uint32_t now = k_uptime_get_32();

    if (handle == 0) {
        const uint32_t secured_for = now - (uint32_t)atomic_get(&slot->secured_at);

        if (secured_for < DISCOVERY_DELAY_MS) {
            wake_sender(DISCOVERY_DELAY_MS - secured_for);
        } else if (!atomic_test_and_set_bit(&slot->flags, FLAG_DISCOVERY_STARTED)) {
            start_discovery(conn, slot);
        }
        return; /* the discovery wakes the sender when it is done */
    }

    const atomic_val_t pending = atomic_get(&slot->in_flight_seq);
    const uint32_t pending_since = slot->in_flight_since;
    bool resend = false;

    if (pending != 0) {
        const uint32_t waited = now - pending_since;
        const uint32_t stuck_after = stuck_after_ms(conn);

        if (waited < stuck_after) {
            /* The acknowledgement wakes the sender; this, if it never comes. */
            wake_sender(stuck_after - waited);
            return;
        }
        LOG_WRN("Layer colour push: write on link slot %u unacknowledged for %u ms on a "
                "live link -- sending again",
                slot_index(slot), waited);
        resend = true;
    }
    if (!resend && slot->sent_valid && slot->sent_generation == generation &&
        slot->sent_rgb == rgb && slot->sent_ms == blank_ms) {
        return;
    }

    /* The same payload zmk_split_central_invoke_behavior() builds: position
     * 0, pressed, the colour in param1 and the blanking time in param2. */
    struct zmk_split_run_behavior_payload payload = {
        .data =
            {
                .position = 0,
                .source = 0,
                .state = 1,
                .param1 = rgb,
                .param2 = blank_ms,
            },
    };
    memcpy(payload.behavior_dev, LAYER_SYNC_BEHAVIOR, sizeof(LAYER_SYNC_BEHAVIOR));

    /* 24 bits of sequence, never 0. Set before the write: the acknowledgement
     * may come before the call returns. */
    slot->next_seq = (slot->next_seq + 1U) & 0xFFFFFFU;
    if (slot->next_seq == 0U) {
        slot->next_seq = 1U;
    }
    const uint32_t seq = slot->next_seq;

    slot->in_flight_since = now;
    atomic_set(&slot->in_flight_seq, (atomic_val_t)seq);

    int err = bt_gatt_write_without_response_cb(
        conn, handle, &payload, sizeof(payload), false, on_write_acknowledged,
        (void *)(((uintptr_t)seq << 8) | slot_index(slot)));

    if (err) {
        /* Nothing went out: what was in flight before still is -- unless
         * the link went down meanwhile, taking it along. */
        const bool same_link = (uint32_t)atomic_get(&slot->link_generation) == generation;

        slot->in_flight_since = pending_since;
        (void)atomic_cas(&slot->in_flight_seq, (atomic_val_t)seq, same_link ? pending : 0);
        if (is_up(conn)) {
            LOG_WRN("Layer colour push on link slot %u failed (%d), retrying",
                    slot_index(slot), err);
            wake_sender(next_retry(slot));
        }
        return;
    }
    atomic_set(&slot->retry_ms, RETRY_MS);
    slot->sent_rgb = rgb;
    slot->sent_ms = blank_ms;
    slot->sent_generation = generation;
    slot->sent_valid = true;
    /* The acknowledgement wakes the sender; this, if it never comes and no
     * change does either. */
    wake_sender(stuck_after_ms(conn));
    LOG_DBG("Pushed layer colour #%06X (blank after %ums) on link slot %u", rgb, blank_ms,
            slot_index(slot));
}

struct secured_links {
    struct bt_conn *conns[CONFIG_BT_MAX_CONN];
    size_t count;
};

/* Peripheral links that are up and encrypted: ZMK's characteristic takes
 * encrypted writes only, and an unencrypted one would be dropped silently. */
static void collect_secured_link(struct bt_conn *conn, void *data) {
    struct secured_links *links = data;

    if (links->count < ARRAY_SIZE(links->conns) && is_peripheral_link(conn) && is_up(conn) &&
        bt_conn_get_security(conn) >= BT_SECURITY_L2) {
        links->conns[links->count++] = bt_conn_ref(conn);
    }
}

static void send_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    uint32_t rgb, blank_ms;

    if (!get_wanted(&rgb, &blank_ms)) {
        return;
    }

    struct secured_links links = {.count = 0};

    /* Collected first, sent after: a write may block for a buffer, which is
     * no place to be inside bt_conn_foreach(). */
    bt_conn_foreach(BT_CONN_TYPE_LE, collect_secured_link, &links);
    for (size_t i = 0; i < links.count; i++) {
        send_to(links.conns[i], rgb, blank_ms);
        bt_conn_unref(links.conns[i]);
    }
}

/* ---- the links coming and going ---- */

static void on_security_changed(struct bt_conn *conn, bt_security_t level,
                                enum bt_security_err err) {
    if (err || level < BT_SECURITY_L2 || !is_peripheral_link(conn)) {
        return;
    }
    atomic_set(&slots[bt_conn_index(conn)].secured_at, (atomic_val_t)k_uptime_get_32());
    wake_sender(DISCOVERY_DELAY_MS);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    ARG_UNUSED(reason);

    if (!is_peripheral_link(conn)) {
        return;
    }

    struct peripheral_slot *slot = &slots[bt_conn_index(conn)];

    /* The write in flight, if any, went down with the link and will not be
     * acknowledged. The next link starts from nothing: it discovers the
     * handle again and is sent the current colour. */
    atomic_inc(&slot->link_generation);
    atomic_set(&slot->run_behavior_handle, 0);
    atomic_set(&slot->in_flight_seq, 0);
    atomic_set(&slot->discovery_tries, 0);
    atomic_set(&slot->retry_ms, RETRY_MS);
    (void)atomic_clear(&slot->flags);
}

BT_CONN_CB_DEFINE(layer_push_conn_callbacks) = {
    .security_changed = on_security_changed,
    .disconnected = on_disconnected,
};

#else /* a wired split */

#include <zmk/behavior.h>
#include <zmk/split/central.h>

static bool sent_valid;
static uint32_t sent_rgb;
static uint32_t sent_ms;
static uint32_t retry_ms = RETRY_MS;

static void send_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    uint32_t rgb, blank_ms;

    if (!get_wanted(&rgb, &blank_ms) ||
        (sent_valid && sent_rgb == rgb && sent_ms == blank_ms)) {
        return;
    }

    struct zmk_behavior_binding binding = {
        .behavior_dev = LAYER_SYNC_BEHAVIOR,
        .param1 = rgb,
        .param2 = blank_ms,
    };
    struct zmk_behavior_binding_event event = {
        .layer = 0,
        .position = 0,
        .timestamp = k_uptime_get(),
    };
    int failed = 0;

    for (uint8_t i = 0; i < ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT; i++) {
        int err = zmk_split_central_invoke_behavior(i, &binding, event, true);

        if (err) {
            /* As ZMK's own path logs it: a peripheral that is not there. */
            LOG_DBG("Layer colour push to peripheral %u failed (%d), retrying", i, err);
            failed = err;
        }
    }
    if (failed) {
        wake_sender(retry_ms);
        retry_ms = MIN(2U * retry_ms, (uint32_t)MAX_RETRY_MS);
        return;
    }
    retry_ms = RETRY_MS;
    sent_rgb = rgb;
    sent_ms = blank_ms;
    sent_valid = true;
    LOG_DBG("Pushed layer colour #%06X (blank after %ums)", rgb, blank_ms);
}

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE) */
