/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Split central: sends each peripheral's LED the layer colour and the
 * indications that concern the whole keyboard, as the lyr_sync behavior
 * (widget.h has the two parameters).
 *
 * Over Bluetooth both are shown on every half at once (led_sync.h): a layer
 * change, or a group of blinks when its turn comes on this half's LED, goes
 * to each ready peripheral with its group number; this half shows it at the
 * start the peripherals answer, timed from the connection event that
 * delivered it. A newer layer colour replaces one not sent yet. A peripheral
 * just connected is sent the current colour on its own once it is ready --
 * that one catches it up and is not synced.
 *
 * The writes go to ZMK's own run-behavior characteristic on the peripheral --
 * the payload zmk_split_central_invoke_behavior() would send, with the group
 * number as the position -- but with bt_gatt_write_without_response_cb(),
 * whose callback comes when the peripheral's link layer has acknowledged the
 * write: that moment is the one both halves time the start from.
 *
 * A write holds a TX context, an ATT TX record and a controller buffer --
 * shared with this half's reports to the computer, 5 TX contexts by default
 * -- until it is acknowledged, normally at the next connection event the
 * peripheral attends. A peripheral that has gone away and is not yet declared
 * lost acknowledges nothing, and with no limit an auto-mouse layer toggling
 * with every touch of the ball would take them all and stall the trackball.
 * So at most WRITES_IN_FLIGHT are unacknowledged per peripheral; the rest
 * wait, and the layer colour waiting is replaced by the next.
 *
 * A write can also stall in the host's ATT queue on a link that is fine: when
 * a write finds no free TX context, Zephyr puts it back and only the next
 * completed write on that link sends it on. A write the link layer had lost
 * would have dropped the link within the supervision timeout; writes still
 * unacknowledged after that, on a link still up, are stuck in that queue.
 * They are written off then and the colour is sent again, which pushes the
 * queue on.
 *
 * Over a wired split there are no shared buffers to protect and no wait for
 * the other half: both go through zmk_split_central_invoke_behavior(), as
 * ZMK sends any behavior, and show there as they arrive.
 *
 * Threads: over Bluetooth, discovery and writes run on a work queue of this
 * file's own, where blocking on a Bluetooth buffer is allowed: it holds up
 * the sender alone -- no other peripheral is written and no group settles
 * meanwhile -- but nothing outside it. The Bluetooth callbacks, the LED
 * thread and the layer events
 * update the state under a lock and wake it; a discovery callback may also
 * continue the discovery, as Zephyr allows. Over a wired split the sender
 * runs on the system work queue, as ZMK's own commands do.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <string.h>

#include <zmk_rgbled_widget/widget.h>

#include "led_sync.h"

LOG_MODULE_DECLARE(rgbled_widget, CONFIG_RGBLED_WIDGET_LOG_LEVEL);

/* The behavior node the peripheral runs; see dts/behaviors/rgbled_widget.dtsi. */
#define LAYER_SYNC_BEHAVIOR "lyr_sync"

/* A discovery or a write the stack refused is tried again after this,
 * doubling with every refusal in a row up to MAX_RETRY_MS. */
#define RETRY_MS     1000
#define MAX_RETRY_MS 30000

/* ---- the layer colour a peripheral is caught up with ---- */

static struct k_spinlock wanted_lock;
static uint32_t wanted_rgb;
static uint32_t wanted_ms;
static bool wanted_set;

static void set_wanted(uint32_t rgb, uint32_t blank_ms) {
    k_spinlock_key_t key = k_spin_lock(&wanted_lock);
    wanted_rgb = rgb;
    wanted_ms = blank_ms;
    wanted_set = true;
    k_spin_unlock(&wanted_lock, key);
}

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
 * up nothing outside the sender. Its deepest path -- a discovery or a write blocking
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

/* When the sender is due next (uptime); 0 when it is not scheduled. Under
 * wake_lock: the check and the reschedule are one step, and a wake-up asked
 * for while the sender runs is kept -- it clears this as it starts. */
static struct k_spinlock wake_lock;
static int64_t wake_due_at;

/* Run the sender in `delay_ms`, or sooner if it is due sooner already: the
 * earliest wake-up asked for wins. */
static void wake_sender(uint32_t delay_ms) {
    const int64_t due_at = k_uptime_get() + delay_ms;
    k_spinlock_key_t key = k_spin_lock(&wake_lock);

    if (wake_due_at == 0 || due_at < wake_due_at) {
        wake_due_at = due_at;
        (void)k_work_reschedule_for_queue(SENDER_QUEUE, &send_work, K_MSEC(delay_ms));
    }
    k_spin_unlock(&wake_lock, key);
}

/* The sender starts: whatever is asked from now on schedules it again. */
static void sender_started(void) {
    k_spinlock_key_t key = k_spin_lock(&wake_lock);

    wake_due_at = 0;
    k_spin_unlock(&wake_lock, key);
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
BUILD_ASSERT(CONFIG_BT_MAX_CONN <= 32, "link slots must fit a 32-bit slot mask");

/* Discovery waits this long after the link is secured, so that ZMK's own --
 * the one that makes the peripheral's keys work -- goes first. */
#define DISCOVERY_DELAY_MS 2000
/* A discovery that ends without the characteristic on a link that is up --
 * Zephyr ends one that way when it has no free request slot for its next
 * step -- is started again, this many times in all. */
#define DISCOVERY_TRIES 3
/* Writes unacknowledged this long past the link's supervision timeout, on a
 * link still up, are stuck in the host's queue. */
#define STUCK_MARGIN_MS 1000
/* Unacknowledged writes per peripheral: a few groups and a colour go out in
 * one connection event, and a peripheral gone quiet holds no more buffers. */
#define WRITES_IN_FLIGHT 4
/* Groups being synced at a time. */
#define SYNC_GROUPS 8

static const struct bt_uuid_128 split_service_uuid =
    BT_UUID_INIT_128(ZMK_SPLIT_BT_SERVICE_UUID);
static const struct bt_uuid_128 run_behavior_uuid =
    BT_UUID_INIT_128(ZMK_SPLIT_BT_CHAR_RUN_BEHAVIOR_UUID);
static const struct bt_uuid_128 sync_service_uuid = BT_UUID_INIT_128(LED_SYNC_SERVICE_UUID);
static const struct bt_uuid_128 answer_char_uuid = BT_UUID_INIT_128(LED_SYNC_ANSWER_CHAR_UUID);

/* ---- one slot per connection, indexed by bt_conn_index() ---- */

enum peripheral_flag {
    /* Secured: ZMK's characteristic takes encrypted writes only. */
    FLAG_LINK_SECURED,
    /* A discovery is under way or done. */
    FLAG_DISCOVERY_STARTED,
    /* The answer characteristic is found: the sender subscribes to it. */
    FLAG_SUBSCRIBE_DUE,
    /* The peripheral answers: what it is sent is synced. */
    FLAG_ANSWERS,
    /* The peripheral has no answers (older firmware): what it is sent shows
     * there on arrival. */
    FLAG_NO_ANSWERS,
    /* Takes writes; rgbled_widget_peripheral_ready() was called. */
    FLAG_READY,
    /* Is sent the current colour, unsynced, before anything else. */
    FLAG_COLOUR_CATCH_UP_DUE,
    /* Its discovery gave up: nothing is sent to it, and nothing waits. */
    FLAG_TAKES_NO_WRITES,
};

struct peripheral_slot {
    atomic_t flags;
    /* Value handle of ZMK's run-behavior characteristic; 0 until found. */
    atomic_t run_behavior_handle;
    /* k_uptime_get_32() when the link was secured: discovery waits
     * DISCOVERY_DELAY_MS from there. */
    atomic_t secured_at;
    /* Bumped on every disconnect. A discovery under way, and the groups
     * sent, count only for the link they were made on. */
    atomic_t link_generation;
    /* Discoveries started on this link. */
    atomic_t discovery_tries;
    /* The next retry delay; RETRY_MS after a success. */
    atomic_t retry_ms;
    /* Writes not acknowledged yet, and since when the oldest of them has
     * waited (k_uptime_get_32()). */
    atomic_t in_flight;
    atomic_t in_flight_since;
    /* Bumped when the writes in flight are written off as stuck: their
     * acknowledgements, if they come, no longer count. */
    atomic_t flight_epoch;
    /* k_uptime_get_32() when it became ready: a group queued before that
     * is not its. */
    atomic_t ready_at;
    /* The rest is the sender's own, and the discovery's. */
    uint32_t discovery_generation;
    struct bt_gatt_discover_params discover;
    struct bt_gatt_discover_params ccc_discover;
    struct bt_gatt_subscribe_params subscribe;
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

/* One write fewer in flight -- never below none: a write-off may have
 * cleared the count under it. */
static void one_less_in_flight(struct peripheral_slot *slot) {
    for (;;) {
        const atomic_val_t count = atomic_get(&slot->in_flight);

        if (count <= 0 || atomic_cas(&slot->in_flight, count, count - 1)) {
            return;
        }
    }
}

/* The delay before the next retry, in ms, doubling it for the one after. */
static uint32_t next_retry(struct peripheral_slot *slot) {
    const uint32_t delay = (uint32_t)atomic_get(&slot->retry_ms);

    atomic_set(&slot->retry_ms, (atomic_val_t)MIN(2U * delay, (uint32_t)MAX_RETRY_MS));
    return delay;
}

uint32_t rgbled_widget_ready_peripheral_slots(void) {
    uint32_t mask = 0;

    for (size_t i = 0; i < ARRAY_SIZE(slots); i++) {
        if (atomic_test_bit(&slots[i].flags, FLAG_READY)) {
            mask |= BIT(i);
        }
    }
    return mask;
}

/* Secured, and not given up on. */
static bool may_take_writes(const struct peripheral_slot *slot) {
    return atomic_test_bit(&slot->flags, FLAG_LINK_SECURED) &&
           !atomic_test_bit(&slot->flags, FLAG_TAKES_NO_WRITES);
}

bool rgbled_widget_peripheral_getting_ready(void) {
    for (size_t i = 0; i < ARRAY_SIZE(slots); i++) {
        if (may_take_writes(&slots[i]) && !atomic_test_bit(&slots[i].flags, FLAG_READY)) {
            return true;
        }
    }
    return false;
}

/* ---- the groups being synced ---- */

static atomic_t last_sync_id;

uint8_t rgbled_widget_sync_new_id(void) {
    for (;;) {
        const uint8_t id = (uint8_t)atomic_inc(&last_sync_id) + 1U;

        if (id != 0) {
            return id;
        }
    }
}

enum group_kind {
    GROUP_FREE,
    GROUP_INDICATION,
    GROUP_LAYER_COLOUR,
};

/* Where a group stands with one peripheral. */
enum group_on_peripheral {
    /* Not one of its peripherals: not ready when it was made -- one that
     * gets ready later is caught up with the colour, and starts with the
     * split-connected blink. */
    GROUP_NOT_FOR_IT,
    GROUP_TO_SEND,
    GROUP_SENT,
    /* Acknowledged: `since` is the moment the start is timed from. */
    GROUP_DELIVERED,
    /* Answered, acknowledged or not yet. */
    GROUP_ANSWERED,
    /* Sent to a peripheral that does not answer: it shows it on arrival. */
    GROUP_SENT_UNSYNCED,
    /* Missed a wait, or went away: the group goes without it. */
    GROUP_LEFT_OUT,
};

struct group_peripheral {
    enum group_on_peripheral state;
    uint32_t link_generation;
    /* TO_SEND: since the group was made; SENT: since it was written;
     * DELIVERED, ANSWERED: when it was delivered, which the start is timed
     * from -- for an answer seen before the acknowledgement, the answer's
     * arrival until the acknowledgement comes. */
    int64_t since;
    /* ANSWERED: the acknowledgement was seen (or no longer waited for). */
    bool acknowledged;
    /* The answer -- which may come before the acknowledgement is seen. */
    enum led_sync_start answer_start;
    uint16_t answer_start_in_ms;
};

struct sync_group {
    enum group_kind kind;
    uint8_t id;
    /* In the order the groups were made: each peripheral is sent them so. */
    uint32_t order;
    /* The lyr_sync parameters; for a layer colour, the colour and its
     * blanking time, and whether it cuts the indications short here. */
    uint32_t param1;
    uint32_t param2;
    bool interrupts;
    struct group_peripheral on[CONFIG_BT_MAX_CONN];
};

static struct k_spinlock groups_lock;
static struct sync_group groups[SYNC_GROUPS];
static uint32_t next_group_order;

/* A free group, under groups_lock; NULL when all are in use. */
static struct sync_group *free_group(void) {
    for (size_t i = 0; i < ARRAY_SIZE(groups); i++) {
        if (groups[i].kind == GROUP_FREE) {
            return &groups[i];
        }
    }
    return NULL;
}

/* The group with that id, under groups_lock. */
static struct sync_group *group_by_id(uint8_t id) {
    for (size_t i = 0; i < ARRAY_SIZE(groups); i++) {
        if (groups[i].kind != GROUP_FREE && groups[i].id == id) {
            return &groups[i];
        }
    }
    return NULL;
}

/* Makes the group for the peripherals in for_slots, under groups_lock.
 * Returns false with none of them. */
static bool make_group(struct sync_group *group, enum group_kind kind, uint8_t id,
                       uint32_t param1, uint32_t param2, bool interrupts, uint32_t for_slots) {
    const int64_t now = k_uptime_get();

    *group = (struct sync_group){
        .kind = kind,
        .id = id,
        .order = next_group_order++,
        .param1 = param1,
        .param2 = param2,
        .interrupts = interrupts,
    };
    for (size_t i = 0; i < ARRAY_SIZE(slots); i++) {
        if (for_slots & BIT(i)) {
            group->on[i] = (struct group_peripheral){
                .state = GROUP_TO_SEND,
                .link_generation = (uint32_t)atomic_get(&slots[i].link_generation),
                .since = now,
            };
        }
    }
    if (for_slots == 0) {
        group->kind = GROUP_FREE;
        return false;
    }
    return true;
}

/* Ready since before queued_at: one that got ready later starts with the
 * split-connected blink, not with what was queued before it. */
static uint32_t slots_ready_since(int64_t queued_at) {
    uint32_t mask = 0;

    for (size_t i = 0; i < ARRAY_SIZE(slots); i++) {
        if (atomic_test_bit(&slots[i].flags, FLAG_READY) &&
            (int32_t)((uint32_t)queued_at - (uint32_t)atomic_get(&slots[i].ready_at)) >= 0) {
            mask |= BIT(i);
        }
    }
    return mask;
}

void rgbled_widget_offer_synced_group(uint8_t sync_id, uint32_t param1, uint32_t param2,
                                      uint32_t slot_mask, int64_t queued_at) {
    const uint32_t for_slots = slot_mask & slots_ready_since(queued_at);
    k_spinlock_key_t key = k_spin_lock(&groups_lock);
    struct sync_group *group = free_group();
    const bool made = group != NULL && make_group(group, GROUP_INDICATION, sync_id, param1,
                                                  param2, false, for_slots);
    k_spin_unlock(&groups_lock, key);

    if (!made) {
        if (for_slots != 0) {
            LOG_WRN("LED sync %u: too many groups at once -- shown here alone", sync_id);
        }
        rgbled_widget_start_synced_group_at(sync_id, k_uptime_get());
        return;
    }
    LOG_DBG("LED sync %u: #%06X x%u for link slots 0x%x", sync_id, param1 & 0xFFFFFFu,
            (param1 >> 24) & 0xFu, for_slots);
    wake_sender(0);
}

/* Ready peripherals get the colour first, once, before what follows. */
static void catch_up_ready_peripherals(void) {
    for (size_t i = 0; i < ARRAY_SIZE(slots); i++) {
        if (atomic_test_bit(&slots[i].flags, FLAG_READY)) {
            atomic_set_bit(&slots[i].flags, FLAG_COLOUR_CATCH_UP_DUE);
        }
    }
}

void rgbled_widget_push_layer_color_to_peripherals(uint32_t rgb, uint32_t blank_ms) {
    set_wanted(rgb, blank_ms);
    catch_up_ready_peripherals();
    wake_sender(0);
}

void rgbled_widget_change_layer_colour_on_all_halves(uint32_t rgb, uint32_t blank_ms,
                                                     bool interrupts) {
    set_wanted(rgb, blank_ms);

    const uint32_t for_slots = rgbled_widget_ready_peripheral_slots();
    k_spinlock_key_t key = k_spin_lock(&groups_lock);

    /* A colour that has gone nowhere yet is replaced: the latest wins. */
    for (size_t i = 0; i < ARRAY_SIZE(groups); i++) {
        struct sync_group *group = &groups[i];

        if (group->kind != GROUP_LAYER_COLOUR) {
            continue;
        }
        bool sent_somewhere = false;

        for (size_t s = 0; s < ARRAY_SIZE(slots); s++) {
            sent_somewhere |= group->on[s].state != GROUP_NOT_FOR_IT &&
                              group->on[s].state != GROUP_TO_SEND;
        }
        if (!sent_somewhere) {
            group->param1 = rgb;
            group->param2 = blank_ms;
            group->interrupts |= interrupts;
            k_spin_unlock(&groups_lock, key);
            LOG_DBG("LED sync %u: layer colour replaced by #%06X", group->id, rgb);
            return;
        }
    }

    struct sync_group *group = free_group();
    const uint8_t id = rgbled_widget_sync_new_id();
    const bool made = group != NULL && make_group(group, GROUP_LAYER_COLOUR, id, rgb, blank_ms,
                                                  interrupts, for_slots);
    k_spin_unlock(&groups_lock, key);

    if (!made) {
        if (for_slots != 0) {
            /* Too many at once: here now, and the peripherals catch up. */
            LOG_WRN("LED sync: too many groups at once -- layer colour #%06X unsynced", rgb);
            catch_up_ready_peripherals();
        }
        rgbled_widget_apply_layer_colour_at(rgb, blank_ms, interrupts, k_uptime_get());
        if (for_slots != 0) {
            wake_sender(0);
        }
        return;
    }
    wake_sender(0);
}

/* ---- answers and acknowledgements ---- */

/* The peripheral acknowledged a write. Never called for a write the link
 * dropped; the disconnect clears the slot then. */
static void on_write_acknowledged(struct bt_conn *conn, void *cookie) {
    ARG_UNUSED(conn);

    const uintptr_t value = (uintptr_t)cookie;
    struct peripheral_slot *slot = &slots[value & 0xFF];
    const uint8_t sync_id = (uint8_t)(value >> 8);
    const uint8_t epoch = (uint8_t)(value >> 16);
    const int64_t now = k_uptime_get();

    if (epoch == (uint8_t)atomic_get(&slot->flight_epoch)) {
        one_less_in_flight(slot);
        atomic_set(&slot->in_flight_since, (atomic_val_t)(uint32_t)now);
    }

    if (sync_id != 0) {
        k_spinlock_key_t key = k_spin_lock(&groups_lock);
        struct sync_group *group = group_by_id(sync_id);

        struct group_peripheral *on = group != NULL ? &group->on[slot_index(slot)] : NULL;

        /* The start is timed from here -- also when the answer came first. */
        if (on != NULL && on->state == GROUP_SENT) {
            on->since = now;
            on->state = GROUP_DELIVERED;
        } else if (on != NULL && on->state == GROUP_ANSWERED && !on->acknowledged) {
            on->since = now;
            on->acknowledged = true;
        }
        k_spin_unlock(&groups_lock, key);
    }
    wake_sender(0);
}

static uint8_t on_answer(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                         const void *data, uint16_t length) {
    if (data == NULL) {
        return BT_GATT_ITER_STOP; /* unsubscribed: the link went down */
    }
    if (length < sizeof(struct led_sync_answer)) {
        return BT_GATT_ITER_CONTINUE;
    }

    struct led_sync_answer answer;
    memcpy(&answer, data, sizeof(answer));
    const uint8_t s = bt_conn_index(conn);
    const int64_t now = k_uptime_get();
    bool taken = false;

    k_spinlock_key_t key = k_spin_lock(&groups_lock);
    struct sync_group *group = group_by_id(answer.sync_id);

    if (group != NULL &&
        (group->on[s].state == GROUP_SENT || group->on[s].state == GROUP_DELIVERED)) {
        struct group_peripheral *on = &group->on[s];

        /* Before the acknowledgement: timed from the answer only if no
           acknowledgement follows (check_waits). */
        on->acknowledged = on->state == GROUP_DELIVERED;
        if (!on->acknowledged) {
            on->since = now;
        }
        on->answer_start = (enum led_sync_start)answer.start;
        on->answer_start_in_ms = answer.start_in_ms;
        on->state = GROUP_ANSWERED;
        taken = true;
    }
    k_spin_unlock(&groups_lock, key);

    if (!taken) {
        LOG_DBG("LED sync %u: answer from link slot %u came too late", answer.sync_id, s);
    }
    wake_sender(0);
    return BT_GATT_ITER_CONTINUE;
}

/* ---- discovery ---- */

/* The peripheral takes writes now: synced if it answers. */
static void ready(struct peripheral_slot *slot, bool answers) {
    if (answers) {
        atomic_set_bit(&slot->flags, FLAG_ANSWERS);
    } else {
        atomic_set_bit(&slot->flags, FLAG_NO_ANSWERS);
    }
    wake_sender(0);
}

/* The discovery ended without what it looked for. The link dropping under it
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
        atomic_set(&slot->run_behavior_handle, 0);
        wake_sender(next_retry(slot));
        return;
    }
    LOG_WRN("Layer colour push: link slot %u has no %s", slot_index(slot), what);
    atomic_set_bit(&slot->flags, FLAG_TAKES_NO_WRITES);
    wake_sender(0); /* the groups waiting for it go without it */
}

static bool discovery_is_stale(const struct peripheral_slot *slot) {
    return slot->discovery_generation != (uint32_t)atomic_get(&slot->link_generation);
}

/* The answers were not found -- Zephyr ends a discovery empty, too, when it
 * has no free request slot: looked for again from the start, and after the
 * last try the peripheral takes writes unsynced. */
static void answers_not_found(struct bt_conn *conn, struct peripheral_slot *slot,
                              const char *what) {
    if (!is_up(conn)) {
        return;
    }
    if (atomic_get(&slot->discovery_tries) < DISCOVERY_TRIES) {
        LOG_DBG("Layer colour push: no %s on link slot %u yet, looking again", what,
                slot_index(slot));
        atomic_set(&slot->run_behavior_handle, 0);
        atomic_clear_bit(&slot->flags, FLAG_DISCOVERY_STARTED);
        wake_sender(next_retry(slot));
        return;
    }
    LOG_INF("Layer colour push: link slot %u has no %s (older firmware?) -- its LED follows "
            "unsynced",
            slot_index(slot), what);
    ready(slot, false);
}

/* Continues the discovery with params. */
static void discover_next(struct bt_conn *conn, struct peripheral_slot *slot,
                          struct bt_gatt_discover_params *params, const char *what) {
    int err = bt_gatt_discover(conn, params);

    if (err) {
        LOG_DBG("Layer colour push: %s discovery on link slot %u refused (%d)", what,
                slot_index(slot), err);
        answers_not_found(conn, slot, what);
    }
}

static uint8_t on_answer_char_found(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    struct bt_gatt_discover_params *params) {
    struct peripheral_slot *slot = CONTAINER_OF(params, struct peripheral_slot, discover);

    if (discovery_is_stale(slot)) {
        return BT_GATT_ITER_STOP;
    }
    if (!attr) {
        answers_not_found(conn, slot, "LED sync answer characteristic");
        return BT_GATT_ITER_STOP;
    }

    /* The end of the service stays in end_handle: the CCC is looked for
     * up to it. */
    slot->subscribe.value_handle = ((const struct bt_gatt_chrc *)attr->user_data)->value_handle;
    slot->subscribe.end_handle = params->end_handle;
    atomic_set_bit(&slot->flags, FLAG_SUBSCRIBE_DUE);
    wake_sender(0);
    return BT_GATT_ITER_STOP;
}

static uint8_t on_sync_service_found(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     struct bt_gatt_discover_params *params) {
    struct peripheral_slot *slot = CONTAINER_OF(params, struct peripheral_slot, discover);

    if (discovery_is_stale(slot)) {
        return BT_GATT_ITER_STOP;
    }
    if (!attr) {
        answers_not_found(conn, slot, "LED sync service");
        return BT_GATT_ITER_STOP;
    }

    params->uuid = &answer_char_uuid.uuid;
    params->func = on_answer_char_found;
    params->start_handle = attr->handle + 1;
    params->end_handle = ((const struct bt_gatt_service_val *)attr->user_data)->end_handle;
    params->type = BT_GATT_DISCOVER_CHARACTERISTIC;
    discover_next(conn, slot, params, "answer characteristic");
    return BT_GATT_ITER_STOP;
}

static uint8_t on_run_behavior_found(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     struct bt_gatt_discover_params *params) {
    struct peripheral_slot *slot = CONTAINER_OF(params, struct peripheral_slot, discover);

    if (discovery_is_stale(slot)) {
        return BT_GATT_ITER_STOP;
    }
    if (!attr) {
        discovery_ended_empty(conn, slot, "ZMK run-behavior characteristic");
        return BT_GATT_ITER_STOP;
    }

    const uint16_t handle = ((const struct bt_gatt_chrc *)attr->user_data)->value_handle;

    atomic_set(&slot->run_behavior_handle, handle);
    atomic_set(&slot->retry_ms, RETRY_MS);
    LOG_DBG("Layer colour push: link slot %u takes writes (handle 0x%04x)", slot_index(slot),
            handle);

    /* Reusing the parameters from inside their own callback is allowed: the
     * procedure is over once this returns STOP. */
    params->uuid = &sync_service_uuid.uuid;
    params->func = on_sync_service_found;
    params->start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    params->end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    params->type = BT_GATT_DISCOVER_PRIMARY;
    discover_next(conn, slot, params, "LED sync service");
    return BT_GATT_ITER_STOP;
}

static uint8_t on_split_service_found(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      struct bt_gatt_discover_params *params) {
    struct peripheral_slot *slot = CONTAINER_OF(params, struct peripheral_slot, discover);

    if (discovery_is_stale(slot)) {
        return BT_GATT_ITER_STOP;
    }
    if (!attr) {
        discovery_ended_empty(conn, slot, "ZMK split service");
        return BT_GATT_ITER_STOP;
    }

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

static void on_subscribed(struct bt_conn *conn, uint8_t err,
                          struct bt_gatt_subscribe_params *params) {
    struct peripheral_slot *slot = CONTAINER_OF(params, struct peripheral_slot, subscribe);

    if (discovery_is_stale(slot) || !is_up(conn)) {
        return;
    }
    if (err) {
        LOG_WRN("Layer colour push: link slot %u refused the answers (0x%02x) -- its LED "
                "follows unsynced",
                slot_index(slot), err);
        ready(slot, false);
        return;
    }
    ready(slot, true);
}

/* From the sender: subscribing may wait for a buffer. */
static void subscribe_to_answers(struct bt_conn *conn, struct peripheral_slot *slot) {
    struct bt_gatt_subscribe_params *params = &slot->subscribe;

    params->notify = on_answer;
    params->subscribe = on_subscribed;
    params->value = BT_GATT_CCC_NOTIFY;
    params->ccc_handle = 0; /* found by Zephyr, up to end_handle */
    params->disc_params = &slot->ccc_discover;
    /* Gone with the link: the next one subscribes afresh. */
    atomic_set_bit(params->flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

    int err = bt_gatt_subscribe(conn, params);

    if (err == -ENOMEM) {
        atomic_set_bit(&slot->flags, FLAG_SUBSCRIBE_DUE);
        wake_sender(next_retry(slot));
    } else if (err) {
        LOG_WRN("Layer colour push: subscribing on link slot %u failed (%d) -- its LED "
                "follows unsynced",
                slot_index(slot), err);
        ready(slot, false);
    }
}

/* ---- sending ---- */

/* How long a write may go unacknowledged on a link that is up before it
 * counts as stuck in the host's queue. */
static uint32_t stuck_after_ms(struct bt_conn *conn) {
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info)) {
        return 32000U + STUCK_MARGIN_MS; /* the longest timeout Bluetooth allows */
    }
    return info.le.timeout * 10U + STUCK_MARGIN_MS;
}

/* Writes the two lyr_sync parameters with the group number (0: unsynced) as
 * the position. Returns whether it went out. */
static bool write_lyr_sync(struct bt_conn *conn, struct peripheral_slot *slot, uint16_t handle,
                           uint8_t sync_id, uint32_t param1, uint32_t param2) {
    /* The same payload zmk_split_central_invoke_behavior() builds: pressed,
     * and the two parameters. */
    struct zmk_split_run_behavior_payload payload = {
        .data =
            {
                .position = sync_id,
                .source = 0,
                .state = 1,
                .param1 = param1,
                .param2 = param2,
            },
    };
    memcpy(payload.behavior_dev, LAYER_SYNC_BEHAVIOR, sizeof(LAYER_SYNC_BEHAVIOR));

    const uint8_t epoch = (uint8_t)atomic_get(&slot->flight_epoch);

    /* Counted before the write: the acknowledgement may come before the
     * call returns. */
    if (atomic_inc(&slot->in_flight) == 0) {
        atomic_set(&slot->in_flight_since, (atomic_val_t)k_uptime_get_32());
    }
    if (epoch != (uint8_t)atomic_get(&slot->flight_epoch)) {
        /* The link went down, or its writes were written off, meanwhile. */
        one_less_in_flight(slot);
        return false;
    }

    int err = bt_gatt_write_without_response_cb(
        conn, handle, &payload, sizeof(payload), false, on_write_acknowledged,
        (void *)(((uintptr_t)epoch << 16) | ((uintptr_t)sync_id << 8) | slot_index(slot)));

    if (err) {
        if (epoch == (uint8_t)atomic_get(&slot->flight_epoch)) {
            one_less_in_flight(slot);
        }
        if (is_up(conn)) {
            LOG_WRN("Layer colour push on link slot %u failed (%d), retrying",
                    slot_index(slot), err);
            wake_sender(next_retry(slot));
        }
        return false;
    }
    atomic_set(&slot->retry_ms, RETRY_MS);
    return true;
}

static bool room_in_flight(struct peripheral_slot *slot) {
    return atomic_get(&slot->in_flight) < WRITES_IN_FLIGHT;
}

/* The next group this peripheral is to be sent, in order -- marked sent, by
 * the sender alone -- or false when there is none. */
static bool take_group_to_send(uint8_t s, bool answers, uint8_t *sync_id, uint32_t *param1,
                               uint32_t *param2) {
    enum group_kind kind = GROUP_FREE;
    struct sync_group *next = NULL;
    k_spinlock_key_t key = k_spin_lock(&groups_lock);

    for (size_t i = 0; i < ARRAY_SIZE(groups); i++) {
        if (groups[i].kind != GROUP_FREE && groups[i].on[s].state == GROUP_TO_SEND &&
            (next == NULL || (int32_t)(groups[i].order - next->order) < 0)) {
            next = &groups[i];
        }
    }
    if (next != NULL) {
        *sync_id = next->id;
        *param1 = next->param1;
        *param2 = next->param2;
        kind = next->kind;
        if (kind == GROUP_LAYER_COLOUR && answers && next->interrupts) {
            /* As this half decided: it saw every change, also those it
             * replaced before they went out. */
            *param1 |= RGBLED_WIDGET_PUSHED_COLOUR_CUTS_INDICATIONS;
        }
        next->on[s].state = answers ? GROUP_SENT : GROUP_SENT_UNSYNCED;
        next->on[s].since = k_uptime_get();
    }
    k_spin_unlock(&groups_lock, key);
    if (kind == GROUP_INDICATION && !answers) {
        /* A peripheral without the answers reads the gap as 16 bits: the
         * dark decision is for synced ones only. */
        *param2 &= ~((uint32_t)0x7u << RGBLED_WIDGET_PUSHED_DARK_SHIFT);
    }
    return next != NULL;
}

/* A write that did not go out: the group is to be sent again. */
static void put_group_back(uint8_t s, uint8_t sync_id) {
    k_spinlock_key_t key = k_spin_lock(&groups_lock);
    struct sync_group *group = group_by_id(sync_id);

    if (group != NULL) {
        group->on[s].state = GROUP_TO_SEND;
    }
    k_spin_unlock(&groups_lock, key);
}

/* Discovery and subscription: whether the peripheral takes writes now. */
static bool get_ready(struct bt_conn *conn, struct peripheral_slot *slot, uint32_t generation,
                      uint32_t now) {
    if (atomic_test_bit(&slot->flags, FLAG_READY)) {
        return true;
    }
    if (atomic_test_and_clear_bit(&slot->flags, FLAG_SUBSCRIBE_DUE)) {
        subscribe_to_answers(conn, slot);
        return false;
    }
    if (atomic_test_bit(&slot->flags, FLAG_ANSWERS) ||
        atomic_test_bit(&slot->flags, FLAG_NO_ANSWERS)) {
        atomic_set(&slot->ready_at, (atomic_val_t)now);
        atomic_set_bit(&slot->flags, FLAG_READY);
        if ((uint32_t)atomic_get(&slot->link_generation) != generation ||
            !atomic_test_bit(&slot->flags, FLAG_LINK_SECURED)) {
            /* The link went down under this: its flags were cleared before
             * READY was set, which belongs to no link now. */
            atomic_clear_bit(&slot->flags, FLAG_READY);
            return false;
        }
        atomic_set_bit(&slot->flags, FLAG_COLOUR_CATCH_UP_DUE);
        LOG_INF("Layer colour push: link slot %u ready%s", slot_index(slot),
                atomic_test_bit(&slot->flags, FLAG_ANSWERS) ? ", LEDs synced" : "");
        rgbled_widget_peripheral_ready(slot_index(slot));
        return true;
    }
    if (atomic_get(&slot->run_behavior_handle) == 0 &&
        !atomic_test_bit(&slot->flags, FLAG_DISCOVERY_STARTED)) {
        const uint32_t secured_for = now - (uint32_t)atomic_get(&slot->secured_at);

        if (secured_for < DISCOVERY_DELAY_MS) {
            wake_sender(DISCOVERY_DELAY_MS - secured_for);
        } else {
            atomic_set_bit(&slot->flags, FLAG_DISCOVERY_STARTED);
            start_discovery(conn, slot);
        }
    }
    return false; /* the discovery wakes the sender when it is done */
}

static void send_to(struct bt_conn *conn) {
    struct peripheral_slot *slot = &slots[bt_conn_index(conn)];
    const uint8_t s = slot_index(slot);
    const uint32_t now = k_uptime_get_32();
    /* Read first: a disconnect from here on changes it. */
    const uint32_t generation = (uint32_t)atomic_get(&slot->link_generation);

    if (!get_ready(conn, slot, generation, now)) {
        return;
    }
    const uint16_t handle = (uint16_t)atomic_get(&slot->run_behavior_handle);

    if (handle == 0) {
        return; /* the link went down: nothing to write to */
    }

    if (atomic_get(&slot->in_flight) > 0) {
        const uint32_t waited = now - (uint32_t)atomic_get(&slot->in_flight_since);
        const uint32_t stuck_after = stuck_after_ms(conn);

        if (waited >= stuck_after) {
            LOG_WRN("Layer colour push: writes on link slot %u unacknowledged for %u ms on a "
                    "live link -- sending the colour again",
                    s, waited);
            atomic_inc(&slot->flight_epoch);
            atomic_set(&slot->in_flight, 0);
            atomic_set_bit(&slot->flags, FLAG_COLOUR_CATCH_UP_DUE);
        } else {
            /* An acknowledgement wakes the sender; this, if none comes. */
            wake_sender(stuck_after - waited);
        }
    }

    uint32_t rgb, blank_ms;

    if (atomic_test_bit(&slot->flags, FLAG_COLOUR_CATCH_UP_DUE) && room_in_flight(slot) &&
        get_wanted(&rgb, &blank_ms)) {
        if (!write_lyr_sync(conn, slot, handle, 0, rgb, blank_ms)) {
            return;
        }
        atomic_clear_bit(&slot->flags, FLAG_COLOUR_CATCH_UP_DUE);
        LOG_DBG("Pushed layer colour #%06X (blank after %ums) on link slot %u", rgb, blank_ms,
                s);
    }

    const bool answers = atomic_test_bit(&slot->flags, FLAG_ANSWERS);
    uint8_t sync_id;
    uint32_t param1, param2;

    while (room_in_flight(slot) && take_group_to_send(s, answers, &sync_id, &param1, &param2)) {
        if (!write_lyr_sync(conn, slot, handle, answers ? sync_id : 0, param1, param2)) {
            put_group_back(s, sync_id);
            return;
        }
        LOG_DBG("LED sync %u: sent to link slot %u", sync_id, s);
    }
}

/* ---- settling the groups ---- */

static const char *const left_out_because[] = {
    [GROUP_TO_SEND] = "not sent",
    [GROUP_SENT] = "no acknowledgement",
    [GROUP_DELIVERED] = "no answer",
};

/* Peripherals left out of a group since the last look: logged after the
 * lock is released. The sender's own. */
static struct {
    uint8_t sync_id;
    uint8_t slot;
    enum group_on_peripheral missed;
} left_out[SYNC_GROUPS];
static size_t left_out_count;

static void note_left_out(uint8_t sync_id, size_t s, enum group_on_peripheral missed) {
    if (left_out_count < ARRAY_SIZE(left_out)) {
        left_out[left_out_count].sync_id = sync_id;
        left_out[left_out_count].slot = (uint8_t)s;
        left_out[left_out_count].missed = missed;
        left_out_count++;
    }
}

static void log_left_out(void) {
    for (size_t i = 0; i < left_out_count; i++) {
        const enum group_on_peripheral missed = left_out[i].missed;
        const int limit = missed == GROUP_TO_SEND ? LED_SYNC_WAIT_TO_BE_SENT_MS
                          : missed == GROUP_SENT  ? LED_SYNC_WAIT_FOR_DELIVERY_MS
                                                  : LED_SYNC_WAIT_FOR_ANSWER_MS;

        LOG_WRN("LED sync %u: link slot %u left out -- %s in %d ms", left_out[i].sync_id,
                left_out[i].slot, left_out_because[missed], limit);
    }
    left_out_count = 0;
}

/* Times out the waits of one peripheral, under groups_lock. Returns when to
 * look again, or INT64_MAX. */
static int64_t check_waits(struct sync_group *group, size_t s, int64_t now) {
    struct group_peripheral *on = &group->on[s];
    int64_t limit;

    switch (on->state) {
    case GROUP_TO_SEND:
        limit = LED_SYNC_WAIT_TO_BE_SENT_MS;
        break;
    case GROUP_SENT:
        limit = LED_SYNC_WAIT_FOR_DELIVERY_MS;
        break;
    case GROUP_DELIVERED:
        limit = LED_SYNC_WAIT_FOR_ANSWER_MS;
        break;
    case GROUP_ANSWERED:
        if (on->acknowledged) {
            return INT64_MAX;
        }
        if (now - on->since >= LED_SYNC_WAIT_FOR_ACK_AFTER_ANSWER_MS ||
            on->link_generation != (uint32_t)atomic_get(&slots[s].link_generation)) {
            /* No acknowledgement after all: timed from the answer. */
            on->acknowledged = true;
            return INT64_MAX;
        }
        return on->since + LED_SYNC_WAIT_FOR_ACK_AFTER_ANSWER_MS;
    default:
        return INT64_MAX;
    }
    if (on->link_generation != (uint32_t)atomic_get(&slots[s].link_generation) ||
        atomic_test_bit(&slots[s].flags, FLAG_TAKES_NO_WRITES)) {
        on->state = GROUP_LEFT_OUT; /* gone, or never to be ready: no wait for it */
        return INT64_MAX;
    }
    if (now - on->since >= limit) {
        note_left_out(group->id, s, on->state);
        on->state = GROUP_LEFT_OUT;
        return INT64_MAX;
    }
    return on->since + limit;
}

struct settled_group {
    enum group_kind kind;
    uint8_t id;
    uint32_t param1;
    uint32_t param2;
    bool interrupts;
    int64_t start_at;
};

/* When a group with every peripheral done starts here: at the latest start
 * the answers give, straight after the previous one if they all say so, at
 * once without an answer. */
static int64_t settled_start(const struct sync_group *group, int64_t now) {
    bool answered = false;
    bool timed = false;
    int64_t start_at = 0;

    for (size_t s = 0; s < ARRAY_SIZE(slots); s++) {
        const struct group_peripheral *on = &group->on[s];

        if (on->state != GROUP_ANSWERED) {
            continue;
        }
        answered = true;
        if (on->answer_start == LED_SYNC_START_IN) {
            const int64_t at =
                on->since + MIN(on->answer_start_in_ms, (uint16_t)LED_SYNC_LONGEST_START_IN_MS);

            if (!timed || at > start_at) {
                start_at = at;
            }
            timed = true;
        }
    }
    if (!answered) {
        return now;
    }
    if (!timed) {
        return group->kind == GROUP_INDICATION ? 0 : now;
    }
    return start_at;
}

/* Takes the groups every peripheral is done with, under groups_lock, and
 * returns how many; *wake_at is when to look again. */
static size_t settle_groups(struct settled_group *settled, int64_t now, int64_t *wake_at) {
    size_t count = 0;

    for (size_t i = 0; i < ARRAY_SIZE(groups); i++) {
        struct sync_group *group = &groups[i];

        if (group->kind == GROUP_FREE) {
            continue;
        }
        bool done = true;

        for (size_t s = 0; s < ARRAY_SIZE(slots); s++) {
            const int64_t again = check_waits(group, s, now);

            *wake_at = MIN(*wake_at, again);
            switch (group->on[s].state) {
            case GROUP_TO_SEND:
            case GROUP_SENT:
            case GROUP_DELIVERED:
                done = false;
                break;
            case GROUP_ANSWERED:
                done = done && group->on[s].acknowledged;
                break;
            default:
                break;
            }
        }
        if (!done) {
            continue;
        }
        settled[count++] = (struct settled_group){
            .kind = group->kind,
            .id = group->id,
            .param1 = group->param1,
            .param2 = group->param2,
            .interrupts = group->interrupts,
            .start_at = settled_start(group, now),
        };
        group->kind = GROUP_FREE;
    }
    return count;
}

static void start_settled(const struct settled_group *group, int64_t now) {
    if (group->start_at == 0) {
        LOG_DBG("LED sync %u: starts straight after the previous", group->id);
    } else if (group->start_at + 5 < now) {
        LOG_WRN("LED sync %u: the answer came %d ms after the start it set", group->id,
                (int)(now - group->start_at));
    } else if (group->start_at - now >= LED_SYNC_LOG_WAITING_START_FROM_MS) {
        /* Waiting for what the other half shows first: worth a line outside
           debugging too. */
        LOG_INF("LED sync %u: starts in %d ms", group->id, (int)(group->start_at - now));
    } else {
        LOG_DBG("LED sync %u: starts in %d ms", group->id, (int)(group->start_at - now));
    }

    switch (group->kind) {
    case GROUP_INDICATION:
        rgbled_widget_start_synced_group_at(group->id, group->start_at);
        break;
    case GROUP_LAYER_COLOUR:
        rgbled_widget_apply_layer_colour_at(group->param1, group->param2, group->interrupts,
                                            group->start_at);
        break;
    default:
        break;
    }
}

/* ---- the sender ---- */

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
    sender_started();

    struct secured_links links = {.count = 0};

    /* Collected first, sent after: a write may block for a buffer, which is
     * no place to be inside bt_conn_foreach(). */
    bt_conn_foreach(BT_CONN_TYPE_LE, collect_secured_link, &links);
    for (size_t i = 0; i < links.count; i++) {
        send_to(links.conns[i]);
        bt_conn_unref(links.conns[i]);
    }

    /* The sender's own; kept off its stack. */
    static struct settled_group settled[SYNC_GROUPS];
    const int64_t now = k_uptime_get();
    int64_t wake_at = INT64_MAX;

    k_spinlock_key_t key = k_spin_lock(&groups_lock);
    const size_t count = settle_groups(settled, now, &wake_at);
    k_spin_unlock(&groups_lock, key);

    log_left_out();

    for (size_t i = 0; i < count; i++) {
        start_settled(&settled[i], now);
    }
    if (wake_at != INT64_MAX) {
        wake_sender((uint32_t)MAX(wake_at - now, 1));
    }
}

/* ---- the links coming and going ---- */

static void on_security_changed(struct bt_conn *conn, bt_security_t level,
                                enum bt_security_err err) {
    if (err || level < BT_SECURITY_L2 || !is_peripheral_link(conn)) {
        return;
    }
    struct peripheral_slot *slot = &slots[bt_conn_index(conn)];

    atomic_set(&slot->secured_at, (atomic_val_t)k_uptime_get_32());
    atomic_set_bit(&slot->flags, FLAG_LINK_SECURED);
    wake_sender(DISCOVERY_DELAY_MS);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    ARG_UNUSED(reason);

    if (!is_peripheral_link(conn)) {
        return;
    }

    struct peripheral_slot *slot = &slots[bt_conn_index(conn)];

    /* The writes in flight went down with the link and will not be
     * acknowledged; the groups waiting for it go without it. The next link
     * starts from nothing: it discovers the handles again and is sent the
     * current colour. */
    atomic_inc(&slot->link_generation);
    atomic_inc(&slot->flight_epoch);
    atomic_set(&slot->run_behavior_handle, 0);
    atomic_set(&slot->in_flight, 0);
    atomic_set(&slot->discovery_tries, 0);
    atomic_set(&slot->retry_ms, RETRY_MS);
    (void)atomic_clear(&slot->flags);
    wake_sender(0);
}

BT_CONN_CB_DEFINE(layer_push_conn_callbacks) = {
    .security_changed = on_security_changed,
    .disconnected = on_disconnected,
};

#else /* a wired split */

#include <zmk/behavior.h>
#include <zmk/split/central.h>

void rgbled_widget_push_layer_color_to_peripherals(uint32_t rgb, uint32_t blank_ms) {
    set_wanted(rgb, blank_ms);
    wake_sender(0);
}

#if PUSH_INDICATIONS
/* Events, in order: a peripheral that falls further behind loses the oldest. */
#define PUSHED_QUEUE_LENGTH 8

static struct k_spinlock pushed_lock;
static struct {
    uint32_t param1;
    uint32_t param2;
} pushed[PUSHED_QUEUE_LENGTH];
/* Sequence number of the next indication queued; that of each one is its
 * place in pushed[] modulo the length. Under pushed_lock. */
static uint32_t pushed_next_seq;

/* The indication numbered `*seq` -- or the oldest still queued, if that one
 * was overwritten. Returns false when none is waiting. */
static bool next_pushed(uint32_t *seq, uint32_t *param1, uint32_t *param2) {
    bool found = false;
    k_spinlock_key_t key = k_spin_lock(&pushed_lock);

    if (pushed_next_seq - *seq > PUSHED_QUEUE_LENGTH) {
        *seq = pushed_next_seq - PUSHED_QUEUE_LENGTH;
    }
    if (*seq != pushed_next_seq) {
        const uint32_t i = *seq % PUSHED_QUEUE_LENGTH;

        *param1 = pushed[i].param1;
        *param2 = pushed[i].param2;
        found = true;
    }
    k_spin_unlock(&pushed_lock, key);
    return found;
}

void rgbled_widget_push_indication_to_peripherals(uint32_t param1, uint32_t param2) {
    k_spinlock_key_t key = k_spin_lock(&pushed_lock);
    const uint32_t i = pushed_next_seq % PUSHED_QUEUE_LENGTH;

    pushed[i].param1 = param1;
    pushed[i].param2 = param2;
    pushed_next_seq++;
    k_spin_unlock(&pushed_lock, key);

    wake_sender(0);
}
#endif

static bool sent_valid;
static uint32_t sent_rgb;
static uint32_t sent_ms;
static uint32_t retry_ms = RETRY_MS;
#if PUSH_INDICATIONS
static uint32_t wired_pushed_seq;
#endif

/* The two lyr_sync parameters to every peripheral. Returns 0 or the last
 * error. */
static int invoke_on_peripherals(uint32_t param1, uint32_t param2) {
    struct zmk_behavior_binding binding = {
        .behavior_dev = LAYER_SYNC_BEHAVIOR,
        .param1 = param1,
        .param2 = param2,
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
            LOG_DBG("Push to peripheral %u failed (%d)", i, err);
            failed = err;
        }
    }
    return failed;
}

/* The colour, if it changed. Returns false when it is to be tried again. */
static bool send_colour(void) {
    uint32_t rgb, blank_ms;

    if (!get_wanted(&rgb, &blank_ms) ||
        (sent_valid && sent_rgb == rgb && sent_ms == blank_ms)) {
        return true;
    }
    if (invoke_on_peripherals(rgb, blank_ms) != 0) {
        wake_sender(retry_ms);
        retry_ms = MIN(2U * retry_ms, (uint32_t)MAX_RETRY_MS);
        return false;
    }
    retry_ms = RETRY_MS;
    sent_rgb = rgb;
    sent_ms = blank_ms;
    sent_valid = true;
    LOG_DBG("Pushed layer colour #%06X (blank after %ums)", rgb, blank_ms);
    return true;
}

static void send_work_cb(struct k_work *work) {
    ARG_UNUSED(work);
    sender_started();

    /* The colour first, as over Bluetooth: a new one cuts what shows. A
     * failure is tried again later; the indications go now all the same. */
    (void)send_colour();

#if PUSH_INDICATIONS
    /* Then the indications to every peripheral, in order; one that is not
     * there misses them -- a wired peripheral coming back is not reported. */
    uint32_t param1, param2;

    while (next_pushed(&wired_pushed_seq, &param1, &param2)) {
        (void)invoke_on_peripherals(param1, param2);
        wired_pushed_seq++;
    }
#endif
}

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE) */
