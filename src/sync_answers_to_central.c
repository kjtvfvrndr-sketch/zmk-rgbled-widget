/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Split peripheral: answers the central's synced indications and layer
 * colours (led_sync.h) -- when this half will start each -- as notifications
 * of a characteristic of its own, which the central subscribes to.
 *
 * The answer is sent from the system work queue, where Zephyr allocates
 * without waiting: a buffer that is not there is tried for again shortly. An
 * answer is only of use before the start it gives, so one that cannot go out
 * by then is dropped, and the central shows that group on its own.
 */

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include "led_sync.h"

LOG_MODULE_DECLARE(rgbled_widget, CONFIG_RGBLED_WIDGET_LOG_LEVEL);

/* A failed send is tried again this often, this many times. */
#define ANSWER_RETRY_MS 3
#define ANSWER_TRIES 5
/* Answers not sent yet; one per group in flight is plenty. */
#define ANSWERS_WAITING 8

static void answers_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    LOG_DBG("LED sync: the central %s the answers",
            value == BT_GATT_CCC_NOTIFY ? "subscribed to" : "unsubscribed from");
}

BT_GATT_SERVICE_DEFINE(led_sync_service, BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(
                                             LED_SYNC_SERVICE_UUID)),
                       BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(LED_SYNC_ANSWER_CHAR_UUID),
                                              BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL,
                                              NULL),
                       BT_GATT_CCC(answers_ccc_changed,
                                   BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT), );

/* The characteristic's value attribute: the service, its declaration, then it. */
#define ANSWER_ATTR (&led_sync_service.attrs[2])

/* ---- the link to the central ---- */

static void find_link_to_central(struct bt_conn *conn, void *data) {
    struct bt_conn **found = data;
    struct bt_conn_info info;

    if (*found == NULL && !bt_conn_get_info(conn, &info) && info.type == BT_CONN_TYPE_LE &&
        info.role == BT_CONN_ROLE_PERIPHERAL && info.state == BT_CONN_STATE_CONNECTED) {
        *found = bt_conn_ref(conn);
    }
}

/* The link to the central, referenced; NULL when there is none. */
static struct bt_conn *link_to_central(void) {
    struct bt_conn *conn = NULL;

    bt_conn_foreach(BT_CONN_TYPE_LE, find_link_to_central, &conn);
    return conn;
}

uint32_t rgbled_widget_sync_shortest_start_in_ms(void) {
    struct bt_conn *conn = link_to_central();
    uint32_t interval_ms = 0;

    if (conn != NULL) {
        struct bt_conn_info info;

        if (!bt_conn_get_info(conn, &info)) {
            /* 1.25 ms units, rounded up. */
            interval_ms = (info.le.interval * 5U + 3U) / 4U;
        }
        bt_conn_unref(conn);
    }
    return interval_ms + LED_SYNC_ANSWER_MARGIN_MS;
}

/* ---- the answers ---- */

static struct k_spinlock answers_lock;
static struct {
    struct led_sync_answer answer;
    /* No use after this uptime: the start it gives has come. */
    int64_t useless_after;
    uint8_t tries;
} waiting[ANSWERS_WAITING];
static uint32_t first_waiting;
static uint32_t waiting_count;

static void send_answers_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(send_answers_work, send_answers_cb);

static void send_answers_cb(struct k_work *work) {
    ARG_UNUSED(work);

    struct bt_conn *conn = link_to_central();

    for (;;) {
        k_spinlock_key_t key = k_spin_lock(&answers_lock);

        if (waiting_count == 0) {
            k_spin_unlock(&answers_lock, key);
            break;
        }
        const struct led_sync_answer answer = waiting[first_waiting].answer;
        const int64_t useless_after = waiting[first_waiting].useless_after;
        k_spin_unlock(&answers_lock, key);

        int err = -ENOTCONN;

        if (k_uptime_get() >= useless_after) {
            err = -ETIMEDOUT;
        } else if (conn != NULL) {
            err = bt_gatt_notify(conn, ANSWER_ATTR, &answer, sizeof(answer));
        }

        key = k_spin_lock(&answers_lock);
        const bool retry = (err == -ENOMEM || err == -ENOBUFS) &&
                           ++waiting[first_waiting].tries < ANSWER_TRIES &&
                           k_uptime_get() < useless_after;
        if (!retry) {
            first_waiting = (first_waiting + 1) % ANSWERS_WAITING;
            waiting_count--;
        }
        k_spin_unlock(&answers_lock, key);

        if (retry) {
            k_work_reschedule(&send_answers_work, K_MSEC(ANSWER_RETRY_MS));
            break;
        }
        if (err == -ETIMEDOUT) {
            LOG_WRN("LED sync %u: answer too late to send -- the central shows it alone",
                    answer.sync_id);
        } else if (err) {
            LOG_WRN("LED sync %u: answer not sent (%d) -- the central shows it alone",
                    answer.sync_id, err);
        } else if (answer.start == LED_SYNC_START_AFTER_PREVIOUS) {
            LOG_DBG("LED sync %u: answered -- straight after the previous", answer.sync_id);
        } else {
            LOG_DBG("LED sync %u: answered -- start in %u ms", answer.sync_id,
                    answer.start_in_ms);
        }
    }

    if (conn != NULL) {
        bt_conn_unref(conn);
    }
}

void rgbled_widget_answer_central(uint8_t sync_id, enum led_sync_start start,
                                  uint32_t start_in_ms) {
    k_spinlock_key_t key = k_spin_lock(&answers_lock);

    if (waiting_count == ANSWERS_WAITING) {
        /* The oldest is the most likely to be late already. */
        first_waiting = (first_waiting + 1) % ANSWERS_WAITING;
        waiting_count--;
    }
    const uint32_t i = (first_waiting + waiting_count) % ANSWERS_WAITING;

    waiting[i].answer = (struct led_sync_answer){
        .sync_id = sync_id,
        .start = (uint8_t)start,
        .start_in_ms = (uint16_t)MIN(start_in_ms, (uint32_t)UINT16_MAX),
    };
    /* An answer to follow the previous group is of use as long as that one
     * is; give it the longest a start may be. */
    waiting[i].useless_after =
        k_uptime_get() + (start == LED_SYNC_START_IN ? MAX(start_in_ms, LED_SYNC_ANSWER_MARGIN_MS)
                                                     : (uint32_t)LED_SYNC_LONGEST_START_IN_MS);
    waiting[i].tries = 0;
    waiting_count++;
    k_spin_unlock(&answers_lock, key);

    k_work_reschedule(&send_answers_work, K_NO_WAIT);
}
