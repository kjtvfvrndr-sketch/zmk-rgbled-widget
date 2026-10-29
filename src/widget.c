#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <stdio.h>
#include <string.h>

#if IS_ENABLED(CONFIG_SOC_FAMILY_NRF)
#include <hal/nrf_power.h>
#endif

#include <zmk/battery.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/split_peripheral_status_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/keymap.h>
#include <zmk/split/bluetooth/peripheral.h>

#include <zmk/split/central.h>
#include <zmk/workqueue.h>

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
#include <zmk/usb.h>
#include <zmk/events/usb_conn_state_changed.h>
#endif

#if IS_ENABLED(CONFIG_BT)
#include <zephyr/bluetooth/conn.h>
#endif

#include <zephyr/logging/log.h>

#include <zmk_rgbled_widget/widget.h>

#include "battery_gradient.h"
#include "led_indication_queue.h"
#include "led_indication_triggers.h"
#include "led_sync.h"

LOG_MODULE_REGISTER(rgbled_widget, CONFIG_RGBLED_WIDGET_LOG_LEVEL);

BUILD_ASSERT(DT_NODE_EXISTS(DT_ALIAS(led_strip)),
             "An alias 'led-strip' is not found for RGBLED_WIDGET");

BUILD_ASSERT(!(SHOW_LAYER_CHANGE && SHOW_LAYER_COLORS),
             "CONFIG_RGBLED_WIDGET_SHOW_LAYER_CHANGE and CONFIG_RGBLED_WIDGET_SHOW_LAYER_COLORS "
             "are mutually exclusive");

// Addressable (WS2812-compatible) single-pixel strip
static const struct device *led_dev = DEVICE_DT_GET(DT_ALIAS(led_strip));

#if SHOW_LAYER_COLORS
static const uint32_t layer_rgb[] = {
    CONFIG_RGBLED_WIDGET_LAYER_0_RGB,  CONFIG_RGBLED_WIDGET_LAYER_1_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_2_RGB,  CONFIG_RGBLED_WIDGET_LAYER_3_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_4_RGB,  CONFIG_RGBLED_WIDGET_LAYER_5_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_6_RGB,  CONFIG_RGBLED_WIDGET_LAYER_7_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_8_RGB,  CONFIG_RGBLED_WIDGET_LAYER_9_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_10_RGB, CONFIG_RGBLED_WIDGET_LAYER_11_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_12_RGB, CONFIG_RGBLED_WIDGET_LAYER_13_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_14_RGB, CONFIG_RGBLED_WIDGET_LAYER_15_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_16_RGB, CONFIG_RGBLED_WIDGET_LAYER_17_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_18_RGB, CONFIG_RGBLED_WIDGET_LAYER_19_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_20_RGB, CONFIG_RGBLED_WIDGET_LAYER_21_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_22_RGB, CONFIG_RGBLED_WIDGET_LAYER_23_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_24_RGB, CONFIG_RGBLED_WIDGET_LAYER_25_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_26_RGB, CONFIG_RGBLED_WIDGET_LAYER_27_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_28_RGB, CONFIG_RGBLED_WIDGET_LAYER_29_RGB,
    CONFIG_RGBLED_WIDGET_LAYER_30_RGB, CONFIG_RGBLED_WIDGET_LAYER_31_RGB,
};

static const uint32_t layer_ms[] = {
    CONFIG_RGBLED_WIDGET_LAYER_0_MS,
    CONFIG_RGBLED_WIDGET_LAYER_1_MS,
    CONFIG_RGBLED_WIDGET_LAYER_2_MS,
    CONFIG_RGBLED_WIDGET_LAYER_3_MS,
    CONFIG_RGBLED_WIDGET_LAYER_4_MS,
    CONFIG_RGBLED_WIDGET_LAYER_5_MS,
    CONFIG_RGBLED_WIDGET_LAYER_6_MS,
    CONFIG_RGBLED_WIDGET_LAYER_7_MS,
    CONFIG_RGBLED_WIDGET_LAYER_8_MS,
    CONFIG_RGBLED_WIDGET_LAYER_9_MS,
    CONFIG_RGBLED_WIDGET_LAYER_10_MS,
    CONFIG_RGBLED_WIDGET_LAYER_11_MS,
    CONFIG_RGBLED_WIDGET_LAYER_12_MS,
    CONFIG_RGBLED_WIDGET_LAYER_13_MS,
    CONFIG_RGBLED_WIDGET_LAYER_14_MS,
    CONFIG_RGBLED_WIDGET_LAYER_15_MS,
    CONFIG_RGBLED_WIDGET_LAYER_16_MS,
    CONFIG_RGBLED_WIDGET_LAYER_17_MS,
    CONFIG_RGBLED_WIDGET_LAYER_18_MS,
    CONFIG_RGBLED_WIDGET_LAYER_19_MS,
    CONFIG_RGBLED_WIDGET_LAYER_20_MS,
    CONFIG_RGBLED_WIDGET_LAYER_21_MS,
    CONFIG_RGBLED_WIDGET_LAYER_22_MS,
    CONFIG_RGBLED_WIDGET_LAYER_23_MS,
    CONFIG_RGBLED_WIDGET_LAYER_24_MS,
    CONFIG_RGBLED_WIDGET_LAYER_25_MS,
    CONFIG_RGBLED_WIDGET_LAYER_26_MS,
    CONFIG_RGBLED_WIDGET_LAYER_27_MS,
    CONFIG_RGBLED_WIDGET_LAYER_28_MS,
    CONFIG_RGBLED_WIDGET_LAYER_29_MS,
    CONFIG_RGBLED_WIDGET_LAYER_30_MS,
    CONFIG_RGBLED_WIDGET_LAYER_31_MS,
};
#endif

// log shorthands
#define LOG_CONN_CENTRAL(index, status, color_label)                                               \
    LOG_INF("Profile %d %s, blinking #%06X", index, status,                                        \
            CONFIG_RGBLED_WIDGET_CONN_RGB_##color_label)
#define LOG_CONN_PERIPHERAL(status, color_label)                                                   \
    LOG_INF("Peripheral %s, blinking #%06X", status,                                               \
            CONFIG_RGBLED_WIDGET_CONN_RGB_##color_label)
#define LOG_BATTERY(battery_level, color_label)                                                    \
    LOG_INF("Battery level %d, blinking #%06X", battery_level,                                     \
            CONFIG_RGBLED_WIDGET_BATTERY_RGB_##color_label)

#define POWER_BREATHE IS_ENABLED(CONFIG_RGBLED_WIDGET_POWER_CONNECTED_BREATHE)

// flag to indicate whether the initial boot up sequence has begun: from then
// on the listeners act
static bool initialized = false;

// Set once the boot report has shown the connection (or a layer colour cut it
// short): connection events blink from then on, and before it the report
// shows how the link settled.
static atomic_t boot_connection_shown;

// Counts the layer colours that cut the indications short. The boot report
// notes it when it begins waiting and gives up if it moved: the keyboard is
// in use.
static atomic_t layer_colour_interrupts;

/*
 * The strip and the queue of indications.
 *
 * Only led_process_thread drives the strip (but for the SLEEP listener's
 * black, below) and only it touches the queue; everyone else sends it
 * commands.
 */

// global brightness scaling, 0..100
static inline uint8_t scale(uint16_t v) {
    return (uint8_t)((v * CONFIG_RGBLED_WIDGET_BRIGHTNESS) / 100);
}

// Set by SLEEP, cleared by any other activity state: from then on the strip
// is only ever sent black. The SLEEP listener blanks the LED, but other threads
// may still run before the power goes off -- a listener that waits, such as
// one closing the split link first -- and this thread would otherwise paint a
// colour pushed from the central, or the rest of a blink, which a WS2812 then
// keeps through System OFF. The lock makes the check and the write one step.
static bool leds_dark_for_sleep;
static K_MUTEX_DEFINE(led_strip_lock);

// Returns whether they were dark already.
static bool keep_leds_dark_for_sleep(bool dark) {
    k_mutex_lock(&led_strip_lock, K_FOREVER);
    const bool was = leds_dark_for_sleep;
    leds_dark_for_sleep = dark;
    k_mutex_unlock(&led_strip_lock);
    return was;
}

static void write_strip(uint32_t rgb) {
    k_mutex_lock(&led_strip_lock, K_FOREVER);
    if (leds_dark_for_sleep) {
        rgb = 0;
    }

    // Per-channel gain, applied as a percentage of the nominal value: 100
    // leaves the channel exactly as written in the hex colour. This is what
    // makes 0xFFFFFF actually render as white -- an asymmetric gain silently
    // turns every hex code into something other than what it says.
    struct led_rgb px = {
        .r = scale((((rgb >> 16) & 0xFF) * CONFIG_RGBLED_WIDGET_GAIN_R) / 100),
        .g = scale((((rgb >> 8) & 0xFF) * CONFIG_RGBLED_WIDGET_GAIN_G) / 100),
        .b = scale(((rgb & 0xFF) * CONFIG_RGBLED_WIDGET_GAIN_B) / 100),
    };

    int err = led_strip_update_rgb(led_dev, &px, 1);
    k_mutex_unlock(&led_strip_lock);
    if (err < 0) {
        LOG_ERR("Failed to update LED strip: %d", err);
    }
}

enum led_command_kind {
    LED_COMMAND_ADD_INDICATION,
    // Central: a group for every half, held at the head of the queue until
    // the peripherals have said when it starts (led_sync.h).
    LED_COMMAND_ADD_SYNCED_GROUP,
    // Peripheral: a group the central syncs: queued at the start this half
    // can manage, which is the answer.
    LED_COMMAND_ADD_SYNCED_FROM_CENTRAL,
    // Central: when a synced group starts.
    LED_COMMAND_SET_SYNCED_START,
    LED_COMMAND_SET_REST,
    // The layer colour as this half last showed it, again: back from idle.
    LED_COMMAND_RESTORE_LAYER_COLOUR,
    LED_COMMAND_STOP_BREATHING,
    LED_COMMAND_CLEAR,
};

struct led_command {
    enum led_command_kind kind;
    // ADD_*: count of these, one after another.
    struct led_indication indication;
    uint8_t count;
    // Synced: the group's number (SET_REST: 0 when not synced).
    uint8_t sync_id;
    // From the central: the uptime its write came, which the start is
    // timed from.
    int64_t received_at;
    // SET_SYNCED_START: the start (0: straight after the group before);
    // SET_REST: when it shows (0: at once).
    int64_t at;
    uint32_t rest_rgb;
    uint32_t rest_blank_ms;
    bool drop_indications;
    // SET_REST: a layer colour -- remembered, once shown, for the way back
    // from idle.
    bool layer_colour;
    // SET_REST: a layer change that waits for the colour before it to have
    // shown its while (RGBLED_WIDGET_LAYER_COLOUR_SHOWN_AT_LEAST_MS).
    bool waits_for_previous_colour;
};

// Room kept for the commands that change state -- a synced start, a layer
// colour, a group from the central: a burst of plain indications cannot take
// it, and the state is not lost to them.
#define LED_COMMANDS_KEPT_FOR_STATE 8

K_MSGQ_DEFINE(led_command_msgq, sizeof(struct led_command),
              RGBLED_WIDGET_INDICATION_QUEUE_LENGTH + LED_COMMANDS_KEPT_FOR_STATE, 4);
// widget.h promises callers this many indications.
BUILD_ASSERT(LED_INDICATION_QUEUE_LENGTH == RGBLED_WIDGET_INDICATION_QUEUE_LENGTH,
             "RGBLED_WIDGET_INDICATION_QUEUE_LENGTH must match LED_INDICATION_QUEUE_LENGTH");

// Returns whether the command was queued.
static bool send_led_command(const struct led_command *command) {
    if (command->kind == LED_COMMAND_ADD_INDICATION) {
        if (k_msgq_num_free_get(&led_command_msgq) <= LED_COMMANDS_KEPT_FOR_STATE ||
            k_msgq_put(&led_command_msgq, command, K_NO_WAIT) != 0) {
            LOG_DBG("LED commands full, indication #%06X dropped", command->indication.rgb);
            return false;
        }
        return true;
    }
    if (k_msgq_put(&led_command_msgq, command, K_NO_WAIT) == 0) {
        return true;
    }
    // A full queue of commands the LED thread has not got to: the state
    // that follows outranks them.
    LOG_WRN("LED commands full, dropping them for a change of state");
    k_msgq_purge(&led_command_msgq);
    return k_msgq_put(&led_command_msgq, command, K_NO_WAIT) == 0;
}

static struct led_indication blink_indication(uint32_t rgb, uint32_t on_ms, uint32_t gap_ms) {
    return (struct led_indication){
        .kind = LED_INDICATION_BLINK, .rgb = rgb, .on_ms = on_ms, .gap_ms = gap_ms};
}

static bool queue_blink(uint32_t rgb, uint32_t on_ms, uint32_t gap_ms) {
    const struct led_command command = {
        .kind = LED_COMMAND_ADD_INDICATION,
        .indication = blink_indication(rgb, on_ms, gap_ms),
        .count = 1,
    };
    return send_led_command(&command);
}

// For other modules (widget.h): count something out on the LED. Each blink is
// an ordinary indication in the same queue, so it waits behind what is already
// showing, and a new layer colour cuts it short like any other.
int rgbled_widget_blink(uint32_t rgb, uint8_t count, uint16_t on_ms, uint16_t off_ms) {
    // on_ms 0 would be an instant blink, and gap_ms 0 falls back to
    // RGBLED_WIDGET_INTERVAL_MS -- keep both what the caller asked for.
    const uint32_t on = MAX(on_ms, 1);
    const uint32_t gap = MAX(off_ms, 1);

    int queued = 0;

    for (uint8_t i = 0; i < count; i++) {
        queued += queue_blink(rgb, on, gap) ? 1 : 0;
    }
    return queued;
}

#if SYNC_WITH_PERIPHERALS
// Central: count of indication on this half and on the peripherals in
// slot_mask at once, sent to them as param1/param2 when its turn comes.
static bool queue_synced_group(const struct led_indication *indication, uint8_t count,
                               uint32_t param1, uint32_t param2, uint32_t slot_mask) {
    struct led_command command = {
        .kind = LED_COMMAND_ADD_SYNCED_GROUP,
        .indication = *indication,
        .count = count,
        .sync_id = rgbled_widget_sync_new_id(),
    };

    command.indication.peripheral_param1 = param1;
    command.indication.peripheral_param2 = param2;
    command.indication.peripheral_slot_mask = slot_mask;
    return send_led_command(&command);
}

void rgbled_widget_start_synced_group_at(uint8_t sync_id, int64_t start_at) {
    const struct led_command command = {
        .kind = LED_COMMAND_SET_SYNCED_START,
        .sync_id = sync_id,
        .at = start_at,
    };
    send_led_command(&command);
}
#endif

// count blinks of rgb that concern the whole keyboard: on a central, on the
// peripherals too (PUSH_INDICATIONS) -- over Bluetooth at once with them.
// gap_ms 0 is RGBLED_WIDGET_INTERVAL_MS. Returns how many were queued.
static int queue_blinks_on_all_halves(uint32_t rgb, uint8_t count, uint32_t on_ms,
                                      uint32_t gap_ms) {
    if (count == 0) {
        return 0;
    }
#if PUSH_INDICATIONS
    // What a peripheral is sent holds 15 at most, and each half shows the
    // same.
    count = MIN(count, 15);
    gap_ms = MIN(gap_ms, RGBLED_WIDGET_PUSHED_GAP_MAX_MS);
    const uint32_t param1 = rgbled_widget_pushed_param1(RGBLED_WIDGET_PUSHED_BLINK, count, rgb);
    const uint32_t param2 = rgbled_widget_pushed_param2((uint16_t)MIN(on_ms, UINT16_MAX),
                                                        (uint16_t)MIN(gap_ms, UINT16_MAX));
#if SYNC_WITH_PERIPHERALS
    const struct led_indication blink = blink_indication(rgb, on_ms, gap_ms);

    return queue_synced_group(&blink, count, param1, param2, UINT32_MAX) ? count : 0;
#else
    rgbled_widget_push_indication_to_peripherals(param1, param2);
#endif
#endif
    int queued = 0;

    for (uint8_t i = 0; i < count; i++) {
        queued += queue_blink(rgb, on_ms, gap_ms) ? 1 : 0;
    }
    return queued;
}

int rgbled_widget_blink_on_all_halves(uint32_t rgb, uint8_t count, uint16_t on_ms,
                                      uint16_t off_ms) {
    // As rgbled_widget_blink(): what the caller asked for, never 0.
    return queue_blinks_on_all_halves(rgb, count, MAX(on_ms, 1), MAX(off_ms, 1));
}

// The colour between indications, at once. With drop_indications it cuts
// them short.
static void set_rest(uint32_t rgb, uint32_t blank_ms, bool drop_indications) {
    if (drop_indications) {
        atomic_inc(&layer_colour_interrupts);
    }
    const struct led_command command = {.kind = LED_COMMAND_SET_REST,
                                        .rest_rgb = rgb,
                                        .rest_blank_ms = blank_ms,
                                        .drop_indications = drop_indications};
    send_led_command(&command);
}

#if !SYNC_WITH_PERIPHERALS && SHOW_LAYER_COLORS
// A new layer colour on this half, at once -- after the one before it has
// shown its while.
static void set_layer_rest(uint32_t rgb, uint32_t blank_ms, bool drop_indications) {
    if (drop_indications) {
        atomic_inc(&layer_colour_interrupts);
    }
    const struct led_command command = {.kind = LED_COMMAND_SET_REST,
                                        .rest_rgb = rgb,
                                        .rest_blank_ms = blank_ms,
                                        .drop_indications = drop_indications,
                                        .layer_colour = true,
                                        .waits_for_previous_colour = true};
    send_led_command(&command);
}
#endif

static void clear_indications(void) {
    const struct led_command command = {.kind = LED_COMMAND_CLEAR};
    k_msgq_purge(&led_command_msgq);
    send_led_command(&command);
}

/*
 * The LED thread's own: resting colours that show at a set moment -- a layer
 * colour synced with the other halves -- in the order they come due.
 */
#define TIMED_RESTS 8

static struct {
    int64_t at;
    uint32_t rgb;
    uint32_t blank_ms;
    bool drop_indications;
    bool layer_colour;
} timed_rests[TIMED_RESTS];
static uint32_t timed_rest_count;

// The layer colour as this half last showed it: what comes back from idle.
static uint32_t shown_layer_rgb;
static uint32_t shown_layer_blank_ms;

// The earliest the next layer colour may show: the one before it shows at
// least RGBLED_WIDGET_LAYER_COLOUR_SHOWN_AT_LEAST_MS -- or until it blanks,
// if sooner -- so that a quick tap still flashes in full. A colour that
// lights nothing (0) holds nothing back. No colour waits longer than that
// time past its own moment, so a burst of changes does not pile up. Every
// half computes it from the same moments, so they stay together.
static int64_t next_layer_colour_not_before;

static int64_t layer_colour_shows_at(int64_t at, uint32_t rgb, uint32_t blank_ms) {
    const int64_t shows_at = MIN(MAX(at, next_layer_colour_not_before),
                                 at + CONFIG_RGBLED_WIDGET_LAYER_COLOUR_SHOWN_AT_LEAST_MS);
    uint32_t held_ms = CONFIG_RGBLED_WIDGET_LAYER_COLOUR_SHOWN_AT_LEAST_MS;

    if (blank_ms > 0) {
        held_ms = MIN(held_ms, blank_ms);
    }
    next_layer_colour_not_before = rgb != 0 ? shows_at + held_ms : shows_at;
    return shows_at;
}

static void apply_rest(struct led_indication_queue *queue, uint32_t rgb, uint32_t blank_ms,
                       bool drop_indications, bool layer_colour, int64_t now) {
    if (layer_colour) {
        shown_layer_rgb = rgb;
        shown_layer_blank_ms = blank_ms;
    }
    led_indication_queue_set_rest(queue, rgb, blank_ms, drop_indications, now);
}

// Shows those due by now; returns when the next one is (or NO_DEADLINE).
static int64_t apply_due_rests(struct led_indication_queue *queue, int64_t now) {
    while (timed_rest_count > 0 && timed_rests[0].at <= now) {
        apply_rest(queue, timed_rests[0].rgb, timed_rests[0].blank_ms,
                   timed_rests[0].drop_indications, timed_rests[0].layer_colour, now);
        timed_rest_count--;
        memmove(&timed_rests[0], &timed_rests[1], timed_rest_count * sizeof(timed_rests[0]));
    }
    return timed_rest_count > 0 ? timed_rests[0].at : LED_INDICATION_NO_DEADLINE;
}

static void add_timed_rest(struct led_indication_queue *queue, int64_t at, uint32_t rgb,
                           uint32_t blank_ms, bool drop_indications, bool layer_colour,
                           int64_t now) {
    if (at <= now) {
        // Due already -- but after those due before it.
        (void)apply_due_rests(queue, now);
        apply_rest(queue, rgb, blank_ms, drop_indications, layer_colour, now);
        return;
    }
    if (timed_rest_count == TIMED_RESTS) {
        // The earliest goes now: a later colour replaces it soon anyway.
        timed_rests[0].at = now;
        (void)apply_due_rests(queue, now);
    }
    uint32_t i = timed_rest_count;

    while (i > 0 && timed_rests[i - 1].at > at) {
        timed_rests[i] = timed_rests[i - 1];
        i--;
    }
    timed_rests[i].at = at;
    timed_rests[i].rgb = rgb;
    timed_rests[i].blank_ms = blank_ms;
    timed_rests[i].drop_indications = drop_indications;
    timed_rests[i].layer_colour = layer_colour;
    timed_rest_count++;
}

static void add_indications(struct led_indication_queue *queue,
                            const struct led_command *command, int64_t now) {
    for (uint8_t i = 0; i < MAX(command->count, 1); i++) {
        if (!led_indication_queue_add(queue, &command->indication, now)) {
            LOG_DBG("LED queue full, indication #%06X dropped", command->indication.rgb);
            return;
        }
    }
}

#if SYNC_WITH_PERIPHERALS
// The first of the group waits for its start; the others follow it.
static void add_synced_group(struct led_indication_queue *queue,
                             const struct led_command *command, int64_t now) {
    struct led_indication indication = command->indication;

    indication.sync_id = command->sync_id;
    indication.waits_for_start = true;
    indication.start_at = 0;
    for (uint8_t i = 0; i < MAX(command->count, 1); i++) {
        if (!led_indication_queue_add(queue, &indication, now)) {
            LOG_DBG("LED queue full, indication #%06X dropped", indication.rgb);
            return;
        }
        indication.waits_for_start = false;
    }
}
#endif

#if SYNC_WITH_CENTRAL
// Straight after a synced group that waits, as the central does with it;
// else when what shows is over, but no sooner than the answer can reach the
// central.
static void add_synced_from_central(struct led_indication_queue *queue,
                                    const struct led_command *command, int64_t now) {
    const bool after_previous = led_indication_queue_synced_waiting(queue);
    struct led_indication indication = command->indication;
    uint32_t start_in_ms = 0;

    indication.sync_id = command->sync_id;
    indication.start_at = 0;
    if (!after_previous) {
        const int64_t earliest =
            command->received_at + rgbled_widget_sync_shortest_start_in_ms();
        const int64_t free_at = led_indication_queue_current_ends_at(queue, now);

        indication.start_at = MAX(earliest, free_at);
        start_in_ms = (uint32_t)(indication.start_at - command->received_at);
    }
    (void)led_indication_queue_add_synced(queue, &indication, MAX(command->count, 1), now);
    rgbled_widget_answer_central(command->sync_id,
                                 after_previous ? LED_SYNC_START_AFTER_PREVIOUS
                                                : LED_SYNC_START_IN,
                                 start_in_ms);
    if (after_previous) {
        LOG_DBG("LED sync %u: #%06X x%u from the central, after the previous",
                command->sync_id, indication.rgb, command->count);
    } else {
        LOG_DBG("LED sync %u: #%06X x%u from the central, starts in %u ms", command->sync_id,
                indication.rgb, command->count, start_in_ms);
    }
}
#endif

static void apply_led_command(struct led_indication_queue *queue,
                              const struct led_command *command, int64_t now) {
    switch (command->kind) {
    case LED_COMMAND_ADD_INDICATION:
        add_indications(queue, command, now);
        break;
#if SYNC_WITH_PERIPHERALS
    case LED_COMMAND_ADD_SYNCED_GROUP:
        add_synced_group(queue, command, now);
        break;
    case LED_COMMAND_SET_SYNCED_START:
        if (!led_indication_queue_set_start(queue, command->sync_id, command->at)) {
            LOG_DBG("LED sync %u: cut short before its start", command->sync_id);
        }
        break;
#endif
#if SYNC_WITH_CENTRAL
    case LED_COMMAND_ADD_SYNCED_FROM_CENTRAL:
        add_synced_from_central(queue, command, now);
        break;
#endif
    case LED_COMMAND_SET_REST: {
        int64_t at = command->at;
#if SYNC_WITH_CENTRAL
        if (command->sync_id != 0) {
            // As an indication with nothing before it: a layer colour cuts
            // what shows short.
            const uint32_t start_in_ms = rgbled_widget_sync_shortest_start_in_ms();

            at = command->received_at + start_in_ms;
            rgbled_widget_answer_central(command->sync_id, LED_SYNC_START_IN, start_in_ms);
            LOG_DBG("LED sync %u: layer colour #%06X from the central, shows in %u ms",
                    command->sync_id, command->rest_rgb, start_in_ms);
        }
#endif
        if (command->waits_for_previous_colour) {
            at = layer_colour_shows_at(at == 0 ? now : at, command->rest_rgb,
                                       command->rest_blank_ms);
        }
        if (at == 0) {
            apply_rest(queue, command->rest_rgb, command->rest_blank_ms,
                       command->drop_indications, command->layer_colour, now);
        } else {
            add_timed_rest(queue, at, command->rest_rgb, command->rest_blank_ms,
                           command->drop_indications, command->layer_colour, now);
        }
        break;
    }
    case LED_COMMAND_RESTORE_LAYER_COLOUR:
        if (shown_layer_rgb != 0) {
            apply_rest(queue, shown_layer_rgb, shown_layer_blank_ms, false, false, now);
        }
        break;
    case LED_COMMAND_STOP_BREATHING:
        led_indication_queue_stop_breathing(queue, now);
        break;
    case LED_COMMAND_CLEAR:
        // Sleep, or a sleep that did not happen: what follows starts afresh.
        timed_rest_count = 0;
        next_layer_colour_not_before = 0;
        led_indication_queue_clear(queue, now);
        break;
    default:
        break;
    }
}

/*
 * USB power and the computer on it
 */

#if IS_ENABLED(CONFIG_SOC_FAMILY_NRF) && NRF_POWER_HAS_USBREG
#define USB_POWER_FROM_VBUS_DETECTOR 1
#else
#define USB_POWER_FROM_VBUS_DETECTOR 0
#endif

static bool usb_power_present(void) {
#if USB_POWER_FROM_VBUS_DETECTOR
    // The VBUS detector, read directly: valid from the first instant after
    // boot, and on a peripheral built without a USB stack.
    return nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
#elif IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    return zmk_usb_is_powered();
#else
    return false;
#endif
}

#if !SPLIT_PERIPHERAL
// Whether a computer has taken the USB connection for keys: the boot report
// waits a moment for it on USB power.
static bool usb_host_ready(void) {
#if IS_ENABLED(CONFIG_ZMK_USB)
    return zmk_usb_is_hid_ready();
#else
    return false;
#endif
}
#endif

#if POWER_BREATHE
static void queue_power_breathe(void) {
    LOG_INF("USB power, breathing #%06X %d times over %d ms",
            CONFIG_RGBLED_WIDGET_BATTERY_RGB_HIGH,
            CONFIG_RGBLED_WIDGET_POWER_CONNECTED_BREATHE_COUNT,
            CONFIG_RGBLED_WIDGET_POWER_CONNECTED_BREATHE_COUNT *
                CONFIG_RGBLED_WIDGET_POWER_CONNECTED_BREATHE_CYCLE_MS);
    const struct led_command command = {
        .kind = LED_COMMAND_ADD_INDICATION,
        .indication = {.kind = LED_INDICATION_BREATHE,
                       .rgb = CONFIG_RGBLED_WIDGET_BATTERY_RGB_HIGH,
                       .on_ms = CONFIG_RGBLED_WIDGET_POWER_CONNECTED_BREATHE_CYCLE_MS,
                       .breathe_cycles = CONFIG_RGBLED_WIDGET_POWER_CONNECTED_BREATHE_COUNT},
    };
    send_led_command(&command);
}

// All of it on the system work queue, as the USB events. Defined statically:
// USB events come before the LED thread starts.
static struct led_power_breathe_trigger power_breathe_trigger;
static void power_check_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(power_check_work, power_check_cb);

enum power_check_state { POWER_CHECK_OFF, POWER_CHECK_ARM, POWER_CHECK_ON };
static atomic_t power_check_state;
// The power the boot report saw: power that came after it is new.
static atomic_t power_seen_by_boot_report;

// Without a USB stack there is no event for the power: look once a second.
#define POWER_POLL_MS 1000

static void power_check_cb(struct k_work *work) {
    ARG_UNUSED(work);

    switch (atomic_get(&power_check_state)) {
    case POWER_CHECK_OFF:
        return;
    case POWER_CHECK_ARM:
        led_power_breathe_trigger_init(&power_breathe_trigger,
                                       atomic_get(&power_seen_by_boot_report));
        atomic_set(&power_check_state, POWER_CHECK_ON);
        break;
    default:
        break;
    }

    const struct led_power_breathe_action action =
        led_power_breathe_trigger_update(&power_breathe_trigger, usb_power_present());

    if (action.stop_breathing) {
        LOG_INF("USB power gone, breathing stopped");
        const struct led_command command = {.kind = LED_COMMAND_STOP_BREATHING};
        send_led_command(&command);
    }
    if (action.start_breathing) {
        // At once: a computer taking the USB connection follows it.
        queue_power_breathe();
    }

    if (!IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
        k_work_reschedule(&power_check_work, K_MSEC(POWER_POLL_MS));
    }
}

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
// Power arriving or going: look now.
static int led_usb_listener_cb(const zmk_event_t *eh) {
    k_work_reschedule(&power_check_work, K_NO_WAIT);
    return 0;
}

ZMK_LISTENER(led_usb_listener, led_usb_listener_cb);
ZMK_SUBSCRIPTION(led_usb_listener, zmk_usb_conn_state_changed);
#endif
#endif // POWER_BREATHE

/*
 * The link
 */

static bool link_is_up(void) {
#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#if IS_ENABLED(CONFIG_ZMK_BLE)
    if (zmk_endpoints_selected().transport == ZMK_TRANSPORT_USB) {
        return true;
    }
    return zmk_ble_active_profile_is_connected();
#else
    return true;
#endif
#elif IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
    return zmk_split_bt_peripheral_is_connected();
#else
    return true;
#endif
}

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_REMIND)
static void restart_conn_remind_period(void);
#endif

/*
 * The halves of a split connected to each other: a colour of its own, so that
 * blue and cyan always mean the computer. Over Bluetooth with LAYER_PUSH the
 * central shows it on itself and the peripheral that came, at once, when that
 * one is ready to be synced with; a peripheral shows none of its own.
 * Without LAYER_PUSH the central shows it alone when the link is secured.
 */
#if !SPLIT_PERIPHERAL && IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
static void queue_split_connected(uint32_t slot_mask) {
    LOG_INF("Halves connected, blinking #%06X", CONFIG_RGBLED_WIDGET_CONN_RGB_SPLIT_CONNECTED);
#if SYNC_WITH_PERIPHERALS
    const struct led_indication blink = blink_indication(
        CONFIG_RGBLED_WIDGET_CONN_RGB_SPLIT_CONNECTED, CONFIG_RGBLED_WIDGET_CONN_BLINK_MS, 0);

    queue_synced_group(&blink, 1,
                       rgbled_widget_pushed_param1(RGBLED_WIDGET_PUSHED_BLINK, 1,
                                                   CONFIG_RGBLED_WIDGET_CONN_RGB_SPLIT_CONNECTED),
                       rgbled_widget_pushed_param2(CONFIG_RGBLED_WIDGET_CONN_BLINK_MS, 0),
                       slot_mask);
#else
    ARG_UNUSED(slot_mask);
    queue_blink(CONFIG_RGBLED_WIDGET_CONN_RGB_SPLIT_CONNECTED, CONFIG_RGBLED_WIDGET_CONN_BLINK_MS,
                0);
#endif
}
#endif

#if !SPLIT_PERIPHERAL
/*
 * The state of the computer's link, as the central shows it: white with the
 * keys on USB (CONN_SHOW_USB), else the active profile's -- blue connected,
 * cyan open for pairing or not connected. Logs it when `log`.
 */
static uint32_t host_state_rgb(bool log) {
    // NOTE: ZMK v0.3.0 API. There is no ZMK_TRANSPORT_NONE in this release --
    // zmk_endpoints_selected() always reports USB or BLE, and "not connected"
    // is derived from the BLE profile state below.
    switch (zmk_endpoints_selected().transport) {
    case ZMK_TRANSPORT_USB:
#if IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_SHOW_USB)
        if (log) {
            LOG_INF("USB connected, blinking #%06X", CONFIG_RGBLED_WIDGET_CONN_RGB_USB);
        }
        return CONFIG_RGBLED_WIDGET_CONN_RGB_USB;
#endif
    default: // ZMK_TRANSPORT_BLE
#if IS_ENABLED(CONFIG_ZMK_BLE)
    {
        const uint8_t profile_index = zmk_ble_active_profile_index();

        if (zmk_ble_active_profile_is_connected()) {
            if (log) {
                LOG_CONN_CENTRAL(profile_index, "connected", CONNECTED);
            }
            return CONFIG_RGBLED_WIDGET_CONN_RGB_CONNECTED;
        }
        if (zmk_ble_active_profile_is_open()) {
            if (log) {
                LOG_CONN_CENTRAL(profile_index, "open", ADVERTISING);
            }
            return CONFIG_RGBLED_WIDGET_CONN_RGB_ADVERTISING;
        }
        if (log) {
            LOG_CONN_CENTRAL(profile_index, "not connected", DISCONNECTED);
        }
        return CONFIG_RGBLED_WIDGET_CONN_RGB_DISCONNECTED;
    }
#else
        return 0;
#endif
    }
}
#endif // !SPLIT_PERIPHERAL

// A central shows the computer's link, on every half (PUSH_INDICATIONS). A
// peripheral shows only its own link to the central when that is down; the
// central sends the rest.
static void indicate_connectivity_internal(void) {
#if !SPLIT_PERIPHERAL
    const uint32_t rgb = host_state_rgb(true);

    if (rgb == 0) {
        return;
    }
    // Behind whatever shows -- a breathe on USB power too.
    queue_blinks_on_all_halves(rgb, 1, CONFIG_RGBLED_WIDGET_CONN_BLINK_MS, 0);
#elif IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
    if (zmk_split_bt_peripheral_is_connected()) {
        return;
    }
    LOG_CONN_PERIPHERAL("not connected", DISCONNECTED);
    queue_blink(CONFIG_RGBLED_WIDGET_CONN_RGB_DISCONNECTED, CONFIG_RGBLED_WIDGET_CONN_BLINK_MS, 0);
#else
    return; // a wired peripheral: no link of its own to show
#endif
#if IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_REMIND)
    restart_conn_remind_period();
#endif
}

#if SPLIT_PERIPHERAL
// The central sends the layer colour each time this half connects; that one
// catches it up and interrupts nothing (led_pushed_colour_interrupts()).
static atomic_t colour_received_since_link_change;
#endif

// debouncing to ignore all but last connectivity event, to prevent repeat blinks
static void indicate_connectivity_cb(struct k_work *work) { indicate_connectivity_internal(); }
static K_WORK_DELAYABLE_DEFINE(indicate_connectivity_work, indicate_connectivity_cb);
void indicate_connectivity() { k_work_reschedule(&indicate_connectivity_work, K_MSEC(16)); }

static int led_output_listener_cb(const zmk_event_t *eh) {
#if SPLIT_PERIPHERAL
    atomic_set(&colour_received_since_link_change, 0);
#endif
#if IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_SHOW_USB) && IS_ENABLED(CONFIG_ZMK_BLE) &&                \
    !SPLIT_PERIPHERAL
    // With the keys on USB a BLE profile coming and going changes nothing
    // about where they go; a change of output raises zmk_endpoint_changed.
    if (as_zmk_ble_active_profile_changed(eh) != NULL &&
        zmk_endpoints_selected().transport == ZMK_TRANSPORT_USB) {
        return 0;
    }
#endif
#if SPLIT_PERIPHERAL && IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
    const struct zmk_split_peripheral_status_changed *status =
        as_zmk_split_peripheral_status_changed(eh);

    if (status != NULL && status->connected) {
        // The central shows the halves connected, here too.
        return 0;
    }
#endif
    if (atomic_get(&boot_connection_shown)) {
        indicate_connectivity();
    }
    return 0;
}

ZMK_LISTENER(led_output_listener, led_output_listener_cb);

#if !SPLIT_PERIPHERAL
// run led_output_listener_cb on endpoint and BLE profile change (on central)
#if IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_SHOW_USB)
ZMK_SUBSCRIPTION(led_output_listener, zmk_endpoint_changed);
#endif
#if IS_ENABLED(CONFIG_ZMK_BLE)
ZMK_SUBSCRIPTION(led_output_listener, zmk_ble_active_profile_changed);
#endif // IS_ENABLED(CONFIG_ZMK_BLE)
#elif IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
// run led_output_listener_cb on peripheral status change event
ZMK_SUBSCRIPTION(led_output_listener, zmk_split_peripheral_status_changed);
#endif

#if SYNC_WITH_PERIPHERALS
// A peripheral is ready to be synced with: the halves connected, on both --
// unless the boot report is still to show it.
void rgbled_widget_peripheral_ready(uint8_t slot) {
    if (atomic_get(&boot_connection_shown)) {
        queue_split_connected(BIT(slot));
    }
}
#elif !SPLIT_PERIPHERAL && IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
/*
 * No LAYER_PUSH: a peripheral's link came up -- secured, as ZMK needs it to
 * use the link: a reconnect attempt that fails to establish (0x3e) fires
 * `connected` too, and never gets there.
 */
static bool is_split_peripheral_link(struct bt_conn *conn) {
    struct bt_conn_info info;

    return !bt_conn_get_info(conn, &info) && info.type == BT_CONN_TYPE_LE &&
           info.role == BT_CONN_ROLE_CENTRAL && info.state == BT_CONN_STATE_CONNECTED &&
           bt_conn_get_security(conn) >= BT_SECURITY_L2;
}

static void count_split_peripheral_link(struct bt_conn *conn, void *data) {
    if (is_split_peripheral_link(conn)) {
        (*(int *)data)++;
    }
}

// A peripheral is connected.
static bool split_peripheral_link_up(void) {
    int count = 0;

    bt_conn_foreach(BT_CONN_TYPE_LE, count_split_peripheral_link, &count);
    return count > 0;
}

static void split_peripheral_up_cb(struct k_work *work) {
    ARG_UNUSED(work);

    if (atomic_get(&boot_connection_shown)) {
        queue_split_connected(0);
    }
}
static K_WORK_DEFINE(split_peripheral_up_work, split_peripheral_up_cb);

static void on_split_link_secured(struct bt_conn *conn, bt_security_t level,
                                  enum bt_security_err err) {
    struct bt_conn_info info;

    if (err == BT_SECURITY_ERR_SUCCESS && level >= BT_SECURITY_L2 &&
        !bt_conn_get_info(conn, &info) && info.type == BT_CONN_TYPE_LE &&
        info.role == BT_CONN_ROLE_CENTRAL) {
        k_work_submit(&split_peripheral_up_work);
    }
}

BT_CONN_CB_DEFINE(rgbled_widget_split_link_callbacks) = {
    .security_changed = on_split_link_secured,
};
#endif

#if SYNC_WITH_PERIPHERALS && PUSH_INDICATIONS && IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
// The battery colour on each half, at once: the central sends the length,
// each shows its own. The central's other colours (BATTERY_SHOW_PERIPHERALS)
// follow on it alone.
static void queue_battery_colours_on_all_halves(const uint32_t *rgbs, int count) {
    if (count <= 0) {
        return;
    }
    struct led_indication blink =
        blink_indication(rgbs[0], CONFIG_RGBLED_WIDGET_BATTERY_BLINK_MS, 0);

    // Each half its own colour: dark before and after on both, whatever
    // their layer colours, so that both time it alike and each stands out.
    blink.dark = LED_DARK_DECIDED | LED_DARK_BEFORE | LED_DARK_AFTER;
    queue_synced_group(&blink, 1,
                       rgbled_widget_pushed_param1(RGBLED_WIDGET_PUSHED_SHOW_BATTERY, 1, 0),
                       rgbled_widget_pushed_param2(CONFIG_RGBLED_WIDGET_BATTERY_BLINK_MS, 0),
                       UINT32_MAX);
    for (int i = 1; i < count; i++) {
        queue_blink(rgbs[i], CONFIG_RGBLED_WIDGET_BATTERY_BLINK_MS, 0);
    }
}
#endif

#if SYNC_WITH_CENTRAL && IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
static uint32_t get_battery_rgb(uint8_t battery_level);
#endif

// An indication the central pushed: count blinks of a colour, or this half's
// battery colour -- synced (sync_id) or at once.
void rgbled_widget_show_pushed_indication(uint32_t param1, uint32_t param2, uint8_t sync_id,
                                          int64_t received_at) {
    const enum rgbled_widget_pushed_kind kind = (param1 >> 28) & 0x7u;
    const uint8_t count = (param1 >> 24) & 0xFu;
    uint32_t rgb = param1 & 0xFFFFFFu;
    const uint16_t on_ms = param2 & 0xFFFFu;
    const uint16_t gap_ms = (param2 >> 16) & RGBLED_WIDGET_PUSHED_GAP_MAX_MS;
    const uint8_t dark = (uint8_t)(param2 >> RGBLED_WIDGET_PUSHED_DARK_SHIFT);

    if (!initialized) {
        return;
    }
    if (kind == RGBLED_WIDGET_PUSHED_SHOW_BATTERY) {
#if SYNC_WITH_CENTRAL && IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
        rgb = get_battery_rgb(zmk_battery_state_of_charge());
#else
        // No battery here to show: nothing, but the central is answered.
#if SYNC_WITH_CENTRAL
        if (sync_id != 0) {
            rgbled_widget_answer_central(sync_id, LED_SYNC_START_IN,
                                         rgbled_widget_sync_shortest_start_in_ms());
        }
#endif
        return;
#endif
    } else {
        LOG_DBG("Blinking #%06X x%u from the central", rgb, count);
    }

    struct led_command command = {
        .kind = LED_COMMAND_ADD_INDICATION,
        .indication = blink_indication(rgb, on_ms, gap_ms),
        .count = kind == RGBLED_WIDGET_PUSHED_SHOW_BATTERY ? 1 : count,
    };

    command.indication.dark = dark;
#if SYNC_WITH_CENTRAL
    if (sync_id != 0) {
        command.kind = LED_COMMAND_ADD_SYNCED_FROM_CENTRAL;
        command.sync_id = sync_id;
        command.received_at = received_at;
    }
#else
    ARG_UNUSED(sync_id);
    ARG_UNUSED(received_at);
#endif
    send_led_command(&command);
}

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_REMIND)
// Periodic reminder while the link is down. Upstream only indicates
// connectivity on state-change events and once at boot; this keeps a steady
// heartbeat so a keyboard that never connected is obvious at a glance.
static void conn_remind_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(conn_remind_work, conn_remind_cb);

// Each connection blink starts the period afresh, so the first reminder comes
// one period after it and they then keep an even rhythm.
static void restart_conn_remind_period(void) {
    k_work_reschedule(&conn_remind_work, K_SECONDS(CONFIG_RGBLED_WIDGET_CONN_REMIND_PERIOD_S));
}

// The colour of a link that is down: open for pairing, or waiting for the
// host (or, on a peripheral, the central) it knows.
static uint32_t link_down_rgb(void) {
#if !SPLIT_PERIPHERAL && IS_ENABLED(CONFIG_ZMK_BLE)
    if (zmk_ble_active_profile_is_open()) {
        return CONFIG_RGBLED_WIDGET_CONN_RGB_ADVERTISING;
    }
#endif
    return CONFIG_RGBLED_WIDGET_CONN_RGB_DISCONNECTED;
}

static void conn_remind_cb(struct k_work *work) {
    // Not before the boot report has shown the link: it may still come.
    if (atomic_get(&boot_connection_shown) && !link_is_up()) {
        // On a central the computer's link: a reminder on every half.
        const uint32_t rgb = link_down_rgb();
#if !SPLIT_PERIPHERAL
        rgbled_widget_blink_on_all_halves(rgb, 1, CONFIG_RGBLED_WIDGET_CONN_REMIND_BLINK_MS,
                                          CONFIG_RGBLED_WIDGET_INTERVAL_MS);
#else
        queue_blink(rgb, CONFIG_RGBLED_WIDGET_CONN_REMIND_BLINK_MS, 0);
#endif
    }
    k_work_reschedule(k_work_delayable_from_work(work),
                      K_SECONDS(CONFIG_RGBLED_WIDGET_CONN_REMIND_PERIOD_S));
}
#endif // IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_REMIND)

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
BUILD_ASSERT(CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_LOW < CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_HIGH,
             "RGBLED_WIDGET_BATTERY_LEVEL_LOW must be below RGBLED_WIDGET_BATTERY_LEVEL_HIGH");

static inline uint32_t get_battery_rgb(uint8_t battery_level) {
    if (battery_level == 0) {
        LOG_INF("Battery level undetermined (zero), blinking #%06X",
                CONFIG_RGBLED_WIDGET_BATTERY_RGB_MISSING);
        return CONFIG_RGBLED_WIDGET_BATTERY_RGB_MISSING;
    }

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_GRADIENT)
    uint32_t rgb = battery_gradient_rgb(
        battery_level, CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_HIGH,
        CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_LOW, CONFIG_RGBLED_WIDGET_BATTERY_RGB_HIGH,
        CONFIG_RGBLED_WIDGET_BATTERY_RGB_MEDIUM, CONFIG_RGBLED_WIDGET_BATTERY_RGB_LOW);
    LOG_INF("Battery level %d, blinking gradient #%06X", battery_level, rgb);
    return rgb;
#else
    if (battery_level >= CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_HIGH) {
        LOG_BATTERY(battery_level, HIGH);
        return CONFIG_RGBLED_WIDGET_BATTERY_RGB_HIGH;
    }
    if (battery_level > CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_LOW) {
        LOG_BATTERY(battery_level, MEDIUM);
        return CONFIG_RGBLED_WIDGET_BATTERY_RGB_MEDIUM;
    }
    LOG_BATTERY(battery_level, LOW);
    return CONFIG_RGBLED_WIDGET_BATTERY_RGB_LOW;
#endif
}

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_SHOW_PERIPHERALS) ||                                   \
    IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_SHOW_ONLY_PERIPHERALS)
#define BATTERY_COLOURS_MAX (1 + ZMK_SPLIT_BLE_PERIPHERAL_COUNT)
#else
#define BATTERY_COLOURS_MAX 1
#endif

// The battery colours to show, waiting a little for levels not read yet.
static int read_battery_colours(uint32_t rgbs[BATTERY_COLOURS_MAX]) {
    int count = 0;
    int retry = 0;

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_SHOW_SELF) ||                                          \
    IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_SHOW_PERIPHERALS)
    uint8_t battery_level = zmk_battery_state_of_charge();
    while (battery_level == 0 && retry++ < 10) {
        k_sleep(K_MSEC(100));
        battery_level = zmk_battery_state_of_charge();
    };

    rgbs[count++] = get_battery_rgb(battery_level);
#endif

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_SHOW_PERIPHERALS) ||                                   \
    IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_SHOW_ONLY_PERIPHERALS)
    for (uint8_t i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        uint8_t peripheral_level;
        int ret = zmk_split_central_get_peripheral_battery_level(i, &peripheral_level);
        if (ret == 0) {
            retry = 0;
            while (peripheral_level == 0 && retry++ < (CONFIG_RGBLED_WIDGET_BATTERY_BLINK_MS +
                                                       CONFIG_RGBLED_WIDGET_INTERVAL_MS) /
                                                          100) {
                k_sleep(K_MSEC(100));
                zmk_split_central_get_peripheral_battery_level(i, &peripheral_level);
            }

            LOG_INF("Got battery level for peripheral %d:", i);
            rgbs[count++] = get_battery_rgb(peripheral_level);
        } else {
            LOG_ERR("Error looking up battery level for peripheral %d", i);
        }
    }
#endif
    ARG_UNUSED(retry);
    return count;
}

static void queue_battery_colours(const uint32_t *rgbs, int count) {
    for (int i = 0; i < count; i++) {
        queue_blink(rgbs[i], CONFIG_RGBLED_WIDGET_BATTERY_BLINK_MS, 0);
    }
}

// The battery key: on a central, each half's own at once (PUSH_INDICATIONS
// over Bluetooth).
void indicate_battery(void) {
    uint32_t rgbs[BATTERY_COLOURS_MAX];
    const int count = read_battery_colours(rgbs);

#if SYNC_WITH_PERIPHERALS && PUSH_INDICATIONS
    queue_battery_colours_on_all_halves(rgbs, count);
#else
    queue_battery_colours(rgbs, count);
#endif
}

static int led_battery_listener_cb(const zmk_event_t *eh) {
    if (!initialized) {
        return 0;
    }

    // check if we are in critical battery levels at state change, blink if we are
    uint8_t battery_level = as_zmk_battery_state_changed(eh)->state_of_charge;

    if (battery_level > 0 && battery_level <= CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_CRITICAL) {
        LOG_BATTERY(battery_level, CRITICAL);
        queue_blink(CONFIG_RGBLED_WIDGET_BATTERY_RGB_CRITICAL,
                    CONFIG_RGBLED_WIDGET_BATTERY_BLINK_MS, 0);
    }
    return 0;
}

// run led_battery_listener_cb on battery state change event
ZMK_LISTENER(led_battery_listener, led_battery_listener_cb);
ZMK_SUBSCRIPTION(led_battery_listener, zmk_battery_state_changed);

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_PULSE)
// Timer-driven low-battery warning. zmk_battery_state_changed only fires when
// the reported percentage actually changes, which at a 10-minute sampling
// interval can be an hour apart near the bottom of the range -- far too rare
// to serve as a warning. So poll the cached value instead.
static void batt_pulse_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(batt_pulse_work, batt_pulse_cb);

static void batt_pulse_cb(struct k_work *work) {
    uint8_t level = zmk_battery_state_of_charge();

    // Not on USB power: it would pulse for the whole charge.
    if (initialized && level > 0 && level <= CONFIG_RGBLED_WIDGET_BATTERY_PULSE_LEVEL &&
        !usb_power_present()) {
        LOG_INF("Battery at %d%%, breathing #%06X", level,
                CONFIG_RGBLED_WIDGET_BATTERY_PULSE_RGB);

        // A breath, as on USB power -- red for want of it -- not blinks:
        // three short red blinks are a refusal (zmk-ble-host-link).
        const struct led_command command = {
            .kind = LED_COMMAND_ADD_INDICATION,
            .indication = {.kind = LED_INDICATION_BREATHE,
                           .rgb = CONFIG_RGBLED_WIDGET_BATTERY_PULSE_RGB,
                           .on_ms = CONFIG_RGBLED_WIDGET_BATTERY_PULSE_BREATHE_CYCLE_MS,
                           .breathe_cycles = CONFIG_RGBLED_WIDGET_BATTERY_PULSE_COUNT},
        };
        send_led_command(&command);
    }

    k_work_reschedule(k_work_delayable_from_work(work),
                      K_SECONDS(CONFIG_RGBLED_WIDGET_BATTERY_PULSE_PERIOD_S));
}
#endif // IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_PULSE)

#endif // IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)

/*
 * The layer colour: the resting colour between indications. A new one cuts
 * the indications short -- the keyboard is in use.
 */

uint32_t led_layer_rgb = 0;

// Dedup partner for led_layer_rgb: what the layer last asked for, whatever the
// blanking has done to the LED since.
static uint32_t led_layer_ms = CONFIG_RGBLED_WIDGET_BLANK_TIMEOUT_MS;

// Applied on a peripheral when the central pushes a new layer colour. Defined
// unconditionally: a peripheral has SHOW_LAYER_COLORS == 0 (it cannot resolve
// layer state itself) yet still needs to display what it is told.
void set_layer_rgb_external(uint32_t rgb, uint32_t blank_ms, uint8_t sync_id,
                            int64_t received_at, bool central_cuts_indications) {
    const bool differs = led_layer_rgb != rgb || led_layer_ms != blank_ms;
    // Another colour cuts the indications short; the same colour with
    // another blanking time only replaces it.
    bool interrupts = led_layer_rgb != rgb;
#if IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    bool received = atomic_get(&colour_received_since_link_change);
    interrupts = led_pushed_colour_interrupts(&received, interrupts);
    atomic_set(&colour_received_since_link_change, received);
#endif
    if (sync_id != 0) {
        // Synced: as the central decided, which saw every change -- also
        // those it replaced before they went out.
        interrupts = central_cuts_indications;
    }
    // A synced one shows -- and is answered -- even when it is the colour
    // here already: the central shows it at that moment too.
    if (!differs && sync_id == 0) {
        return;
    }
    led_layer_rgb = rgb;
    led_layer_ms = blank_ms;

    /* DBG: arrives with every layer change on the central. */
    LOG_DBG("Applying pushed layer colour #%06X, blank after %dms%s", rgb, blank_ms,
            interrupts ? ", indications cut short" : "");
    if (interrupts) {
        atomic_inc(&layer_colour_interrupts);
    }

    const struct led_command command = {
        .kind = LED_COMMAND_SET_REST,
        .sync_id = sync_id,
        .received_at = received_at,
        .rest_rgb = rgb,
        .rest_blank_ms = blank_ms,
        .drop_indications = interrupts,
        .layer_colour = true,
#if SYNC_WITH_CENTRAL
        // A colour the central catches this half up with is not one of the
        // changes both halves time alike.
        .waits_for_previous_colour = sync_id != 0,
#else
        .waits_for_previous_colour = true,
#endif
    };
    send_led_command(&command);
}

#if SYNC_WITH_PERIPHERALS
void rgbled_widget_apply_layer_colour_at(uint32_t rgb, uint32_t blank_ms, bool interrupts,
                                         int64_t at) {
    const struct led_command command = {
        .kind = LED_COMMAND_SET_REST,
        .at = MAX(at, 1),
        .rest_rgb = rgb,
        .rest_blank_ms = blank_ms,
        .drop_indications = interrupts,
        .layer_colour = true,
        .waits_for_previous_colour = true,
    };
    send_led_command(&command);
}
#endif

#if SHOW_LAYER_COLORS

// The boot report and the layer events both set the colour.
static K_MUTEX_DEFINE(layer_colour_lock);

// The colour the layer wants, noted without showing it: the boot report
// shows it once it is done, and a layer event meanwhile compares against it.
static void note_initial_layer_colour(void) {
    k_mutex_lock(&layer_colour_lock, K_FOREVER);
    const uint8_t index = zmk_keymap_highest_layer_active();
    led_layer_rgb = layer_rgb[index];
    led_layer_ms = layer_ms[index];
    k_mutex_unlock(&layer_colour_lock);
}

static void show_initial_layer_colour(void) {
    k_mutex_lock(&layer_colour_lock, K_FOREVER);
    LOG_INF("Setting initial layer color #%06X", led_layer_rgb);
    const struct led_command command = {.kind = LED_COMMAND_SET_REST,
                                        .rest_rgb = led_layer_rgb,
                                        .rest_blank_ms = led_layer_ms,
                                        .layer_colour = true};
    send_led_command(&command);
#if LAYER_PUSH
    rgbled_widget_push_layer_color_to_peripherals(led_layer_rgb, led_layer_ms);
#endif
    k_mutex_unlock(&layer_colour_lock);
}

static void update_layer_color(void) {
    k_mutex_lock(&layer_colour_lock, K_FOREVER);
    const uint8_t index = zmk_keymap_highest_layer_active();

    if (led_layer_rgb != layer_rgb[index] || led_layer_ms != layer_ms[index]) {
        // Another colour cuts the indications short; the same colour with
        // another blanking time only replaces it.
        const bool interrupts = led_layer_rgb != layer_rgb[index];
        led_layer_rgb = layer_rgb[index];
        led_layer_ms = layer_ms[index];
        /* DBG: fires on every layer change, and the auto-mouse layer changes
           all the time. */
        LOG_DBG("Setting layer color to #%06X for layer %d, blank after %dms", led_layer_rgb,
                index, layer_ms[index]);
#if SYNC_WITH_PERIPHERALS
        // Here when the peripherals can show it too.
        if (interrupts) {
            atomic_inc(&layer_colour_interrupts);
        }
        rgbled_widget_change_layer_colour_on_all_halves(led_layer_rgb, led_layer_ms, interrupts);
#else
        set_layer_rest(led_layer_rgb, led_layer_ms, interrupts);
#if LAYER_PUSH
        rgbled_widget_push_layer_color_to_peripherals(led_layer_rgb, layer_ms[index]);
#endif
#endif
    }
    k_mutex_unlock(&layer_colour_lock);
}

static int led_layer_color_listener_cb(const zmk_event_t *eh) {
    if (initialized) {
        update_layer_color();
    }
    return 0;
}

// run layer_color_listener_cb on layer status change event
ZMK_LISTENER(led_layer_color_listener, led_layer_color_listener_cb);
ZMK_SUBSCRIPTION(led_layer_color_listener, zmk_layer_state_changed);
#endif // SHOW_LAYER_COLORS

/*
 * Power handling, compiled for BOTH split roles.
 *
 * A WS2812 latches the last colour it was sent and keeps driving its die with
 * no MCU involvement whatsoever. Entering System OFF therefore does NOT turn it
 * off -- the LED stays lit until the battery is flat. It must be explicitly
 * sent black before sleeping.
 *
 * This cannot live under SHOW_LAYER_COLORS: that gate is false on a split
 * peripheral, yet the peripheral does hold a persistent colour pushed from the
 * central via set_layer_rgb_external().
 */
static int led_activity_listener_cb(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);

    if (ev == NULL) {
        return 0;
    }

    if (ev->state != ZMK_ACTIVITY_SLEEP && keep_leds_dark_for_sleep(false)) {
        // A sleep that did not happen: the LED may light again -- but not
        // with what was queued while it was dark (a reminder, say); the
        // layer colour, and then the state below, decide what it shows now.
        clear_indications();
        if (initialized) {
            set_rest(led_layer_rgb, led_layer_ms, false);
        }
    }

    switch (ev->state) {
    case ZMK_ACTIVITY_SLEEP:
        LOG_INF("Entering sleep, turning LED off");
        keep_leds_dark_for_sleep(true);
        clear_indications();
        write_strip(0);
        break;

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_OFF_ON_IDLE)
    case ZMK_ACTIVITY_IDLE:
        LOG_INF("Going idle, turning LED off");
        set_rest(0, 0, false);
        break;

    case ZMK_ACTIVITY_ACTIVE:
        // The layer colour as this half last showed it: one still to show
        // at its synced or held moment comes then, here as on the other half.
        if (initialized) {
            const struct led_command command = {.kind = LED_COMMAND_RESTORE_LAYER_COLOUR};
            send_led_command(&command);
        }
        break;
#endif

    default:
        break;
    }

    return 0;
}

ZMK_LISTENER(led_activity_listener, led_activity_listener_cb);
ZMK_SUBSCRIPTION(led_activity_listener, zmk_activity_state_changed);

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
void indicate_layer(void) {
    uint8_t index = zmk_keymap_highest_layer_active();
    /* DBG for the same reason: one line per layer change. */
    LOG_DBG("Blinking %d times #%06X for layer change", index,
            CONFIG_RGBLED_WIDGET_LAYER_RGB);

    for (int i = 0; i < index; i++) {
        const bool last = i == index - 1;
        queue_blink(CONFIG_RGBLED_WIDGET_LAYER_RGB, CONFIG_RGBLED_WIDGET_LAYER_BLINK_MS,
                    last ? 0 : CONFIG_RGBLED_WIDGET_LAYER_BLINK_MS);
    }
}
#endif // !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

#if SHOW_LAYER_CHANGE
static void indicate_layer_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(layer_indicate_work, indicate_layer_cb);

static int led_layer_listener_cb(const zmk_event_t *eh) {
    // ignore if not initialized yet or layer off events
    if (initialized && as_zmk_layer_state_changed(eh)->state) {
        k_work_reschedule(&layer_indicate_work, K_MSEC(CONFIG_RGBLED_WIDGET_LAYER_DEBOUNCE_MS));
    }
    return 0;
}

static void indicate_layer_cb(struct k_work *work) { indicate_layer(); }

ZMK_LISTENER(led_layer_listener, led_layer_listener_cb);
ZMK_SUBSCRIPTION(led_layer_listener, zmk_layer_state_changed);
#endif // SHOW_LAYER_CHANGE

// Touched by led_process_thread only.
static struct led_indication_queue led_queue;

#if SYNC_WITH_PERIPHERALS || SYNC_WITH_CENTRAL
#define LED_PROCESS_STACK_SIZE 1536
#else
#define LED_PROCESS_STACK_SIZE 1024
#endif
// Synced groups handed to the sender at a time: as many as it writes at
// once to a peripheral.
#define WRITES_OFFERED_AT_ONCE 4

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_LOG_LED_THREAD_STALLS)
// What the LED thread last said about itself, for the stall watch below:
// when it last went round its loop (k_uptime_get_32()), when it means to
// go round next (0: when a command comes), how many indications wait, and
// whether the first holds for a synced start.
static atomic_t led_thread_ran_at_ms;
static atomic_t led_thread_due_at_ms;
static atomic_t led_thread_indications_waiting;
static atomic_t led_thread_holds_for_synced_start;

static void led_thread_says_it_ran(void) {
    atomic_set(&led_thread_ran_at_ms, (atomic_val_t)k_uptime_get_32());
}

static void led_thread_says_it_waits(int64_t next_change_at) {
    atomic_set(&led_thread_due_at_ms, next_change_at == LED_INDICATION_NO_DEADLINE
                                          ? 0
                                          : (atomic_val_t)(uint32_t)MAX(next_change_at, 1));
    atomic_set(&led_thread_indications_waiting, (atomic_val_t)led_queue.waiting_count);
    atomic_set(&led_thread_holds_for_synced_start,
               led_indication_queue_synced_waiting(&led_queue));
}
#else
static inline void led_thread_says_it_ran(void) {}
static inline void led_thread_says_it_waits(int64_t next_change_at) {
    ARG_UNUSED(next_change_at);
}
#endif

extern void led_process_thread(void *d0, void *d1, void *d2) {
    ARG_UNUSED(d0);
    ARG_UNUSED(d1);
    ARG_UNUSED(d2);

    led_indication_queue_init(&led_queue, CONFIG_RGBLED_WIDGET_INTERVAL_MS);

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_REMIND)
    k_work_reschedule(&conn_remind_work, K_SECONDS(CONFIG_RGBLED_WIDGET_CONN_REMIND_PERIOD_S));
#endif

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_PULSE)
    k_work_reschedule(&batt_pulse_work, K_SECONDS(CONFIG_RGBLED_WIDGET_BATTERY_PULSE_PERIOD_S));
#endif

    // Written after every command too: a write the sleep guard turned black
    // is not what the strip should keep once the guard lifts.
    bool strip_is_stale = true;
    uint32_t written_rgb = 0;

#if SYNC_WITH_PERIPHERALS
    uint32_t holds_given_up = 0;
#endif

    while (true) {
        led_thread_says_it_ran();
        const int64_t next_rest_at = apply_due_rests(&led_queue, k_uptime_get());
        int64_t next_change_at;
        const uint32_t rgb =
            led_indication_queue_colour(&led_queue, k_uptime_get(), &next_change_at);
        if (strip_is_stale || rgb != written_rgb) {
            write_strip(rgb);
            written_rgb = rgb;
            strip_is_stale = false;
        }
        next_change_at = MIN(next_change_at, next_rest_at);

#if SYNC_WITH_PERIPHERALS
        // Synced groups whose turn has come go to the peripherals.
        static struct led_indication offers[WRITES_OFFERED_AT_ONCE]; // off the stack
        const uint32_t offered =
            led_indication_queue_take_offers(&led_queue, k_uptime_get(), offers,
                                             ARRAY_SIZE(offers), rgbled_widget_sync_new_id);

        for (uint32_t i = 0; i < offered; i++) {
            rgbled_widget_offer_synced_group(
                offers[i].sync_id, offers[i].peripheral_param1,
                offers[i].peripheral_param2 |
                    ((uint32_t)offers[i].dark << RGBLED_WIDGET_PUSHED_DARK_SHIFT),
                offers[i].peripheral_slot_mask, offers[i].added_at);
        }
        if (offered > 0) {
            // Held from now: shown anyway if no start ever comes.
            next_change_at =
                MIN(next_change_at, k_uptime_get() + LED_SYNC_UNANSWERED_HOLD_LIMIT_MS);
        }
        if (led_queue.unanswered_holds_given_up != holds_given_up) {
            holds_given_up = led_queue.unanswered_holds_given_up;
            LOG_WRN("LED sync: a group got no start in %d ms -- shown here alone",
                    LED_SYNC_UNANSWERED_HOLD_LIMIT_MS);
        }
#endif

        k_timeout_t wait = K_FOREVER;
        if (next_change_at != LED_INDICATION_NO_DEADLINE) {
            wait = K_MSEC(MAX(next_change_at - k_uptime_get(), 0));
        }
        led_thread_says_it_waits(next_change_at);

        struct led_command command;
        if (k_msgq_get(&led_command_msgq, &command, wait) == 0) {
            do {
                apply_led_command(&led_queue, &command, k_uptime_get());
            } while (k_msgq_get(&led_command_msgq, &command, K_NO_WAIT) == 0);
            strip_is_stale = true;
        }
    }
}

// define led_process_thread, start running it 100 ms after boot. Synced, it
// also hands groups to the sender or answers the central, and logs more.
K_THREAD_DEFINE(led_process_tid, LED_PROCESS_STACK_SIZE, led_process_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 100);

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_LOG_LED_THREAD_STALLS)
/*
 * The stall watch: once a second, from the system work queue, whether the
 * LED thread still goes round its loop when it should -- with commands
 * waiting for it, or past the time it meant to go round. One WRN when it has
 * not for LED_THREAD_STALL_MS, with what it was doing; one INF when it moves
 * again. Only reads what the thread published.
 */
#define LED_THREAD_STALL_MS 3000
#define LED_THREAD_STALL_CHECK_MS 1000

static void led_thread_stall_check_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(led_thread_stall_check, led_thread_stall_check_cb);

// What a pending LED thread waits on: the commands -- its own wait, so not
// stalled after all -- the strip, or something else, whose address
// zephyr.map names.
static void describe_led_thread_wait(char *buf, size_t len) {
    const _wait_q_t *on = led_process_tid->base.pended_on;

    if (on == NULL) {
        snprintf(buf, len, "not pending on anything");
    } else if (on == &led_command_msgq.wait_q) {
        snprintf(buf, len, "waiting for a command");
    } else if (on == &led_strip_lock.wait_q) {
        snprintf(buf, len, "waiting for the strip lock");
    } else {
        snprintf(buf, len, "pending on the object at %p (see zephyr.map)", (void *)on);
    }
}

static void led_thread_stall_check_cb(struct k_work *work) {
    ARG_UNUSED(work);
    static bool stalled;
    static uint32_t stalled_since_ms;

    const uint32_t now = k_uptime_get_32();
    const uint32_t ran_at = (uint32_t)atomic_get(&led_thread_ran_at_ms);
    const uint32_t due_at = (uint32_t)atomic_get(&led_thread_due_at_ms);
    const uint32_t commands_waiting = k_msgq_num_used_get(&led_command_msgq);
    const uint32_t since_ran = now - ran_at;
    const bool overdue = due_at != 0 && (int32_t)(now - due_at) > LED_THREAD_STALL_MS;

    if (!stalled && since_ran > LED_THREAD_STALL_MS && (commands_waiting > 0 || overdue)) {
        char state[32];
        char waits_on[64];

        stalled = true;
        stalled_since_ms = ran_at;
        describe_led_thread_wait(waits_on, sizeof(waits_on));
        LOG_WRN("LED thread stalled: not round its loop for %u ms; %u commands waiting, %s; "
                "thread %s, %s; %u indications queued%s",
                since_ran, commands_waiting, overdue ? "overdue" : "not due",
                k_thread_state_str(led_process_tid, state, sizeof(state)), waits_on,
                (uint32_t)atomic_get(&led_thread_indications_waiting),
                atomic_get(&led_thread_holds_for_synced_start)
                    ? ", the first holding for a synced start"
                    : "");
    } else if (stalled && ran_at != stalled_since_ms) {
        stalled = false;
        LOG_INF("LED thread moving again after %u ms", ran_at - stalled_since_ms);
    }
    k_work_reschedule(&led_thread_stall_check, K_MSEC(LED_THREAD_STALL_CHECK_MS));
}

static int led_thread_stall_watch_init(void) {
    // The thread starts 100 ms after boot; it has gone round once by then.
    k_work_reschedule(&led_thread_stall_check, K_MSEC(LED_THREAD_STALL_MS));
    return 0;
}

SYS_INIT(led_thread_stall_watch_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif

/*
 * The boot report -- after a power-on and after deep sleep alike, which ends
 * in a reset. First the power, at once: a green breathe on USB power, else
 * the battery colour. Then the links, in a queue behind it, as soon as they
 * have settled -- what is known by the end of the power part follows it
 * straight away. A central waits LED_BOOT_WAIT_FOR_LINK_MS at most (and
 * LED_BOOT_WAIT_FOR_PERIPHERAL_READY_MS more for a peripheral getting ready),
 * then shows the halves connected (CONN_RGB_SPLIT_CONNECTED) and the
 * computer's link on every half. A peripheral waits for its central up to
 * LED_BOOT_PERIPHERAL_WAITS_FOR_CENTRAL_MS: there, the central shows the
 * rest; not there, cyan. What comes later shows by its own blink.
 */

// The links may show now, elapsed_ms into the report.
static bool boot_links_settled(int64_t elapsed_ms) {
#if SPLIT_PERIPHERAL
    // Connected: the central shows the rest, here too. Else, in the end,
    // that it is not. A wired peripheral has no link of its own to show.
    return link_is_up() || elapsed_ms >= LED_BOOT_PERIPHERAL_WAITS_FOR_CENTRAL_MS;
#else
    const bool host_settled = led_boot_connection_settled(
        elapsed_ms, link_is_up(), IS_ENABLED(CONFIG_ZMK_USB), usb_power_present(),
        usb_host_ready());
#if SYNC_WITH_PERIPHERALS
    // A peripheral, ready to be synced with -- and waited for a while longer
    // if it is connected but not ready yet.
    const bool peripheral_getting_ready = rgbled_widget_peripheral_getting_ready();

    if (elapsed_ms >= LED_BOOT_WAIT_FOR_LINK_MS) {
        return !peripheral_getting_ready ||
               elapsed_ms >= LED_BOOT_WAIT_FOR_LINK_MS + LED_BOOT_WAIT_FOR_PERIPHERAL_READY_MS;
    }
    return host_settled && rgbled_widget_ready_peripheral_slots() != 0 &&
           !peripheral_getting_ready;
#elif IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
    return elapsed_ms >= LED_BOOT_WAIT_FOR_LINK_MS || (host_settled && split_peripheral_link_up());
#else
    return host_settled;
#endif
#endif
}

// The links, as they settled: the halves connected, then the computer's link.
static void show_links_at_boot(void) {
#if SPLIT_PERIPHERAL
    // Down: this half shows it. Up: the central shows the rest, here too.
    indicate_connectivity_internal();
#else
#if SYNC_WITH_PERIPHERALS
    const uint32_t ready = rgbled_widget_ready_peripheral_slots();

    if (ready != 0) {
        queue_split_connected(ready);
    }
#elif IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
    if (split_peripheral_link_up()) {
        queue_split_connected(0);
    }
#endif
    // On the peripherals too.
    indicate_connectivity_internal();
#endif
}

extern void led_init_thread(void *d0, void *d1, void *d2) {
    ARG_UNUSED(d0);
    ARG_UNUSED(d1);
    ARG_UNUSED(d2);

    const int64_t began_at = k_uptime_get();
    const atomic_val_t interrupts_before = atomic_get(&layer_colour_interrupts);
#if SHOW_LAYER_COLORS
    note_initial_layer_colour();
#endif
    initialized = true;

    // The power, at once.
    const bool usb_power = usb_power_present();

    LOG_INF("Boot report: %s first", usb_power ? "USB power" : "the battery");
#if POWER_BREATHE
    if (usb_power) {
        queue_power_breathe();
    } else
#endif
    {
#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
        // Reading may take a second: a layer colour meanwhile drops it.
        uint32_t rgbs[BATTERY_COLOURS_MAX];
        const int count = read_battery_colours(rgbs);
        if (atomic_get(&layer_colour_interrupts) == interrupts_before) {
            queue_battery_colours(rgbs, count);
        }
#endif
    }

    // The links, behind it.
#if SPLIT_PERIPHERAL
    LOG_INF("Boot report: waiting for the central, at most %d ms",
            LED_BOOT_PERIPHERAL_WAITS_FOR_CENTRAL_MS);
#else
    LOG_INF("Boot report: waiting for the links, at most %d ms", LED_BOOT_WAIT_FOR_LINK_MS);
#endif
    while (!boot_links_settled(k_uptime_get() - began_at) &&
           atomic_get(&layer_colour_interrupts) == interrupts_before) {
        k_sleep(K_MSEC(50));
    }

    if (atomic_get(&layer_colour_interrupts) == interrupts_before) {
        LOG_INF("Boot report: the links after %d ms", (int)(k_uptime_get() - began_at));
        // Connection events blink from here on: one that comes while the
        // report reads the state is not lost, at most shown twice.
        atomic_set(&boot_connection_shown, 1);
        show_links_at_boot();
    } else {
        LOG_INF("Boot report dropped: the layer colour changed first");
        atomic_set(&boot_connection_shown, 1);
    }

#if SHOW_LAYER_COLORS
    // After a layer event in the report the colour is shown and pushed already.
    if (atomic_get(&layer_colour_interrupts) == interrupts_before) {
        show_initial_layer_colour();
    }
#endif

#if POWER_BREATHE
    // From here on power that comes is new.
    atomic_set(&power_seen_by_boot_report, usb_power);
    atomic_set(&power_check_state, POWER_CHECK_ARM);
    k_work_reschedule(&power_check_work, K_NO_WAIT);
#endif

    LOG_INF("Finished initializing LED widget");
}

// run init thread on boot for the boot report
K_THREAD_DEFINE(led_init_tid, 1024, led_init_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 200);
