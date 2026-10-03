#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

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

#include <zephyr/logging/log.h>

#include <zmk_rgbled_widget/widget.h>

#include "battery_gradient.h"
#include "led_indication_queue.h"
#include "led_indication_triggers.h"

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
// and the breathe on USB power each note it when they begin waiting and give
// up if it moved: the keyboard is in use.
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
    LED_COMMAND_SET_REST,
    LED_COMMAND_STOP_BREATHING,
    LED_COMMAND_CLEAR,
};

struct led_command {
    enum led_command_kind kind;
    struct led_indication indication;
    uint32_t rest_rgb;
    uint32_t rest_blank_ms;
    bool drop_indications;
};

K_MSGQ_DEFINE(led_command_msgq, sizeof(struct led_command), 16, 4);

static void send_led_command(const struct led_command *command) {
    if (k_msgq_put(&led_command_msgq, command, K_NO_WAIT) == 0) {
        return;
    }
    if (command->kind == LED_COMMAND_ADD_INDICATION) {
        LOG_DBG("LED commands full, indication #%06X dropped", command->indication.rgb);
        return;
    }
    // A full queue of commands the LED thread has not got to: the state
    // that follows outranks them.
    LOG_WRN("LED commands full, dropping them for a change of state");
    k_msgq_purge(&led_command_msgq);
    k_msgq_put(&led_command_msgq, command, K_NO_WAIT);
}

static void queue_blink(uint32_t rgb, uint32_t on_ms, uint32_t gap_ms, bool preempts_breathe) {
    const struct led_command command = {
        .kind = LED_COMMAND_ADD_INDICATION,
        .indication = {.kind = LED_INDICATION_BLINK,
                       .rgb = rgb,
                       .on_ms = on_ms,
                       .gap_ms = gap_ms,
                       .preempts_breathe = preempts_breathe},
    };
    send_led_command(&command);
}

// The colour between indications. With drop_indications it cuts them short
// and shows at once.
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

static void clear_indications(void) {
    const struct led_command command = {.kind = LED_COMMAND_CLEAR};
    k_msgq_purge(&led_command_msgq);
    send_led_command(&command);
}

static void apply_led_command(struct led_indication_queue *queue,
                              const struct led_command *command, int64_t now) {
    switch (command->kind) {
    case LED_COMMAND_ADD_INDICATION:
        if (!led_indication_queue_add(queue, &command->indication, now)) {
            LOG_DBG("LED queue full, indication #%06X dropped", command->indication.rgb);
        }
        break;
    case LED_COMMAND_SET_REST:
        led_indication_queue_set_rest(queue, command->rest_rgb, command->rest_blank_ms,
                                      command->drop_indications, now);
        break;
    case LED_COMMAND_STOP_BREATHING:
        led_indication_queue_stop_breathing(queue, now);
        break;
    case LED_COMMAND_CLEAR:
        led_indication_queue_clear(queue, now);
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

// Whether a computer has taken the USB connection for keys.
static bool usb_host_ready(void) {
#if IS_ENABLED(CONFIG_ZMK_USB)
    return zmk_usb_is_hid_ready();
#else
    return false;
#endif
}

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
    const int64_t now = k_uptime_get();

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

    const struct led_power_breathe_action action = led_power_breathe_trigger_update(
        &power_breathe_trigger, now, usb_power_present(), IS_ENABLED(CONFIG_ZMK_USB),
        usb_host_ready(), (uint32_t)atomic_get(&layer_colour_interrupts));

    if (action.stop_breathing) {
        LOG_INF("USB power gone, breathing stopped");
        const struct led_command command = {.kind = LED_COMMAND_STOP_BREATHING};
        send_led_command(&command);
    }
    if (action.start_breathing) {
        queue_power_breathe();
    }

    if (action.check_again_at >= 0) {
        k_work_reschedule(&power_check_work, K_MSEC(MAX(action.check_again_at - now, 0)));
    } else if (!IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
        k_work_reschedule(&power_check_work, K_MSEC(POWER_POLL_MS));
    }
}

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
// Power arriving or going, and a computer taking the connection: look now.
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

static void indicate_connectivity_internal(void) {
    uint32_t rgb = 0;

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    // NOTE: ZMK v0.3.0 API. There is no ZMK_TRANSPORT_NONE in this release --
    // zmk_endpoints_selected() always reports USB or BLE, and "not connected"
    // is derived from the BLE profile state below.
    switch (zmk_endpoints_selected().transport) {
    case ZMK_TRANSPORT_USB:
#if IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_SHOW_USB)
        LOG_INF("USB connected, blinking #%06X", CONFIG_RGBLED_WIDGET_CONN_RGB_USB);
        rgb = CONFIG_RGBLED_WIDGET_CONN_RGB_USB;
        break;
#endif
    default: // ZMK_TRANSPORT_BLE
#if IS_ENABLED(CONFIG_ZMK_BLE)
    {
        uint8_t profile_index = zmk_ble_active_profile_index();
        if (zmk_ble_active_profile_is_connected()) {
            LOG_CONN_CENTRAL(profile_index, "connected", CONNECTED);
            rgb = CONFIG_RGBLED_WIDGET_CONN_RGB_CONNECTED;
        } else if (zmk_ble_active_profile_is_open()) {
            LOG_CONN_CENTRAL(profile_index, "open", ADVERTISING);
            rgb = CONFIG_RGBLED_WIDGET_CONN_RGB_ADVERTISING;
        } else {
            LOG_CONN_CENTRAL(profile_index, "not connected", DISCONNECTED);
            rgb = CONFIG_RGBLED_WIDGET_CONN_RGB_DISCONNECTED;
        }
    }
#endif
        break;
    }
#elif IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
    if (zmk_split_bt_peripheral_is_connected()) {
        LOG_CONN_PERIPHERAL("connected", CONNECTED);
        rgb = CONFIG_RGBLED_WIDGET_CONN_RGB_CONNECTED;
    } else {
        LOG_CONN_PERIPHERAL("not connected", DISCONNECTED);
        rgb = CONFIG_RGBLED_WIDGET_CONN_RGB_DISCONNECTED;
    }
#endif

    if (rgb == 0) {
        return; // a wired peripheral: no link of its own to show
    }
    // A breathe on USB power gives way to it and starts over after it.
    queue_blink(rgb, CONFIG_RGBLED_WIDGET_CONN_BLINK_MS, 0, true);
#if IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_REMIND)
    restart_conn_remind_period();
#endif
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
// The central sends the layer colour each time this half connects; that one
// catches it up and interrupts nothing (led_pushed_colour_interrupts()).
static atomic_t colour_received_since_link_change;
#endif

static int led_output_listener_cb(const zmk_event_t *eh) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    atomic_set(&colour_received_since_link_change, 0);
#endif
#if IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_SHOW_USB) && IS_ENABLED(CONFIG_ZMK_BLE) &&                \
    (!IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL))
    // With the keys on USB a BLE profile coming and going changes nothing
    // about where they go; a change of output raises zmk_endpoint_changed.
    if (as_zmk_ble_active_profile_changed(eh) != NULL &&
        zmk_endpoints_selected().transport == ZMK_TRANSPORT_USB) {
        return 0;
    }
#endif
    if (atomic_get(&boot_connection_shown)) {
        indicate_connectivity();
    }
    return 0;
}

// debouncing to ignore all but last connectivity event, to prevent repeat blinks
static void indicate_connectivity_cb(struct k_work *work) { indicate_connectivity_internal(); }
static K_WORK_DELAYABLE_DEFINE(indicate_connectivity_work, indicate_connectivity_cb);
void indicate_connectivity() { k_work_reschedule(&indicate_connectivity_work, K_MSEC(16)); }

ZMK_LISTENER(led_output_listener, led_output_listener_cb);

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
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
#if (!IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)) &&                  \
    IS_ENABLED(CONFIG_ZMK_BLE)
    if (zmk_ble_active_profile_is_open()) {
        return CONFIG_RGBLED_WIDGET_CONN_RGB_ADVERTISING;
    }
#endif
    return CONFIG_RGBLED_WIDGET_CONN_RGB_DISCONNECTED;
}

static void conn_remind_cb(struct k_work *work) {
    if (initialized && !link_is_up()) {
        queue_blink(link_down_rgb(), CONFIG_RGBLED_WIDGET_CONN_REMIND_BLINK_MS, 0, false);
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
        queue_blink(rgbs[i], CONFIG_RGBLED_WIDGET_BATTERY_BLINK_MS, 0, false);
    }
}

void indicate_battery(void) {
    uint32_t rgbs[BATTERY_COLOURS_MAX];
    queue_battery_colours(rgbs, read_battery_colours(rgbs));
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
                    CONFIG_RGBLED_WIDGET_BATTERY_BLINK_MS, 0, false);
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
        LOG_INF("Battery at %d%%, pulsing #%06X", level,
                CONFIG_RGBLED_WIDGET_BATTERY_PULSE_RGB);

        // N blinks, each followed by the resting colour for the gap.
        for (int i = 0; i < CONFIG_RGBLED_WIDGET_BATTERY_PULSE_COUNT; i++) {
            queue_blink(CONFIG_RGBLED_WIDGET_BATTERY_PULSE_RGB,
                        CONFIG_RGBLED_WIDGET_BATTERY_PULSE_MS,
                        CONFIG_RGBLED_WIDGET_BATTERY_PULSE_GAP_MS, false);
        }
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
void set_layer_rgb_external(uint32_t rgb, uint32_t blank_ms) {
    const bool differs = led_layer_rgb != rgb || led_layer_ms != blank_ms;
    // Another colour cuts the indications short; the same colour with
    // another blanking time only replaces it.
    bool interrupts = led_layer_rgb != rgb;
#if IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    bool received = atomic_get(&colour_received_since_link_change);
    interrupts = led_pushed_colour_interrupts(&received, interrupts);
    atomic_set(&colour_received_since_link_change, received);
#endif
    if (!differs) {
        return;
    }
    led_layer_rgb = rgb;
    led_layer_ms = blank_ms;

    /* DBG: arrives with every layer change on the central. */
    LOG_DBG("Applying pushed layer colour #%06X, blank after %dms%s", rgb, blank_ms,
            interrupts ? ", indications cut short" : "");
    set_rest(rgb, blank_ms, interrupts);
}

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
    set_rest(led_layer_rgb, led_layer_ms, false);
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
        set_rest(led_layer_rgb, led_layer_ms, interrupts);
#if LAYER_PUSH
        rgbled_widget_push_layer_color_to_peripherals(led_layer_rgb, layer_ms[index]);
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
        // Restore what the layer wants. On a peripheral led_layer_rgb holds
        // the colour last pushed by the central.
        if (initialized && led_layer_rgb != 0) {
            set_rest(led_layer_rgb, led_layer_ms, false);
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
                    last ? 0 : CONFIG_RGBLED_WIDGET_LAYER_BLINK_MS, false);
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

    while (true) {
        int64_t next_change_at;
        const uint32_t rgb =
            led_indication_queue_colour(&led_queue, k_uptime_get(), &next_change_at);
        if (strip_is_stale || rgb != written_rgb) {
            write_strip(rgb);
            written_rgb = rgb;
            strip_is_stale = false;
        }

        k_timeout_t wait = K_FOREVER;
        if (next_change_at != LED_INDICATION_NO_DEADLINE) {
            wait = K_MSEC(MAX(next_change_at - k_uptime_get(), 0));
        }

        struct led_command command;
        if (k_msgq_get(&led_command_msgq, &command, wait) == 0) {
            do {
                apply_led_command(&led_queue, &command, k_uptime_get());
            } while (k_msgq_get(&led_command_msgq, &command, K_NO_WAIT) == 0);
            strip_is_stale = true;
        }
    }
}

// define led_process_thread with stack size 1024, start running it 100 ms after
// boot
K_THREAD_DEFINE(led_process_tid, 1024, led_process_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 100);

// The boot report -- after a power-on and after deep sleep alike, which ends
// in a reset: first the connection, once it has settled, then the battery,
// or with USB power the breathe in its place.
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

    LOG_INF("Boot report: waiting for the link, at most %d ms", LED_BOOT_WAIT_FOR_LINK_MS);
    while (!led_boot_connection_settled(k_uptime_get() - began_at, link_is_up(),
                                        IS_ENABLED(CONFIG_ZMK_USB), usb_power_present(),
                                        usb_host_ready()) &&
           atomic_get(&layer_colour_interrupts) == interrupts_before) {
        k_sleep(K_MSEC(50));
    }

    const bool usb_power = usb_power_present();
    if (atomic_get(&layer_colour_interrupts) == interrupts_before) {
        LOG_INF("Boot report after %d ms%s", (int)(k_uptime_get() - began_at),
                usb_power ? ", on USB power" : "");
        // Connection events blink from here on: one that comes while the
        // blink below reads the state is not lost, at most shown twice.
        atomic_set(&boot_connection_shown, 1);
        indicate_connectivity_internal();
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
