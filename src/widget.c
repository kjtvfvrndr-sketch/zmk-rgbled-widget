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

#include <zephyr/logging/log.h>

#include <zmk_rgbled_widget/widget.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

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

// a blink work item as specified by the color (0xRRGGBB) and duration
struct blink_item {
    uint32_t rgb;
    uint32_t duration_ms;
    uint32_t sleep_ms;
};

// flag to indicate whether the initial boot up sequence is complete
static bool initialized = false;

// track current color for persistent indicators (layer color)
uint32_t led_current_rgb = 0;

// global brightness scaling, 0..100
static inline uint8_t scale(uint16_t v) {
    return (uint8_t)((v * CONFIG_RGBLED_WIDGET_BRIGHTNESS) / 100);
}

#if CONFIG_RGBLED_WIDGET_BLANK_TIMEOUT_MS > 0
static struct k_work_delayable blank_work;
#endif

// low-level method to control the LED
static void set_rgb_leds(uint32_t rgb, uint32_t duration_ms) {
    // GREEN_TRIM compensates for the green die being perceptually brighter
    struct led_rgb px = {
        .r = scale((rgb >> 16) & 0xFF),
        .g = scale((((rgb >> 8) & 0xFF) * CONFIG_RGBLED_WIDGET_GREEN_TRIM) / 100),
        .b = scale(rgb & 0xFF),
    };

    int err = led_strip_update_rgb(led_dev, &px, 1);
    if (err < 0) {
        LOG_ERR("Failed to update LED strip: %d", err);
    }

    if (duration_ms > 0) {
        k_sleep(K_MSEC(duration_ms));
    }
    led_current_rgb = rgb;

#if CONFIG_RGBLED_WIDGET_BLANK_TIMEOUT_MS > 0
    // Arm (or cancel) the blanking timer from the single place that ever
    // touches the strip, so every path is covered: layer colours, colours
    // pushed from the central, and the tail of a blink sequence.
    if (rgb != 0) {
        k_work_reschedule(&blank_work, K_MSEC(CONFIG_RGBLED_WIDGET_BLANK_TIMEOUT_MS));
    } else {
        k_work_cancel_delayable(&blank_work);
    }
#endif
}


// define message queue of blink work items, that will be processed by a
// separate thread
K_MSGQ_DEFINE(led_msgq, sizeof(struct blink_item), 16, 1);

#if CONFIG_RGBLED_WIDGET_BLANK_TIMEOUT_MS > 0
//
// A split peripheral only ever sees ZMK_ACTIVITY_IDLE on the ACTIVE -> IDLE
// transition (set_state() in app/src/activity.c returns early when the state
// is unchanged). Type only on the central and the peripheral goes idle once,
// gets blanked, and then every pushed layer colour lights it again with no
// further idle event to ever turn it back off -- so it stays lit for hours.
//
// This timer is independent of the activity state and therefore covers that
// case, on both roles.
//
static void blank_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    // Go through the queue rather than calling set_rgb_leds() here, so the
    // strip is only ever driven from led_process_thread.
    struct blink_item blank = {.rgb = 0, .duration_ms = 0};
    k_msgq_put(&led_msgq, &blank, K_NO_WAIT);
}
#endif

static void indicate_connectivity_internal(void) {
    struct blink_item blink = {.duration_ms = CONFIG_RGBLED_WIDGET_CONN_BLINK_MS};

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    // NOTE: ZMK v0.3.0 API. There is no ZMK_TRANSPORT_NONE in this release --
    // zmk_endpoints_selected() always reports USB or BLE, and "not connected"
    // is derived from the BLE profile state below.
    switch (zmk_endpoints_selected().transport) {
    case ZMK_TRANSPORT_USB:
#if IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_SHOW_USB)
        LOG_INF("USB connected, blinking #%06X", CONFIG_RGBLED_WIDGET_CONN_RGB_USB);
        blink.rgb = CONFIG_RGBLED_WIDGET_CONN_RGB_USB;
        break;
#endif
    default: // ZMK_TRANSPORT_BLE
#if IS_ENABLED(CONFIG_ZMK_BLE)
    {
        uint8_t profile_index = zmk_ble_active_profile_index();
        if (zmk_ble_active_profile_is_connected()) {
            LOG_CONN_CENTRAL(profile_index, "connected", CONNECTED);
            blink.rgb = CONFIG_RGBLED_WIDGET_CONN_RGB_CONNECTED;
        } else if (zmk_ble_active_profile_is_open()) {
            LOG_CONN_CENTRAL(profile_index, "open", ADVERTISING);
            blink.rgb = CONFIG_RGBLED_WIDGET_CONN_RGB_ADVERTISING;
        } else {
            LOG_CONN_CENTRAL(profile_index, "not connected", DISCONNECTED);
            blink.rgb = CONFIG_RGBLED_WIDGET_CONN_RGB_DISCONNECTED;
        }
    }
#endif
        break;
    }
#elif IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
    if (zmk_split_bt_peripheral_is_connected()) {
        LOG_CONN_PERIPHERAL("connected", CONNECTED);
        blink.rgb = CONFIG_RGBLED_WIDGET_CONN_RGB_CONNECTED;
    } else {
        LOG_CONN_PERIPHERAL("not connected", DISCONNECTED);
        blink.rgb = CONFIG_RGBLED_WIDGET_CONN_RGB_DISCONNECTED;
    }
#endif

    k_msgq_put(&led_msgq, &blink, K_NO_WAIT);
}

static int led_output_listener_cb(const zmk_event_t *eh) {
    if (initialized) {
        indicate_connectivity();
    }
    return 0;
}

// debouncing to ignore all but last connectivity event, to prevent repeat blinks
static struct k_work_delayable indicate_connectivity_work;
static void indicate_connectivity_cb(struct k_work *work) { indicate_connectivity_internal(); }
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
static struct k_work_delayable conn_remind_work;

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

static void conn_remind_cb(struct k_work *work) {
    if (initialized && !link_is_up()) {
        struct blink_item blink = {.rgb = CONFIG_RGBLED_WIDGET_CONN_RGB_DISCONNECTED,
                                   .duration_ms = CONFIG_RGBLED_WIDGET_CONN_BLINK_MS};
        k_msgq_put(&led_msgq, &blink, K_NO_WAIT);
    }
    k_work_reschedule(k_work_delayable_from_work(work),
                      K_SECONDS(CONFIG_RGBLED_WIDGET_CONN_REMIND_PERIOD_S));
}
#endif // IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_REMIND)

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
static inline uint32_t get_battery_rgb(uint8_t battery_level) {
    if (battery_level == 0) {
        LOG_INF("Battery level undetermined (zero), blinking #%06X",
                CONFIG_RGBLED_WIDGET_BATTERY_RGB_MISSING);
        return CONFIG_RGBLED_WIDGET_BATTERY_RGB_MISSING;
    }

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_GRADIENT)
    // Sweep hue from 0 deg (red, empty) to 120 deg (green, full) at full
    // saturation and value. Integer math only; GREEN_TRIM is applied later,
    // once, inside set_rgb_leds().
    uint16_t h = (uint16_t)battery_level * 120 / 100;
    uint8_t r = (h < 60) ? 255 : (uint8_t)(((120 - h) * 255) / 60);
    uint8_t g = (h < 60) ? (uint8_t)((h * 255) / 60) : 255;
    uint32_t rgb = ((uint32_t)r << 16) | ((uint32_t)g << 8);
    LOG_INF("Battery level %d, blinking gradient #%06X", battery_level, rgb);
    return rgb;
#else
    if (battery_level >= CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_HIGH) {
        LOG_BATTERY(battery_level, HIGH);
        return CONFIG_RGBLED_WIDGET_BATTERY_RGB_HIGH;
    }
    if (battery_level >= CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_LOW) {
        LOG_BATTERY(battery_level, MEDIUM);
        return CONFIG_RGBLED_WIDGET_BATTERY_RGB_MEDIUM;
    }
    LOG_BATTERY(battery_level, LOW);
    return CONFIG_RGBLED_WIDGET_BATTERY_RGB_LOW;
#endif
}

void indicate_battery(void) {
    struct blink_item blink = {.duration_ms = CONFIG_RGBLED_WIDGET_BATTERY_BLINK_MS};
    int retry = 0;

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_SHOW_SELF) ||                                          \
    IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_SHOW_PERIPHERALS)
    uint8_t battery_level = zmk_battery_state_of_charge();
    while (battery_level == 0 && retry++ < 10) {
        k_sleep(K_MSEC(100));
        battery_level = zmk_battery_state_of_charge();
    };

    blink.rgb = get_battery_rgb(battery_level);
    k_msgq_put(&led_msgq, &blink, K_NO_WAIT);
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
            blink.rgb = get_battery_rgb(peripheral_level);
            k_msgq_put(&led_msgq, &blink, K_NO_WAIT);
        } else {
            LOG_ERR("Error looking up battery level for peripheral %d", i);
        }
    }
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

        struct blink_item blink = {.duration_ms = CONFIG_RGBLED_WIDGET_BATTERY_BLINK_MS,
                                   .rgb = CONFIG_RGBLED_WIDGET_BATTERY_RGB_CRITICAL};
        k_msgq_put(&led_msgq, &blink, K_NO_WAIT);
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
static struct k_work_delayable batt_pulse_work;

static inline bool vbus_present(void) {
#if IS_ENABLED(CONFIG_SOC_FAMILY_NRF)
    // Read the USB regulator status directly: CONFIG_ZMK_USB (and with it
    // zmk_usb_is_powered()) is unavailable on a split peripheral, which would
    // otherwise pulse red for the entire charge cycle.
    return nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
#else
    return false;
#endif
}

static void batt_pulse_cb(struct k_work *work) {
    uint8_t level = zmk_battery_state_of_charge();

    if (initialized && level > 0 && level <= CONFIG_RGBLED_WIDGET_BATTERY_PULSE_LEVEL &&
        !vbus_present()) {
        LOG_INF("Battery at %d%%, pulsing #%06X", level,
                CONFIG_RGBLED_WIDGET_BATTERY_PULSE_RGB);

        // Each queued item is rendered by led_process_thread as "colour for
        // duration_ms, then the layer colour for sleep_ms", so N items produce
        // N blinks with gaps and a clean return to the layer colour.
        struct blink_item blink = {.rgb = CONFIG_RGBLED_WIDGET_BATTERY_PULSE_RGB,
                                   .duration_ms = CONFIG_RGBLED_WIDGET_BATTERY_PULSE_MS,
                                   .sleep_ms = CONFIG_RGBLED_WIDGET_BATTERY_PULSE_GAP_MS};
        for (int i = 0; i < CONFIG_RGBLED_WIDGET_BATTERY_PULSE_COUNT; i++) {
            k_msgq_put(&led_msgq, &blink, K_NO_WAIT);
        }
    }

    k_work_reschedule(k_work_delayable_from_work(work),
                      K_SECONDS(CONFIG_RGBLED_WIDGET_BATTERY_PULSE_PERIOD_S));
}
#endif // IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_PULSE)

#endif // IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)

uint32_t led_layer_rgb = 0;

// Applied on a peripheral when the central pushes a new layer colour. Defined
// unconditionally: a peripheral has SHOW_LAYER_COLORS == 0 (it cannot resolve
// layer state itself) yet still needs to display what it is told.
void set_layer_rgb_external(uint32_t rgb) {
    if (led_layer_rgb == rgb) {
        return;
    }
    led_layer_rgb = rgb;

    struct blink_item color = {.rgb = rgb};
    LOG_INF("Applying pushed layer colour #%06X", rgb);
    k_msgq_put(&led_msgq, &color, K_NO_WAIT);
}

#if SHOW_LAYER_COLORS

#if LAYER_PUSH
// Mirror the layer color onto every peripheral. param1 of a split behavior
// invocation is a uint32_t, so the full 0xRRGGBB value fits and the peripheral
// needs no copy of the layer table.
//
// MUST NOT run inline in the layer-change listener. ZMK dispatches events
// synchronously on the raising thread, so that listener executes in the key
// processing path -- and split_bt_invoke_behavior_payload() calls
// k_msgq_put(..., K_MSEC(100)) on a queue only
// CONFIG_ZMK_SPLIT_BLE_CENTRAL_SPLIT_RUN_QUEUE_SIZE deep (default 5). A full
// queue would therefore stall key handling for up to 100 ms per layer change.
//
// Deferring to ZMK's low priority work queue also coalesces bursts for free:
// only the most recent colour is ever sent, since rapid layer changes just
// overwrite pending_push_rgb before the single work item runs.
//
static uint32_t pending_push_rgb;
static uint32_t last_pushed_rgb;
static int64_t last_push_uptime;
static struct k_work_delayable push_layer_work;

static void push_layer_rgb_work_cb(struct k_work *work) {
    ARG_UNUSED(work);

    if (pending_push_rgb == last_pushed_rgb) {
        return;
    }
    last_pushed_rgb = pending_push_rgb;
    last_push_uptime = k_uptime_get();

    struct zmk_behavior_binding binding = {
        .behavior_dev = "lyr_sync",
        .param1 = pending_push_rgb,
        .param2 = 0,
    };
    struct zmk_behavior_binding_event event = {
        .layer = 0,
        .position = 0,
        .timestamp = k_uptime_get(),
    };

    for (uint8_t i = 0; i < ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT; i++) {
        int err = zmk_split_central_invoke_behavior(i, &binding, event, true);
        if (err) {
            LOG_DBG("Could not push layer color to peripheral %d: %d", i, err);
        }
    }
}

//
// Leading-edge rate limit. An isolated layer change is pushed immediately; a
// burst collapses into at most one write per MIN_INTERVAL_MS, and the final
// colour always lands because pending_push_rgb is overwritten in place.
//
// This matters because bt_gatt_write_without_response() draws from the same
// ACL TX pool (BT_L2CAP_TX_BUF_COUNT defaults to BT_BUF_ACL_TX_COUNT) that the
// central uses for HID reports to the host. The split link drains slowly --
// with the default ZMK_SPLIT_BLE_PREF_LATENCY of 30 the peripheral only
// listens about 4 times a second -- so unthrottled pushes park buffers on that
// connection and starve the trackball's reports to the host.
//
// An auto-activated mouse layer toggles on every trackball move/stop cycle, so
// without this the push rate easily exceeds what the link can drain.
//
static void push_layer_rgb(uint32_t rgb) {
    pending_push_rgb = rgb;

    int64_t since = k_uptime_get() - last_push_uptime;
    k_timeout_t delay = (since >= CONFIG_RGBLED_WIDGET_LAYER_PUSH_MIN_INTERVAL_MS)
                            ? K_NO_WAIT
                            : K_MSEC(CONFIG_RGBLED_WIDGET_LAYER_PUSH_MIN_INTERVAL_MS - since);

    k_work_reschedule_for_queue(zmk_workqueue_lowprio_work_q(), &push_layer_work, delay);
}
#endif // LAYER_PUSH

void update_layer_color(void) {
    uint8_t index = zmk_keymap_highest_layer_active();

    if (led_layer_rgb != layer_rgb[index]) {
        led_layer_rgb = layer_rgb[index];
        struct blink_item color = {.rgb = led_layer_rgb};
        LOG_INF("Setting layer color to #%06X for layer %d", led_layer_rgb, index);
        k_msgq_put(&led_msgq, &color, K_NO_WAIT);
#if LAYER_PUSH
        push_layer_rgb(led_layer_rgb);
#endif
    }
}

static int led_layer_color_listener_cb(const zmk_event_t *eh) {
    // Activity transitions are handled by led_activity_listener below, which is
    // compiled for both split roles. Ignore them here.
    if (as_zmk_activity_state_changed(eh) != NULL) {
        return 0;
    }

    // it must be a layer change event instead
    if (initialized) {
        update_layer_color();
    }
    return 0;
}

// run layer_color_listener_cb on layer status change event and activity state event
ZMK_LISTENER(led_layer_color_listener, led_layer_color_listener_cb);
ZMK_SUBSCRIPTION(led_layer_color_listener, zmk_layer_state_changed);
ZMK_SUBSCRIPTION(led_layer_color_listener, zmk_activity_state_changed);
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

    switch (ev->state) {
    case ZMK_ACTIVITY_SLEEP:
        LOG_INF("Entering sleep, turning LED off");
        set_rgb_leds(0, 0);
        break;

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_OFF_ON_IDLE)
    case ZMK_ACTIVITY_IDLE:
        LOG_INF("Going idle, turning LED off");
        set_rgb_leds(0, 0);
        break;

    case ZMK_ACTIVITY_ACTIVE:
        // Restore whatever colour was current before we went dark. On a
        // peripheral this is the colour last pushed by the central.
        if (initialized) {
            set_rgb_leds(led_layer_rgb, 0);
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
    static const struct blink_item blink = {.duration_ms = CONFIG_RGBLED_WIDGET_LAYER_BLINK_MS,
                                            .rgb = CONFIG_RGBLED_WIDGET_LAYER_RGB,
                                            .sleep_ms = CONFIG_RGBLED_WIDGET_LAYER_BLINK_MS};
    static const struct blink_item last_blink = {.duration_ms = CONFIG_RGBLED_WIDGET_LAYER_BLINK_MS,
                                                 .rgb = CONFIG_RGBLED_WIDGET_LAYER_RGB};
    LOG_INF("Blinking %d times #%06X for layer change", index,
            CONFIG_RGBLED_WIDGET_LAYER_RGB);

    for (int i = 0; i < index; i++) {
        if (i < index - 1) {
            k_msgq_put(&led_msgq, &blink, K_NO_WAIT);
        } else {
            k_msgq_put(&led_msgq, &last_blink, K_NO_WAIT);
        }
    }
}
#endif // !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

#if SHOW_LAYER_CHANGE
static struct k_work_delayable layer_indicate_work;

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

extern void led_process_thread(void *d0, void *d1, void *d2) {
    ARG_UNUSED(d0);
    ARG_UNUSED(d1);
    ARG_UNUSED(d2);

    k_work_init_delayable(&indicate_connectivity_work, indicate_connectivity_cb);

#if CONFIG_RGBLED_WIDGET_BLANK_TIMEOUT_MS > 0
    k_work_init_delayable(&blank_work, blank_work_cb);
#endif

#if LAYER_PUSH
    k_work_init_delayable(&push_layer_work, push_layer_rgb_work_cb);
#endif

#if SHOW_LAYER_CHANGE
    k_work_init_delayable(&layer_indicate_work, indicate_layer_cb);
#endif

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_CONN_REMIND)
    k_work_init_delayable(&conn_remind_work, conn_remind_cb);
    k_work_reschedule(&conn_remind_work, K_SECONDS(CONFIG_RGBLED_WIDGET_CONN_REMIND_PERIOD_S));
#endif

#if IS_ENABLED(CONFIG_RGBLED_WIDGET_BATTERY_PULSE)
    k_work_init_delayable(&batt_pulse_work, batt_pulse_cb);
    k_work_reschedule(&batt_pulse_work, K_SECONDS(CONFIG_RGBLED_WIDGET_BATTERY_PULSE_PERIOD_S));
#endif

    while (true) {
        // wait until a blink item is received and process it
        struct blink_item blink;
        k_msgq_get(&led_msgq, &blink, K_FOREVER);
        if (blink.duration_ms > 0) {
            LOG_DBG("Got a blink item from msgq, color #%06X, duration %d", blink.rgb,
                    blink.duration_ms);

            // Blink the leds, using a separation blink if necessary
            if (blink.rgb == led_current_rgb && blink.rgb > 0) {
                set_rgb_leds(0, CONFIG_RGBLED_WIDGET_INTERVAL_MS);
            }
            set_rgb_leds(blink.rgb, blink.duration_ms);
            if (blink.rgb == led_layer_rgb && blink.rgb > 0) {
                set_rgb_leds(0, CONFIG_RGBLED_WIDGET_INTERVAL_MS);
            }
            // wait interval before processing another blink
            set_rgb_leds(led_layer_rgb,
                         blink.sleep_ms > 0 ? blink.sleep_ms : CONFIG_RGBLED_WIDGET_INTERVAL_MS);

        } else {
            LOG_DBG("Got a layer color item from msgq, color #%06X", blink.rgb);
            set_rgb_leds(blink.rgb, 0);
        }
    }
}

// define led_process_thread with stack size 1024, start running it 100 ms after
// boot
K_THREAD_DEFINE(led_process_tid, 1024, led_process_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 100);

extern void led_init_thread(void *d0, void *d1, void *d2) {
    ARG_UNUSED(d0);
    ARG_UNUSED(d1);
    ARG_UNUSED(d2);

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
    // check and indicate battery level on thread start
    LOG_INF("Indicating initial battery status");

    indicate_battery();

    // wait until blink should be displayed for further checks
    k_sleep(K_MSEC(CONFIG_RGBLED_WIDGET_BATTERY_BLINK_MS + CONFIG_RGBLED_WIDGET_INTERVAL_MS));
#endif // IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)

    // check and indicate current profile or peripheral connectivity status
    LOG_INF("Indicating initial connectivity status");
    indicate_connectivity();

#if SHOW_LAYER_COLORS
    LOG_INF("Setting initial layer color");
    update_layer_color();
#endif // SHOW_LAYER_COLORS

    initialized = true;
    LOG_INF("Finished initializing LED widget");
}

// run init thread on boot for initial battery+output checks
K_THREAD_DEFINE(led_init_tid, 1024, led_init_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 200);
