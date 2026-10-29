#pragma once

#define SHOW_LAYER_CHANGE                                                                          \
    (IS_ENABLED(CONFIG_RGBLED_WIDGET_SHOW_LAYER_CHANGE)) &&                                        \
        (!IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL))

#define SHOW_LAYER_COLORS                                                                          \
    (IS_ENABLED(CONFIG_RGBLED_WIDGET_SHOW_LAYER_COLORS)) &&                                        \
        (!IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL))

// Pushing the layer colour to peripherals only makes sense on a split central.
#define LAYER_PUSH                                                                                 \
    (IS_ENABLED(CONFIG_RGBLED_WIDGET_LAYER_PUSH)) &&                                               \
        (IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL))

// ...and the indications that concern the whole keyboard along with it.
#define PUSH_INDICATIONS                                                                           \
    (LAYER_PUSH) && (IS_ENABLED(CONFIG_RGBLED_WIDGET_PUSH_INDICATIONS_TO_PERIPHERALS))

// A split peripheral: its LED shows what the central pushes, too.
#define SPLIT_PERIPHERAL (IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL))

/*
 * What goes to a peripheral over the lyr_sync behavior: its two parameters,
 * and over Bluetooth the number of the synced group as the event's position
 * (0: not synced, shown on arrival -- see src/led_sync.h). A layer colour:
 * param1 0xRRGGBB, and for a synced one bit 30 set when it cuts the
 * indications short -- as the central decided for every half -- param2 its
 * blanking time in ms. An indication: param1
 * with bit 31 set, the kind in bits 28-30, a count in bits 24-27 and
 * 0xRRGGBB below; param2 its length (ms) in bits 0-15, the gap after each
 * blink (ms) in bits 16-28, and in bits 29-31 whether a synced blink is dark
 * before and after it, as the central decided for every half (enum led_dark
 * in src/led_indication_queue.h).
 */
#define RGBLED_WIDGET_PUSHED_INDICATION_FLAG 0x80000000u
#define RGBLED_WIDGET_PUSHED_COLOUR_CUTS_INDICATIONS 0x40000000u

enum rgbled_widget_pushed_kind {
    // count blinks of the colour: rgbled_widget_blink_on_all_halves().
    RGBLED_WIDGET_PUSHED_BLINK = 0,
    // Each half its own battery colour, one blink of the length given (the
    // colour bits unused): the battery key.
    RGBLED_WIDGET_PUSHED_SHOW_BATTERY = 1,
};

static inline uint32_t rgbled_widget_pushed_param1(enum rgbled_widget_pushed_kind kind,
                                                   uint8_t count, uint32_t rgb) {
    return RGBLED_WIDGET_PUSHED_INDICATION_FLAG | (((uint32_t)kind & 0x7u) << 28) |
           (((uint32_t)count & 0xFu) << 24) | (rgb & 0xFFFFFFu);
}

#define RGBLED_WIDGET_PUSHED_GAP_MAX_MS 0x1FFFu
#define RGBLED_WIDGET_PUSHED_DARK_SHIFT 29

static inline uint32_t rgbled_widget_pushed_param2(uint16_t on_ms, uint16_t gap_ms) {
    const uint32_t gap = gap_ms < RGBLED_WIDGET_PUSHED_GAP_MAX_MS ? gap_ms
                                                                  : RGBLED_WIDGET_PUSHED_GAP_MAX_MS;
    return (gap << 16) | on_ms;
}

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
void indicate_battery(void);
#endif

void indicate_connectivity(void);

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
void indicate_layer(void);
#endif

// For other modules: count blinks of rgb (0xRRGGBB), each on_ms long and
// followed by off_ms of the resting colour, queued behind whatever the LED is
// showing. Callable from any thread. A full queue drops blinks rather than
// waiting. Returns the number of blinks the command queue took; the LED
// thread may still drop some later (its own queue full, or a change of state
// clearing what is queued).
// CONFIG_RGBLED_WIDGET_BLINK_API says this exists.
int rgbled_widget_blink(uint32_t rgb, uint8_t count, uint16_t on_ms, uint16_t off_ms);

// The same, for what concerns the whole keyboard rather than this half: on a
// split central with RGBLED_WIDGET_PUSH_INDICATIONS_TO_PERIPHERALS it shows
// on every connected peripheral too -- over Bluetooth at the same moment on
// every half, when its turn comes on this one (at most 15 blinks). Elsewhere
// it is rgbled_widget_blink().
// CONFIG_RGBLED_WIDGET_BLINK_ON_ALL_HALVES_API says this exists.
int rgbled_widget_blink_on_all_halves(uint32_t rgb, uint8_t count, uint16_t on_ms,
                                      uint16_t off_ms);

// How many indications (blinks) the LED queues at most, for callers that
// queue several at once: past it, rgbled_widget_blink() drops the rest.
#define RGBLED_WIDGET_INDICATION_QUEUE_LENGTH 16

// Applied on a peripheral when the central pushes a layer colour: sync_id
// its synced group (0: at once), received_at the uptime the write came, and
// for a synced one whether it cuts the indications short (else this half
// decides).
void set_layer_rgb_external(uint32_t rgb, uint32_t blank_ms, uint8_t sync_id,
                            int64_t received_at, bool central_cuts_indications);

// Applied on a peripheral when the central pushes an indication (the two
// lyr_sync parameters, RGBLED_WIDGET_PUSHED_INDICATION_FLAG set), likewise.
void rgbled_widget_show_pushed_indication(uint32_t param1, uint32_t param2, uint8_t sync_id,
                                          int64_t received_at);

#if LAYER_PUSH
// Central: the colour every peripheral is caught up with -- sent at once to
// those ready (src/layer_push_to_peripherals.c), not synced; a layer change
// goes synced over Bluetooth. Returns at once; the write follows.
void rgbled_widget_push_layer_color_to_peripherals(uint32_t rgb, uint32_t blank_ms);
#endif

#if PUSH_INDICATIONS && !IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
// Central of a wired split: an indication (rgbled_widget_pushed_param1/2)
// for every peripheral, shown there as it arrives. Returns at once.
void rgbled_widget_push_indication_to_peripherals(uint32_t param1, uint32_t param2);
#endif
