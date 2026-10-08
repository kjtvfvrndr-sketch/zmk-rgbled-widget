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

// How many indications (blinks) the LED queues at most, for callers that
// queue several at once: past it, rgbled_widget_blink() drops the rest.
#define RGBLED_WIDGET_INDICATION_QUEUE_LENGTH 16

// Applied on a peripheral when the central pushes a new layer colour.
void set_layer_rgb_external(uint32_t rgb, uint32_t blank_ms);

#if LAYER_PUSH
// Central: show this colour on every peripheral's LED
// (src/layer_push_to_peripherals.c). Returns at once; the write follows.
void rgbled_widget_push_layer_color_to_peripherals(uint32_t rgb, uint32_t blank_ms);
#endif
