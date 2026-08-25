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

#if IS_ENABLED(CONFIG_ZMK_BLE)
void indicate_connectivity(void);
#endif

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
void indicate_layer(void);
#endif

// Applied on a peripheral when the central pushes a new layer colour.
void set_layer_rgb_external(uint32_t rgb);
