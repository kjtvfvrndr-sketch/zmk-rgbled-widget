target_sources_ifdef(CONFIG_RGBLED_WIDGET app PRIVATE src/widget.c src/led_indication_queue.c)
if(CONFIG_RGBLED_WIDGET_LAYER_PUSH AND CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  target_sources(app PRIVATE src/layer_push_to_peripherals.c)
endif()
target_sources_ifdef(CONFIG_ZMK_BEHAVIOR_RGBLED_WIDGET app PRIVATE src/behaviors/behavior_rgbled_widget.c)
target_sources_ifdef(CONFIG_ZMK_BEHAVIOR_RGBLED_SYNC app PRIVATE src/behaviors/behavior_rgbled_sync.c)

zephyr_include_directories(include)
