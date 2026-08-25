/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_rgbled_sync

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/behavior.h>

#include <zmk_rgbled_widget/widget.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static int behavior_rgb_sync_init(const struct device *dev) { return 0; }

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
#if IS_ENABLED(CONFIG_RGBLED_WIDGET)
    // param1 carries the full 0xRRGGBB value pushed by the central, so this
    // side needs no copy of the layer colour table.
    LOG_DBG("Received layer colour #%06X from central", binding->param1);
    set_layer_rgb_external(binding->param1);
#endif
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_rgb_sync_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
    // Deliberately NOT BEHAVIOR_LOCALITY_GLOBAL: the central invokes this
    // explicitly per peripheral, and a global locality would re-broadcast it.
    .locality = BEHAVIOR_LOCALITY_EVENT_SOURCE,
};

#define RGBSYNC_INST(n)                                                                            \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_rgb_sync_init, NULL, NULL, NULL, POST_KERNEL,               \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_rgb_sync_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RGBSYNC_INST)
