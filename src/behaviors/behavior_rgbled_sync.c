/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_rgbled_sync

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/behavior.h>

#include <zmk_rgbled_widget/widget.h>

LOG_MODULE_DECLARE(rgbled_widget, CONFIG_RGBLED_WIDGET_LOG_LEVEL);

static int behavior_rgb_sync_init(const struct device *dev) { return 0; }

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
#if IS_ENABLED(CONFIG_RGBLED_WIDGET)
    // Taken as the write came: a synced start is timed from here.
    const int64_t received_at = k_uptime_get();
    // The central sends the number of the synced group as the position.
    const uint8_t sync_id = (uint8_t)event.position;

    if (binding->param1 & RGBLED_WIDGET_PUSHED_INDICATION_FLAG) {
        // An indication for the whole keyboard (widget.h has the layout).
        rgbled_widget_show_pushed_indication(binding->param1, binding->param2, sync_id,
                                             received_at);
        return ZMK_BEHAVIOR_OPAQUE;
    }
    // param1 carries the full 0xRRGGBB value pushed by the central, so this
    // side needs no copy of the layer colour table -- and above it whether
    // a synced one cuts the indications short.
    const uint32_t rgb = binding->param1 & 0xFFFFFFu;

    LOG_DBG("Received layer colour #%06X (blank after %dms) from central%s", rgb,
            binding->param2, sync_id != 0 ? ", synced" : "");
    set_layer_rgb_external(rgb, binding->param2, sync_id, received_at,
                           (binding->param1 & RGBLED_WIDGET_PUSHED_COLOUR_CUTS_INDICATIONS) != 0);
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