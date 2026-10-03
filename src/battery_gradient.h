/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * The battery colour with RGBLED_WIDGET_BATTERY_GRADIENT: the HIGH colour at
 * the high level and above, the LOW colour at the low level and below, the
 * MEDIUM colour halfway between the two levels, and a blend in between.
 *
 * A plain blend of two colours dims halfway when they sit far apart -- blue
 * and red meet at a half-bright purple -- so the blend is scaled to the
 * brightness of its ends, taken as their brightest channel. Between colours
 * next to each other (green, yellow, red) a plain blend already keeps it, and
 * the result is the usual hue sweep.
 *
 * Integer math only; no Zephyr dependency, so a host test can include it.
 */

#pragma once

#include <stdint.h>

static inline uint8_t battery_gradient_channel(uint32_t rgb, int shift) {
    return (uint8_t)((rgb >> shift) & 0xFF);
}

static inline uint8_t battery_gradient_brightest(uint32_t rgb) {
    uint8_t m = battery_gradient_channel(rgb, 16);

    if (battery_gradient_channel(rgb, 8) > m) {
        m = battery_gradient_channel(rgb, 8);
    }
    if (battery_gradient_channel(rgb, 0) > m) {
        m = battery_gradient_channel(rgb, 0);
    }
    return m;
}

/* `from` blended toward `to` by num/den (0 <= num <= den, den > 0), at the
 * brightness the two ends have at that point. Worked in 1/1024ths of a step
 * and rounded once at the end, so the result does not depend on direction. */
static inline uint32_t battery_gradient_blend(uint32_t from, uint32_t to, int32_t num,
                                              int32_t den) {
    int32_t c[3];
    int32_t brightest = 0;

    for (int i = 0; i < 3; i++) {
        const int shift = 16 - 8 * i;
        const int32_t a = battery_gradient_channel(from, shift);
        const int32_t b = battery_gradient_channel(to, shift);

        c[i] = a * 1024 + (b - a) * 1024 * num / den;
        if (c[i] > brightest) {
            brightest = c[i];
        }
    }

    const int32_t a_max = battery_gradient_brightest(from);
    const int32_t b_max = battery_gradient_brightest(to);
    const int32_t target = a_max * 1024 + (b_max - a_max) * 1024 * num / den;
    uint32_t rgb = 0;

    for (int i = 0; i < 3; i++) {
        /* Scaled to the target brightness, back to 0..255, rounded. */
        int32_t v = brightest > 0 ? (int32_t)(((int64_t)c[i] * target / brightest + 512) / 1024)
                                  : 0;

        if (v > 255) {
            v = 255;
        }
        rgb |= (uint32_t)v << (16 - 8 * i);
    }
    return rgb;
}

/* The colour for `level` percent; `high` > `low`. */
static inline uint32_t battery_gradient_rgb(uint8_t level, uint8_t high, uint8_t low,
                                            uint32_t rgb_high, uint32_t rgb_medium,
                                            uint32_t rgb_low) {
    if (level >= high) {
        return rgb_high;
    }
    if (level <= low) {
        return rgb_low;
    }

    /* In halves of a percent, so that the midpoint is exact. */
    const int32_t span = (int32_t)high - (int32_t)low;
    const int32_t at = 2 * (int32_t)level - (int32_t)high - (int32_t)low;

    if (at >= 0) {
        return battery_gradient_blend(rgb_medium, rgb_high, at, span);
    }
    return battery_gradient_blend(rgb_low, rgb_medium, at + span, span);
}
