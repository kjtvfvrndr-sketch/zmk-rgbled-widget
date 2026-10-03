/*
 * What the LED shows from moment to moment: indications one after another,
 * each followed by a gap, and between them the resting colour -- the layer
 * colour, until its blanking time runs out.
 *
 * Plain C with no Zephyr in it, so the host tests drive it with a clock of
 * their own. widget.c owns one, touches it from the LED thread only, and
 * writes to the strip whatever led_indication_queue_colour() returns.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define LED_INDICATION_QUEUE_LENGTH 16
#define LED_INDICATION_NO_DEADLINE INT64_MAX
#define LED_BREATHE_FRAME_MS 20

enum led_indication_kind {
    // on_ms in rgb, dark for interval_ms before it if the LED shows rgb
    // already, and after it if the resting colour is rgb too.
    LED_INDICATION_BLINK,
    // breathe_cycles times from dark up to rgb and back, on_ms each.
    LED_INDICATION_BREATHE,
};

struct led_indication {
    enum led_indication_kind kind;
    uint32_t rgb;
    uint32_t on_ms;
    uint32_t breathe_cycles;
    // The resting colour after it, before the next indication; 0 means
    // interval_ms.
    uint32_t gap_ms;
    // A connection blink: added while a breathe is running it is shown at
    // once, and the breathe starts over after it; added while one waits it
    // goes ahead of it.
    bool preempts_breathe;
    // Set by led_indication_queue_add().
    int64_t added_at;
};

enum led_indication_phase {
    LED_PHASE_RESTING,
    LED_PHASE_DARK_BEFORE,
    LED_PHASE_ON,
    LED_PHASE_DARK_AFTER,
    LED_PHASE_GAP,
};

struct led_indication_queue {
    uint32_t interval_ms;

    struct led_indication waiting[LED_INDICATION_QUEUE_LENGTH];
    uint32_t first_waiting;
    uint32_t waiting_count;

    struct led_indication current;
    enum led_indication_phase phase;
    int64_t phase_started_at;
    int64_t phase_ends_at;

    uint32_t rest_rgb;
    uint32_t rest_blank_ms;
    int64_t rest_shown_since;

    uint32_t shown_rgb;
};

void led_indication_queue_init(struct led_indication_queue *queue, uint32_t interval_ms);

// Returns false when the queue is full and the indication was dropped.
bool led_indication_queue_add(struct led_indication_queue *queue,
                              const struct led_indication *indication, int64_t now);

// The colour to show between indications, blanked blank_ms after it is shown
// (0: never). With drop_indications the running and waiting ones end, and it
// shows at once.
void led_indication_queue_set_rest(struct led_indication_queue *queue, uint32_t rgb,
                                   uint32_t blank_ms, bool drop_indications, int64_t now);

// Ends a running breathe and drops the waiting ones.
void led_indication_queue_stop_breathing(struct led_indication_queue *queue, int64_t now);

// Drops every indication and the resting colour.
void led_indication_queue_clear(struct led_indication_queue *queue, int64_t now);

// The colour to show at now; *next_change_at is when it may change next
// without anything added (LED_INDICATION_NO_DEADLINE: not by itself).
uint32_t led_indication_queue_colour(struct led_indication_queue *queue, int64_t now,
                                     int64_t *next_change_at);

// rgb at the brightness a breathe of cycle_ms has elapsed_ms in: dark at the
// start of each cycle, rgb itself halfway.
uint32_t led_breathe_rgb(uint32_t rgb, uint32_t elapsed_ms, uint32_t cycle_ms);
