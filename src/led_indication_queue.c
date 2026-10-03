#include "led_indication_queue.h"

#include <stddef.h>

#define WAITING_INDEX(queue, position)                                                             \
    (((queue)->first_waiting + (position)) % LED_INDICATION_QUEUE_LENGTH)

void led_indication_queue_init(struct led_indication_queue *queue, uint32_t interval_ms) {
    *queue = (struct led_indication_queue){.interval_ms = interval_ms, .phase = LED_PHASE_RESTING};
}

static uint32_t gap_after(const struct led_indication_queue *queue,
                          const struct led_indication *indication) {
    return indication->gap_ms > 0 ? indication->gap_ms : queue->interval_ms;
}

static uint32_t on_duration(const struct led_indication *indication) {
    if (indication->kind == LED_INDICATION_BREATHE) {
        return indication->on_ms * indication->breathe_cycles;
    }
    return indication->on_ms;
}

static void enter_phase(struct led_indication_queue *queue, enum led_indication_phase phase,
                        int64_t at, uint32_t duration_ms) {
    queue->phase = phase;
    queue->phase_started_at = at;
    queue->phase_ends_at = at + duration_ms;
    if (phase == LED_PHASE_GAP) {
        queue->rest_shown_since = at;
    }
}

// Puts it at position among the waiting ones; on a full queue the last
// waiting one gives way.
static void insert_waiting(struct led_indication_queue *queue, uint32_t position,
                           const struct led_indication *indication) {
    if (queue->waiting_count == LED_INDICATION_QUEUE_LENGTH) {
        queue->waiting_count--;
    }
    if (position > queue->waiting_count) {
        position = queue->waiting_count;
    }
    for (uint32_t i = queue->waiting_count; i > position; i--) {
        queue->waiting[WAITING_INDEX(queue, i)] = queue->waiting[WAITING_INDEX(queue, i - 1)];
    }
    queue->waiting[WAITING_INDEX(queue, position)] = *indication;
    queue->waiting_count++;
}

bool led_indication_queue_add(struct led_indication_queue *queue,
                              const struct led_indication *indication, int64_t now) {
    struct led_indication added = *indication;
    added.added_at = now;

    if (added.preempts_breathe) {
        if (queue->phase == LED_PHASE_ON && queue->current.kind == LED_INDICATION_BREATHE &&
            now < queue->phase_ends_at) {
            struct led_indication breathe_again = queue->current;
            breathe_again.added_at = now;
            insert_waiting(queue, 0, &breathe_again);
            insert_waiting(queue, 0, &added);
            queue->phase = LED_PHASE_RESTING;
            queue->phase_started_at = now;
            return true;
        }
        // Ahead of a breathe that waits, too.
        for (uint32_t i = 0; i < queue->waiting_count; i++) {
            if (queue->waiting[WAITING_INDEX(queue, i)].kind == LED_INDICATION_BREATHE) {
                insert_waiting(queue, i, &added);
                return true;
            }
        }
    }

    if (queue->waiting_count == LED_INDICATION_QUEUE_LENGTH) {
        return false;
    }
    queue->waiting[WAITING_INDEX(queue, queue->waiting_count)] = added;
    queue->waiting_count++;
    return true;
}

void led_indication_queue_set_rest(struct led_indication_queue *queue, uint32_t rgb,
                                   uint32_t blank_ms, bool drop_indications, int64_t now) {
    queue->rest_rgb = rgb;
    queue->rest_blank_ms = blank_ms;
    queue->rest_shown_since = now;
    if (drop_indications) {
        queue->waiting_count = 0;
        queue->phase = LED_PHASE_RESTING;
        queue->phase_started_at = now;
    }
}

void led_indication_queue_stop_breathing(struct led_indication_queue *queue, int64_t now) {
    uint32_t kept = 0;
    for (uint32_t i = 0; i < queue->waiting_count; i++) {
        const struct led_indication waiting = queue->waiting[WAITING_INDEX(queue, i)];
        if (waiting.kind != LED_INDICATION_BREATHE) {
            queue->waiting[WAITING_INDEX(queue, kept)] = waiting;
            kept++;
        }
    }
    queue->waiting_count = kept;

    if (queue->phase == LED_PHASE_ON && queue->current.kind == LED_INDICATION_BREATHE) {
        enter_phase(queue, LED_PHASE_GAP, now, gap_after(queue, &queue->current));
    }
}

void led_indication_queue_clear(struct led_indication_queue *queue, int64_t now) {
    queue->waiting_count = 0;
    queue->phase = LED_PHASE_RESTING;
    queue->phase_started_at = now;
    queue->rest_rgb = 0;
    queue->rest_blank_ms = 0;
    queue->rest_shown_since = now;
}

static void start_next(struct led_indication_queue *queue, int64_t now) {
    queue->current = queue->waiting[queue->first_waiting];
    queue->first_waiting = (queue->first_waiting + 1) % LED_INDICATION_QUEUE_LENGTH;
    queue->waiting_count--;

    // From the end of the last gap if it was waiting by then, so a late
    // wake-up does not stretch the sequence; else from when it came.
    int64_t start = queue->phase_started_at;
    if (queue->current.added_at > start) {
        start = queue->current.added_at;
    }
    if (start > now) {
        start = now;
    }

    if (queue->current.kind == LED_INDICATION_BLINK && queue->current.rgb != 0 &&
        queue->current.rgb == queue->shown_rgb) {
        enter_phase(queue, LED_PHASE_DARK_BEFORE, start, queue->interval_ms);
    } else {
        enter_phase(queue, LED_PHASE_ON, start, on_duration(&queue->current));
    }
}

static void end_phase(struct led_indication_queue *queue) {
    const int64_t at = queue->phase_ends_at;
    const uint32_t gap_ms = gap_after(queue, &queue->current);

    switch (queue->phase) {
    case LED_PHASE_DARK_BEFORE:
        enter_phase(queue, LED_PHASE_ON, at, on_duration(&queue->current));
        break;
    case LED_PHASE_ON:
        if (queue->current.kind == LED_INDICATION_BLINK && queue->current.rgb != 0 &&
            queue->current.rgb == queue->rest_rgb) {
            enter_phase(queue, LED_PHASE_DARK_AFTER, at, queue->interval_ms);
        } else {
            enter_phase(queue, LED_PHASE_GAP, at, gap_ms);
        }
        break;
    case LED_PHASE_DARK_AFTER:
        enter_phase(queue, LED_PHASE_GAP, at, gap_ms);
        break;
    case LED_PHASE_GAP:
    default:
        queue->phase = LED_PHASE_RESTING;
        queue->phase_started_at = at;
        break;
    }
}

static int64_t earlier(int64_t a, int64_t b) { return a < b ? a : b; }

static uint32_t rest_colour(struct led_indication_queue *queue, int64_t now,
                            int64_t *next_change_at) {
    if (queue->rest_rgb != 0 && queue->rest_blank_ms > 0) {
        const int64_t blank_at = queue->rest_shown_since + queue->rest_blank_ms;
        if (now >= blank_at) {
            queue->rest_rgb = 0;
            queue->rest_blank_ms = 0;
        } else {
            *next_change_at = earlier(*next_change_at, blank_at);
        }
    }
    return queue->rest_rgb;
}

uint32_t led_breathe_rgb(uint32_t rgb, uint32_t elapsed_ms, uint32_t cycle_ms) {
    if (cycle_ms < 2) {
        return 0;
    }
    const uint32_t position = elapsed_ms % cycle_ms;
    const uint32_t half = cycle_ms / 2;

    // x rises from 0 to 1 over the first half of the cycle and falls back
    // over the second, in 1/65536.
    uint64_t x = position < half ? (uint64_t)position * 65536u / half
                                 : (uint64_t)(cycle_ms - position) * 65536u / (cycle_ms - half);
    if (x > 65536u) {
        x = 65536u;
    }
    // smoothstep, x²(3 - 2x): an even swell with no corner at either end,
    // close to a raised cosine without one.
    const uint64_t x_squared = (x * x) >> 16;
    const uint64_t swell = (x_squared * (3u * 65536u - 2u * x)) >> 16;
    // The eye judges brightness on a curve: squaring the swell makes the
    // light seem to rise and fall evenly instead of rushing up from dark.
    const uint64_t level = (swell * swell) >> 16;

    uint32_t out = 0;
    for (int shift = 0; shift <= 16; shift += 8) {
        const uint64_t channel = (rgb >> shift) & 0xFF;
        out |= (uint32_t)((channel * level + 32768u) >> 16) << shift;
    }
    return out;
}

static uint32_t phase_colour(struct led_indication_queue *queue, int64_t now,
                             int64_t *next_change_at) {
    *next_change_at = queue->phase_ends_at;

    switch (queue->phase) {
    case LED_PHASE_ON:
        if (queue->current.kind == LED_INDICATION_BREATHE) {
            const int64_t elapsed = now - queue->phase_started_at;
            const int64_t next_frame =
                queue->phase_started_at +
                (elapsed / LED_BREATHE_FRAME_MS + 1) * LED_BREATHE_FRAME_MS;
            *next_change_at = earlier(next_frame, queue->phase_ends_at);
            return led_breathe_rgb(queue->current.rgb, (uint32_t)elapsed, queue->current.on_ms);
        }
        return queue->current.rgb;
    case LED_PHASE_GAP:
        return rest_colour(queue, now, next_change_at);
    default:
        return 0;
    }
}

uint32_t led_indication_queue_colour(struct led_indication_queue *queue, int64_t now,
                                     int64_t *next_change_at) {
    uint32_t rgb;
    for (;;) {
        if (queue->phase == LED_PHASE_RESTING) {
            if (queue->waiting_count > 0) {
                start_next(queue, now);
                continue;
            }
            *next_change_at = LED_INDICATION_NO_DEADLINE;
            rgb = rest_colour(queue, now, next_change_at);
            break;
        }
        if (now < queue->phase_ends_at) {
            rgb = phase_colour(queue, now, next_change_at);
            break;
        }
        end_phase(queue);
    }
    queue->shown_rgb = rgb;
    return rgb;
}
