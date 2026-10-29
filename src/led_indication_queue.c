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
    if (queue->waiting_count == LED_INDICATION_QUEUE_LENGTH) {
        return false;
    }
    struct led_indication added = *indication;
    added.added_at = now;
    added.offered_at = 0;
    queue->waiting[WAITING_INDEX(queue, queue->waiting_count)] = added;
    queue->waiting_count++;
    return true;
}

uint32_t led_indication_queue_add_synced(struct led_indication_queue *queue,
                                         const struct led_indication *indication, uint32_t count,
                                         int64_t now) {
    // Behind the synced ones that wait, ahead of this half's own.
    uint32_t position = 0;
    for (uint32_t i = 0; i < queue->waiting_count; i++) {
        if (queue->waiting[WAITING_INDEX(queue, i)].sync_id != 0) {
            position = i + 1;
        }
    }

    uint32_t queued = 0;
    for (uint32_t i = 0; i < count && position < LED_INDICATION_QUEUE_LENGTH; i++) {
        struct led_indication added = *indication;
        added.added_at = now;
        added.offered_at = 0;
        added.waits_for_start = false;
        if (i > 0) {
            added.start_at = 0; // straight after the one before
        }
        insert_waiting(queue, position, &added);
        position++;
        queued++;
    }
    return queued;
}

bool led_indication_queue_synced_waiting(const struct led_indication_queue *queue) {
    for (uint32_t i = 0; i < queue->waiting_count; i++) {
        if (queue->waiting[WAITING_INDEX(queue, i)].sync_id != 0) {
            return true;
        }
    }
    return false;
}

static bool dark_after_on(const struct led_indication_queue *queue,
                          const struct led_indication *indication) {
    if (indication->kind != LED_INDICATION_BLINK) {
        return false;
    }
    if (indication->dark & LED_DARK_DECIDED) {
        return (indication->dark & LED_DARK_AFTER) != 0;
    }
    return indication->rgb != 0 && indication->rgb == queue->rest_rgb;
}

static bool dark_before(const struct led_indication_queue *queue,
                        const struct led_indication *indication) {
    if (indication->kind != LED_INDICATION_BLINK) {
        return false;
    }
    if (indication->dark & LED_DARK_DECIDED) {
        return (indication->dark & LED_DARK_BEFORE) != 0;
    }
    return indication->rgb != 0 && indication->rgb == queue->shown_rgb;
}

int64_t led_indication_queue_current_ends_at(const struct led_indication_queue *queue,
                                             int64_t now) {
    const struct led_indication *current = &queue->current;
    const int64_t gap = gap_after(queue, current);
    const int64_t dark_after = dark_after_on(queue, current) ? queue->interval_ms : 0;
    int64_t ends_at;

    switch (queue->phase) {
    case LED_PHASE_DARK_BEFORE:
        ends_at = queue->phase_ends_at + on_duration(current) + dark_after + gap;
        break;
    case LED_PHASE_ON:
        ends_at = queue->phase_ends_at + dark_after + gap;
        break;
    case LED_PHASE_DARK_AFTER:
        ends_at = queue->phase_ends_at + gap;
        break;
    case LED_PHASE_GAP:
        ends_at = queue->phase_ends_at;
        break;
    case LED_PHASE_RESTING:
    default:
        ends_at = now;
        break;
    }
    return ends_at > now ? ends_at : now;
}

uint32_t led_indication_queue_take_offers(struct led_indication_queue *queue, int64_t now,
                                          struct led_indication *offers, uint32_t max,
                                          uint8_t (*new_id)(void)) {
    uint32_t taken = 0;

    // Their turn has come when nothing shows, or a synced one does: that one
    // runs on every half alike, so what follows it can be agreed on now.
    if (queue->phase != LED_PHASE_RESTING && queue->current.sync_id == 0) {
        return 0;
    }
    for (uint32_t i = 0; i < queue->waiting_count && taken < max; i++) {
        struct led_indication *waiting = &queue->waiting[WAITING_INDEX(queue, i)];

        if (waiting->sync_id == 0) {
            break; // this half's own goes first: not past it
        }
        if (waiting->waits_for_start && waiting->offered_at == 0) {
            const uint8_t old_id = waiting->sync_id;
            const uint8_t id = new_id();
            uint8_t dark = waiting->dark;

            if (!(dark & LED_DARK_DECIDED)) {
                const bool as_rest = waiting->kind == LED_INDICATION_BLINK &&
                                     waiting->rgb != 0 &&
                                     waiting->rgb == queue->rest_rgb_as_set;

                dark = LED_DARK_DECIDED | (as_rest ? LED_DARK_BEFORE | LED_DARK_AFTER : 0);
            }
            waiting->offered_at = now > 0 ? now : 1;
            // The whole group -- this run of it, nothing further back --
            // alike.
            for (uint32_t j = i; j < queue->waiting_count; j++) {
                struct led_indication *member = &queue->waiting[WAITING_INDEX(queue, j)];

                if (member->sync_id != old_id || (j > i && member->waits_for_start)) {
                    break;
                }
                member->sync_id = id;
                member->dark = dark;
            }
            offers[taken++] = *waiting;
        }
    }
    return taken;
}

bool led_indication_queue_set_start(struct led_indication_queue *queue, uint8_t sync_id,
                                    int64_t start_at) {
    for (uint32_t i = 0; i < queue->waiting_count; i++) {
        struct led_indication *waiting = &queue->waiting[WAITING_INDEX(queue, i)];

        if (waiting->sync_id == sync_id && waiting->waits_for_start) {
            waiting->waits_for_start = false;
            waiting->start_at = start_at;
            return true;
        }
    }
    return false;
}

void led_indication_queue_set_rest(struct led_indication_queue *queue, uint32_t rgb,
                                   uint32_t blank_ms, bool drop_indications, int64_t now) {
    queue->rest_rgb = rgb;
    queue->rest_rgb_as_set = rgb;
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
    queue->rest_rgb_as_set = 0;
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
    // A synced one starts when the halves agreed, however late this runs.
    if (queue->current.start_at > start) {
        start = queue->current.start_at;
    }
    if (start > now) {
        start = now;
    }

    if (dark_before(queue, &queue->current)) {
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
        if (dark_after_on(queue, &queue->current)) {
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

// When the first waiting one may start: a synced one at its start, one that
// waits for its start not before it is given one -- or, unanswered, at the
// hold limit.
static int64_t next_may_start_at(struct led_indication_queue *queue, int64_t now) {
    struct led_indication *first = &queue->waiting[queue->first_waiting];

    if (first->waits_for_start) {
        if (first->offered_at == 0) {
            return LED_INDICATION_NO_DEADLINE; // handed out when its turn comes
        }
        const int64_t give_up_at = first->offered_at + LED_SYNC_UNANSWERED_HOLD_LIMIT_MS;

        if (now < give_up_at) {
            return give_up_at;
        }
        first->waits_for_start = false;
        first->start_at = now;
        queue->unanswered_holds_given_up++;
    }
    return first->start_at > now ? first->start_at : now;
}

uint32_t led_indication_queue_colour(struct led_indication_queue *queue, int64_t now,
                                     int64_t *next_change_at) {
    uint32_t rgb;
    for (;;) {
        if (queue->phase == LED_PHASE_RESTING) {
            *next_change_at = LED_INDICATION_NO_DEADLINE;
            if (queue->waiting_count > 0) {
                const int64_t may_start_at = next_may_start_at(queue, now);

                if (may_start_at <= now) {
                    start_next(queue, now);
                    continue;
                }
                *next_change_at = may_start_at;
            }
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
