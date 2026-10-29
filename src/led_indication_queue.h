/*
 * What the LED shows from moment to moment: indications one after another,
 * each followed by a gap, and between them the resting colour -- the layer
 * colour, until its blanking time runs out.
 *
 * Some indications are shown on every half of a split at once (synced): the
 * central and its peripherals agree when each starts (led_sync.h).
 * On the central such a group waits at the head of the queue until the
 * peripherals have said when they can show it; on a peripheral it is queued
 * with that start already set, ahead of the half's own indications that
 * wait, behind the one that shows.
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

// A synced group on the central that was offered to the peripherals and
// never given a start is shown anyway this long after the offer. The sender
// always answers well within it (layer_push_to_peripherals.c); this only
// keeps a lost answer from holding the LED for good.
#define LED_SYNC_UNANSWERED_HOLD_LIMIT_MS 8000

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

    // Shown on every half at once: the number the central gave its group;
    // 0 for this half's own.
    uint8_t sync_id;
    // A blink dark before and after it (enum led_dark): decided for a
    // synced group by the central, so both halves time it alike; else each
    // half decides by the colour it shows.
    uint8_t dark;
    // Central, the first of a synced group: waits at the head of the queue
    // until led_indication_queue_set_start() says when it starts.
    bool waits_for_start;
    // Not started before this uptime (ms), and timed from it even when
    // started later; 0: as soon as its turn comes.
    int64_t start_at;
    // Central, the first of a synced group: what the peripherals are sent
    // for the whole group, and which of them (by link slot bit).
    uint32_t peripheral_param1;
    uint32_t peripheral_param2;
    uint32_t peripheral_slot_mask;

    // Set by the queue.
    int64_t added_at;
    // When led_indication_queue_take_offers() handed it out; 0: not yet.
    int64_t offered_at;
};

enum led_dark {
    // Decided: the two below say it. Else by colour: dark before if the LED
    // shows the blink's colour already, after if the resting colour is it.
    LED_DARK_DECIDED = 1,
    LED_DARK_BEFORE = 2,
    LED_DARK_AFTER = 4,
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
    // The resting colour as set, kept when it blanks: the same on every half
    // of a split, as the layer colour is.
    uint32_t rest_rgb_as_set;

    uint32_t shown_rgb;

    // Synced groups shown after LED_SYNC_UNANSWERED_HOLD_LIMIT_MS without a
    // start: widget.c logs each.
    uint32_t unanswered_holds_given_up;
};

void led_indication_queue_init(struct led_indication_queue *queue, uint32_t interval_ms);

// Queues it behind everything. Returns false when the queue is full and the
// indication was dropped.
bool led_indication_queue_add(struct led_indication_queue *queue,
                              const struct led_indication *indication, int64_t now);

// Peripheral: a synced group from the central -- count copies of
// indication, the first with its start_at, the others each straight after
// the one before. Queued behind the synced groups that wait, ahead of this
// half's own; on a full queue the last of its own give way. Returns how many
// were queued.
uint32_t led_indication_queue_add_synced(struct led_indication_queue *queue,
                                         const struct led_indication *indication, uint32_t count,
                                         int64_t now);

// Whether a synced group waits, not started yet: one queued behind it starts
// straight after it.
bool led_indication_queue_synced_waiting(const struct led_indication_queue *queue);

// When what shows now -- with its gap -- is over: now when nothing shows.
int64_t led_indication_queue_current_ends_at(const struct led_indication_queue *queue,
                                             int64_t now);

// Central: the synced groups at the head of the queue whose turn has come
// and that were not handed out yet -- the first of each, at most max, in
// order -- marked as handed out, numbered afresh by new_id() (so that one
// never meets an older group's number), their dark before and after decided
// (by the resting colour as set) for every half. Their peripherals are sent
// them now.
uint32_t led_indication_queue_take_offers(struct led_indication_queue *queue, int64_t now,
                                          struct led_indication *offers, uint32_t max,
                                          uint8_t (*new_id)(void));

// Central: synced group sync_id starts at start_at, an uptime: one already
// past starts at once, as far into it as the other halves are; 0 starts it
// straight after what is ahead of it. Returns false when it is no longer
// queued -- a layer colour cut it short.
bool led_indication_queue_set_start(struct led_indication_queue *queue, uint8_t sync_id,
                                    int64_t start_at);

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
