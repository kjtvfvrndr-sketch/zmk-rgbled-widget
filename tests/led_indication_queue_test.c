/*
 * Host tests for the LED queue: cc -I../src led_indication_queue_test.c
 * ../src/led_indication_queue.c && ./a.out
 */
#include <assert.h>
#include <stdio.h>

#include "led_indication_queue.h"

#define INTERVAL 500
#define RED 0xFF0000
#define BLUE 0x0000FF
#define GREEN 0x00FF00
#define MAGENTA 0xFF00FF

static uint32_t at(struct led_indication_queue *q, int64_t now) {
    int64_t next;
    return led_indication_queue_colour(q, now, &next);
}

static int64_t next_at(struct led_indication_queue *q, int64_t now) {
    int64_t next;
    (void)led_indication_queue_colour(q, now, &next);
    return next;
}

// Group numbers as the central gives them when it offers: 100, 101, ...
static uint8_t next_id = 100;
static uint8_t new_id(void) { return next_id++; }

static struct led_indication blink(uint32_t rgb, uint32_t on_ms) {
    return (struct led_indication){.kind = LED_INDICATION_BLINK, .rgb = rgb, .on_ms = on_ms};
}

// A peripheral's synced group goes ahead of its own waiting ones, behind the
// one that shows, at the start it was given.
static void test_peripheral_synced_ahead_of_own(void) {
    struct led_indication_queue q;
    led_indication_queue_init(&q, INTERVAL);

    struct led_indication battery = blink(GREEN, 2000);
    struct led_indication reminder = blink(RED, 300);
    assert(led_indication_queue_add(&q, &battery, 0));
    assert(at(&q, 0) == GREEN);
    assert(led_indication_queue_add(&q, &reminder, 100));

    // Battery 0..2000, its gap to 2500.
    assert(led_indication_queue_current_ends_at(&q, 100) == 2500);
    assert(!led_indication_queue_synced_waiting(&q));

    struct led_indication magenta = blink(MAGENTA, 1000);
    magenta.sync_id = 7;
    magenta.start_at = 2500;
    assert(led_indication_queue_add_synced(&q, &magenta, 1, 100) == 1);
    assert(led_indication_queue_synced_waiting(&q));

    assert(at(&q, 1999) == GREEN);
    assert(at(&q, 2000) == 0); // gap, no resting colour
    assert(at(&q, 2500) == MAGENTA);
    assert(at(&q, 3499) == MAGENTA);
    assert(at(&q, 3500) == 0);
    assert(at(&q, 4000) == RED); // its own after
    printf("ok peripheral synced ahead of own\n");
}

// A start in the future keeps the LED resting until then, and a late wake-up
// keeps the agreed timeline.
static void test_start_at_kept_when_late(void) {
    struct led_indication_queue q;
    led_indication_queue_init(&q, INTERVAL);

    struct led_indication b = blink(BLUE, 1000);
    b.sync_id = 3;
    b.start_at = 1000;
    led_indication_queue_add_synced(&q, &b, 1, 0);
    assert(at(&q, 0) == 0);
    assert(next_at(&q, 10) == 1000);
    // The thread wakes 30 ms late: the blink still ends at 2000.
    assert(at(&q, 1030) == BLUE);
    assert(at(&q, 1999) == BLUE);
    assert(at(&q, 2000) == 0);
    printf("ok start kept when late\n");
}

// A group: the first at its start, the rest straight after each other.
static void test_group_follows(void) {
    struct led_indication_queue q;
    led_indication_queue_init(&q, INTERVAL);

    struct led_indication r = blink(RED, 200);
    r.gap_ms = 200;
    r.sync_id = 9;
    r.start_at = 100;
    assert(led_indication_queue_add_synced(&q, &r, 3, 0) == 3);
    assert(at(&q, 100) == RED);
    assert(at(&q, 300) == 0);
    assert(at(&q, 500) == RED);
    assert(at(&q, 800) == 0);
    assert(at(&q, 900) == RED);
    assert(at(&q, 1100) == 0);
    printf("ok group follows\n");
}

// Central: a group waiting for its start holds the head and what is behind
// it; it is offered once when its turn comes.
static void test_central_hold_and_offer(void) {
    struct led_indication_queue q;
    led_indication_queue_init(&q, INTERVAL);

    struct led_indication power = {.kind = LED_INDICATION_BREATHE, .rgb = GREEN, .on_ms = 1000,
                                   .breathe_cycles = 2};
    assert(led_indication_queue_add(&q, &power, 0));
    (void)at(&q, 0);

    struct led_indication white = blink(0xFFFFFF, 1000);
    white.sync_id = 4;
    white.waits_for_start = true;
    white.peripheral_param1 = 0x1234;
    assert(led_indication_queue_add(&q, &white, 300));
    struct led_indication own = blink(RED, 300);
    assert(led_indication_queue_add(&q, &own, 400));

    struct led_indication offers[4];
    (void)at(&q, 500);
    assert(led_indication_queue_take_offers(&q, 500, offers, 4, new_id) == 0); // breathing
    (void)at(&q, 2500);                                                 // breathe and gap over
    assert(q.phase == LED_PHASE_RESTING);
    assert(led_indication_queue_take_offers(&q, 2500, offers, 4, new_id) == 1);
    assert(offers[0].peripheral_param1 == 0x1234);
    const uint8_t white_id = offers[0].sync_id;
    assert(led_indication_queue_take_offers(&q, 2510, offers, 4, new_id) == 0); // once
    assert(at(&q, 2600) == 0);                                          // held
    assert(!led_indication_queue_set_start(&q, 4, 2700)); // its number before the offer
    assert(led_indication_queue_set_start(&q, white_id, 2700));
    assert(at(&q, 2699) == 0);
    assert(at(&q, 2700) == 0xFFFFFF);
    assert(at(&q, 3700) == 0);
    assert(at(&q, 4200) == RED);
    printf("ok central hold and offer\n");
}

// An offered group that never gets a start shows at the hold limit.
static void test_central_hold_limit(void) {
    struct led_indication_queue q;
    led_indication_queue_init(&q, INTERVAL);

    struct led_indication b = blink(BLUE, 1000);
    b.sync_id = 5;
    b.waits_for_start = true;
    led_indication_queue_add(&q, &b, 0);
    struct led_indication offers[4];
    (void)at(&q, 0);
    assert(led_indication_queue_take_offers(&q, 10, offers, 4, new_id) == 1);
    assert(next_at(&q, 20) == 10 + LED_SYNC_UNANSWERED_HOLD_LIMIT_MS);
    assert(at(&q, 10 + LED_SYNC_UNANSWERED_HOLD_LIMIT_MS) == BLUE);
    assert(q.unanswered_holds_given_up == 1);
    printf("ok central hold limit\n");
}

// Groups queued back to back are offered together; a group behind one of
// this half's own waits for it.
static void test_offers_stop_at_own(void) {
    struct led_indication_queue q;
    led_indication_queue_init(&q, INTERVAL);

    struct led_indication a = blink(MAGENTA, 1000);
    a.sync_id = 1;
    a.waits_for_start = true;
    struct led_indication b = blink(BLUE, 1000);
    b.sync_id = 2;
    b.waits_for_start = true;
    struct led_indication own = blink(RED, 300);
    struct led_indication c = blink(BLUE, 1000);
    c.sync_id = 3;
    c.waits_for_start = true;
    led_indication_queue_add(&q, &a, 0);
    led_indication_queue_add(&q, &b, 0);
    led_indication_queue_add(&q, &own, 0);
    led_indication_queue_add(&q, &c, 0);
    struct led_indication offers[4];
    (void)at(&q, 0);
    assert(led_indication_queue_take_offers(&q, 0, offers, 4, new_id) == 2);
    assert(offers[0].peripheral_param1 == 0 && offers[1].sync_id == offers[0].sync_id + 1);
    // b straight after a.
    assert(led_indication_queue_set_start(&q, offers[0].sync_id, 100));
    assert(led_indication_queue_set_start(&q, offers[1].sync_id, 0)); /* 0: straight after a */
    assert(at(&q, 100) == MAGENTA);
    assert(at(&q, 1100) == 0);
    assert(at(&q, 1600) == BLUE);
    printf("ok offers stop at own\n");
}

// A layer colour cuts a held group: its start finds nothing.
static void test_colour_cuts_held(void) {
    struct led_indication_queue q;
    led_indication_queue_init(&q, INTERVAL);
    struct led_indication b = blink(BLUE, 1000);
    b.sync_id = 6;
    b.waits_for_start = true;
    led_indication_queue_add(&q, &b, 0);
    led_indication_queue_set_rest(&q, GREEN, 0, true, 10);
    assert(!led_indication_queue_set_start(&q, 6, 20));
    assert(at(&q, 30) == GREEN);
    printf("ok colour cuts held\n");
}

// What shows now ends with its gap, the dark after a blink in the resting
// colour counted.
static void test_current_ends_at(void) {
    struct led_indication_queue q;
    led_indication_queue_init(&q, INTERVAL);
    led_indication_queue_set_rest(&q, BLUE, 0, false, 0);
    assert(at(&q, 0) == BLUE);
    struct led_indication b = blink(BLUE, 1000);
    led_indication_queue_add(&q, &b, 0);
    (void)at(&q, 0); // dark before: shown is blue
    assert(q.phase == LED_PHASE_DARK_BEFORE);
    assert(led_indication_queue_current_ends_at(&q, 0) == 500 + 1000 + 500 + 500);
    printf("ok current ends at\n");
}

// The central decides dark before and after for a synced group by the
// resting colour as set -- even blanked -- and every member gets it.
static void test_offer_decides_dark(void) {
    struct led_indication_queue q;
    led_indication_queue_init(&q, INTERVAL);
    led_indication_queue_set_rest(&q, BLUE, 100, false, 0);
    assert(at(&q, 0) == BLUE);
    assert(at(&q, 100) == 0); // blanked
    struct led_indication b = blink(BLUE, 300);
    b.sync_id = 8;
    b.waits_for_start = true;
    led_indication_queue_add(&q, &b, 200);
    b.waits_for_start = false;
    led_indication_queue_add(&q, &b, 200);
    struct led_indication offers[4];
    (void)at(&q, 200);
    assert(led_indication_queue_take_offers(&q, 200, offers, 4, new_id) == 1);
    assert(offers[0].dark == (LED_DARK_DECIDED | LED_DARK_BEFORE | LED_DARK_AFTER));
    assert(led_indication_queue_set_start(&q, offers[0].sync_id, 1000));
    // Dark before though the LED shows black: as on the other half.
    assert(at(&q, 1000) == 0);
    assert(q.phase == LED_PHASE_DARK_BEFORE);
    assert(at(&q, 1500) == BLUE);
    printf("ok offer decides dark\n");
}

// A peripheral follows the central's decision, not its own colours.
static void test_decided_dark_followed(void) {
    struct led_indication_queue q;
    led_indication_queue_init(&q, INTERVAL);
    led_indication_queue_set_rest(&q, GREEN, 0, false, 0);
    assert(at(&q, 0) == GREEN);
    struct led_indication g = blink(GREEN, 1000);
    g.sync_id = 4;
    g.start_at = 100;
    g.dark = LED_DARK_DECIDED; // the central said: none
    led_indication_queue_add_synced(&q, &g, 1, 0);
    assert(at(&q, 100) == GREEN);
    assert(q.phase == LED_PHASE_ON);
    assert(at(&q, 1100) == GREEN); // gap, no dark after
    assert(q.phase == LED_PHASE_GAP);
    printf("ok decided dark followed\n");
}

// Numbering at the offer renames only that group's run, not an older group
// further back that happens to share its number.
static void test_offer_renames_only_its_run(void) {
    struct led_indication_queue q;
    led_indication_queue_init(&q, INTERVAL);
    struct led_indication a = blink(BLUE, 300);
    a.sync_id = 7;
    a.waits_for_start = true;
    struct led_indication own = blink(RED, 300);
    struct led_indication b = a; // same number, a later group behind
    led_indication_queue_add(&q, &a, 0);
    led_indication_queue_add(&q, &own, 0);
    led_indication_queue_add(&q, &b, 0);
    struct led_indication offers[4];
    (void)at(&q, 0);
    assert(led_indication_queue_take_offers(&q, 0, offers, 4, new_id) == 1);
    assert(offers[0].sync_id != 7);
    assert(q.waiting[(q.first_waiting + 2) % LED_INDICATION_QUEUE_LENGTH].sync_id == 7);
    printf("ok offer renames only its run\n");
}

int main(void) {
    test_peripheral_synced_ahead_of_own();
    test_start_at_kept_when_late();
    test_group_follows();
    test_central_hold_and_offer();
    test_central_hold_limit();
    test_offers_stop_at_own();
    test_colour_cuts_held();
    test_current_ends_at();
    test_offer_decides_dark();
    test_decided_dark_followed();
    test_offer_renames_only_its_run();
    printf("all LED queue tests passed\n");
    return 0;
}
