/*
 * Showing an indication or a layer colour on every half of a Bluetooth split
 * at once.
 *
 * The central sends it to a peripheral when its turn comes on the central's
 * own LED -- as the lyr_sync behavior, its group number in the event's
 * position. Writes reach the peripheral only when it attends a connection
 * event: with its latency that may be a while. Both halves time the start
 * from that one event:
 *
 *  - The peripheral takes the write in it and works out when it can start:
 *    when what it shows now is over, and no earlier than
 *    rgbled_widget_sync_shortest_start_in_ms() from now. It answers at once
 *    over a characteristic of its own (struct led_sync_answer). Having
 *    something to send it attends the very next connection event: the
 *    answer reaches the central one connection interval later.
 *  - The central learns of the write's link-layer acknowledgement in that
 *    same event and starts at that moment plus what the answer says -- the
 *    acknowledgement alone is the reference, also when the answer is seen
 *    first. The shortest start, a connection interval and a margin, lets the
 *    answer arrive before then.
 *
 * A group sent while another synced one waits on the peripheral starts
 * straight after it there: the answer says so, and the central does the
 * same. A peripheral that is not ready when the group's turn comes is not
 * sent it; one that does not answer in time, or goes away, is left out. The
 * central shows the group on its own then.
 *
 * This header is private to the widget: widget.c, layer_push_to_peripherals.c
 * (central) and sync_answers_to_central.c (peripheral).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <zmk_rgbled_widget/widget.h>

// Central: the layer colour and the split-connected blink shown on every half
// at once -- and, with PUSH_INDICATIONS, what concerns the whole keyboard.
#define SYNC_WITH_PERIPHERALS (LAYER_PUSH) && IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
// Peripheral: shows what the central syncs, and answers it.
#define SYNC_WITH_CENTRAL (SPLIT_PERIPHERAL) && IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)

/* ---- over the air ---- */

// The peripheral's service and characteristic for the answers (notify).
#define LED_SYNC_SERVICE_UUID                                                                      \
    BT_UUID_128_ENCODE(0x8f3a1c52, 0x6b1e, 0x4d1a, 0x9c3e, 0x5a7d2b6e0f10)
#define LED_SYNC_ANSWER_CHAR_UUID                                                                  \
    BT_UUID_128_ENCODE(0x8f3a1c52, 0x6b1e, 0x4d1a, 0x9c3e, 0x5a7d2b6e0f11)

enum led_sync_start {
    // start_in_ms after the connection event that brought the write.
    LED_SYNC_START_IN = 0,
    // Straight after the synced group before it.
    LED_SYNC_START_AFTER_PREVIOUS = 1,
};

struct led_sync_answer {
    uint8_t sync_id;
    uint8_t start; // enum led_sync_start
    uint16_t start_in_ms;
} __attribute__((packed));

/* ---- timing ---- */

// Added to the connection interval for the shortest start: the time the
// peripheral needs to have its answer out for the next event, and the
// central to take it in.
#define LED_SYNC_ANSWER_MARGIN_MS 20

// Central: a group goes to the peripherals that are ready when its turn
// comes. How long it waits for room to be written (WRITES_IN_FLIGHT), then
// for the write's acknowledgement, and after it for the answer. A
// peripheral that misses one is left out.
#define LED_SYNC_WAIT_TO_BE_SENT_MS 2000
#define LED_SYNC_WAIT_FOR_DELIVERY_MS 1000
#define LED_SYNC_WAIT_FOR_ANSWER_MS 1000
// An answer can come before this half sees the write's acknowledgement --
// even in the same connection event, while a parameter change runs. The start
// is timed from the acknowledgement: waited for this long after the answer,
// then timed from the answer's arrival.
#define LED_SYNC_WAIT_FOR_ACK_AFTER_ANSWER_MS 200
// The latest start a peripheral may ask for; later it is shown there late.
#define LED_SYNC_LONGEST_START_IN_MS 5000
// A start this far off waits for what the peripheral shows first, not for
// the link: logged at INF, the usual tens of ms only at DBG.
#define LED_SYNC_LOG_WAITING_START_FROM_MS 200

/* ---- central: widget.c <-> layer_push_to_peripherals.c ---- */

#if SYNC_WITH_PERIPHERALS
// A new group number, never 0.
uint8_t rgbled_widget_sync_new_id(void);

// The link slots of peripherals that are ready to be synced with, and
// whether one is connected but not ready yet.
uint32_t rgbled_widget_ready_peripheral_slots(void);
bool rgbled_widget_peripheral_getting_ready(void);

// Called by the sender when the peripheral on link slot `slot` is ready.
void rgbled_widget_peripheral_ready(uint8_t slot);

// The LED thread: synced group sync_id, queued at queued_at, has come to
// its turn. The peripherals in slot_mask ready since before it was queued are
// sent param1/param2 (widget.h); the start comes back through
// rgbled_widget_start_synced_group_at().
void rgbled_widget_offer_synced_group(uint8_t sync_id, uint32_t param1, uint32_t param2,
                                      uint32_t slot_mask, int64_t queued_at);

// Called by the sender: synced group sync_id starts at start_at (an uptime;
// 0: straight after the one before it).
void rgbled_widget_start_synced_group_at(uint8_t sync_id, int64_t start_at);

// A new layer colour for every half: shown here when the ready peripherals
// can show it too, at once with none.
void rgbled_widget_change_layer_colour_on_all_halves(uint32_t rgb, uint32_t blank_ms,
                                                     bool interrupts);

// Called by the sender: the layer colour shows here at `at` (an uptime).
void rgbled_widget_apply_layer_colour_at(uint32_t rgb, uint32_t blank_ms, bool interrupts,
                                         int64_t at);
#endif

/* ---- peripheral: widget.c <-> sync_answers_to_central.c ---- */

#if SYNC_WITH_CENTRAL
// The earliest a synced group may start after the write came: one
// connection interval of the link to the central and the margin.
uint32_t rgbled_widget_sync_shortest_start_in_ms(void);

// Sends the answer for group sync_id. Returns at once.
void rgbled_widget_answer_central(uint8_t sync_id, enum led_sync_start start,
                                  uint32_t start_in_ms);
#endif
