/*
 * When the boot report and the breathe on USB power are shown -- the
 * decisions, kept free of Zephyr so the host tests can drive them. widget.c
 * feeds them the state of the links and the power.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

// At boot on USB power, how long the boot report waits for a computer to
// take the USB connection before it shows the links: it does in a few
// hundred milliseconds (the host's 100 ms attach debounce, the reset, the
// descriptors), more through a hub. A charger never does.
#define LED_WAIT_FOR_USB_HOST_MS 1000

// A central's boot report waits for the links this long at most, then shows
// what there is (a peripheral: LED_BOOT_PERIPHERAL_WAITS_FOR_CENTRAL_MS).
#define LED_BOOT_WAIT_FOR_LINK_MS 3000

// A peripheral waits this long for its central before it shows that it is
// not there (cyan). Finding it can take a few seconds -- the central may be
// searching for it at low power -- and cyan for a link that comes moments
// later only misleads; the link coming ends the wait at once.
#define LED_BOOT_PERIPHERAL_WAITS_FOR_CENTRAL_MS 6000

// A central whose peripheral is connected by then but not ready to be synced
// with (its discovery comes 2 s after the link is secured) waits this much
// longer for it, so that the halves connected show before the computer's
// link, on both at once.
#define LED_BOOT_WAIT_FOR_PERIPHERAL_READY_MS 3000

// Whether the boot report may show the connection now, elapsed_ms after it
// began. With USB power on a half with a USB output the output is decided
// first: a computer may take the connection, a charger never will.
static inline bool led_boot_connection_settled(int64_t elapsed_ms, bool link_up,
                                               bool usb_output_possible, bool usb_power,
                                               bool usb_host_ready) {
    if (elapsed_ms >= LED_BOOT_WAIT_FOR_LINK_MS) {
        return true;
    }
    const bool output_decided = !usb_output_possible || !usb_power || usb_host_ready ||
                                elapsed_ms >= LED_WAIT_FOR_USB_HOST_MS;
    return output_decided && link_up;
}

// The breathe when USB power arrives on a running half: at once, ahead of
// the connection blink a computer taking the USB connection brings -- that
// one waits in the queue behind it.
struct led_power_breathe_trigger {
    bool power;
};

struct led_power_breathe_action {
    bool start_breathing;
    bool stop_breathing;
};

static inline void led_power_breathe_trigger_init(struct led_power_breathe_trigger *trigger,
                                                  bool power) {
    *trigger = (struct led_power_breathe_trigger){.power = power};
}

// Call on every change of the power.
static inline struct led_power_breathe_action
led_power_breathe_trigger_update(struct led_power_breathe_trigger *trigger, bool power) {
    const struct led_power_breathe_action action = {
        .start_breathing = power && !trigger->power,
        .stop_breathing = !power && trigger->power,
    };

    trigger->power = power;
    return action;
}

// A peripheral is sent the layer colour each time it connects to the
// central; that one only catches it up and interrupts nothing. Each one
// after it that differs is a layer change, and does.
static inline bool led_pushed_colour_interrupts(bool *colour_received_since_link_up,
                                                bool colour_differs) {
    const bool interrupts = *colour_received_since_link_up && colour_differs;
    *colour_received_since_link_up = true;
    return interrupts;
}
