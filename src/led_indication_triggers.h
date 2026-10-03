/*
 * When the boot report and the breathe on USB power are shown -- the
 * decisions, kept free of Zephyr so the host tests can drive them. widget.c
 * feeds them the state of the links and the power.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

// How long to wait for a computer to take the USB connection once power is
// there: it does in a few hundred milliseconds (the host's 100 ms attach
// debounce, the reset, the descriptors), more through a hub. A charger never
// does, and its breathe starts this long after the power.
#define LED_WAIT_FOR_USB_HOST_MS 1000

// Once the computer has the USB connection, how long to leave ZMK to switch
// the output and the connection blink to queue (its debounce is 16 ms)
// before the breathe goes in after it.
#define LED_SETTLE_AFTER_USB_HOST_MS 50

// The boot report waits for the link this long at most, then shows what
// there is.
#define LED_BOOT_WAIT_FOR_LINK_MS 3000

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

// The breathe when USB power arrives on a running half.
struct led_power_breathe_trigger {
    bool power;
    bool pending;
    int64_t power_came_at;
    int64_t usb_host_ready_at; // -1: not yet
    uint32_t layer_interrupts_when_power_came;
};

struct led_power_breathe_action {
    bool start_breathing;
    bool stop_breathing;
    int64_t check_again_at; // -1: only on the next change
};

static inline void led_power_breathe_trigger_init(struct led_power_breathe_trigger *trigger,
                                                  bool power) {
    *trigger = (struct led_power_breathe_trigger){.power = power, .usb_host_ready_at = -1};
}

// Call on every change of the power or the USB connection, and at
// check_again_at. usb_output_possible: the half can send keys over USB,
// so a computer is waited for. layer_interrupts counts the layer colours
// that interrupted the indications: one since the power came cancels the
// breathe -- the keyboard is in use.
static inline struct led_power_breathe_action
led_power_breathe_trigger_update(struct led_power_breathe_trigger *trigger, int64_t now,
                                 bool power, bool usb_output_possible, bool usb_host_ready,
                                 uint32_t layer_interrupts) {
    struct led_power_breathe_action action = {.check_again_at = -1};

    if (power && !trigger->power) {
        trigger->pending = true;
        trigger->power_came_at = now;
        trigger->usb_host_ready_at = -1;
        trigger->layer_interrupts_when_power_came = layer_interrupts;
    } else if (!power && trigger->power) {
        trigger->pending = false;
        action.stop_breathing = true;
    }
    trigger->power = power;

    if (!trigger->pending) {
        return action;
    }
    if (layer_interrupts != trigger->layer_interrupts_when_power_came) {
        trigger->pending = false;
        return action;
    }
    if (usb_host_ready && trigger->usb_host_ready_at < 0) {
        trigger->usb_host_ready_at = now;
    }

    int64_t due = trigger->power_came_at;
    if (usb_output_possible) {
        due += LED_WAIT_FOR_USB_HOST_MS;
        if (trigger->usb_host_ready_at >= 0 &&
            trigger->usb_host_ready_at + LED_SETTLE_AFTER_USB_HOST_MS < due) {
            due = trigger->usb_host_ready_at + LED_SETTLE_AFTER_USB_HOST_MS;
        }
    }
    if (now >= due) {
        trigger->pending = false;
        action.start_breathing = true;
    } else {
        action.check_again_at = due;
    }
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
