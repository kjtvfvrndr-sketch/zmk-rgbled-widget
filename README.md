# LED indicators using an RGB LED

> [!IMPORTANT]
> This module uses a versioning scheme that is compatible with ZMK versions.
> As a general rule, the `main` branch is targeting compatibility with ZMK's `main`.
>
> **If you have build failures with ZMK's latest release (like `v0.3`) make sure to [use the corresponding revision](#installation) for `zmk-rgbled-widget` in your `west.yml`**.

This is a [ZMK module](https://zmk.dev/docs/features/modules) containing a simple widget that utilizes a (typically built-in) RGB LED controlled by three separate GPIOs.
It is used to indicate battery level and BLE connection status in a minimalist way.

## Features

<details>
  <summary>Short video demo</summary>
  See below video for a short demo, running through power on, profile switching and power offs.

  https://github.com/caksoylar/zmk-rgbled-widget/assets/7876996/cfd89dd1-ff24-4a33-8563-2fdad2a828d4
</details>

### Boot report

On power-on and on waking from deep sleep, first the connection, once it has settled — the link up, at most 3 s, and on USB power a computer given up to 1 s to take the keys — then the battery level, or with USB power the breathe below in its place.

### Battery status

- Blink the battery level in the boot report as a gradient: green at `CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_HIGH` (80%) and above, yellow halfway, red at `CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_LOW` (10%) and below — the colours are `CONFIG_RGBLED_WIDGET_BATTERY_RGB_HIGH`, `_MEDIUM` and `_LOW`
  - With `CONFIG_RGBLED_WIDGET_BATTERY_GRADIENT=n` the three colours switch at the two levels instead
  - See [options](#battery-levels-for-splits) for showing battery levels for splits
- Pulse 🔴 three short blinks every minute while the battery is at `CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_LOW` or below and not on USB power (`CONFIG_RGBLED_WIDGET_BATTERY_PULSE`, on by default)
- Breathe 🟢 twice, 2 s each, from dark up to `CONFIG_RGBLED_WIDGET_BATTERY_RGB_HIGH` and back, when USB power arrives (`CONFIG_RGBLED_WIDGET_POWER_CONNECTED_BREATHE`, on by default) — it tells of the power, not of charging
  - If the keys move to USB with it, that blink comes first and the breathe after it, in full; a charger breathes after a second
  - A connection blink meanwhile shows at once and the breathe starts over; it stops when the power goes

### Connection status

- Blink 🔵 for connected, cyan for open (waiting to pair) or for a paired profile whose host is not connected, in the boot report and on every change (only on central side for splits)
- Blink ⚪ when the keys go to USB, and the BLE status when they go back (`CONFIG_RGBLED_WIDGET_CONN_SHOW_USB`, on by default): it follows where the keys go, not the power — a charger, or a cable while the keys stay on Bluetooth, changes nothing
- Blink 🔵 for connected, cyan for disconnected on peripheral side of splits
- While the link is down, repeat a short blink in its colour every 5 s (`CONFIG_RGBLED_WIDGET_CONN_REMIND`, on by default), counted from the last connection blink and kept up while idle, until the link is up or the keyboard sleeps

### Layer state

You can pick one of the following methods (off by default) to indicate the highest active layer:

- Enable `CONFIG_RGBLED_WIDGET_SHOW_LAYER_CHANGE` to show the highest active layer on every layer activation
  using a sequence of N cyan color blinks, where N is the zero-based index of the layer, or
- Enable `CONFIG_RGBLED_WIDGET_SHOW_LAYER_COLORS` to assign each layer its own color, which will remain on while that layer is the highest active layer
  - On a split central the colour is also shown on the peripherals (`CONFIG_RGBLED_WIDGET_LAYER_PUSH`, on by default with layer colours; `=n` turns it off)

The layer is resolved on the central part of a split keyboard, since peripheral parts aren't aware of the layer information; layer colours are mirrored to the peripherals as above, the blink sequence is not.

A new layer colour ends whatever the LED is showing and what waits to be shown — blinks, the battery, the breathe, the boot report — and shows at once: the keyboard is in use. The same colour again ends nothing, nor does the colour a peripheral is sent each time it connects.

> [!TIP]
> Also see [below](#showing-status-on-demand) for keymap behaviors you can use to show the battery and connection status on demand.

## Installation

To use, first add this module to your `config/west.yml` by adding a new entry to `projects`:

```yaml west.yml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: v0.3           # Your ZMK version
      import: app/west.yml
    - name: zmk-rgbled-widget  # <-- new entry
      url: https://github.com/caksoylar/zmk-rgbled-widget
      revision: v0.3           # MUST match your ZMK version!
  self:
    path: config
```

For more information, including instructions for building locally, check out the ZMK docs on [building with modules](https://zmk.dev/docs/features/modules#building-with-modules).

Then, if you are using one of the boards supported by the [`rgbled_adapter`](boards/shields/rgbled_adapter) shield such as Xiao BLE,
just add the `rgbled_adapter` as an additional shield to your build, e.g. in `build.yaml`:

```yaml build.yaml
---
include:
  - board: xiao_ble//zmk
    shield: hummingbird rgbled_adapter
```

For other keyboards, see the ["Adding support" section](#adding-support-in-custom-boardsshields) below.

## Showing status on demand

This module also defines keymap [behaviors](https://zmk.dev/docs/keymaps/behaviors) to let you show battery or connection status on demand:

```dts
#include <behaviors/rgbled_widget.dtsi>  // needed to use the behaviors

/ {
    keymap {
        ...
        some_layer {
            bindings = <
                ...
                &ind_bat  // indicate battery level
                &ind_con  // indicate connectivity status
                ...
            >;
        };
    };
};
```

When you invoke the behavior by pressing the corresponding key (or combo), it will trigger the corresponding indicator on the LED.
This will happen on all keyboard parts for split keyboards, so make sure to flash firmware to all parts after enabling.

> [!NOTE]
> The behaviors can be used even when you use split keyboards with different controllers that don't all support the widget.
> Make sure that you use the `rgbled_adapter` shield (or enable `CONFIG_RGBLED_WIDGET` if not using the adapter) _only_ for the keyboard parts that support it.

## Battery levels for splits

For split keyboards, each part will indicate its own battery level with a single battery blink, by default.
However, for some scenarios like keyboards with dongles and no RGB LED on the peripherals, you might want the central part to show the battery levels of peripherals too.
This can be done by enabling one of the below settings:

- `CONFIG_RGBLED_WIDGET_BATTERY_SHOW_PERIPHERALS`: Blink for battery level of self and then the peripherals, in order
- `CONFIG_RGBLED_WIDGET_BATTERY_SHOW_ONLY_PERIPHERALS`: Blink for battery level of only the peripherals, in order

These two settings only apply to split central parts.
The order of blinks for peripherals is determined by the initial pairing order for the split parts.
If a part is currently disconnected, a magenta/purple ([configurable](#configuration-details)) blink will be displayed.

## Configuration details

<details>
<summary>General</summary>

| Name                               | Description                                    | Default |
| ---------------------------------- | ---------------------------------------------- | ------- |
| `CONFIG_RGBLED_WIDGET_INTERVAL_MS` | Minimum wait duration between two blinks in ms | 500     |

</details>

<details>
<summary>Battery-related</summary>

| Name                                          | Description                                                           | Default       |
| --------------------------------------------- | --------------------------------------------------------------------- | ------------- |
| `CONFIG_RGBLED_WIDGET_BATTERY_BLINK_MS`       | Duration of battery level blink in ms                                 | 2000          |
| `CONFIG_RGBLED_WIDGET_BATTERY_GRADIENT`       | Blend the three colours instead of switching at the levels            | `y`           |
| `CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_HIGH`     | Fully the high colour at and above this percentage                    | 80            |
| `CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_LOW`      | Fully the low colour at and below this percentage                     | 10            |
| `CONFIG_RGBLED_WIDGET_BATTERY_RGB_HIGH`       | High battery colour                                                   | `0x00FF00`    |
| `CONFIG_RGBLED_WIDGET_BATTERY_RGB_MEDIUM`     | Medium battery colour (the gradient's midpoint)                       | `0xFFFF00`    |
| `CONFIG_RGBLED_WIDGET_BATTERY_RGB_LOW`        | Low battery colour                                                    | `0xFF0000`    |
| `CONFIG_RGBLED_WIDGET_BATTERY_RGB_MISSING`    | Colour for battery not detected, or peripheral disconnected           | `0xFF00FF`    |
| `CONFIG_RGBLED_WIDGET_BATTERY_PULSE`          | Pulse while the battery is low                                        | `y`           |
| `CONFIG_RGBLED_WIDGET_BATTERY_PULSE_LEVEL`    | Pulse at and below this percentage                                    | `LEVEL_LOW`   |
| `CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_CRITICAL` | One blink on every reported level at or below this (0 = off)          | 0             |
| `CONFIG_RGBLED_WIDGET_BATTERY_RGB_CRITICAL`   | Colour of that blink                                                  | `0xFF0000`    |
| `CONFIG_RGBLED_WIDGET_POWER_CONNECTED_BREATHE` | Breathe in `BATTERY_RGB_HIGH` when USB power arrives                 | `y`           |
| `CONFIG_RGBLED_WIDGET_POWER_CONNECTED_BREATHE_COUNT` | Breaths                                                         | 2             |
| `CONFIG_RGBLED_WIDGET_POWER_CONNECTED_BREATHE_CYCLE_MS` | One breath, dark to full and back, in ms                     | 2000          |

Only one of the options below can be enabled.
The non-default ones (second and third below) only work on central parts of splits.

| Name                                                 | Description                                             | Default |
| ---------------------------------------------------- | ------------------------------------------------------- | ------- |
| `CONFIG_RGBLED_WIDGET_BATTERY_SHOW_SELF`             | Indicate battery level from self only                   | `y`     |
| `CONFIG_RGBLED_WIDGET_BATTERY_SHOW_PERIPHERALS`      | On a split central, also show peripheral battery levels | `n`     |
| `CONFIG_RGBLED_WIDGET_BATTERY_SHOW_ONLY_PERIPHERALS` | On a split central, show only peripheral battery levels | `n`     |

</details>

<details>
<summary>Connectivity-related</summary>

| Name                                           | Description                                                 | Default      |
| ---------------------------------------------- | ----------------------------------------------------------- | ------------ |
| `CONFIG_RGBLED_WIDGET_CONN_BLINK_MS`           | Duration of a connection event blink in ms                  | 1000         |
| `CONFIG_RGBLED_WIDGET_CONN_REMIND`             | Repeat a short blink while the link is down                 | `y`          |
| `CONFIG_RGBLED_WIDGET_CONN_REMIND_PERIOD_S`    | Seconds between reminder blinks                             | 5            |
| `CONFIG_RGBLED_WIDGET_CONN_REMIND_BLINK_MS`    | Duration of a reminder blink in ms                          | 300          |
| `CONFIG_RGBLED_WIDGET_CONN_SHOW_USB`           | Show USB indicator instead of BLE status if it has priority | `y`          |
| `CONFIG_RGBLED_WIDGET_CONN_RGB_CONNECTED`      | Colour for a connected profile                              | `0x0000FF`   |
| `CONFIG_RGBLED_WIDGET_CONN_RGB_ADVERTISING`    | Colour for a profile open for pairing                       | `0x00FFFF`   |
| `CONFIG_RGBLED_WIDGET_CONN_RGB_DISCONNECTED`   | Colour for a paired profile without its host                | `0x00FFFF`   |
| `CONFIG_RGBLED_WIDGET_CONN_RGB_USB`            | Colour for USB endpoint active                              | `0xFFFFFF`   |

</details>

<details>
<summary>Layers-related</summary>

Layers are resolved on non-splits and central parts of splits; layer colours are mirrored to the peripherals.

Below settings enable and configure the sequence-based layer indicator.

| Name                                     | Description                                                                  | Default    |
| ---------------------------------------- | ---------------------------------------------------------------------------- | ---------- |
| `CONFIG_RGBLED_WIDGET_SHOW_LAYER_CHANGE` | Indicate highest active layer on each layer change with a sequence of blinks | `n`        |
| `CONFIG_RGBLED_WIDGET_LAYER_BLINK_MS`    | Blink and wait duration for layer indicator                                  | 100        |
| `CONFIG_RGBLED_WIDGET_LAYER_COLOR`       | Color to use for layer indicator                                             | Cyan (`6`) |
| `CONFIG_RGBLED_WIDGET_LAYER_DEBOUNCE_MS` | Wait duration after a layer change before showing the highest active layer   | 100        |

Below settings enable and configure the color-based layer indicator.

| Name                                     | Description                                                                | Default       |
| ---------------------------------------- | -------------------------------------------------------------------------- | ------------- |
| `CONFIG_RGBLED_WIDGET_SHOW_LAYER_COLORS` | Indicate highest active layer with a constant configurable color per layer | `n`           |
| `CONFIG_RGBLED_WIDGET_LAYER_PUSH`        | On a split central, show the layer colour on the peripherals too            | `y`           |
| `CONFIG_RGBLED_WIDGET_LAYER_PUSH_STACK_SIZE` | Stack of the thread that sends it over Bluetooth                         | 1792, 1280 without logging |
| `CONFIG_RGBLED_WIDGET_LAYER_PUSH_THREAD_PRIORITY` | Its priority                                                        | 10            |
| `CONFIG_RGBLED_WIDGET_LAYER_0_COLOR`     | Color to use for the base layer                                            | Black (`0`)   |
| `CONFIG_RGBLED_WIDGET_LAYER_1_COLOR`     | Color to use for layer 1                                                   | Red (`1`)     |
| `CONFIG_RGBLED_WIDGET_LAYER_2_COLOR`     | Color to use for layer 2                                                   | Green (`2`)   |
| `CONFIG_RGBLED_WIDGET_LAYER_3_COLOR`     | Color to use for layer 3                                                   | Yellow (`3`)  |
| `CONFIG_RGBLED_WIDGET_LAYER_4_COLOR`     | Color to use for layer 4                                                   | Blue (`4`)    |
| `CONFIG_RGBLED_WIDGET_LAYER_5_COLOR`     | Color to use for layer 5                                                   | Magenta (`5`) |
| `CONFIG_RGBLED_WIDGET_LAYER_6_COLOR`     | Color to use for layer 6                                                   | Cyan (`6`)    |
| `CONFIG_RGBLED_WIDGET_LAYER_7_COLOR`     | Color to use for layer 7                                                   | White (`7`)   |
| `CONFIG_RGBLED_WIDGET_LAYER_xx_COLOR`    | Color to use for layer xx (change xx to the layer number to change)        | Black (`0`)   |

</details>

<details>
<summary>Mapping for color values</summary>
Color settings use the following integer values:

| Color        | Value |
| ------------ | ----- |
| Black (none) | `0`   |
| Red          | `1`   |
| Green        | `2`   |
| Yellow       | `3`   |
| Blue         | `4`   |
| Magenta      | `5`   |
| Cyan         | `6`   |
| White        | `7`   |

</details>

You can add these settings to your conf file to modify the config values. You can use a `config/rgbled_adapter.conf` file in your ZMK config repo if you use the adapter shield as mentioned above, or `config/<keyboard>.conf`. E.g. in `config/rgbled_adapter.conf`:

```ini
CONFIG_RGBLED_WIDGET_INTERVAL_MS=250
CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_HIGH=60
CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_LOW=15
```

## Adding support in custom boards/shields

To be able to use this widget, you need three LEDs controlled by GPIOs (_not_ smart LEDs), ideally red, green and blue colors.
Once you have these LED definitions in your board/shield, simply set the appropriate `aliases` to the RGB LED node labels.

As an example, here is a definition for three LEDs connected to VCC and separate GPIOs for a nRF52840 controller:

```dts
/ {
    aliases {
        led-red = &led0;
        led-green = &led1;
        led-blue = &led2;
    };

    leds {
        compatible = "gpio-leds";
        status = "okay";
        led0: led_0 {
            gpios = <&gpio0 26 GPIO_ACTIVE_LOW>; // red LED, connected to P0.26
        };
        led1: led_1 {
            gpios = <&gpio0 30 GPIO_ACTIVE_LOW>; // green LED, connected to P0.30
        };
        led2: led_2 {
            gpios = <&gpio0 6 GPIO_ACTIVE_LOW>;  // blue LED, connected to P0.06
        };
    };
};
```

(If the LEDs are wired between GPIO and GND instead, use `GPIO_ACTIVE_HIGH` flag.)

Finally, turn on the widget in the configuration:

```ini
CONFIG_RGBLED_WIDGET=y
```
