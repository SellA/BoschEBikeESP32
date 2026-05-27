# BoschEBike ESP32 Multi-Mode Bridge

ESP32 firmware that connects to a Bosch smart system eBike through the Bosch Live Data Interface (LDI) and republishes the bike data to watches, head units, and fitness apps.

The `feature/multi-mode` branch extends the original transparent Suunto bridge with runtime-selectable BLE output modes:

```text
[ Bosch eBike ] -- BLE LDI --> [ ESP32 ] -- selected BLE mode --> [ Watch / head unit / app ]
                    client                 server
```

## What is new in this branch

Compared with `master`, this branch adds:

- Runtime-selectable firmware modes stored in NVS, configurable from the web UI.
- Standard BLE Cycling Power Service (CPS) mode.
- Standard BLE Cycling Speed and Cadence Service (CSC) mode.
- Combined Power + Cadence mode using one CPS service with extended wheel and crank revolution data.
- Native Suunto-compatible cadence encoding in Power + Cadence mode: Bosch cadence rpm is represented through standard CPS crank revolution deltas.
- GAP Appearance advertising for power and speed/cadence sensors, so fitness clients classify the bridge correctly.
- Cycling Power Control Point (0x2A66), including crank length support and cumulative wheel revolution reset.
- SC Control Point (0x2A55) for CSC compliance.
- Runtime simulation mode toggled from the web UI and persisted in NVS.
- Configurable BLE base device name, persisted in NVS.
- BLE debug ring buffer available from the web UI at `/log`.
- Bridge LiPo battery voltage/percentage monitoring on IO35.
- PlatformIO environment for `lolin32_lite` in addition to `esp32dev`.
- PlatformIO environment for `esp32-s3-zero`, with native USB CDC enabled.

## Operating modes

Select the mode from the web dashboard at `http://192.168.4.1`. The setting is stored in NVS and survives reboot.

| Mode | Name | BLE service exposed to the client | Typical use |
|---|---|---|---|
| 1 | Suunto Bridge | Bosch LDI service (custom UUID) | Original transparent bridge for the companion SuuntoPlus app. |
| 2 | Power Sensor | Cycling Power Service, CPS `0x1818` | Standard power-only sensor for watches, bike computers, and apps. |
| 3 | Speed & Cadence | Cycling Speed and Cadence Service, CSC `0x1816` | Standard speed/cadence sensor. |
| 4 | Power + Cadence | Cycling Power Service, CPS `0x1818` with power, wheel, and crank data | Recommended standard mode when the client should receive power, speed, and cadence as one power sensor. |

BLE names are composed from the configured base name plus a mode suffix:

| Mode | Example BLE name |
|---|---|
| 1 | `BoschEBike Bridge` |
| 2 | `BoschEBike Power` |
| 3 | `BoschEBike SpeedCadence` |
| 4 | `BoschEBike PowerCadence` |

If a watch or head unit has already paired with the bridge in another mode, remove the old sensor pairing before pairing again.

## Features

- Bosch LDI client connection to the eBike.
- Web dashboard at `http://192.168.4.1` showing speed, cadence, power, battery, odometer, ambient light, bridge battery, BLE state, and status flags.
- OTA firmware update from `http://192.168.4.1/update`.
- Runtime mode selection from the web UI.
- Runtime simulation toggle from the web UI.
- Configurable BLE base device name from the web UI.
- Optional BLE debug log from `http://192.168.4.1/log`.
- Standard BLE output modes for clients that do not run the dedicated SuuntoPlus app.

## Compatible hardware

The firmware targets ESP32 boards based on ESP32-WROOM-32 or ESP32-WROVER modules.

| Board | PlatformIO environment | Notes |
|---|---|---|
| ESP32 DevKit style boards | `esp32dev` | Generic ESP32 Dev Module target. |
| WEMOS / LOLIN32 Lite | `lolin32_lite` | Includes `upload_port = COM10` and `monitor_port = COM10` in `platformio.ini`. |
| ESP32-S3-Zero | `esp32-s3-zero` | Uses PlatformIO board `esp32-s3-devkitc-1` as the closest generic ESP32-S3 target, with USB CDC on boot enabled. |
| AZDelivery ESP32 DevKit V4 | `esp32dev` | Common ESP32 DevKit variant. |
| DOIT ESP32 DevKit v1 | `esp32dev` | Equivalent DevKit layout. |
| NodeMCU-32S | `esp32dev` | Check USB-UART driver. |
| Lolin D32 / D32 Pro | `esp32dev` | Similar ESP32 target; adjust upload port if needed. |

ESP32-S3 boards are now supported through the `esp32-s3-zero` environment. Battery sensing is disabled by default on that target because the common ESP32-S3-Zero layout does not expose the LOLIN-style VBAT divider used on GPIO35. If you add your own divider, override `BRIDGE_BATTERY_ADC_PIN` in `build_flags`.

## Software requirements

- Visual Studio Code with the PlatformIO extension, or PlatformIO Core CLI.
- USB-UART driver for the board:
  - CP2102: Silicon Labs VCP driver.
  - CH340/CH341: WCH CH341 driver.

## Flashing

Clone the repository and select this branch:

```bash
git clone https://github.com/SellA/BoschEBikeESP32.git
cd BoschEBikeESP32
git checkout feature/multi-mode
```

Build and upload for a generic ESP32 DevKit:

```bash
pio run -e esp32dev -t upload
```

Build and upload for LOLIN32 Lite:

```bash
pio run -e lolin32_lite -t upload
```

Build and upload for ESP32-S3-Zero:

```bash
pio run -e esp32-s3-zero -t upload
```

Open the serial monitor at 115200 baud:

```bash
pio device monitor -e lolin32_lite
```

For ESP32-S3-Zero:

```bash
pio device monitor -e esp32-s3-zero
```

## Web UI

At boot, the ESP32 starts a Wi-Fi access point:

```text
SSID: BoschEBike Bridge
Password: password
URL: http://192.168.4.1
```

The web UI provides:

- Current live Bosch data.
- eBike and client BLE connection state.
- Mode selector for modes 1-4.
- BLE base name editor.
- Simulation enable/disable toggle.
- BLE debug enable/disable toggle.
- Link to the BLE debug log.
- OTA firmware update page.

The Wi-Fi AP is intended for configuration and diagnostics. It automatically shuts down after the boot/configuration window when idle to reduce power usage.

## First-time Bosch pairing

The Bosch smart system only allows registered accessories to connect to the LDI service.

Requirements:

- Bosch smart system with drive unit firmware v19 or newer.
- eBike Flow app installed and connected to the bike.

Pair the ESP32 with the bike:

1. Power on the bike and open the eBike Flow app.
2. Go to eBike settings, Components, Add new device.
3. Select Accessories.
4. Power up the ESP32.
5. Confirm the bridge when it appears in the Flow app.
6. The bridge remains registered and reconnects automatically on later power-ups.

Only one Bosch LDI accessory can be connected at a time. A Kiox display or another Bosch accessory can compete for the same connection slot.

## Connecting clients

### Mode 1: Suunto Bridge

Mode 1 keeps the original behavior from `master`: the ESP32 exposes the Bosch LDI custom service and forwards protobuf payloads without conversion.

Use this mode with the companion SuuntoPlus app:

```text
BoschEBikeSuunto
```

Repository:

```text
https://github.com/SellA/BoschEBikeSuunto
```

### Mode 2: Power Sensor

Mode 2 exposes a standard Cycling Power Service (`0x1818`) with instantaneous power.

Use this when the client should see the bridge as a standard power meter and cadence/speed are not required from this bridge.

### Mode 3: Speed & Cadence

Mode 3 exposes a standard Cycling Speed and Cadence Service (`0x1816`).

The ESP32 converts Bosch speed and cadence into cumulative wheel and crank revolutions as required by the CSC specification. Wheel circumference defaults to 2105 mm and can be overridden in `platformio.ini`:

```ini
build_flags =
    -DWHEEL_CIRCUMFERENCE_MM=2105
```

### Mode 4: Power + Cadence

Mode 4 exposes a single standard Cycling Power Service (`0x1818`) with extended measurement fields:

- instantaneous power,
- cumulative wheel revolutions,
- last wheel event time,
- cumulative crank revolutions,
- last crank event time.

This follows the pattern used by many real cycling power meters. The client receives one power sensor and derives speed/cadence from the standard wheel/crank fields.

Cadence handling in this branch is intentionally value-preserving: Bosch cadence rpm is encoded by emitting one synthetic crank revolution per notification and setting the crank event-time delta to one revolution at the Bosch cadence. This avoids the intermittent zero cadence samples produced by a purely physical fractional-revolution accumulator at low cadence.

## Simulation mode

Simulation is toggled from the web UI and stored in NVS. It no longer requires editing and rebuilding the firmware.

When enabled, the ESP32 skips the real bike connection and generates synthetic data:

- speed: 15-35 km/h,
- cadence: 0-120 rpm,
- power: 0-500 W,
- battery: 75%.

The generated data is emitted through the currently selected mode, so all four BLE output modes can be tested without a bike.

## BLE debug log

Enable BLE debug from the web UI, then open:

```text
http://192.168.4.1/log
```

The log is an in-memory ring buffer with connection, advertising, GATT discovery, CCCD, control point, and rate-limited LDI data entries. It is useful for diagnosing pairing, notification subscription, and Bosch reconnect behavior.

Use:

```text
http://192.168.4.1/clearlog
```

to clear the buffer.

## OTA update

Build the firmware:

```bash
pio run -e lolin32_lite
```

For ESP32-S3-Zero:

```bash
pio run -e esp32-s3-zero
```

Connect to the ESP32 Wi-Fi AP and open:

```text
http://192.168.4.1/update
```

Upload:

```text
.pio/build/lolin32_lite/firmware.bin
```

Use `.pio/build/esp32dev/firmware.bin` if you built the `esp32dev` environment.
Use `.pio/build/esp32-s3-zero/firmware.bin` if you built the `esp32-s3-zero` environment.

## Bosch LDI fields

The ESP32 decodes Bosch LDI protobuf notifications and maps these fields into its internal live data model:

| Field | Type | Value |
|---|---|---|
| 1 | uint | Speed x 100, km/h |
| 2 | uint | Cadence, rpm |
| 5 | uint | Motor power, W |
| 9 | uint | Ambient light x 1000, lux |
| 10 | uint | Battery, percent |
| 12 | uint | Odometer x 1000, km |
| 17 | uint | Light state: 0 off, 1 on, 2 auto |
| 21 | bool | System locked |
| 22 | bool | Charger connected |
| 23 | bool | Light reserve active |
| 24 | bool | Diagnostics active |
| 25 | bool | Bike stationary / not driving |

## HTTP endpoints

| Endpoint | Purpose |
|---|---|
| `/` | Web dashboard. |
| `/data` | Current status and live data as JSON. |
| `/status` | Basic AP/web status JSON. |
| `/setmode?mode=1..4` | Store mode and reboot. |
| `/setname?name=...` | Store BLE base device name and reboot. |
| `/setsim?sim=0|1` | Store simulation setting and reboot. |
| `/setdebug?debug=0|1` | Store BLE debug setting and reboot. |
| `/log` | BLE debug log. |
| `/clearlog` | Clear BLE debug log. |
| `/update` | OTA update page and upload target. |

## Dependencies

- NimBLE-Arduino `^1.4.3`
- Arduino ESP32 framework through PlatformIO
- Preferences
- WebServer
- WiFi
- Update

## Related projects

- `BoschEBikeSuunto`: SuuntoPlus app for Mode 1 LDI bridge operation.
