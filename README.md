# BoschEBike ESP32 Bridge

Transparent BLE bridge between a **Bosch eBike** and a **Suunto** watch. The ESP32 sits in the middle as a proxy: the bike sees it as a Suunto accessory, the watch sees it as the bike. Data is forwarded as raw protobuf with no conversion.

```
[ Bosch eBike ] ──BLE LDI──► [ ESP32 ] ──BLE LDI──► [ Suunto ]
                  (client)                (server)
```

## Features

- Transparent BLE proxy using the same LDI protocol as the bike
- Live web dashboard at `http://192.168.4.1` (speed, cadence, power, battery, odometer, ambient light, status flags)
- OTA firmware update via browser at `http://192.168.4.1/update`
- Simulation mode (`SIM_ENABLED = true`) for development without a real bike

## Compatible hardware

The firmware targets `esp32dev` (PlatformIO), compatible with any board based on the **ESP32-WROOM-32** or **ESP32-WROVER** module.

| Board | Notes |
|---|---|
| **AZDelivery ESP32 DevKit V4** | Most common in Europe, tested |
| **DOIT ESP32 DevKit v1** | Equivalent, widely available |
| **NodeMCU-32S** | Compatible, same pinout |
| **Lolin D32 / D32 Pro** | Compatible, more compact form factor |
| **ESP32-WROVER-IE / DevKitC** | Compatible (PSRAM not used) |

> Boards based on **ESP32-C3**, **ESP32-S2**, or **ESP32-S3** are not compatible without code changes (different NimBLE API on those variants).

## Software requirements

- [Visual Studio Code](https://code.visualstudio.com/) + [PlatformIO extension](https://platformio.org/install/ide?install=vscode)
- or [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation/index.html)

## Flashing the ESP32

### 1. Clone the repository

```bash
git clone https://github.com/SellA/BoschEBikeESP32.git
cd BoschEBikeESP32
```

### 2. Open in VS Code with PlatformIO

Open the folder in VS Code. PlatformIO will detect the project and download dependencies (NimBLE-Arduino) automatically.

### 3. Connect the ESP32 via USB

Make sure the USB-UART driver is installed:
- **CP2102** (AZDelivery, DOIT): [Silicon Labs driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
- **CH340** (NodeMCU-32S, cheap clones): [CH340 driver](https://www.wch-ic.com/downloads/CH341SER_EXE.html)

### 4. Upload the firmware

Click the **Upload** button (→) in the PlatformIO status bar, or from the terminal:

```bash
pio run --target upload
```

### 5. Verify

Open the serial monitor (115200 baud), or connect to the Wi-Fi network `BoschEBike Bridge` (password: `password`) and open `http://192.168.4.1`.

## OTA update (wireless)

1. Build the project in PlatformIO → produces `.pio/build/esp32dev/firmware.bin`
2. Connect to the `BoschEBike Bridge` Wi-Fi network
3. Go to `http://192.168.4.1/update`
4. Select the `firmware.bin` file and click **Upload firmware**

The ESP32 reboots automatically after a successful update.

## First-time pairing

### Requirements

- Bosch smart system with **drive unit firmware v19 or newer** (check in Flow app: Bike → Settings → System Information → Drive Unit Software Version)
- **eBike Flow app** installed on your phone and connected to the bike

### Step 1 — Register the bridge in the Bosch Flow app

The bike only allows LDI connections from devices registered as accessories.

1. Power on the bike and open the **eBike Flow app**
2. Go to **eBike settings (top right) → Components → Add new device**
3. Select **Accessories**
4. Power up the ESP32 — it starts advertising as an LDI accessory
5. The app will detect the bridge; tap to confirm the pairing
6. The bridge now appears under **Components** in the Flow app

> This step is required only once. The bike will remember the bridge and reconnect automatically on subsequent power-ups.

### Step 2 — Connect the Suunto watch

Once the bike is paired with the ESP32:

1. Power on the bike and the ESP32 — the bike connects to the bridge automatically
2. The ESP32 switches to advertising as `BoschEBike` (LDI server mode)
3. On the Suunto: start a workout, swipe to the SuuntoPlus screen and select **Bosch eBike** — the watch connects automatically via BLE

See [BoschEBikeSuunto](https://github.com/SellA/BoschEBikeSuunto) for Suunto app installation instructions.

> **Note:** only one LDI accessory can be connected at a time. If a Kiox 500 display or a Bosch GPS device is also paired, it will compete for the same connection slot.

## Simulation mode

To develop the Suunto app without a physical bike, set `SIM_ENABLED = true` in [src/main.cpp](src/main.cpp#L33):

```cpp
static const bool SIM_ENABLED = true;
```

The ESP32 immediately advertises as `BoschEBike` and generates synthetic data (speed 15–35 km/h, cadence 70–95 rpm, power 120–280 W) in a loop.

## LDI protocol (Bosch)

Data is sent as binary protobuf over BLE notifications:

| Field | Type | Value |
|---|---|---|
| 1 | uint | Speed × 100 (km/h) |
| 2 | uint | Cadence (rpm) |
| 5 | uint | Power (W) |
| 9 | uint | Ambient light × 1000 (lux) |
| 10 | uint | Battery (%) |
| 12 | uint | Odometer × 1000 (km) |
| 17 | uint | Light state (0=off, 1=on, 2=auto) |
| 21 | bool | System locked |
| 22 | bool | Charger connected |
| 23 | bool | Light reserve |
| 24 | bool | Diagnostics active |
| 25 | bool | Bike stationary |

## Dependencies

- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) `^1.4.3`

## Related

- [BoschEBikeSuunto](https://github.com/SellA/BoschEBikeSuunto) — the Suunto watch app that displays live data from this bridge
