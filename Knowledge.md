# BoschEBike Bridge — Technical Knowledge

Reference document for future modifications. Captures protocol quirks, debugging
findings and non-obvious design decisions so you don't have to rediscover them.

---

## Bosch LDI Protocol

### UUIDs
| Role | UUID |
|------|------|
| Service | `0000eb20-eaa2-11e9-81b4-2a2ae2dbcce4` |
| Characteristic (notify) | `0000eb21-eaa2-11e9-81b4-2a2ae2dbcce4` |

The bike advertises a **solicitation UUID** (AD type `0x15`) and only connects
to peripherals that advertise the same UUID back. The ESP32 must therefore
advertise this UUID in phase 1 to attract the Bosch system unit.

### Protobuf payload (LDI Live Data)
All fields are varint (wire type 0). Fixed-point encoding:

| Field | Value |
|-------|-------|
| 1 | speed × 100 → divide by 100 for km/h |
| 2 | cadence in RPM |
| 5 | motor power in W |
| 9 | ambient light × 1000 → divide by 1000 for lux |
| 10 | battery SoC in % |
| 12 | odometer × 1000 → divide by 1000 for km |
| 17 | bike light: 0=off, 1=on dim, 2=on full |
| 21 | system locked (bool) |
| 22 | charger connected (bool) |
| 23 | light reserve (bool) |
| 24 | diagnostics active (bool) |
| 25 | not driving (bool) |

Custom bridge-only fields appended before forwarding to Suunto:
| Field | Value |
|-------|-------|
| 100 | no-ebike flag (1 = bike not connected) |
| 101 | bridge battery % |
| 102 | bridge battery mV |

---

## BLE Security (Bonding)

The Bosch eBike **requires SC + Just Works bonding** (`BLE_SM_PAIR_AUTHREQ_SC | BLE_SM_PAIR_AUTHREQ_BOND`) to complete the LDI encryption handshake. Without it the bike will not send notify data.

**Old false belief**: bonding was thought to cause Suunto to disconnect.
**Actual root causes** (both fixed):
1. NimBLE stores 128-bit-string UUIDs as 128-bit in the GATT attribute table; strict clients (Suunto, Garmin) reject these even for standard services — always use `NimBLEUUID((uint16_t)0x1818)` style constructors for standard 16-bit UUIDs.
2. Missing SC/CP Control Point characteristics (CPCP for CPS, SCCP for CSC) — strict clients disconnect immediately when these are absent.

---

## UUID Format Gotcha (NimBLE)

When creating GATT services and characteristics with NimBLE, **never pass a UUID string** for standard BT SIG services/characteristics:
```cpp
// BAD — stored as 128-bit, rejected by strict clients
pServer->createService("00001818-0000-1000-8000-00805f9b34fb");

// GOOD — stored as 16-bit as expected
pServer->createService(NimBLEUUID((uint16_t)0x1818));
```
This applies to ALL standard UUIDs (0x1816, 0x1818, 0x2A5B, 0x2A63, 0x2A65, 0x2A66, etc.).
The custom Bosch LDI UUID is 128-bit and must use the string form.

---

## GATT Discovery Flow (bike side)

Connection → `openGattClient()` →
1. Negotiate MTU (247) + data length (251 bytes)
2. `ble_gattc_exchange_mtu` → callback `gattMtuCB`
3. Discover LDI service by UUID → `gattSvcCB`
4. Discover LDI characteristic → `gattChrCB`
5. Discover CCCD descriptor → `gattDscCB`
6. Write CCCD with `0x0001` (notify ON) → `gattWriteCccdCB`
7. Done — LDI notifications start flowing

If any step fails, retry after 3 s. If LDI service not found at all, full reset
and re-advertise for eBike.

**Important**: after CCCD write succeeds, `cscLastUpdateMs` is reset to 0 so
the first real LDI packet does not accumulate fake distance for the 2–5 s
GATT discovery gap.

---

## Advertising Phases

**Phase 1 — attract eBike:**
- AD type `0x15` (128-bit Service Solicitation UUID) = Bosch LDI service UUID
- Interval: 50–160 ms (0x0050–0x00A0)
- The bike looks for this and connects as GATT server (ESP32 is GATT client)

**Phase 2 — attract fitness client:**
Started either when GATT is ready (ebikeGattReady) or immediately in sim mode.

| Mode | AD type | Value |
|------|---------|-------|
| Suunto Bridge | `0x07` 128-bit UUID | LDI service UUID |
| Power Sensor | `0x03` 16-bit UUID | `0x1818` |
| Speed & Cadence | `0x03` 16-bit UUID | `0x1816` |
| Power + Cadence | `0x03` 16-bit UUID | `0x1818` |

For sensor modes (not Suunto), also include AD type `0x19` (Appearance):
- `0x0484` = Cycling: Power Sensor
- `0x0483` = Cycling: Speed and Cadence Sensor

This is required for Suunto, Garmin and Wahoo to categorize the sensor correctly
and offer it under the right sport activity.

---

## CPS (Cycling Power Service, 0x1818) Compliance

Mandatory characteristics:
- `0x2A63` Power Measurement — NOTIFY | READ
- `0x2A65` CPS Feature — READ, value = 32-bit flags
- `0x2A5D` Sensor Location — READ, value `0x00` (Other)
- `0x2A66` Cycling Power Control Point — WRITE | INDICATE

**CPS Feature flags used:**
- Bit 14 (`0x00004000`): Crank Length Adjustment Supported → makes CPCP mandatory
  and causes fitness clients to ask for crank length during pairing, correctly
  classifying this as a power POD rather than a generic sensor.
- Bits 4+5 (`0x00000030`): Wheel/Crank Revolution Data Supported (MODE_POWER_CADENCE only)

**CPCP opcodes implemented:**
| Opcode | Function |
|--------|----------|
| `0x01` | Set Cumulative Wheel Revolutions (MODE_POWER_CADENCE) |
| `0x04` | Set Crank Length (stored, no effect on power output) |
| `0x05` | Request Crank Length |
| others | "Op Code Not Supported" (0x02) |

Response format: `[0x20, opcode, result_code, ...optional_data]`, sent as INDICATE.

---

## CSC (Cycling Speed & Cadence, 0x1816) Compliance

Mandatory characteristics:
- `0x2A5B` CSC Measurement — NOTIFY | READ
- `0x2A5C` CSC Feature — READ, value = `0x0003` (Wheel + Crank Revolution Data)
- `0x2A5D` Sensor Location — READ, value `0x00` (Other)
- `0x2A55` SC Control Point — WRITE | INDICATE

**CSC Feature bit 0** (Wheel Revolution Data) makes SCCP mandatory. Without SCCP
strict clients (Suunto, Garmin) disconnect immediately.

**SCCP opcodes implemented:**
| Opcode | Function |
|--------|----------|
| `0x01` | Set Cumulative Value (reset wheel rev counter) |
| others | "Not Supported" (0x02) |

Response: `[0x10, opcode, result]`, sent as INDICATE.

---

## Measurement Packet Formats

### CPS — Power only (MODE_POWER_SENSOR), 4 bytes
```
flags(2 LE, 0x0000) | power_w(2 LE signed)
```

### CPS — Power + Wheel + Crank (MODE_POWER_CADENCE), 14 bytes
```
flags(2 LE, 0x0030) | power_w(2 LE) | cum_wheel(4 LE) |
wheel_evt(2 LE, 1/2048 s) | cum_crank(2 LE) | crank_evt(2 LE, 1/1024 s)
```
Cadence is not a direct field — the client derives it from successive
crank revolution deltas. Each notification increments `cscCrankRevTotal` by 1
and sets the event timestamp to exactly one revolution at the Bosch RPM.

### CSC — Speed + Cadence (MODE_SPEED_CADENCE), 11 bytes
```
flags(1, 0x03) | cum_wheel(4 LE) | wheel_evt(2 LE, 1/1024 s) |
cum_crank(2 LE) | crank_evt(2 LE, 1/1024 s)
```

**Timestamp resolution difference:**
- CSC: 1/1024 s per tick
- CPS wheel event: 1/2048 s per tick (different spec!)
- CPS crank event: 1/1024 s per tick (same as CSC)

---

## Fractional Revolution Accumulator

Speed and cadence from the bike arrive at irregular intervals (typically 1–2 s).
The BLE spec expects cumulative revolution counts, not instantaneous values.

To avoid jerky counters a fractional accumulator is used:
```
cscWheelRevFrac  += revPerSec * dtSec
newRevs           = floor(cscWheelRevFrac)
cscWheelRevFrac  -= newRevs
cscWheelRevTotal += newRevs
```
The **totals** are preserved across bike disconnections so the client sees a
smooth continuation. The **fractional parts** are reset on disconnect to avoid
a phantom partial revolution being added when the bike reconnects.

---

## Bridge Battery (hardware)

LOLIN32 Lite exposes a voltage-divided VBAT signal on **GPIO35** (divide by 2).
Reading: average 8 ADC samples, multiply by 2 to get mV, then look up % from
the LiPo discharge curve defined in `lipoPercentFromMv()`.

ESP32-S3-Zero does not wire GPIO35 to anything useful. Set
`-DBRIDGE_BATTERY_ADC_PIN=-1` in build flags for boards without battery sense.
The ADC pin is checked at runtime (`BATTERY_ADC_PIN < 0`) and the feature is
silently disabled.

Sampling interval: 5 s. Resolution: 12-bit, attenuation ADC_11db.

---

## LED (Startup Blink)

The startup LED blinks for 2 s after boot on `LED_BUILTIN` (fallback: GPIO2).

Some boards have active-low LEDs (LOLIN32 Lite), some active-high (ESP32-S3-Zero).
Control via build flag:
- `-DSTARTUP_LED_ACTIVE_LOW=1` (default)
- `-DSTARTUP_LED_ACTIVE_LOW=0` for active-high boards

---

## NVS Keys (namespace "ebike")

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `mode` | uint8 | 1 | Firmware mode (1–4) |
| `dname` | string | "BoschEBike" | Base device name (max 20 chars) |
| `sim` | uint8 | 0 | Simulation enabled (0/1) |
| `bdbg` | uint8 | 0 | BLE debug log enabled (0/1) |

On first flash (NVS never written) all values fall back to defaults.

---

## Web Server Timeout Logic

Wi-Fi AP is started at boot and shut down automatically to save power:
- **Boot window** (default 60 s): if no client connects within this window, AP turns off.
- **Idle timeout** (default 60 s): after the last client disconnects, AP turns off
  after this delay (gives time to reconnect).

Both timeouts are configurable via build flags:
```
-DWEB_BOOT_WINDOW_MS=60000
-DWEB_IDLE_AFTER_CLIENT_MS=60000
```

The AP station count is polled every loop tick; a non-zero count resets the
idle timer and sets `webHadClient = true` to switch from boot-window to
idle-timeout mode.

---

## Simulation Mode

Generates triangle waves for speed, cadence and power to allow testing without
a real bike:
- Speed: 15–35 km/h, period 20 s
- Cadence: 0–120 RPM, period 15 s
- Power: 0–500 W, period 12 s

When sim is active, advertising for the fitness client starts immediately at
boot without waiting for the bike. The bike-side GATT stack is completely idle.

---

## No-eBike Notifications

When a fitness client is connected but the bike is not, the bridge sends
zero-value data every 1 s (instead of going silent). For Suunto Bridge mode this
includes field 100 (`BRIDGE_NO_EBIKE_FIELD = 1`) so the app can display a
"no bike" status rather than stale values.

---

## GAP Appearance (0x2A01)

Set at boot via `ble_svc_gap_device_appearance_set()` based on mode:
- MODE_POWER_SENSOR / MODE_POWER_CADENCE → `0x0484` (Cycling: Power Sensor)
- MODE_SPEED_CADENCE → `0x0483` (Cycling: Speed and Cadence Sensor)
- MODE_SUUNTO_BRIDGE → not set (custom service, fitness clients don't need it)

Some fitness watches read this characteristic after connecting to decide which
sport activity to assign the sensor to.

---

## Known Working Clients

| Client | Modes tested | Notes |
|--------|-------------|-------|
| Suunto watch | 1 (Suunto Bridge) | Requires LDI UUID in AD, strict GATT compliance |
| Suunto app (Android/iOS) | 2, 3, 4 | Checks for CPCP/SCCP, rejects 128-bit standard UUIDs |
| Garmin | 2, 3 | Similar strictness to Suunto |
| Wahoo | 2, 3 | Less strict, but appearance AD helps categorization |
