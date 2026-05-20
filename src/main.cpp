/*
 * BoschEBike Multi-Mode Bridge
 *
 * ── Overview ──────────────────────────────────────────────────────────────────
 * The ESP32 sits between a Bosch eBike and a BLE client (Suunto watch, Garmin,
 * Wahoo, or any fitness app). The operating mode is selected at runtime via the
 * web UI and stored in NVS so it survives reboots. Flash many boards with the
 * same firmware, then configure each one independently.
 *
 * ── Modes ─────────────────────────────────────────────────────────────────────
 *   1 — Suunto Bridge   transparent LDI proxy → Suunto watch / custom app
 *   2 — Power Sensor    standard Cycling Power Service (CPS, UUID 0x1818)
 *   3 — Speed & Cadence standard Cycling Speed & Cadence (CSC, UUID 0x1816)
 *
 * ── BLE roles ─────────────────────────────────────────────────────────────────
 *   GATT Client  →  toward the bike (Bosch LDI spec, same in all modes)
 *   GATT Server  →  toward the client (mode-specific service/characteristics)
 *
 * ── Advertising flow ──────────────────────────────────────────────────────────
 *   Phase 1: LDI solicitation (AD type 0x15) → attracts the Bosch system unit
 *   Phase 2: mode-specific service UUID      → attracts the BLE client
 *
 * ── Bosch LDI UUIDs ───────────────────────────────────────────────────────────
 *   Service:        0000eb20-eaa2-11e9-81b4-2a2ae2dbcce4
 *   Characteristic: 0000eb21-eaa2-11e9-81b4-2a2ae2dbcce4  (notify)
 *
 * ── Protobuf payload (LDI Live Data) ─────────────────────────────────────────
 *   Field  1: speed × 100 (÷100 → km/h)          Field 17: bike light (0/1/2)
 *   Field  2: cadence (rpm)                        Field 21: system locked
 *   Field  5: motor power (W)                      Field 22: charger connected
 *   Field  9: ambient light × 1000 (÷1000 → lux)  Field 23: light reserve
 *   Field 10: battery SoC (%)                      Field 24: diagnostics
 *   Field 12: odometer × 1000 (÷1000 → km)         Field 25: not driving
 *
 * ── NVS ───────────────────────────────────────────────────────────────────────
 *   Namespace "ebike", key "mode" (uint8): 1 / 2 / 3
 *   Changed via web UI → saved to NVS → reboot to apply
 *
 * ── Web UI / OTA ──────────────────────────────────────────────────────────────
 *   Wi-Fi AP: "BoschEBike Bridge" / "password"
 *   Dashboard:  http://192.168.4.1
 *   OTA update: http://192.168.4.1/update  (upload firmware.bin)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <stdarg.h>
#include "nimble/nimble/host/services/gap/include/services/gap/ble_svc_gap.h"
#include "nimble/nimble/host/include/host/ble_gatt.h"
#include "nimble/nimble/host/include/host/ble_hs_mbuf.h"

// ─── Firmware mode ────────────────────────────────────────────────────────────
enum FirmwareMode : uint8_t {
    MODE_SUUNTO_BRIDGE  = 1,
    MODE_POWER_SENSOR   = 2,
    MODE_SPEED_CADENCE  = 3,
    MODE_POWER_CADENCE  = 4   // CPS (0x1818) + CSC (0x1816) on the same device
};

// Default mode used when NVS has never been written (first flash).
static const FirmwareMode DEFAULT_MODE = MODE_SUUNTO_BRIDGE;

static FirmwareMode gMode = DEFAULT_MODE;

// ─── Device name ──────────────────────────────────────────────────────────────
// The full BLE device name is "<base> Bridge|Power|SpeedCadence".
// gBaseDeviceName is stored in NVS (key "dname"); gDeviceName is composed at
// boot and used everywhere (NimBLEDevice::init, advertising scan response).
static char gBaseDeviceName[21] = "BoschEBike";  // max 20 chars + null
static char gDeviceName[34]     = {};             // base + longest suffix " PowerCadence"

static void buildDeviceName() {
    const char* suffix =
        gMode == MODE_SUUNTO_BRIDGE ? " Bridge"        :
        gMode == MODE_POWER_SENSOR  ? " Power"         :
        gMode == MODE_SPEED_CADENCE ? " SpeedCadence"  : " PowerCadence";
    snprintf(gDeviceName, sizeof(gDeviceName), "%s%s", gBaseDeviceName, suffix);
}

// ─── Simulation ───────────────────────────────────────────────────────────────
// When gSimEnabled is true the ESP32 generates synthetic data and advertises
// immediately for the client, without connecting to a real bike.
// Toggled via web UI (/setsim) and stored in NVS key "sim".
static bool gSimEnabled = false;

// ─── BLE Debug Log ────────────────────────────────────────────────────────────
// Toggled via web UI (/setdebug) and stored in NVS key "bdbg".
// Events are written to a fixed-size ring buffer in RAM and served as plain
// text at GET /log — readable from a browser or via WebFetch at 192.168.4.1.
static bool gBleDebug = false;

static const int DBLOG_ENTRIES = 100;
static const int DBLOG_LINE    = 80;   // 12 chars timestamp + 67 chars message

static char     dblogBuf[DBLOG_ENTRIES][DBLOG_LINE];
static int      dblogHead      = 0;    // next write slot (oldest when full)
static int      dblogCount     = 0;
static uint32_t dblogLastLdiMs = 0;    // rate-limit for LDI data entries

static void dblog(const char* fmt, ...) {
    if (!gBleDebug) return;
    char tmp[DBLOG_LINE];
    int n = snprintf(tmp, sizeof(tmp), "[%9lu] ", (unsigned long)millis());
    va_list ap; va_start(ap, fmt);
    vsnprintf(tmp + n, sizeof(tmp) - n, fmt, ap);
    va_end(ap);
    strlcpy(dblogBuf[dblogHead], tmp, DBLOG_LINE);
    dblogHead = (dblogHead + 1) % DBLOG_ENTRIES;
    if (dblogCount < DBLOG_ENTRIES) dblogCount++;
}

#define WIFI_AP_SSID "BoschEBike Bridge"
#define WIFI_AP_PASS "password"

// Bosch LDI UUIDs (bike client side and Suunto server side)
#define LDI_SVC_UUID  "0000eb20-eaa2-11e9-81b4-2a2ae2dbcce4"
#define LDI_CHAR_UUID "0000eb21-eaa2-11e9-81b4-2a2ae2dbcce4"

// Standard BLE sensor service / characteristic UUIDs
#define CPS_SVC_UUID    "00001818-0000-1000-8000-00805f9b34fb"
#define CPS_MEAS_UUID   "00002a63-0000-1000-8000-00805f9b34fb"
#define CPS_FEAT_UUID   "00002a65-0000-1000-8000-00805f9b34fb"
#define CSC_SVC_UUID    "00001816-0000-1000-8000-00805f9b34fb"
#define CSC_MEAS_UUID   "00002a5b-0000-1000-8000-00805f9b34fb"
#define CSC_FEAT_UUID   "00002a5c-0000-1000-8000-00805f9b34fb"
#define SENSOR_LOC_UUID "00002a5d-0000-1000-8000-00805f9b34fb"

// Wheel circumference for CSC speed calculation (mm).
// Override in platformio.ini build_flags: -DWHEEL_CIRCUMFERENCE_MM=2105
#ifndef WHEEL_CIRCUMFERENCE_MM
#define WHEEL_CIRCUMFERENCE_MM 2105
#endif

// LOLIN D32 battery divider: VBAT → 100 kΩ → IO35 → 100 kΩ → GND
static const int      BATTERY_ADC_PIN            = 35;
static const uint32_t BATTERY_SAMPLE_INTERVAL_MS = 5000;

#ifdef LED_BUILTIN
static const int STARTUP_LED_PIN = LED_BUILTIN;
#else
static const int STARTUP_LED_PIN = 2;
#endif
static const bool     STARTUP_LED_ACTIVE_LOW = true;
static const uint32_t STARTUP_LED_ON_MS      = 2000;
static uint32_t startupLedOffMs  = 0;
static bool     startupLedActive = false;

static void setStartupLed(bool on) {
    digitalWrite(STARTUP_LED_PIN, STARTUP_LED_ACTIVE_LOW ? !on : on);
}
static void startStartupLed() {
    pinMode(STARTUP_LED_PIN, OUTPUT);
    setStartupLed(true);
    startupLedOffMs  = millis() + STARTUP_LED_ON_MS;
    startupLedActive = true;
}
static void updateStartupLed() {
    if (startupLedActive && (int32_t)(millis() - startupLedOffMs) >= 0) {
        setStartupLed(false);
        startupLedActive = false;
    }
}

// ─── Decoded live data ────────────────────────────────────────────────────────
struct LiveData {
    float   speedKmh     = 0.0f;
    int32_t cadenceRpm   = 0;
    int32_t powerW       = 0;
    float   ambientLux   = 0.0f;
    int32_t batterySoc   = 0;
    float   odometerKm   = 0.0f;
    int     bikeLight    = 0;
    bool    systemLocked = false;
    bool    chargerConn  = false;
    bool    lightReserve = false;
    bool    diagActive   = false;
    bool    notDriving   = false;
    bool    valid        = false;
};
static LiveData gData;

// ─── Manual protobuf decoder ──────────────────────────────────────────────────
struct VInt { uint64_t v; int p; };

static VInt readVarint(const uint8_t* d, int pos, int len) {
    uint64_t v = 0; int sh = 0; uint8_t b;
    do {
        if (pos >= len) break;
        b = d[pos++];
        v |= (uint64_t)(b & 0x7f) << sh;
        sh += 7;
    } while (b & 0x80);
    return { v, pos };
}

static void decodeLiveData(const uint8_t* data, size_t len) {
    LiveData ld = gData;
    int pos = 0;
    while (pos < (int)len) {
        auto tag = readVarint(data, pos, len); pos = tag.p;
        int fn = (int)(tag.v >> 3), wt = (int)(tag.v & 7);
        if (wt == 0) {
            auto v = readVarint(data, pos, len); pos = v.p;
            switch (fn) {
                case 1:  ld.speedKmh     = v.v / 100.0f;  break;
                case 2:  ld.cadenceRpm   = (int32_t)v.v;  break;
                case 5:  ld.powerW       = (int32_t)v.v;  break;
                case 9:  ld.ambientLux   = v.v / 1000.0f; break;
                case 10: ld.batterySoc   = (int32_t)v.v;  break;
                case 12: ld.odometerKm   = v.v / 1000.0f; break;
                case 17: ld.bikeLight    = (int)v.v;      break;
                case 21: ld.systemLocked = v.v != 0;      break;
                case 22: ld.chargerConn  = v.v != 0;      break;
                case 23: ld.lightReserve = v.v != 0;      break;
                case 24: ld.diagActive   = v.v != 0;      break;
                case 25: ld.notDriving   = v.v != 0;      break;
            }
        } else if (wt == 2) {
            auto sl = readVarint(data, pos, len); pos = sl.p + (int)sl.v;
        } else break;
    }
    ld.valid = true;
    gData = ld;
}

// ─── Protobuf encoder (bridge mode simulation / no-ebike packet) ──────────────
static int encodeVarint(uint8_t* buf, uint64_t val) {
    int n = 0;
    do {
        buf[n] = (uint8_t)((val & 0x7f) | (val >= 0x80 ? 0x80 : 0x00));
        val >>= 7; n++;
    } while (val);
    return n;
}
static int encodeField(uint8_t* buf, int fieldNum, uint64_t val) {
    int n = encodeVarint(buf, (uint64_t)(fieldNum << 3));
    n += encodeVarint(buf + n, val);
    return n;
}

// Triangle wave: oscillates between lo and hi with the given period
static int32_t triWave(uint32_t ms, uint32_t periodMs, int32_t lo, int32_t hi) {
    uint32_t phase = ms % periodMs;
    int32_t  half  = (int32_t)(periodMs / 2);
    int32_t  range = hi - lo;
    return (int32_t)phase < half
        ? lo + ((int32_t)phase * range) / half
        : hi - (((int32_t)phase - half) * range) / half;
}

static uint32_t nextSimNotifyMs     = 0;
static uint32_t nextNoEbikeNotifyMs = 0;

static const int BRIDGE_NO_EBIKE_FIELD        = 100;
static const int BRIDGE_BATTERY_PERCENT_FIELD = 101;
static const int BRIDGE_BATTERY_MV_FIELD      = 102;

static uint32_t nextBatterySampleMs  = 0;
static uint16_t bridgeBatteryMv      = 0;
static uint8_t  bridgeBatteryPercent = 0;

static uint8_t lipoPercentFromMv(uint16_t mv) {
    struct Point { uint16_t mv; uint8_t pct; };
    static const Point curve[] = {
        {4200,100},{4100,90},{4000,80},{3920,70},{3850,60},
        {3790,50},{3750,40},{3710,30},{3670,20},{3610,10},{3300,0}
    };
    if (mv >= curve[0].mv) return 100;
    for (size_t i = 1; i < sizeof(curve)/sizeof(curve[0]); ++i) {
        if (mv >= curve[i].mv) {
            const Point h = curve[i-1], l = curve[i];
            return l.pct + ((uint32_t)(mv - l.mv) * (h.pct - l.pct)) / (h.mv - l.mv);
        }
    }
    return 0;
}

static void updateBridgeBattery(bool force = false) {
    uint32_t now = millis();
    if (!force && (int32_t)(now - nextBatterySampleMs) < 0) return;
    nextBatterySampleMs = now + BATTERY_SAMPLE_INTERVAL_MS;
    uint32_t accMv = 0;
    for (int i = 0; i < 8; ++i) { accMv += analogReadMilliVolts(BATTERY_ADC_PIN); delay(1); }
    bridgeBatteryMv      = (uint16_t)((accMv / 8) * 2);
    bridgeBatteryPercent = lipoPercentFromMv(bridgeBatteryMv);
}

// ─── CPS state ───────────────────────────────────────────────────────────────
// Crank length in 1/10 mm units (1725 = 172.5 mm, a common default).
// Updated via Cycling Power Control Point opcode 0x04.
// Power data comes from the eBike, so this value is only stored and reported
// back to the client — it has no effect on the power calculation.
static uint16_t cpsCrankLengthTenthMm = 1725;
// Wheel event timestamp for MODE_POWER_CADENCE CPS measurement.
// CPS spec uses 1/2048 s resolution (different from CSC which uses 1/1024 s).
static uint16_t cpsCombWheelEventT = 0;

// ─── CSC accumulator state ────────────────────────────────────────────────────
static float    cscWheelRevFrac    = 0.0f;
static uint32_t cscWheelRevTotal   = 0;
static uint16_t cscLastWheelEventT = 0;
static float    cscCrankRevFrac    = 0.0f;
static uint16_t cscCrankRevTotal   = 0;
static uint16_t cscLastCrankEventT = 0;
static uint32_t cscLastUpdateMs    = 0;

// ─── BLE server characteristics (one per mode, only one non-null at runtime) ──
static NimBLEServer*         pServer  = nullptr;
static NimBLECharacteristic* pLdiChar = nullptr;  // MODE_SUUNTO_BRIDGE
static NimBLECharacteristic* pCpsChar = nullptr;  // MODE_POWER_SENSOR
static NimBLECharacteristic* pCscChar = nullptr;  // MODE_SPEED_CADENCE

// ─── BLE state ────────────────────────────────────────────────────────────────
static uint32_t nextGattRetryMs  = 0;
static uint16_t ebikeConnHandle  = BLE_HS_CONN_HANDLE_NONE;
static uint16_t clientConnHandle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t ldiSvcStart      = 0;
static uint16_t ldiSvcEnd        = 0;
static uint16_t ldiCharHandle    = 0;
static uint16_t ldiCccdHandle    = 0;
static volatile bool gattDiscoveryActive = false;

static volatile bool flagEbikePeripheralConn = false;
static volatile bool flagEbikeDisconn        = false;
static volatile bool flagClientConn          = false;
static volatile bool flagClientDisconn       = false;
static volatile bool ebikeConnected          = false;
static volatile bool ebikeGattReady          = false;
static volatile bool clientConnected         = false;
static volatile bool ebikeEncrypted          = false;
static volatile bool gattStartPending        = false;
static uint16_t      lastMtu                 = 0;

static NimBLEAddress ebikeAddr("");
static const ble_uuid128_t LDI_SVC_UUID_RAW = BLE_UUID128_INIT(
    0xe4,0xcc,0xdb,0xe2,0x2a,0x2a,0xb4,0x81,
    0xe9,0x11,0xa2,0xea,0x20,0xeb,0x00,0x00
);
static const ble_uuid128_t LDI_CHAR_UUID_RAW = BLE_UUID128_INIT(
    0xe4,0xcc,0xdb,0xe2,0x2a,0x2a,0xb4,0x81,
    0xe9,0x11,0xa2,0xea,0x21,0xeb,0x00,0x00
);
static const ble_uuid16_t CCCD_UUID_RAW = BLE_UUID16_INIT(0x2902);

static void startAdvertisingForEbike();
static void startAdvertisingForClient();
static bool openGattClient();

// ─── Notify helpers ───────────────────────────────────────────────────────────
static void notifyLdiData(const uint8_t* data, size_t len) {
    if (!clientConnected || !pLdiChar) return;
    uint8_t buf[256];
    if (len + 12 <= sizeof(buf)) {
        memcpy(buf, data, len);
        uint8_t* p = buf + len;
        p += encodeField(p, BRIDGE_BATTERY_PERCENT_FIELD, bridgeBatteryPercent);
        p += encodeField(p, BRIDGE_BATTERY_MV_FIELD, bridgeBatteryMv);
        pLdiChar->setValue(buf, (size_t)(p - buf));
    } else {
        pLdiChar->setValue(data, len);
    }
    pLdiChar->notify();
}

static void notifyCpsData() {
    if (!clientConnected || !pCpsChar) return;
    int16_t power = (int16_t)gData.powerW;
    // CPS Measurement: flags(2) + instantaneous power(2) — flags=0x0000
    uint8_t buf[4] = { 0x00, 0x00, (uint8_t)(power & 0xFF), (uint8_t)(power >> 8) };
    pCpsChar->setValue(buf, 4);
    pCpsChar->notify();
}

// MODE_POWER_CADENCE: CPS Measurement with power + wheel revolution + crank revolution
// data packed into a single packet. This is the standard way real power meters (e.g.
// Favero Assioma, Garmin Vector) expose all three metrics over a single CPS service.
//
// Flags: 0x0030 = Wheel Revolution Data Present (bit 4) + Crank Revolution Data Present (bit 5)
// Packet layout (14 bytes):
//   flags(2LE) | power(2LE) | cum_wheel(4LE) | wheel_time(2LE,1/2048s) |
//   cum_crank(2LE) | crank_time(2LE,1/1024s)
static void notifyCpsCombinedData() {
    if (!clientConnected || !pCpsChar) return;

    uint32_t now = millis();
    uint32_t dt  = cscLastUpdateMs ? (now - cscLastUpdateMs) : 0;
    cscLastUpdateMs = now;

    if (dt > 0 && dt < 5000) {
        float dtSec = dt / 1000.0f;
        if (gData.speedKmh > 0.1f) {
            float speedMs = gData.speedKmh / 3.6f;
            cscWheelRevFrac += speedMs * dtSec / (WHEEL_CIRCUMFERENCE_MM / 1000.0f);
            uint32_t newRevs = (uint32_t)cscWheelRevFrac;
            if (newRevs > 0) {
                cscWheelRevFrac  -= newRevs;
                cscWheelRevTotal += newRevs;
                cpsCombWheelEventT = (uint16_t)(((uint64_t)now * 2048) / 1000);
            }
        }
        if (gData.cadenceRpm > 0) {
            cscCrankRevFrac += gData.cadenceRpm / 60.0f * dtSec;
            uint16_t newRevs = (uint16_t)cscCrankRevFrac;
            if (newRevs > 0) {
                cscCrankRevFrac   -= newRevs;
                cscCrankRevTotal  += newRevs;
                cscLastCrankEventT = (uint16_t)(((uint64_t)now * 1024) / 1000);
            }
        }
    }

    int16_t power = (int16_t)gData.powerW;
    uint8_t buf[14];
    buf[0]  = 0x30; buf[1] = 0x00;
    buf[2]  = (uint8_t)(power);        buf[3]  = (uint8_t)(power >> 8);
    buf[4]  = (uint8_t)(cscWheelRevTotal);
    buf[5]  = (uint8_t)(cscWheelRevTotal >> 8);
    buf[6]  = (uint8_t)(cscWheelRevTotal >> 16);
    buf[7]  = (uint8_t)(cscWheelRevTotal >> 24);
    buf[8]  = (uint8_t)(cpsCombWheelEventT);
    buf[9]  = (uint8_t)(cpsCombWheelEventT >> 8);
    buf[10] = (uint8_t)(cscCrankRevTotal);
    buf[11] = (uint8_t)(cscCrankRevTotal >> 8);
    buf[12] = (uint8_t)(cscLastCrankEventT);
    buf[13] = (uint8_t)(cscLastCrankEventT >> 8);
    pCpsChar->setValue(buf, 14);
    pCpsChar->notify();
}

static void notifyCscData() {
    if (!clientConnected || !pCscChar) return;

    uint32_t now = millis();
    uint32_t dt  = cscLastUpdateMs ? (now - cscLastUpdateMs) : 0;
    cscLastUpdateMs = now;

    if (dt > 0 && dt < 5000) {
        float dtSec = dt / 1000.0f;

        if (gData.speedKmh > 0.1f) {
            float speedMs = gData.speedKmh / 3.6f;
            cscWheelRevFrac += speedMs * dtSec / (WHEEL_CIRCUMFERENCE_MM / 1000.0f);
            uint32_t newRevs = (uint32_t)cscWheelRevFrac;
            if (newRevs > 0) {
                cscWheelRevFrac -= newRevs;
                cscWheelRevTotal += newRevs;
                // Timestamp of last wheel event in 1/1024 s units (wraps uint16)
                cscLastWheelEventT = (uint16_t)(((uint64_t)now * 1024) / 1000);
            }
        }

        if (gData.cadenceRpm > 0) {
            cscCrankRevFrac += gData.cadenceRpm / 60.0f * dtSec;
            uint16_t newRevs = (uint16_t)cscCrankRevFrac;
            if (newRevs > 0) {
                cscCrankRevFrac -= newRevs;
                cscCrankRevTotal += newRevs;
                cscLastCrankEventT = (uint16_t)(((uint64_t)now * 1024) / 1000);
            }
        }
    }

    // CSC Measurement: flags(1) + wheel_revs(4) + wheel_time(2) + crank_revs(2) + crank_time(2)
    uint8_t buf[11];
    buf[0]  = 0x03;  // flags: Wheel Revolution Data + Crank Revolution Data
    buf[1]  = (uint8_t)(cscWheelRevTotal);
    buf[2]  = (uint8_t)(cscWheelRevTotal >> 8);
    buf[3]  = (uint8_t)(cscWheelRevTotal >> 16);
    buf[4]  = (uint8_t)(cscWheelRevTotal >> 24);
    buf[5]  = (uint8_t)(cscLastWheelEventT);
    buf[6]  = (uint8_t)(cscLastWheelEventT >> 8);
    buf[7]  = (uint8_t)(cscCrankRevTotal);
    buf[8]  = (uint8_t)(cscCrankRevTotal >> 8);
    buf[9]  = (uint8_t)(cscLastCrankEventT);
    buf[10] = (uint8_t)(cscLastCrankEventT >> 8);
    pCscChar->setValue(buf, 11);
    pCscChar->notify();
}

static void handleLdiPayload(const uint8_t* data, size_t len) {
    decodeLiveData(data, len);
    // Rate-limited log: at most one entry every 10 s so the buffer is not flooded
    uint32_t nowL = millis();
    if (nowL - dblogLastLdiMs >= 10000) {
        dblogLastLdiMs = nowL;
        char hex[13]; int hi = 0;
        for (size_t j = 0; j < len && j < 6; j++)
            hi += snprintf(hex + hi, sizeof(hex) - hi, "%02x", data[j]);
        dblog("LDI: %uB spd=%.1f pwr=%d bat=%d [%s]",
              (unsigned)len, gData.speedKmh, gData.powerW, gData.batterySoc, hex);
    }
    if      (gMode == MODE_SUUNTO_BRIDGE) notifyLdiData(data, len);
    else if (gMode == MODE_POWER_SENSOR)  notifyCpsData();
    else if (gMode == MODE_SPEED_CADENCE) notifyCscData();
    else                                  notifyCpsCombinedData();  // MODE_POWER_CADENCE
}

// ─── Simulation / no-ebike data ───────────────────────────────────────────────
static void generateAndNotifySimData() {
    uint32_t t = millis();
    gData.speedKmh   = triWave(t, 20000, 1500, 3500) / 100.0f;
    gData.cadenceRpm = triWave(t, 15000,   70,   95);
    gData.powerW     = triWave(t, 12000,  120,  280);
    gData.batterySoc = 75;
    gData.odometerKm = 1234.0f;
    gData.valid      = true;

    if (gMode == MODE_SUUNTO_BRIDGE) {
        uint8_t buf[32]; uint8_t* p = buf;
        p += encodeField(p,  1, (uint64_t)(gData.speedKmh * 100));
        p += encodeField(p,  2, gData.cadenceRpm);
        p += encodeField(p,  5, gData.powerW);
        p += encodeField(p, 10, gData.batterySoc);
        p += encodeField(p, 12, (uint64_t)(gData.odometerKm * 1000));
        notifyLdiData(buf, (size_t)(p - buf));
    } else if (gMode == MODE_POWER_SENSOR) {
        notifyCpsData();
    } else if (gMode == MODE_SPEED_CADENCE) {
        notifyCscData();
    } else {  // MODE_POWER_CADENCE
        notifyCpsCombinedData();
    }
}

static void notifyNoEbikeData() {
    if (gMode == MODE_SUUNTO_BRIDGE) {
        uint8_t buf[32]; uint8_t* p = buf;
        p += encodeField(p,  1, 0);
        p += encodeField(p,  2, 0);
        p += encodeField(p,  5, 0);
        p += encodeField(p, 10, 0);
        p += encodeField(p, 12, 0);
        p += encodeField(p, BRIDGE_NO_EBIKE_FIELD, 1);
        size_t len = (size_t)(p - buf);
        decodeLiveData(buf, len);
        notifyLdiData(buf, len);
    } else if (gMode == MODE_POWER_SENSOR) {
        notifyCpsData();   // sends 0 W
    } else if (gMode == MODE_SPEED_CADENCE) {
        notifyCscData();   // sends unchanged cumulative counts (speed = 0)
    } else {  // MODE_POWER_CADENCE
        notifyCpsCombinedData();
    }
}

// ─── GATT client callbacks (bike side) ───────────────────────────────────────
static int gattWriteCccdCB(uint16_t, const struct ble_gatt_error* error,
                           struct ble_gatt_attr*, void*) {
    if (error->status == 0) {
        dblog("GATT: CCCD write OK, GATT ready");
        ebikeGattReady      = true;
        gattDiscoveryActive = false;
        nextGattRetryMs     = 0;
        // Reset the CSC timestamp so the first real LDI packet does not
        // accumulate distance for the entire GATT discovery gap (2–5 s).
        cscLastUpdateMs = 0;
        if (!clientConnected) startAdvertisingForClient();
    } else {
        dblog("GATT: CCCD write err %d, retry 3s", error->status);
        gattDiscoveryActive = false;
        nextGattRetryMs     = millis() + 3000;
    }
    return 0;
}

static void writeLdiCccd(uint16_t connHandle);

static int gattDscCB(uint16_t connHandle, const struct ble_gatt_error* error,
                     uint16_t, const struct ble_gatt_dsc* dsc, void*) {
    if (error->status == 0 && dsc &&
        ble_uuid_cmp(&dsc->uuid.u, &CCCD_UUID_RAW.u) == 0) {
        ldiCccdHandle = dsc->handle;
        dblog("GATT: CCCD h=%u", dsc->handle);
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        if (ldiCccdHandle == 0) {
            dblog("GATT: CCCD not found, retry 3s");
            gattDiscoveryActive = false;
            nextGattRetryMs = millis() + 3000;
            return 0;
        }
        writeLdiCccd(connHandle);
    } else if (error->status != 0) {
        dblog("GATT: dsc err %d, retry 3s", error->status);
        gattDiscoveryActive = false;
        nextGattRetryMs = millis() + 3000;
    }
    return 0;
}

static void writeLdiCccd(uint16_t connHandle) {
    if (ldiCccdHandle == 0) {
        gattDiscoveryActive = false;
        nextGattRetryMs = millis() + 3000;
        return;
    }
    dblog("GATT: write CCCD h=%u notify-ON", ldiCccdHandle);
    const uint16_t notifyOn = 0x0001;
    int rc = ble_gattc_write_flat(connHandle, ldiCccdHandle,
                                  &notifyOn, sizeof(notifyOn),
                                  gattWriteCccdCB, nullptr);
    if (rc != 0) {
        gattDiscoveryActive = false;
        nextGattRetryMs = millis() + 3000;
    }
}

static int gattChrCB(uint16_t connHandle, const struct ble_gatt_error* error,
                     const struct ble_gatt_chr* chr, void*) {
    if (error->status == 0 && chr) {
        ldiCharHandle = chr->val_handle;
        dblog("GATT: LDI chr h=%u", chr->val_handle);
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        if (ldiCharHandle == 0) {
            dblog("GATT: LDI chr not found, retry 3s");
            gattDiscoveryActive = false;
            nextGattRetryMs = millis() + 3000;
            return 0;
        }
        int rc = ble_gattc_disc_all_dscs(connHandle, ldiCharHandle + 1,
                                         ldiSvcEnd, gattDscCB, nullptr);
        if (rc != 0) {
            if (ldiCharHandle + 1 <= ldiSvcEnd) {
                dblog("GATT: dsc disc fail, CCCD fallback h=%u", ldiCharHandle + 1);
                ldiCccdHandle = ldiCharHandle + 1;
                writeLdiCccd(connHandle);
            } else {
                dblog("GATT: chr err %d, retry 3s", error->status);
                gattDiscoveryActive = false;
                nextGattRetryMs = millis() + 3000;
            }
        }
    } else if (error->status != 0) {
        dblog("GATT: chr err %d, retry 3s", error->status);
        gattDiscoveryActive = false;
        nextGattRetryMs = millis() + 3000;
    }
    return 0;
}

static int gattSvcCB(uint16_t connHandle, const struct ble_gatt_error* error,
                     const struct ble_gatt_svc* service, void*) {
    if (error->status == 0 && service) {
        ldiSvcStart = service->start_handle;
        ldiSvcEnd   = service->end_handle;
        dblog("GATT: LDI svc [%u..%u]", service->start_handle, service->end_handle);
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        if (ldiSvcStart == 0 || ldiSvcEnd == 0) {
            dblog("GATT: LDI svc not found, restarting");
            gattDiscoveryActive = false;
            ebikeConnected      = false;
            ebikeEncrypted      = false;
            gattStartPending    = false;
            ebikeConnHandle     = BLE_HS_CONN_HANDLE_NONE;
            nextGattRetryMs     = 0;
            startAdvertisingForEbike();
            return 0;
        }
        int rc = ble_gattc_disc_chrs_by_uuid(connHandle, ldiSvcStart, ldiSvcEnd,
                                             &LDI_CHAR_UUID_RAW.u, gattChrCB, nullptr);
        if (rc != 0) {
            dblog("GATT: chr disc fail err %d, retry 3s", rc);
            gattDiscoveryActive = false;
            nextGattRetryMs = millis() + 3000;
        }
    } else if (error->status != 0) {
        dblog("GATT: svc err %d, retry 3s", error->status);
        gattDiscoveryActive = false;
        nextGattRetryMs = millis() + 3000;
    }
    return 0;
}

static void startLdiServiceDiscovery(uint16_t connHandle) {
    ldiSvcStart = ldiSvcEnd = ldiCharHandle = ldiCccdHandle = 0;
    int rc = ble_gattc_disc_svc_by_uuid(connHandle, &LDI_SVC_UUID_RAW.u,
                                        gattSvcCB, nullptr);
    if (rc != 0) {
        gattDiscoveryActive = false;
        nextGattRetryMs = millis() + 3000;
    }
}

static int gattMtuCB(uint16_t connHandle, const struct ble_gatt_error*,
                     uint16_t mtu, void*) {
    lastMtu = mtu;
    dblog("GATT: MTU=%u, svc discovery", mtu);
    startLdiServiceDiscovery(connHandle);
    return 0;
}

static int customGapHandler(struct ble_gap_event* event, void*) {
    if (event->type == BLE_GAP_EVENT_ENC_CHANGE &&
        event->enc_change.conn_handle == ebikeConnHandle) {
        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(ebikeConnHandle, &desc);
        ebikeEncrypted = (event->enc_change.status == 0 && rc == 0 &&
                          desc.sec_state.encrypted);
        dblog("ENC: h=%u enc=%d status=%d",
              event->enc_change.conn_handle, (int)ebikeEncrypted,
              event->enc_change.status);
        if (ebikeEncrypted && gattStartPending) {
            gattStartPending = false;
            openGattClient();
        }
    }
    if (event->type == BLE_GAP_EVENT_NOTIFY_RX &&
        event->notify_rx.conn_handle == ebikeConnHandle &&
        event->notify_rx.attr_handle == ldiCharHandle) {
        uint8_t buf[244];
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len > sizeof(buf)) len = sizeof(buf);
        if (os_mbuf_copydata(event->notify_rx.om, 0, len, buf) == 0)
            handleLdiPayload(buf, len);
    }
    return 0;
}

// ─── BLE server callbacks ─────────────────────────────────────────────────────
class ServerCB : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, ble_gap_conn_desc* desc) override {
        if (!gSimEnabled && !ebikeConnected) {
            ebikeAddr       = NimBLEAddress(desc->peer_ota_addr);
            ebikeConnHandle = desc->conn_handle;
            ebikeConnected  = true;
            ebikeEncrypted  = desc->sec_state.encrypted;
            flagEbikePeripheralConn = true;
            dblog("CONN: eBike h=%u enc=%d", desc->conn_handle,
                  (int)desc->sec_state.encrypted);
        } else {
            clientConnHandle = desc->conn_handle;
            flagClientConn   = true;
            dblog("CONN: Client h=%u", desc->conn_handle);
        }
    }
    void onDisconnect(NimBLEServer*, ble_gap_conn_desc* desc) override {
        if (!gSimEnabled && desc->conn_handle == ebikeConnHandle) {
            ebikeConnected      = false;
            ebikeGattReady      = false;
            gattDiscoveryActive = false;
            ebikeEncrypted      = false;
            gattStartPending    = false;
            ebikeConnHandle     = BLE_HS_CONN_HANDLE_NONE;
            flagEbikeDisconn    = true;
            dblog("DISC: eBike h=%u", desc->conn_handle);
        } else {
            if (desc->conn_handle == clientConnHandle)
                clientConnHandle = BLE_HS_CONN_HANDLE_NONE;
            flagClientDisconn = true;
            dblog("DISC: Client h=%u", desc->conn_handle);
        }
    }
    bool onConfirmPIN(uint32_t) override { return true; }
};

// ─── BLE advertising ──────────────────────────────────────────────────────────
static void startAdvertisingForEbike() {
    dblog("ADV: eBike solicitation started");
    auto* adv = NimBLEDevice::getAdvertising();
    adv->reset();
    const uint8_t payload[] = {
        0x02, 0x01, 0x06,
        0x11, 0x15,  // AD type 0x15: 128-bit Service Solicitation UUID
        0xe4,0xcc,0xdb,0xe2,0x2a,0x2a,0xb4,0x81,
        0xe9,0x11,0xa2,0xea,0x20,0xeb,0x00,0x00
    };
    NimBLEAdvertisementData advData;
    advData.addData(std::string((const char*)payload, sizeof(payload)));
    adv->setAdvertisementData(advData);
    NimBLEAdvertisementData scan;
    scan.setName(gDeviceName);
    adv->setScanResponseData(scan);
    adv->setScanResponse(true);
    adv->setMinInterval(0x0050);
    adv->setMaxInterval(0x00a0);
    adv->start();
}

static void startAdvertisingForClient() {
    dblog("ADV: client started mode=%d", (int)gMode);
    auto* adv = NimBLEDevice::getAdvertising();
    adv->reset();
    NimBLEAdvertisementData advData;
    NimBLEAdvertisementData scan;

    if (gMode == MODE_SUUNTO_BRIDGE) {
        // AD type 0x07: Complete List of 128-bit Service UUIDs (LDI UUID)
        const uint8_t payload[] = {
            0x02, 0x01, 0x06,
            0x11, 0x07,
            0xe4,0xcc,0xdb,0xe2,0x2a,0x2a,0xb4,0x81,
            0xe9,0x11,0xa2,0xea,0x20,0xeb,0x00,0x00
        };
        advData.addData(std::string((const char*)payload, sizeof(payload)));
        scan.setName(gDeviceName);
    } else {
        // AD type 0x03: Complete List of 16-bit Service UUIDs
        // AD type 0x19: Appearance — required for fitness clients (Suunto, Garmin,
        // Wahoo) to categorise the sensor and offer it in the correct activity.
        //   0x0484 = Cycling: Power Sensor  (BT Assigned Numbers §2.6.3)
        //   0x0483 = Cycling: Speed and Cadence Sensor
        uint16_t uuid16     = (gMode == MODE_SPEED_CADENCE) ? 0x1816 : 0x1818;
        uint16_t appearance = (gMode == MODE_SPEED_CADENCE) ? 0x0483 : 0x0484;
        const uint8_t payload[] = {
            0x02, 0x01, 0x06,
            0x03, 0x03, (uint8_t)(uuid16 & 0xFF),     (uint8_t)(uuid16 >> 8),
            0x03, 0x19, (uint8_t)(appearance & 0xFF), (uint8_t)(appearance >> 8)
        };
        advData.addData(std::string((const char*)payload, sizeof(payload)));
        scan.setName(gDeviceName);
    }

    adv->setAdvertisementData(advData);
    adv->setScanResponseData(scan);
    adv->setScanResponse(true);
    adv->setMinInterval(0x0050);
    adv->setMaxInterval(0x00a0);
    adv->start();
}

static bool openGattClient() {
    if (!ebikeConnected || ebikeConnHandle == BLE_HS_CONN_HANDLE_NONE) return false;
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(ebikeConnHandle, &desc) != 0) return false;
    ebikeEncrypted = desc.sec_state.encrypted;
    dblog("GATT: open client h=%u enc=%d", ebikeConnHandle, (int)ebikeEncrypted);
    gattStartPending    = false;
    ebikeGattReady      = false;
    gattDiscoveryActive = true;
    ldiSvcStart = ldiSvcEnd = ldiCharHandle = ldiCccdHandle = 0;
    NimBLEDevice::setMTU(247);
    ble_gap_set_data_len(ebikeConnHandle, 251, (251 + 14) * 8);
    int rc = ble_gattc_exchange_mtu(ebikeConnHandle, gattMtuCB, nullptr);
    if (rc != 0) startLdiServiceDiscovery(ebikeConnHandle);
    return true;
}

static void resetAndAdvertiseForEbike() {
    dblog("RESET: full reset, adv for eBike");
    ebikeConnected      = ebikeGattReady = false;
    gattDiscoveryActive = ebikeEncrypted = gattStartPending = false;
    ebikeConnHandle     = BLE_HS_CONN_HANDLE_NONE;
    ldiSvcStart = ldiSvcEnd = ldiCharHandle = ldiCccdHandle = 0;
    flagEbikePeripheralConn = flagEbikeDisconn = false;
    gData               = LiveData{};
    nextGattRetryMs     = 0;
    nextNoEbikeNotifyMs = 0;  // send first no-ebike notification immediately
    // Reset CSC fractional accumulators; totals are preserved so the client
    // sees a smooth continuation (no backward jump) when the bike reconnects.
    cscWheelRevFrac = cscCrankRevFrac = 0.0f;
    cscLastUpdateMs = 0;
    startAdvertisingForEbike();
}

// ─── Cycling Power Control Point callbacks (CPS mode) ────────────────────────
// The Cycling Power Control Point (0x2A66) is mandatory when CPS Feature bit 14
// (Crank Length Adjustment Supported) is set. Fitness clients use it to configure
// and verify crank length during pairing; without it they consider the sensor
// non-compliant and may not offer it in cycling activities.
// We implement opcodes 0x04 (Set Crank Length) and 0x05 (Request Crank Length).
// All other opcodes get "Op Code Not Supported" (0x02). The crank length value
// is stored but has no effect on power output — power comes from the eBike.
class CpControlPointCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) override {
        std::string val = pChar->getValue();
        if (val.empty()) return;
        uint8_t opcode   = (uint8_t)val[0];
        uint8_t respCode = 0x01;  // Success
        uint8_t extra[2] = {};
        int     extraLen = 0;

        switch (opcode) {
            case 0x01:  // Set Cumulative Wheel Revolutions (used in MODE_POWER_CADENCE)
                if (val.size() >= 5) {
                    uint32_t cumVal = (uint8_t)val[1] | ((uint8_t)val[2] << 8) |
                                      ((uint8_t)val[3] << 16) | ((uint8_t)val[4] << 24);
                    cscWheelRevTotal = cumVal;
                    cscWheelRevFrac  = 0.0f;
                    dblog("CPCP: set cumulative wheel=%lu", (unsigned long)cumVal);
                } else {
                    respCode = 0x03;  // Invalid Parameter
                }
                break;
            case 0x04:  // Set Crank Length (param: uint16_t, 1/10 mm)
                if (val.size() >= 3) {
                    cpsCrankLengthTenthMm = (uint8_t)val[1] | ((uint8_t)val[2] << 8);
                    dblog("CPCP: set crank=%u (1/10mm)", cpsCrankLengthTenthMm);
                } else {
                    respCode = 0x03;  // Invalid Parameter
                }
                break;
            case 0x05:  // Request Crank Length — respond with current value
                extra[0] = (uint8_t)(cpsCrankLengthTenthMm & 0xFF);
                extra[1] = (uint8_t)(cpsCrankLengthTenthMm >> 8);
                extraLen = 2;
                dblog("CPCP: request crank=%u", cpsCrankLengthTenthMm);
                break;
            default:
                respCode = 0x02;  // Op Code Not Supported
                dblog("CPCP: opcode=0x%02x not supported", opcode);
                break;
        }

        uint8_t resp[5] = { 0x20, opcode, respCode, extra[0], extra[1] };
        pChar->setValue(resp, 3 + extraLen);
        pChar->indicate();
    }
};

// ─── SC Control Point callbacks (CSC mode) ────────────────────────────────────
// The SC Control Point (0x2A55) is mandatory when Wheel Revolution Data is
// supported (CSC Feature bit 0). Suunto and other strict clients check for it
// and disconnect if it is absent. We implement opcode 0x01 (Set Cumulative
// Value) to let the client reset the wheel rev counter; all other opcodes get
// "Opcode Not Supported" (0x02).
class ScControlPointCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) override {
        std::string val = pChar->getValue();
        if (val.empty()) return;
        uint8_t opcode = (uint8_t)val[0];
        uint8_t result = 0x01;  // Success
        if (opcode == 0x01 && val.size() >= 5) {
            uint32_t cumVal = (uint8_t)val[1] | ((uint8_t)val[2] << 8) |
                              ((uint8_t)val[3] << 16) | ((uint8_t)val[4] << 24);
            cscWheelRevTotal = cumVal;
            cscWheelRevFrac  = 0.0f;
            dblog("SCCP: set cumulative=%lu", (unsigned long)cumVal);
        } else {
            result = 0x02;  // Opcode Not Supported
            dblog("SCCP: opcode=0x%02x not supported", opcode);
        }
        uint8_t resp[3] = { 0x10, opcode, result };
        pChar->setValue(resp, 3);
        pChar->indicate();
    }
};

// ─── Web server ───────────────────────────────────────────────────────────────
static WebServer webServer(80);
static bool     webOk             = false;
static bool     wifiActive        = false;
static bool     webHadClient      = false;
static uint32_t webStartedMs      = 0;
static uint32_t lastWebClientSeenMs = 0;

static const uint32_t WEB_BOOT_WINDOW_MS       = 60000;
static const uint32_t WEB_IDLE_AFTER_CLIENT_MS = 60000;

static const char INDEX_HTML[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<title>BoschEBike Bridge</title>
<style>
*{box-sizing:border-box}
body{background:#0d1117;color:#e6edf3;font-family:monospace;padding:16px;margin:0}
h1{color:#58a6ff;font-size:18px;margin:0 0 4px}
#st{font-size:13px;margin-bottom:10px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;max-width:500px}
.card{background:#161b22;border-radius:8px;padding:12px}
.lbl{color:#8b949e;font-size:10px;letter-spacing:.5px;margin-bottom:2px}
.val{font-size:28px;font-weight:bold}
.unit{color:#8b949e;font-size:13px}
.flags{background:#161b22;border-radius:8px;padding:10px;margin-top:8px;max-width:500px}
.f{display:inline-block;margin:3px;padding:3px 10px;border-radius:4px;font-size:12px}
.on{background:#1f6feb}.off{background:#21262d;color:#484f58}
.msect{background:#161b22;border-radius:8px;padding:10px;margin-top:8px;max-width:500px}
.mbtn{background:#21262d;color:#e6edf3;border:1px solid #30363d;border-radius:6px;
  padding:6px 14px;cursor:pointer;font-family:monospace;font-size:12px;margin:3px}
.mbtn.act{background:#1f6feb;border-color:#1f6feb}
#mst{font-size:11px;color:#8b949e;margin-top:6px;min-height:16px}
.dinput{display:flex;gap:6px;margin-top:6px;align-items:center}
.dinput input{background:#0d1117;color:#e6edf3;border:1px solid #30363d;border-radius:6px;
  padding:5px 10px;font-family:monospace;font-size:13px;flex:1;min-width:0}
.dinput input:focus{outline:none;border-color:#58a6ff}
.dinput button{background:#21262d;color:#e6edf3;border:1px solid #30363d;border-radius:6px;
  padding:5px 12px;cursor:pointer;font-family:monospace;font-size:12px;white-space:nowrap}
.dinput button:hover{background:#30363d}
#dnprev{font-size:11px;color:#58a6ff;margin-top:4px;min-height:16px}
#dnst{font-size:11px;color:#8b949e;margin-top:2px;min-height:16px}
#ts{margin-top:10px;color:#484f58;font-size:11px}
</style></head><body>
<h1>BoschEBike Bridge</h1>
<div id="st" style="color:#3fb950">Connecting...</div>
<div class="grid">
  <div class="card"><div class="lbl">SPEED</div><span class="val" id="spd">-</span> <span class="unit">km/h</span></div>
  <div class="card"><div class="lbl">CADENCE</div><span class="val" id="cad">-</span> <span class="unit">rpm</span></div>
  <div class="card"><div class="lbl">POWER</div><span class="val" id="pwr" style="color:#3fb950">-</span> <span class="unit">W</span></div>
  <div class="card"><div class="lbl">BATTERY</div><span class="val" id="bat" style="color:#58a6ff">-</span> <span class="unit">%</span></div>
  <div class="card"><div class="lbl">BRIDGE BATTERY</div><span class="val" id="bbat" style="color:#d29922">-</span> <span class="unit">%</span></div>
  <div class="card"><div class="lbl">ODOMETER</div><span class="val" id="odo">-</span> <span class="unit">km</span></div>
  <div class="card"><div class="lbl">AMBIENT LIGHT</div><span class="val" id="lux" style="color:#d29922">-</span> <span class="unit">lux</span></div>
</div>
<div class="flags">
  <span class="f off" id="fl_lt">Light: -</span>
  <span class="f off" id="fl_lk">Lock</span>
  <span class="f off" id="fl_ch">Charger</span>
  <span class="f off" id="fl_rv">Reserve</span>
  <span class="f off" id="fl_dg">Diag</span>
  <span class="f off" id="fl_st">Stationary</span>
  <span class="f off" id="fl_cl">Client</span>
</div>
<div class="msect">
  <div class="lbl">FIRMWARE MODE</div>
  <div style="margin-top:6px">
    <button class="mbtn" id="mb1" onclick="setMode(1)">Suunto Bridge</button>
    <button class="mbtn" id="mb2" onclick="setMode(2)">Power Sensor</button>
    <button class="mbtn" id="mb3" onclick="setMode(3)">Speed &amp; Cadence</button>
    <button class="mbtn" id="mb4" onclick="setMode(4)">Power + Cadence</button>
  </div>
  <div id="mst"></div>
</div>
<div class="msect">
  <div class="lbl">DEVICE SETTINGS</div>
  <div class="dinput">
    <input type="text" id="dname" maxlength="20" placeholder="BoschEBike" autocomplete="off" spellcheck="false">
    <button onclick="saveName()">Save &amp; Reboot</button>
  </div>
  <div id="dnprev"></div>
  <div id="dnst"></div>
</div>
<div class="msect">
  <div class="lbl">SIMULATION</div>
  <div style="margin-top:6px">
    <button class="mbtn" id="sb0" onclick="setSim(0)">Real eBike</button>
    <button class="mbtn" id="sb1" onclick="setSim(1)">Simulated</button>
  </div>
  <div id="sst" style="font-size:11px;color:#8b949e;margin-top:6px;min-height:16px"></div>
</div>
<div class="msect">
  <div class="lbl">BLE DEBUG LOG</div>
  <div style="margin-top:6px;display:flex;align-items:center;flex-wrap:wrap;gap:4px">
    <button class="mbtn" id="db0" onclick="setDebug(0)">Disabled</button>
    <button class="mbtn" id="db1" onclick="setDebug(1)">Enabled</button>
    <a href="/log" target="_blank" style="color:#58a6ff;font-size:12px;margin-left:6px;text-decoration:none">View log &#x2197;</a>
    <button class="mbtn" onclick="clearLog()">Clear</button>
  </div>
  <div id="dbst" style="font-size:11px;color:#8b949e;margin-top:6px;min-height:16px"></div>
</div>
<div id="ts"></div>
<script>
const LT=['−','OFF','ON'];
const MN=['','Suunto Bridge','Power Sensor','Speed & Cadence','Power + Cadence'];
const MS=['',' Bridge',' Power',' SpeedCadence',' PowerCadence'];
let busy=false;
let nameFocused=false;
document.getElementById('dname').addEventListener('focus',()=>{nameFocused=true;});
document.getElementById('dname').addEventListener('blur',()=>{nameFocused=false;});
document.getElementById('dname').addEventListener('input',function(){
  var sfx=MS[currentMode]||'';
  document.getElementById('dnprev').textContent=this.value?'BLE name: '+this.value+sfx:'';
});
var currentMode=1;
function f(id,on,txt){var e=document.getElementById(id);e.textContent=txt;e.className='f '+(on?'on':'off');}
function setMode(m){
  document.getElementById('mst').textContent='Saving...';
  fetch('/setmode?mode='+m,{cache:'no-store'})
    .then(r=>r.text()).then(t=>{document.getElementById('mst').textContent=t;})
    .catch(()=>{document.getElementById('mst').textContent='Error';});
}
function saveName(){
  var n=document.getElementById('dname').value.trim();
  if(!n){document.getElementById('dnst').textContent='Name cannot be empty';return;}
  document.getElementById('dnst').textContent='Saving...';
  fetch('/setname?name='+encodeURIComponent(n),{cache:'no-store'})
    .then(r=>r.text()).then(t=>{document.getElementById('dnst').textContent=t;})
    .catch(()=>{document.getElementById('dnst').textContent='Error';});
}
function setSim(s){
  document.getElementById('sst').textContent='Saving...';
  fetch('/setsim?sim='+s,{cache:'no-store'})
    .then(r=>r.text()).then(t=>{document.getElementById('sst').textContent=t;})
    .catch(()=>{document.getElementById('sst').textContent='Error';});
}
function setDebug(d){
  document.getElementById('dbst').textContent='Saving...';
  fetch('/setdebug?debug='+d,{cache:'no-store'})
    .then(r=>r.text()).then(t=>{document.getElementById('dbst').textContent=t;})
    .catch(()=>{document.getElementById('dbst').textContent='Error';});
}
function clearLog(){
  fetch('/clearlog',{cache:'no-store'})
    .then(r=>r.text()).then(t=>{document.getElementById('dbst').textContent=t;})
    .catch(()=>{document.getElementById('dbst').textContent='Error';});
}
function pollData(){
  if(busy)return;busy=true;
  fetch('/data',{cache:'no-store'}).then(r=>r.json()).then(d=>{
    currentMode=d.mode;
    var s=document.getElementById('st');
    var mn='['+MN[d.mode]+']';
    if(d.sim){s.style.color='#a371f7';s.textContent='SIMULATION '+mn+' | Client: '+(d.client?'connected':'advertising');}
    else if(!d.ebike){s.style.color='#d29922';s.textContent=mn+' waiting for bike...';}
    else if(!d.gatt){s.style.color='#d29922';s.textContent=mn+' Bike connected | GATT: discovering...';}
    else{s.style.color='#3fb950';s.textContent=mn+' Bike ready | Client: '+(d.client?'connected':'advertising');}
    var v=d.valid;
    document.getElementById('spd').textContent=v?d.speed.toFixed(1):'-';
    document.getElementById('cad').textContent=v?d.cadence:'-';
    document.getElementById('pwr').textContent=v?d.power:'-';
    document.getElementById('bat').textContent=v?d.battery:'-';
    document.getElementById('bbat').textContent=d.bridge_battery;
    document.getElementById('odo').textContent=v?d.odometer.toFixed(1):'-';
    document.getElementById('lux').textContent=v?d.ambient.toFixed(0):'-';
    f('fl_lt',d.bike_light===2,'Light: '+(LT[d.bike_light]||'-'));
    f('fl_lk',d.locked,'Lock');
    f('fl_ch',d.charger,'Charger');
    f('fl_rv',d.reserve,'Reserve');
    f('fl_dg',d.diag,'Diag');
    f('fl_st',d.standstill,'Stationary');
    f('fl_cl',d.client,'Client');
    for(var i=1;i<=4;i++)document.getElementById('mb'+i).className='mbtn'+(d.mode==i?' act':'');
    document.getElementById('sb0').className='mbtn'+(d.sim?'':' act');
    document.getElementById('sb1').className='mbtn'+(d.sim?' act':'');
    document.getElementById('db0').className='mbtn'+(d.debug?'':' act');
    document.getElementById('db1').className='mbtn'+(d.debug?' act':'');
    if(!nameFocused){
      document.getElementById('dname').value=d.base_name||'';
      document.getElementById('dnprev').textContent='BLE name: '+d.device_name;
    }
  }).catch(()=>{document.getElementById('st').textContent='Web connection error';})
    .finally(()=>{busy=false;});
}
pollData();
setInterval(pollData,1000);
</script></body></html>)html";

static void handleRoot() {
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send_P(200, "text/html", INDEX_HTML);
}

static void handleStatus() {
    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"ap_ip\":\"%s\",\"wifi_mode\":\"ap\",\"web\":true}",
        WiFi.softAPIP().toString().c_str());
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "application/json", buf);
}

static void handleSetMode() {
    if (!webServer.hasArg("mode")) {
        webServer.send(400, "text/plain", "Missing mode");
        return;
    }
    int m = webServer.arg("mode").toInt();
    if (m < 1 || m > 4) {
        webServer.send(400, "text/plain", "Invalid mode (1-4)");
        return;
    }
    Preferences prefs;
    prefs.begin("ebike", false);
    prefs.putUChar("mode", (uint8_t)m);
    prefs.end();

    const char* names[] = { "", "Suunto Bridge", "Power Sensor", "Speed & Cadence", "Power + Cadence" };
    char buf[64];
    snprintf(buf, sizeof(buf), "Mode set to %s. Rebooting...", names[m]);
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "text/plain", buf);
    delay(300);
    ESP.restart();
}

static void handleSetName() {
    if (!webServer.hasArg("name")) {
        webServer.send(400, "text/plain", "Missing name");
        return;
    }
    String name = webServer.arg("name");
    name.trim();
    if (name.length() == 0 || name.length() > 20) {
        webServer.send(400, "text/plain", "Name must be 1-20 characters");
        return;
    }
    for (char c : name) {
        if (!isalnum(c) && c != ' ' && c != '-' && c != '_') {
            webServer.send(400, "text/plain", "Only letters, digits, spaces, - and _ allowed");
            return;
        }
    }
    Preferences prefs;
    prefs.begin("ebike", false);
    prefs.putString("dname", name);
    prefs.end();

    char buf[64];
    snprintf(buf, sizeof(buf), "Name set to \"%s\". Rebooting...", name.c_str());
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "text/plain", buf);
    delay(300);
    ESP.restart();
}

static void handleSetSim() {
    if (!webServer.hasArg("sim")) {
        webServer.send(400, "text/plain", "Missing sim parameter");
        return;
    }
    int s = webServer.arg("sim").toInt();
    if (s != 0 && s != 1) {
        webServer.send(400, "text/plain", "sim must be 0 or 1");
        return;
    }
    Preferences prefs;
    prefs.begin("ebike", false);
    prefs.putUChar("sim", (uint8_t)s);
    prefs.end();

    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "text/plain",
        s ? "Simulation enabled. Rebooting..." : "Simulation disabled. Rebooting...");
    delay(300);
    ESP.restart();
}

static const char UPDATE_HTML[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BoschEBike Bridge OTA</title>
<style>
*{box-sizing:border-box}
body{background:#0d1117;color:#e6edf3;font-family:system-ui,sans-serif;margin:0;padding:20px}
main{max-width:520px}
h1{font-size:20px;margin:0 0 16px}
form{background:#161b22;border-radius:8px;padding:16px}
input,button{width:100%;font-size:15px;margin-top:12px}
button{background:#1f6feb;color:white;border:0;border-radius:6px;padding:10px}
p{color:#8b949e;font-size:13px;line-height:1.4}
</style></head><body><main>
<h1>Firmware update</h1>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin" required>
<button type="submit">Upload firmware</button>
</form>
<p>Use the file <code>.pio/build/lolin32_lite/firmware.bin</code>. The ESP32 reboots automatically after a successful update.</p>
</main></body></html>)html";

static void handleUpdatePage() {
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send_P(200, "text/html", UPDATE_HTML);
}

static void handleUpdateUpload() {
    HTTPUpload& upload = webServer.upload();
    if      (upload.status == UPLOAD_FILE_START)   { if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial); }
    else if (upload.status == UPLOAD_FILE_WRITE)   { if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial); }
    else if (upload.status == UPLOAD_FILE_END)     { if (!Update.end(true)) Update.printError(Serial); }
    else if (upload.status == UPLOAD_FILE_ABORTED) { Update.abort(); }
}

static void handleUpdateDone() {
    bool ok = !Update.hasError();
    webServer.sendHeader("Connection", "close");
    webServer.send(ok ? 200 : 500, "text/plain",
                   ok ? "Update complete. Rebooting..." : "Update failed.");
    if (ok) { delay(500); ESP.restart(); }
}

static void handleData() {
    LiveData d = gData;
    char buf[820];
    snprintf(buf, sizeof(buf),
        "{\"sim\":%s,\"mode\":%d,\"base_name\":\"%s\",\"device_name\":\"%s\","
        "\"ebike\":%s,\"gatt\":%s,\"client\":%s,\"valid\":%s,"
        "\"speed\":%.2f,\"cadence\":%d,\"power\":%d,\"battery\":%d,"
        "\"bridge_battery\":%u,\"bridge_battery_mv\":%u,"
        "\"odometer\":%.2f,\"ambient\":%.1f,\"bike_light\":%d,"
        "\"locked\":%s,\"charger\":%s,\"reserve\":%s,\"diag\":%s,\"standstill\":%s,"
        "\"debug\":%s}",
        gSimEnabled     ? "true" : "false",
        (int)gMode,
        gBaseDeviceName,
        gDeviceName,
        ebikeConnected  ? "true" : "false",
        ebikeGattReady  ? "true" : "false",
        clientConnected ? "true" : "false",
        d.valid         ? "true" : "false",
        d.speedKmh, d.cadenceRpm, d.powerW, d.batterySoc,
        bridgeBatteryPercent, bridgeBatteryMv,
        d.odometerKm, d.ambientLux, d.bikeLight,
        d.systemLocked ? "true" : "false",
        d.chargerConn  ? "true" : "false",
        d.lightReserve ? "true" : "false",
        d.diagActive   ? "true" : "false",
        d.notDriving   ? "true" : "false",
        gBleDebug      ? "true" : "false"
    );
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "application/json", buf);
}

static void handleLog() {
    // Return ring buffer contents as plain text, oldest entry first.
    // Served at GET /log — readable from a browser or via WebFetch.
    static char logBuf[DBLOG_ENTRIES * DBLOG_LINE + 64];
    int pos = 0;
    if (dblogCount == 0) {
        pos = snprintf(logBuf, sizeof(logBuf), "(log empty)\n");
    } else {
        int start = (dblogHead - dblogCount + DBLOG_ENTRIES) % DBLOG_ENTRIES;
        for (int i = 0; i < dblogCount && pos < (int)sizeof(logBuf) - 2; i++) {
            int idx = (start + i) % DBLOG_ENTRIES;
            pos += snprintf(logBuf + pos, sizeof(logBuf) - pos, "%s\n", dblogBuf[idx]);
        }
    }
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "text/plain", logBuf);
}

static void handleClearLog() {
    dblogHead = dblogCount = 0;
    dblogLastLdiMs = 0;
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "text/plain", "Log cleared");
}

static void handleSetDebug() {
    if (!webServer.hasArg("debug")) {
        webServer.send(400, "text/plain", "Missing debug parameter");
        return;
    }
    int d = webServer.arg("debug").toInt();
    if (d != 0 && d != 1) {
        webServer.send(400, "text/plain", "debug must be 0 or 1");
        return;
    }
    Preferences prefs;
    prefs.begin("ebike", false);
    prefs.putUChar("bdbg", (uint8_t)d);
    prefs.end();
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "text/plain",
        d ? "Debug enabled. Rebooting..." : "Debug disabled. Rebooting...");
    delay(300);
    ESP.restart();
}

static void startWebDebug() {
    IPAddress apIp(192, 168, 4, 1);
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    WiFi.softAPConfig(apIp, apIp, IPAddress(255, 255, 255, 0));
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, 1, false, 4);

    webServer.on("/",        handleRoot);
    webServer.on("/data",    handleData);
    webServer.on("/status",  handleStatus);
    webServer.on("/setmode", handleSetMode);
    webServer.on("/setname", handleSetName);
    webServer.on("/setsim",    handleSetSim);
    webServer.on("/setdebug",  handleSetDebug);
    webServer.on("/log",       handleLog);
    webServer.on("/clearlog",  handleClearLog);
    webServer.on("/update",  HTTP_GET,  handleUpdatePage);
    webServer.on("/update",  HTTP_POST, handleUpdateDone, handleUpdateUpload);
    webServer.begin();

    webStartedMs        = millis();
    lastWebClientSeenMs = webStartedMs;
    webHadClient        = false;
    webOk               = true;
    wifiActive          = true;
}

static void stopWebDebug() {
    webServer.close();
    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    webOk      = false;
    wifiActive = false;
}

static void manageWebPower() {
    if (!wifiActive) return;
    uint32_t now      = millis();
    uint8_t  stations = WiFi.softAPgetStationNum();
    if (stations > 0) {
        webHadClient        = true;
        lastWebClientSeenMs = now;
        return;
    }
    uint32_t timeoutMs   = webHadClient ? WEB_IDLE_AFTER_CLIENT_MS : WEB_BOOT_WINDOW_MS;
    uint32_t referenceMs = webHadClient ? lastWebClientSeenMs : webStartedMs;
    if ((int32_t)(now - referenceMs - timeoutMs) >= 0) stopWebDebug();
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    startStartupLed();

    // Load settings from NVS (falls back to defaults on first boot)
    {
        Preferences prefs;
        prefs.begin("ebike", true);
        uint8_t stored = prefs.getUChar("mode", (uint8_t)DEFAULT_MODE);
        if (stored >= 1 && stored <= 4) gMode = (FirmwareMode)stored;
        prefs.getString("dname", "BoschEBike").toCharArray(gBaseDeviceName, sizeof(gBaseDeviceName));
        gSimEnabled = prefs.getUChar("sim",  0) != 0;
        gBleDebug   = prefs.getUChar("bdbg", 0) != 0;
        prefs.end();
    }
    buildDeviceName();

    analogReadResolution(12);
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
    updateBridgeBattery(true);

    startWebDebug();

    NimBLEDevice::init(gDeviceName);
    // Bridge mode: Bosch bike requires SC + bonding → set globally.
    // Power/CSC modes: standard fitness clients (Suunto, Garmin, Wahoo) expect an
    // open connection with no security request. Setting BOND globally would cause
    // NimBLE to send a Security Request PDU on every client connection, making the
    // client disconnect before it can write the CCCD to enable notifications.
    // The bike can still initiate its own encryption in all modes.
    if (gMode == MODE_SUUNTO_BRIDGE) {
        NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_SC | BLE_SM_PAIR_AUTHREQ_BOND);
        NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    } else {
        NimBLEDevice::setSecurityAuth(false, false, false);
    }
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    // Set GAP Appearance characteristic (0x2A01) so connected clients can read
    // the device type. Some fitness watches read this after connecting to decide
    // which activity sport this sensor belongs to.
    if      (gMode == MODE_POWER_SENSOR  ||
             gMode == MODE_POWER_CADENCE)  ble_svc_gap_device_appearance_set(0x0484);
    else if (gMode == MODE_SPEED_CADENCE)  ble_svc_gap_device_appearance_set(0x0483);
    NimBLEDevice::setCustomGapHandler(customGapHandler);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCB());

    if (gMode == MODE_SUUNTO_BRIDGE) {
        auto* svc = pServer->createService(NimBLEUUID(LDI_SVC_UUID));
        pLdiChar  = svc->createCharacteristic(NimBLEUUID(LDI_CHAR_UUID),
                                               NIMBLE_PROPERTY::NOTIFY);
        svc->start();
    } else if (gMode == MODE_POWER_SENSOR) {
        // Use explicit 16-bit UUIDs — NimBLE stores 128-bit-string UUIDs as
        // 128-bit in the attribute table, which strict fitness clients may reject.
        auto* svc = pServer->createService(NimBLEUUID((uint16_t)0x1818));
        pCpsChar  = svc->createCharacteristic(NimBLEUUID((uint16_t)0x2A63),
                                               NIMBLE_PROPERTY::NOTIFY |
                                               NIMBLE_PROPERTY::READ);
        auto* feat = svc->createCharacteristic(NimBLEUUID((uint16_t)0x2A65),
                                                NIMBLE_PROPERTY::READ);
        // Bit 14: Crank Length Adjustment Supported — makes Cycling Power
        // Control Point (0x2A66) mandatory and prompts fitness clients to ask
        // for crank length during pairing, properly classifying this as a
        // power POD rather than a generic sensor.
        uint32_t featVal = 0x00004000;
        feat->setValue((uint8_t*)&featVal, 4);
        auto* loc = svc->createCharacteristic(NimBLEUUID((uint16_t)0x2A5D),
                                               NIMBLE_PROPERTY::READ);
        uint8_t locVal = 0x00;  // Other
        loc->setValue(&locVal, 1);
        // Cycling Power Control Point: mandatory when bit 14 of CPS Feature is set
        auto* cpcp = svc->createCharacteristic(NimBLEUUID((uint16_t)0x2A66),
                                                NIMBLE_PROPERTY::WRITE |
                                                NIMBLE_PROPERTY::INDICATE);
        cpcp->setCallbacks(new CpControlPointCB());
        svc->start();
    } else if (gMode == MODE_SPEED_CADENCE) {
        // Use explicit 16-bit UUIDs (see CPS comment above).
        auto* svc = pServer->createService(NimBLEUUID((uint16_t)0x1816));
        pCscChar  = svc->createCharacteristic(NimBLEUUID((uint16_t)0x2A5B),
                                               NIMBLE_PROPERTY::NOTIFY |
                                               NIMBLE_PROPERTY::READ);
        auto* feat = svc->createCharacteristic(NimBLEUUID((uint16_t)0x2A5C),
                                                NIMBLE_PROPERTY::READ);
        uint16_t featVal = 0x0003;  // Wheel Revolution Data + Crank Revolution Data
        feat->setValue((uint8_t*)&featVal, 2);
        auto* loc = svc->createCharacteristic(NimBLEUUID((uint16_t)0x2A5D),
                                               NIMBLE_PROPERTY::READ);
        uint8_t locVal = 0x00;
        loc->setValue(&locVal, 1);
        // SC Control Point: mandatory when Wheel Revolution Data is supported
        // (CSC Feature bit 0 = 1). Clients that find it absent consider the
        // sensor non-compliant and disconnect immediately.
        auto* sccp = svc->createCharacteristic(NimBLEUUID((uint16_t)0x2A55),
                                                NIMBLE_PROPERTY::WRITE |
                                                NIMBLE_PROPERTY::INDICATE);
        sccp->setCallbacks(new ScControlPointCB());
        svc->start();
    } else {
        // MODE_POWER_CADENCE: single CPS service with Wheel Revolution Data (bit 4)
        // and Crank Revolution Data (bit 5) packed into the CPS Measurement packet.
        // This is the standard approach of real power meters (Favero Assioma, Garmin
        // Vector): one 0x1818 service carrying power + speed + cadence together, so
        // fitness clients pair it as a power sensor and get all three metrics.
        auto* svc = pServer->createService(NimBLEUUID((uint16_t)0x1818));
        pCpsChar  = svc->createCharacteristic(NimBLEUUID((uint16_t)0x2A63),
                                               NIMBLE_PROPERTY::NOTIFY |
                                               NIMBLE_PROPERTY::READ);
        auto* feat = svc->createCharacteristic(NimBLEUUID((uint16_t)0x2A65),
                                                NIMBLE_PROPERTY::READ);
        // Bits 4+5: Wheel/Crank Revolution Data Supported (enables extended packet)
        // Bit 14:   Crank Length Adjustment Supported (prompts crank length at pairing)
        uint32_t featVal = 0x00004030;
        feat->setValue((uint8_t*)&featVal, 4);
        auto* loc = svc->createCharacteristic(NimBLEUUID((uint16_t)0x2A5D),
                                               NIMBLE_PROPERTY::READ);
        uint8_t locVal = 0x00;
        loc->setValue(&locVal, 1);
        auto* cpcp = svc->createCharacteristic(NimBLEUUID((uint16_t)0x2A66),
                                                NIMBLE_PROPERTY::WRITE |
                                                NIMBLE_PROPERTY::INDICATE);
        cpcp->setCallbacks(new CpControlPointCB());
        svc->start();
    }

    if (gSimEnabled) {
        startAdvertisingForClient();
    } else {
        resetAndAdvertiseForEbike();
    }
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    updateStartupLed();
    updateBridgeBattery();

    if (webOk) {
        webServer.handleClient();
        manageWebPower();
    }

    if (!gSimEnabled) {
        if (flagEbikeDisconn) {
            flagEbikeDisconn = false;
            resetAndAdvertiseForEbike();
            return;
        }
        if (flagEbikePeripheralConn) {
            flagEbikePeripheralConn = false;
            if (openGattClient()) nextGattRetryMs = 0;
            else { ebikeGattReady = false; nextGattRetryMs = millis() + 3000; }
            return;
        }
        if (ebikeConnected && !ebikeGattReady && nextGattRetryMs != 0 &&
            (int32_t)(millis() - nextGattRetryMs) >= 0) {
            nextGattRetryMs = millis() + 3000;
            if (openGattClient()) nextGattRetryMs = 0;
        }
    }

    if (flagClientConn) {
        flagClientConn  = false;
        clientConnected = true;
        nextSimNotifyMs = nextNoEbikeNotifyMs = 0;
    }
    if (flagClientDisconn) {
        flagClientDisconn = false;
        clientConnected   = false;
        nextSimNotifyMs   = nextNoEbikeNotifyMs = 0;
        if (gSimEnabled || ebikeGattReady) startAdvertisingForClient();
    }

    if (!gSimEnabled && clientConnected && !ebikeConnected &&
        (int32_t)(millis() - nextNoEbikeNotifyMs) >= 0) {
        notifyNoEbikeData();
        nextNoEbikeNotifyMs = millis() + 1000;
    }

    if (gSimEnabled && clientConnected &&
        (int32_t)(millis() - nextSimNotifyMs) >= 0) {
        generateAndNotifySimData();
        nextSimNotifyMs = millis() + 500;
    }

    delay(1);
}
