/*
 * BoschEBike Multi-Mode Bridge
 *
 * ESP32 bridge between a Bosch eBike and any BLE fitness client (Suunto,
 * Garmin, Wahoo, apps). Mode is selected from the web UI and stored in NVS.
 *
 * Modes:
 *   1  Suunto Bridge   - transparent LDI proxy for Suunto watch / custom app
 *   2  Power Sensor    - Cycling Power Service (CPS, UUID 0x1818)
 *   3  Speed & Cadence - Cycling Speed & Cadence Service (CSC, UUID 0x1816)
 *   4  Power + Cadence - CPS (0x1818) with wheel + crank data packed in
 *
 * BLE:
 *   GATT Client -> bike (Bosch LDI, same in all modes)
 *   GATT Server -> fitness client (mode-specific service/characteristics)
 *
 * Advertising:
 *   Phase 1: LDI solicitation (AD type 0x15) to attract the Bosch system unit
 *   Phase 2: mode-specific service UUID to attract the fitness client
 *
 * Bosch LDI UUIDs:
 *   Service:  0000eb20-eaa2-11e9-81b4-2a2ae2dbcce4
 *   Char:     0000eb21-eaa2-11e9-81b4-2a2ae2dbcce4  (notify)
 *
 * LDI protobuf fields:
 *   1=speed*100  2=cadence  5=power  9=ambient*1000  10=battery%
 *   12=odometer*1000  17=light  21=locked  22=charger  23=reserve  24=diag  25=standstill
 *
 * NVS namespace "ebike": mode(u8), dname(str), sim(u8), bdbg(u8)
 *
 * Web UI / OTA:
 *   AP "BoschEBike Bridge" / "password"
 *   http://192.168.4.1        dashboard
 *   http://192.168.4.1/update  OTA upload
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

#define FIRMWARE_VERSION "1.0.0"

// Firmware mode
enum FirmwareMode : uint8_t {
    MODE_SUUNTO_BRIDGE  = 1,
    MODE_POWER_SENSOR   = 2,
    MODE_SPEED_CADENCE  = 3,
    MODE_POWER_CADENCE  = 4   // CPS (0x1818) + CSC (0x1816) on the same device
};

// Default mode used when NVS has never been written (first flash).
static const FirmwareMode DEFAULT_MODE = MODE_SUUNTO_BRIDGE;

static FirmwareMode gMode = DEFAULT_MODE;

// Device name
// gBaseDeviceName is stored in NVS (key "dname"); gDeviceName is built at boot
// by appending a mode suffix and used everywhere (BLE init, scan response).
static char gBaseDeviceName[21] = "BoschEBike";  // max 20 chars + null
static char gDeviceName[34]     = {};             // base + longest suffix " PowerCadence"

static void buildDeviceName() {
    const char* suffix =
        gMode == MODE_SUUNTO_BRIDGE ? " Bridge"        :
        gMode == MODE_POWER_SENSOR  ? " Power"         :
        gMode == MODE_SPEED_CADENCE ? " SpeedCadence"  : " PowerCadence";
    snprintf(gDeviceName, sizeof(gDeviceName), "%s%s", gBaseDeviceName, suffix);
}

// Simulation
// When true, synthetic data is generated and advertised immediately without
// connecting to a real bike. Toggled via web UI (/setsim), stored in NVS "sim".
static bool gSimEnabled = false;

// BLE debug log
// Ring buffer in RAM, served as plain text at GET /log.
// Toggled via web UI (/setdebug), stored in NVS "bdbg".
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

// Optional bridge battery sense input.
// ESP32 classic boards such as LOLIN32 Lite often expose a divided VBAT signal
// on GPIO35. ESP32-S3-Zero does not provide this by default, so leave it
// disabled there unless you wire your own divider and override the macro.
#ifndef BRIDGE_BATTERY_ADC_PIN
#define BRIDGE_BATTERY_ADC_PIN 35
#endif
static const int      BATTERY_ADC_PIN            = BRIDGE_BATTERY_ADC_PIN;
static const uint32_t BATTERY_SAMPLE_INTERVAL_MS = 5000;

#ifdef LED_BUILTIN
static const int STARTUP_LED_PIN = LED_BUILTIN;
#else
static const int STARTUP_LED_PIN = 2;
#endif
#ifndef STARTUP_LED_ACTIVE_LOW
#define STARTUP_LED_ACTIVE_LOW 1
#endif
static const bool     STARTUP_LED_IS_ACTIVE_LOW = STARTUP_LED_ACTIVE_LOW != 0;
static const uint32_t STARTUP_LED_ON_MS      = 2000;
static uint32_t startupLedOffMs  = 0;
static bool     startupLedActive = false;

static void setStartupLed(bool on) {
    digitalWrite(STARTUP_LED_PIN, STARTUP_LED_IS_ACTIVE_LOW ? !on : on);
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

// Live data
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

// Protobuf decoder
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

// Raw field capture for reverse engineering.
// Captures every protobuf field seen in real LDI packets from the bike.
// Only called from handleLdiPayload() — not triggered by sim or no-ebike data.
static const int RE_MAX_FIELD = 128;

struct ReField {
    bool     seen      = false;
    uint8_t  wireType  = 0;      // 0=varint, 2=length-delimited
    uint64_t value     = 0;      // decoded value (varint) or byte count (len-delim)
    uint64_t prevValue = 0;
    bool     changed   = false;  // true if value differed from previous packet
    uint32_t seenCount = 0;
    uint8_t  bytes[16] = {};     // first 16 raw bytes of length-delimited fields
    uint8_t  byteLen   = 0;
};

static ReField  reFields[RE_MAX_FIELD];
static uint32_t rePacketCount  = 0;
static uint32_t reLastPacketMs = 0;

static void captureRawFields(const uint8_t* data, size_t len) {
    for (int i = 0; i < RE_MAX_FIELD; i++) reFields[i].changed = false;
    int pos = 0;
    while (pos < (int)len) {
        auto tag = readVarint(data, pos, len); pos = tag.p;
        int fn = (int)(tag.v >> 3), wt = (int)(tag.v & 7);
        if (wt == 0) {
            auto v = readVarint(data, pos, len); pos = v.p;
            if (fn > 0 && fn < RE_MAX_FIELD) {
                ReField& rf  = reFields[fn];
                rf.changed   = rf.seen && (rf.value != v.v);
                rf.prevValue = rf.value;
                rf.value     = v.v;
                rf.seen      = true;
                rf.wireType  = 0;
                rf.seenCount++;
            }
        } else if (wt == 2) {
            auto sl = readVarint(data, pos, len);
            if (fn > 0 && fn < RE_MAX_FIELD) {
                ReField& rf  = reFields[fn];
                rf.seen      = true;
                rf.wireType  = 2;
                rf.value     = sl.v;
                rf.byteLen   = (uint8_t)min((size_t)16, (size_t)sl.v);
                memcpy(rf.bytes, data + sl.p, rf.byteLen);
                rf.changed   = true;
                rf.seenCount++;
            }
            pos = sl.p + (int)sl.v;
        } else break;
    }
    rePacketCount++;
    reLastPacketMs = millis();
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

// Protobuf encoder (simulation / no-ebike packets)
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
    if (BATTERY_ADC_PIN < 0) {
        bridgeBatteryMv = 0;
        bridgeBatteryPercent = 0;
        return;
    }
    uint32_t now = millis();
    if (!force && (int32_t)(now - nextBatterySampleMs) < 0) return;
    nextBatterySampleMs = now + BATTERY_SAMPLE_INTERVAL_MS;
    uint32_t accMv = 0;
    for (int i = 0; i < 8; ++i) { accMv += analogReadMilliVolts(BATTERY_ADC_PIN); delay(1); }
    bridgeBatteryMv      = (uint16_t)((accMv / 8) * 2);
    bridgeBatteryPercent = lipoPercentFromMv(bridgeBatteryMv);
}

// CPS state
// Crank length in 1/10 mm (1725 = 172.5 mm). Updated via CPCP opcode 0x04;
// stored and reported back but has no effect on power (that comes from the bike).
static uint16_t cpsCrankLengthTenthMm = 1725;
// Wheel event timestamp for MODE_POWER_CADENCE (1/2048 s, differs from CSC's 1/1024 s).
static uint16_t cpsCombWheelEventT = 0;

// CSC accumulator state
static float    cscWheelRevFrac    = 0.0f;
static uint32_t cscWheelRevTotal   = 0;
static uint16_t cscLastWheelEventT = 0;
static float    cscCrankRevFrac    = 0.0f;
static uint16_t cscCrankRevTotal   = 0;
static uint16_t cscLastCrankEventT = 0;
static uint32_t cscLastUpdateMs    = 0;

// BLE server characteristics (one per mode, only one non-null at runtime)
static NimBLEServer*         pServer  = nullptr;
static NimBLECharacteristic* pLdiChar = nullptr;  // MODE_SUUNTO_BRIDGE
static NimBLECharacteristic* pCpsChar = nullptr;  // MODE_POWER_SENSOR
static NimBLECharacteristic* pCscChar = nullptr;  // MODE_SPEED_CADENCE

// BLE state
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

// Notify helpers
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

// CPS measurement with power + wheel + crank packed together (flags 0x0030).
// Layout (14 bytes): flags(2LE) power(2LE) cum_wheel(4LE) wheel_evt(2LE,1/2048s)
//                   cum_crank(2LE) crank_evt(2LE,1/1024s)
// Cadence is derived by the client from successive crank revolution deltas.
static void notifyCpsCombinedData() {
    if (!clientConnected || !pCpsChar) return;

    uint32_t now = millis();
    uint32_t dt  = cscLastUpdateMs ? (now - cscLastUpdateMs) : 0;
    cscLastUpdateMs = now;

    if (dt > 0 && dt < 5000) {
        float dtSec = dt / 1000.0f;
        if (gData.speedKmh > 0.1f) {
            float revPerSec = (gData.speedKmh / 3.6f) / (WHEEL_CIRCUMFERENCE_MM / 1000.0f);
            cscWheelRevFrac += revPerSec * dtSec;
            uint32_t newRevs = (uint32_t)cscWheelRevFrac;
            if (newRevs > 0) {
                cscWheelRevFrac  -= newRevs;
                cscWheelRevTotal += newRevs;
                // Interpolate: the last revolution completed (frac/revPerSec) seconds ago.
                uint32_t eventMs = now - (uint32_t)(cscWheelRevFrac / revPerSec * 1000.0f);
                cpsCombWheelEventT = (uint16_t)(((uint64_t)eventMs * 2048) / 1000);
            }
        }
    }

    if (gData.cadenceRpm > 0) {
        uint32_t crankPeriodTicks = (61440UL + ((uint32_t)gData.cadenceRpm / 2)) /
                                    (uint32_t)gData.cadenceRpm;
        if (crankPeriodTicks == 0) crankPeriodTicks = 1;
        cscCrankRevTotal++;
        cscLastCrankEventT = (uint16_t)(cscLastCrankEventT + (uint16_t)crankPeriodTicks);
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
            float revPerSec = (gData.speedKmh / 3.6f) / (WHEEL_CIRCUMFERENCE_MM / 1000.0f);
            cscWheelRevFrac += revPerSec * dtSec;
            uint32_t newRevs = (uint32_t)cscWheelRevFrac;
            if (newRevs > 0) {
                cscWheelRevFrac  -= newRevs;
                cscWheelRevTotal += newRevs;
                uint32_t eventMs = now - (uint32_t)(cscWheelRevFrac / revPerSec * 1000.0f);
                cscLastWheelEventT = (uint16_t)(((uint64_t)eventMs * 1024) / 1000);
            }
        }

        if (gData.cadenceRpm > 0) {
            float crankRevPerSec = gData.cadenceRpm / 60.0f;
            cscCrankRevFrac += crankRevPerSec * dtSec;
            uint16_t newRevs = (uint16_t)cscCrankRevFrac;
            if (newRevs > 0) {
                cscCrankRevFrac  -= newRevs;
                cscCrankRevTotal += newRevs;
                uint32_t eventMs = now - (uint32_t)(cscCrankRevFrac / crankRevPerSec * 1000.0f);
                cscLastCrankEventT = (uint16_t)(((uint64_t)eventMs * 1024) / 1000);
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
    captureRawFields(data, len);
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

// Simulation / no-ebike data
static void generateAndNotifySimData() {
    uint32_t t = millis();
    gData.speedKmh   = triWave(t, 20000, 1500, 3500) / 100.0f;
    gData.cadenceRpm = triWave(t, 15000,    0,  120);
    gData.powerW     = triWave(t, 12000,    0,  500);
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

// GATT client callbacks (bike side)
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

// BLE server callbacks
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

// BLE advertising
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

// Cycling Power Control Point callbacks (CPS mode)
// Mandatory when CPS Feature bit 14 is set. Handles Set/Request Crank Length
// (opcodes 0x04/0x05); everything else gets "Op Code Not Supported" (0x02).
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

// SC Control Point callbacks (CSC mode)
// Mandatory when Wheel Revolution Data is supported (CSC Feature bit 0).
// Opcode 0x01 resets the wheel rev counter; all others get "Not Supported".
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

// Web server
static WebServer webServer(80);
static bool     webOk             = false;
static bool     wifiActive        = false;
static bool     webHadClient      = false;
static uint32_t webStartedMs      = 0;
static uint32_t lastWebClientSeenMs = 0;

#ifndef WEB_BOOT_WINDOW_MS
#define WEB_BOOT_WINDOW_MS 60000
#endif
#ifndef WEB_IDLE_AFTER_CLIENT_MS
#define WEB_IDLE_AFTER_CLIENT_MS 60000
#endif
static const uint32_t WEB_BOOT_WINDOW_TIMEOUT_MS = WEB_BOOT_WINDOW_MS;
static const uint32_t WEB_IDLE_AFTER_CLIENT_TIMEOUT_MS = WEB_IDLE_AFTER_CLIENT_MS;

static const char INDEX_HTML[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BoschEBike Bridge</title>
<link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><text y='.9em' font-size='90'>&#x26A1;</text></svg>">
<style>
:root{--bg:#0d1117;--sf:#161b22;--sf2:#1c2128;--bd:#30363d;--tx:#e6edf3;--mu:#8b949e;--bl:#58a6ff;--gn:#3fb950;--ye:#d29922;--pu:#a371f7;--re:#f85149}
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--tx);font-family:system-ui,-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;font-size:14px;line-height:1.5;padding:16px}
.wrap{max-width:520px}
/* header */
.hdr{display:flex;align-items:flex-start;justify-content:space-between;gap:8px;margin-bottom:12px;flex-wrap:wrap}
.hdr-l h1{font-size:18px;font-weight:600;color:var(--bl);line-height:1.2}
.hdr-l .ver{font-size:11px;color:var(--mu);margin-top:1px}
.hdr-r{display:flex;flex-direction:column;align-items:flex-end;gap:5px}
/* status pill */
.st-pill{display:flex;align-items:center;gap:6px;background:var(--sf);border:1px solid var(--bd);border-radius:20px;padding:4px 12px;font-size:12px;white-space:nowrap}
.st-dot{width:8px;height:8px;border-radius:50%;background:var(--mu);flex-shrink:0;transition:background .3s}
.st-dot.gn{background:var(--gn);box-shadow:0 0 6px var(--gn)}
.st-dot.ye{background:var(--ye);box-shadow:0 0 6px var(--ye)}
.st-dot.pu{background:var(--pu);box-shadow:0 0 6px var(--pu)}
/* lang switcher */
.lang-sw{display:flex;gap:2px;background:var(--sf);border:1px solid var(--bd);border-radius:6px;padding:2px}
.lang-btn{background:transparent;border:none;border-radius:4px;color:var(--mu);cursor:pointer;font:11px/1 system-ui,-apple-system,sans-serif;padding:3px 7px;transition:background .15s,color .15s}
.lang-btn:hover:not(.act){background:var(--bd);color:var(--tx)}
.lang-btn.act{background:var(--bl);color:#fff}
/* sections */
.sec{background:var(--sf);border:1px solid var(--bd);border-radius:10px;padding:14px;margin-bottom:10px}
.sec-t{font-size:10px;font-weight:600;letter-spacing:.8px;text-transform:uppercase;color:var(--mu);margin-bottom:10px}
/* sensor grid */
.sgrid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.s-lbl{font-size:10px;font-weight:600;letter-spacing:.5px;text-transform:uppercase;color:var(--mu);margin-bottom:2px}
.s-val{font-family:ui-monospace,'Cascadia Code',Consolas,monospace;font-size:26px;font-weight:700;line-height:1;color:var(--tx)}
.s-unit{font-size:12px;color:var(--mu);margin-left:2px}
hr.div{border:none;border-top:1px solid var(--bd);margin:10px 0}
/* flags */
.flags{display:flex;flex-wrap:wrap;gap:5px}
.fl{display:inline-flex;align-items:center;padding:3px 10px;border-radius:20px;font-size:12px;border:1px solid transparent;transition:background .2s,color .2s}
.fl.on{background:#1f6feb33;border-color:var(--bl);color:var(--bl)}
.fl.ye{background:#d2992233;border-color:var(--ye);color:var(--ye)}
.fl.off{background:transparent;border-color:var(--bd);color:var(--mu)}
/* mode buttons */
.mbtns{display:flex;flex-wrap:wrap;gap:6px}
.mbtn{background:var(--sf2);color:var(--tx);border:1px solid var(--bd);border-radius:6px;padding:7px 14px;cursor:pointer;font:13px/1 system-ui,-apple-system,sans-serif;transition:background .15s,border-color .15s}
.mbtn:hover{background:var(--bd)}
.mbtn.act{background:#1f6feb33;border-color:var(--bl);color:var(--bl)}
.m-desc{font-size:11px;color:var(--mu);margin-top:7px;min-height:14px}
/* inputs */
.irow{display:flex;gap:6px}
.irow input{flex:1;background:var(--bg);color:var(--tx);border:1px solid var(--bd);border-radius:6px;padding:7px 10px;font:13px/1 system-ui,-apple-system,sans-serif;min-width:0}
.irow input:focus{outline:none;border-color:var(--bl)}
/* buttons */
.btn{background:var(--sf2);color:var(--tx);border:1px solid var(--bd);border-radius:6px;padding:7px 14px;cursor:pointer;font:12px/1 system-ui,-apple-system,sans-serif;white-space:nowrap;text-decoration:none;display:inline-block}
.btn:hover{background:var(--bd)}
/* toggle pair */
.tpair{display:flex;gap:6px}
.tbtn{flex:1;background:var(--sf2);color:var(--mu);border:1px solid var(--bd);border-radius:6px;padding:7px;cursor:pointer;font:12px/1 system-ui,-apple-system,sans-serif;text-align:center;transition:background .15s,color .15s}
.tbtn:hover:not(.act){background:var(--bd);color:var(--tx)}
.tbtn.act{background:#1f6feb33;border-color:var(--bl);color:var(--bl)}
/* status/feedback lines */
.sl{font-size:11px;color:var(--mu);margin-top:6px;min-height:16px}
.nm-prev{font-size:11px;color:var(--bl);margin-top:4px;min-height:16px}
/* footer */
.ftr{margin-top:14px;display:flex;justify-content:space-between;align-items:center}
.ftr a{color:var(--bl);text-decoration:none;font-size:12px}
.ftr a:hover{text-decoration:underline}
#ts{font-size:11px;color:var(--bd)}
/* error banner */
#errB{display:none;background:#f8514933;border:1px solid var(--re);border-radius:8px;padding:8px 12px;color:var(--re);font-size:12px;margin-bottom:10px}
</style></head><body>
<div class="wrap">
<div id="errB"></div>
<div class="hdr">
  <div class="hdr-l">
    <h1>BoschEBike Bridge</h1>
    <div class="ver" id="verLine">&#x2014;</div>
  </div>
  <div class="hdr-r">
    <div class="st-pill"><div class="st-dot" id="stDot"></div><span id="stTxt">&#x2014;</span></div>
    <div class="lang-sw">
      <button class="lang-btn" data-lang="en" onclick="setLang('en')">EN</button>
      <button class="lang-btn" data-lang="it" onclick="setLang('it')">IT</button>
      <button class="lang-btn" data-lang="de" onclick="setLang('de')">DE</button>
      <button class="lang-btn" data-lang="fr" onclick="setLang('fr')">FR</button>
      <button class="lang-btn" data-lang="es" onclick="setLang('es')">ES</button>
    </div>
  </div>
</div>
<div class="sec">
  <div class="sec-t" data-i18n="liveData">Live Data</div>
  <div class="sgrid">
    <div><div class="s-lbl" data-i18n="speed">Speed</div><span class="s-val" id="spd">&#x2014;</span><span class="s-unit">km/h</span></div>
    <div><div class="s-lbl" data-i18n="cadence">Cadence</div><span class="s-val" id="cad">&#x2014;</span><span class="s-unit">rpm</span></div>
    <div><div class="s-lbl" data-i18n="power">Power</div><span class="s-val" id="pwr" style="color:var(--gn)">&#x2014;</span><span class="s-unit">W</span></div>
    <div><div class="s-lbl" data-i18n="bikeBat">Bike Battery</div><span class="s-val" id="bat" style="color:var(--bl)">&#x2014;</span><span class="s-unit">%</span></div>
    <div><div class="s-lbl" data-i18n="bridgeBat">Bridge Battery</div><span class="s-val" id="bbat" style="color:var(--ye)">&#x2014;</span><span class="s-unit">%</span></div>
    <div><div class="s-lbl" data-i18n="odometer">Odometer</div><span class="s-val" id="odo">&#x2014;</span><span class="s-unit">km</span></div>
    <div><div class="s-lbl" data-i18n="ambient">Ambient Light</div><span class="s-val" id="lux" style="color:var(--ye)">&#x2014;</span><span class="s-unit">lux</span></div>
  </div>
  <hr class="div">
  <div class="flags">
    <span class="fl off" id="fl_lt">&#x2014;</span>
    <span class="fl off" id="fl_lk">&#x2014;</span>
    <span class="fl off" id="fl_ch">&#x2014;</span>
    <span class="fl off" id="fl_rv">&#x2014;</span>
    <span class="fl off" id="fl_dg">&#x2014;</span>
    <span class="fl off" id="fl_st">&#x2014;</span>
    <span class="fl off" id="fl_cl">&#x2014;</span>
  </div>
</div>
<div class="sec">
  <div class="sec-t" data-i18n="fwMode">Firmware Mode</div>
  <div class="mbtns">
    <button class="mbtn" id="mb1" onclick="setMode(1)">Suunto Bridge</button>
    <button class="mbtn" id="mb2" onclick="setMode(2)">Power Sensor</button>
    <button class="mbtn" id="mb3" onclick="setMode(3)">Speed &amp; Cadence</button>
    <button class="mbtn" id="mb4" onclick="setMode(4)">Power + Cadence</button>
  </div>
  <div class="m-desc" id="mDesc"></div>
  <div class="sl" id="mst"></div>
</div>
<div class="sec">
  <div class="sec-t" data-i18n="devName">Device Name</div>
  <div class="irow">
    <input type="text" id="dname" maxlength="20" placeholder="BoschEBike" autocomplete="off" spellcheck="false">
    <button class="btn" onclick="saveName()" id="saveBtn">Save</button>
  </div>
  <div class="nm-prev" id="dnprev"></div>
  <div class="sl" id="dnst"></div>
</div>
<div class="sec">
  <div class="sec-t" data-i18n="dataSrc">Data Source</div>
  <div class="tpair">
    <button class="tbtn" id="sb0" onclick="setSim(0)" data-i18n="realEbike">Real eBike</button>
    <button class="tbtn" id="sb1" onclick="setSim(1)" data-i18n="simulation">Simulation</button>
  </div>
  <div class="sl" id="sst"></div>
</div>
<div class="sec">
  <div class="sec-t" data-i18n="bleDebug">BLE Debug Log</div>
  <div class="tpair" style="margin-bottom:8px">
    <button class="tbtn" id="db0" onclick="setDebug(0)" data-i18n="disabled">Disabled</button>
    <button class="tbtn" id="db1" onclick="setDebug(1)" data-i18n="enabled">Enabled</button>
  </div>
  <div style="display:flex;gap:8px">
    <a href="/log" target="_blank" class="btn" id="viewLogBtn">View log &#x2197;</a>
    <button class="btn" onclick="clearLog()" id="clearBtn">Clear</button>
  </div>
  <div class="sl" id="dbst"></div>
</div>
<div class="ftr"><span id="ts"></span><div style="display:flex;gap:14px"><a href="/explorer">Field Explorer &#x2197;</a><a href="/update" id="otaLink">Firmware update &#x2197;</a></div></div>
</div>
<script>
const T={
en:{liveData:'Live Data',speed:'Speed',cadence:'Cadence',power:'Power',bikeBat:'Bike Battery',bridgeBat:'Bridge Battery',odometer:'Odometer',ambient:'Ambient Light',fwMode:'Firmware Mode',devName:'Device Name',saveBtn:'Save',dataSrc:'Data Source',realEbike:'Real eBike',simulation:'Simulation',bleDebug:'BLE Debug Log',disabled:'Disabled',enabled:'Enabled',viewLog:'View log ↗',clearBtn:'Clear',otaLink:'Firmware update ↗',loading:'—',connecting:'Connecting…',waitBike:'Waiting for eBike…',gattInit:'eBike connected · Initializing…',readyWait:'Ready · Waiting for client',readyConn:'Ready · Client connected',simWait:'Simulation · Waiting for client',simConn:'Simulation · Client connected',connLost:'Connection lost',connRetry:'Connection lost — retrying…',saving:'Saving…',errConn:'Error — check connection',nameEmpty:'Name cannot be empty',bleName:'BLE name: ',updated:'Updated ',mDesc:['','Transparent LDI proxy for Suunto watch app','Standard Cycling Power Service (CPS/BLE)','Standard Speed & Cadence Service (CSC/BLE)','Power (CPS) and Speed & Cadence (CSC) combined'],lightOff:'Light: OFF',lightOn:'Light: ON',lightDash:'Light: —',flagLocked:'Locked',flagCharger:'Charger',flagReserve:'Reserve',flagDiag:'Diag',flagStat:'Stationary',flagClient:'Client connected',confirmMode:(n)=>`Switch to ${n} mode?\n\nThe bridge will reboot to apply the change.`,confirmName:(n)=>`Rename device to "${n}"?\n\nThe bridge will reboot to apply the change.`,confirmSimOn:'Switch to simulation mode?\n\nThe bridge will use simulated data instead of the real eBike. It will reboot to apply.',confirmSimOff:'Switch to real eBike mode?\n\nThe bridge will use live data from the bike. It will reboot to apply.'},
it:{liveData:'Dati in tempo reale',speed:'Velocità',cadence:'Cadenza',power:'Potenza',bikeBat:'Batteria bici',bridgeBat:'Batteria bridge',odometer:'Contachilometri',ambient:'Luce ambiente',fwMode:'Modalità firmware',devName:'Nome dispositivo',saveBtn:'Salva',dataSrc:'Sorgente dati',realEbike:'eBike reale',simulation:'Simulazione',bleDebug:'Log debug BLE',disabled:'Disattivato',enabled:'Attivato',viewLog:'Visualizza log ↗',clearBtn:'Cancella',otaLink:'Aggiornamento firmware ↗',loading:'—',connecting:'Connessione…',waitBike:'In attesa di eBike…',gattInit:'eBike connessa · Inizializzazione…',readyWait:'Pronta · In attesa del client',readyConn:'Pronta · Client connesso',simWait:'Simulazione · In attesa del client',simConn:'Simulazione · Client connesso',connLost:'Connessione persa',connRetry:'Connessione persa — nuovo tentativo…',saving:'Salvataggio…',errConn:'Errore — verifica la connessione',nameEmpty:'Il nome non può essere vuoto',bleName:'Nome BLE: ',updated:'Aggiornato alle ',mDesc:['','Proxy LDI trasparente per app Suunto','Servizio standard Cycling Power (CPS/BLE)','Servizio standard Speed & Cadence (CSC/BLE)','Power (CPS) e Speed & Cadence (CSC) combinati'],lightOff:'Luce: OFF',lightOn:'Luce: ON',lightDash:'Luce: —',flagLocked:'Bloccato',flagCharger:'Caricatore',flagReserve:'Riserva',flagDiag:'Diagnosi',flagStat:'Fermo',flagClient:'Client connesso',confirmMode:(n)=>`Passare alla modalità ${n}?\n\nIl bridge si riavvierà per applicare la modifica.`,confirmName:(n)=>`Rinominare il dispositivo in "${n}"?\n\nIl bridge si riavvierà per applicare la modifica.`,confirmSimOn:'Attivare la simulazione?\n\nIl bridge userà dati simulati. Si riavvierà per applicare.',confirmSimOff:'Tornare alla modalità eBike reale?\n\nIl bridge userà i dati live dalla bici. Si riavvierà per applicare.'},
de:{liveData:'Live-Daten',speed:'Geschwindigkeit',cadence:'Trittfrequenz',power:'Leistung',bikeBat:'Akku Fahrrad',bridgeBat:'Akku Bridge',odometer:'Kilometerstand',ambient:'Umgebungslicht',fwMode:'Firmware-Modus',devName:'Gerätename',saveBtn:'Speichern',dataSrc:'Datenquelle',realEbike:'Echtes eBike',simulation:'Simulation',bleDebug:'BLE-Debug-Log',disabled:'Deaktiviert',enabled:'Aktiviert',viewLog:'Log anzeigen ↗',clearBtn:'Löschen',otaLink:'Firmware-Update ↗',loading:'—',connecting:'Verbinden…',waitBike:'Warte auf eBike…',gattInit:'eBike verbunden · Initialisierung…',readyWait:'Bereit · Warte auf Client',readyConn:'Bereit · Client verbunden',simWait:'Simulation · Warte auf Client',simConn:'Simulation · Client verbunden',connLost:'Verbindung verloren',connRetry:'Verbindung verloren — erneuter Versuch…',saving:'Speichern…',errConn:'Fehler — Verbindung prüfen',nameEmpty:'Name darf nicht leer sein',bleName:'BLE-Name: ',updated:'Aktualisiert ',mDesc:['','Transparenter LDI-Proxy für Suunto-App','Standard Cycling Power Service (CPS/BLE)','Standard Speed & Cadence Service (CSC/BLE)','Power (CPS) und Speed & Cadence (CSC) kombiniert'],lightOff:'Licht: AUS',lightOn:'Licht: EIN',lightDash:'Licht: —',flagLocked:'Gesperrt',flagCharger:'Ladegerät',flagReserve:'Reserve',flagDiag:'Diagnose',flagStat:'Stillstand',flagClient:'Client verbunden',confirmMode:(n)=>`Zu Modus ${n} wechseln?\n\nDie Bridge wird neu gestartet, um die Änderung anzuwenden.`,confirmName:(n)=>`Gerät in "${n}" umbenennen?\n\nDie Bridge wird neu gestartet, um die Änderung anzuwenden.`,confirmSimOn:'Simulationsmodus aktivieren?\n\nDie Bridge verwendet simulierte Daten. Sie startet neu, um die Änderung anzuwenden.',confirmSimOff:'Zurück zum echten eBike-Modus?\n\nDie Bridge verwendet Live-Daten vom Fahrrad. Sie startet neu.'},
fr:{liveData:'Données en direct',speed:'Vitesse',cadence:'Cadence',power:'Puissance',bikeBat:'Batterie vélo',bridgeBat:'Batterie bridge',odometer:'Compteur km',ambient:'Lumière ambiante',fwMode:'Mode firmware',devName:'Nom du dispositif',saveBtn:'Enregistrer',dataSrc:'Source de données',realEbike:'eBike réel',simulation:'Simulation',bleDebug:'Journal debug BLE',disabled:'Désactivé',enabled:'Activé',viewLog:'Voir le journal ↗',clearBtn:'Effacer',otaLink:'Mise à jour firmware ↗',loading:'—',connecting:'Connexion…',waitBike:'En attente de l’eBike…',gattInit:'eBike connecté · Initialisation…',readyWait:'Prêt · En attente du client',readyConn:'Prêt · Client connecté',simWait:'Simulation · En attente du client',simConn:'Simulation · Client connecté',connLost:'Connexion perdue',connRetry:'Connexion perdue — nouvelle tentative…',saving:'Enregistrement…',errConn:'Erreur — vérifier la connexion',nameEmpty:'Le nom ne peut pas être vide',bleName:'Nom BLE : ',updated:'Mis à jour à ',mDesc:['','Proxy LDI transparent pour app Suunto','Service standard Cycling Power (CPS/BLE)','Service standard Speed & Cadence (CSC/BLE)','Power (CPS) et Speed & Cadence (CSC) combinés'],lightOff:'Lumière : OFF',lightOn:'Lumière : ON',lightDash:'Lumière : —',flagLocked:'Verrouillé',flagCharger:'Chargeur',flagReserve:'Réserve',flagDiag:'Diagnostic',flagStat:'À l’arrêt',flagClient:'Client connecté',confirmMode:(n)=>`Passer en mode ${n} ?\n\nLe bridge va redémarrer pour appliquer le changement.`,confirmName:(n)=>`Renommer le dispositif en « ${n} » ?\n\nLe bridge va redémarrer pour appliquer le changement.`,confirmSimOn:'Activer la simulation ?\n\nLe bridge utilisera des données simulées. Il redémarrera pour appliquer.',confirmSimOff:'Revenir au mode eBike réel ?\n\nLe bridge utilisera les données en direct. Il redémarrera pour appliquer.'},
es:{liveData:'Datos en vivo',speed:'Velocidad',cadence:'Cadencia',power:'Potencia',bikeBat:'Batería bici',bridgeBat:'Batería bridge',odometer:'Odómetro',ambient:'Luz ambiental',fwMode:'Modo de firmware',devName:'Nombre del dispositivo',saveBtn:'Guardar',dataSrc:'Fuente de datos',realEbike:'eBike real',simulation:'Simulación',bleDebug:'Log de debug BLE',disabled:'Desactivado',enabled:'Activado',viewLog:'Ver log ↗',clearBtn:'Limpiar',otaLink:'Actualizar firmware ↗',loading:'—',connecting:'Conectando…',waitBike:'Esperando eBike…',gattInit:'eBike conectada · Inicializando…',readyWait:'Lista · Esperando cliente',readyConn:'Lista · Cliente conectado',simWait:'Simulación · Esperando cliente',simConn:'Simulación · Cliente conectado',connLost:'Conexión perdida',connRetry:'Conexión perdida — reintentando…',saving:'Guardando…',errConn:'Error — verifica la conexión',nameEmpty:'El nombre no puede estar vacío',bleName:'Nombre BLE: ',updated:'Actualizado a las ',mDesc:['','Proxy LDI transparente para app Suunto','Servicio estándar Cycling Power (CPS/BLE)','Servicio estándar Speed & Cadence (CSC/BLE)','Power (CPS) y Speed & Cadence (CSC) combinados'],lightOff:'Luz: OFF',lightOn:'Luz: ON',lightDash:'Luz: —',flagLocked:'Bloqueado',flagCharger:'Cargador',flagReserve:'Reserva',flagDiag:'Diagnóstico',flagStat:'Parado',flagClient:'Cliente conectado',confirmMode:(n)=>`¿Cambiar al modo ${n}?\n\nEl bridge se reiniciará para aplicar el cambio.`,confirmName:(n)=>`¿Renombrar el dispositivo a "${n}"?\n\nEl bridge se reiniciará para aplicar el cambio.`,confirmSimOn:'¿Activar la simulación?\n\nEl bridge usará datos simulados. Se reiniciará para aplicar.',confirmSimOff:'¿Volver al modo eBike real?\n\nEl bridge usará datos en vivo de la bici. Se reiniciará para aplicar.'}
};
const MN=['','Suunto Bridge','Power Sensor','Speed & Cadence','Power + Cadence'];
const MS=['',' Bridge',' Power',' SpeedCadence',' PowerCadence'];
let busy=false,errCnt=0,curMode=1,nameFoc=false;
let L=localStorage.getItem('lang')||'en';
if(!T[L])L='en';

function t(k){return T[L][k]||k;}

function setLang(l){
  L=l;localStorage.setItem('lang',l);
  document.querySelectorAll('[data-i18n]').forEach(el=>{
    const k=el.dataset.i18n;
    if(T[L][k]!==undefined)el.textContent=T[L][k];
  });
  document.getElementById('saveBtn').textContent=t('saveBtn');
  document.getElementById('viewLogBtn').textContent=t('viewLog');
  document.getElementById('clearBtn').textContent=t('clearBtn');
  document.getElementById('otaLink').textContent=t('otaLink');
  document.querySelectorAll('.lang-btn').forEach(b=>b.classList.toggle('act',b.dataset.lang===l));
  updateFlags(lastD);
  if(lastD)document.getElementById('mDesc').textContent=t('mDesc')[lastD.mode]||'';
}

const dnEl=document.getElementById('dname');
dnEl.addEventListener('focus',()=>{nameFoc=true;});
dnEl.addEventListener('blur',()=>{nameFoc=false;});
dnEl.addEventListener('input',function(){
  const sfx=MS[curMode]||'';
  document.getElementById('dnprev').textContent=this.value?t('bleName')+this.value+sfx:'';
});

function setDot(cls,txt){
  const d=document.getElementById('stDot');
  d.className='st-dot'+(cls?' '+cls:'');
  document.getElementById('stTxt').textContent=txt;
}

function flagEl(id,state,txt){
  const e=document.getElementById(id);
  e.textContent=txt;
  e.className='fl '+(state==='ye'?'ye':state?'on':'off');
}

let lastD=null;
function updateFlags(d){
  if(!d)return;
  flagEl('fl_lt',d.bike_light===2?true:(d.bike_light===1?'ye':false),d.bike_light===2?t('lightOn'):(d.bike_light===1?t('lightOff'):t('lightDash')));
  flagEl('fl_lk',d.locked,t('flagLocked'));
  flagEl('fl_ch',d.charger,t('flagCharger'));
  flagEl('fl_rv',d.reserve,t('flagReserve'));
  flagEl('fl_dg',d.diag,t('flagDiag'));
  flagEl('fl_st',d.standstill,t('flagStat'));
  flagEl('fl_cl',d.client,t('flagClient'));
}

function api(url,stId){
  document.getElementById(stId).textContent=t('saving');
  fetch(url,{cache:'no-store'})
    .then(r=>r.text()).then(tx=>{document.getElementById(stId).textContent=tx;})
    .catch(()=>{document.getElementById(stId).textContent=t('errConn');});
}

function setMode(m){
  if(!confirm(t('confirmMode')(MN[m])))return;
  api('/setmode?mode='+m,'mst');
}

function saveName(){
  const n=dnEl.value.trim();
  if(!n){document.getElementById('dnst').textContent=t('nameEmpty');return;}
  const full=n+(MS[curMode]||'');
  if(!confirm(t('confirmName')(full)))return;
  api('/setname?name='+encodeURIComponent(n),'dnst');
}

function setSim(s){
  if(!confirm(s?t('confirmSimOn'):t('confirmSimOff')))return;
  api('/setsim?sim='+s,'sst');
}

function setDebug(d){api('/setdebug?debug='+d,'dbst');}

function clearLog(){
  fetch('/clearlog',{cache:'no-store'})
    .then(r=>r.text()).then(tx=>{document.getElementById('dbst').textContent=tx;})
    .catch(()=>{document.getElementById('dbst').textContent=t('errConn');});
}

function pollData(){
  if(busy)return;busy=true;
  fetch('/data',{cache:'no-store'}).then(r=>r.json()).then(d=>{
    lastD=d;errCnt=0;
    document.getElementById('errB').style.display='none';
    curMode=d.mode;
    if(d.sim)setDot('pu',d.client?t('simConn'):t('simWait'));
    else if(!d.ebike)setDot('ye',t('waitBike'));
    else if(!d.gatt)setDot('ye',t('gattInit'));
    else setDot('gn',d.client?t('readyConn'):t('readyWait'));
    const v=d.valid;
    document.getElementById('spd').textContent=v?d.speed.toFixed(1):'—';
    document.getElementById('cad').textContent=v?d.cadence:'—';
    document.getElementById('pwr').textContent=v?d.power:'—';
    document.getElementById('bat').textContent=v?d.battery:'—';
    document.getElementById('bbat').textContent=d.bridge_battery;
    document.getElementById('odo').textContent=v?d.odometer.toFixed(1):'—';
    document.getElementById('lux').textContent=v?d.ambient.toFixed(0):'—';
    updateFlags(d);
    document.getElementById('mDesc').textContent=t('mDesc')[d.mode]||'';
    for(let i=1;i<=4;i++)document.getElementById('mb'+i).className='mbtn'+(d.mode===i?' act':'');
    document.getElementById('sb0').className='tbtn'+(d.sim?'':' act');
    document.getElementById('sb1').className='tbtn'+(d.sim?' act':'');
    document.getElementById('db0').className='tbtn'+(d.debug?'':' act');
    document.getElementById('db1').className='tbtn'+(d.debug?' act':'');
    if(!nameFoc){
      dnEl.value=d.base_name||'';
      document.getElementById('dnprev').textContent=t('bleName')+d.device_name;
    }
    document.getElementById('verLine').textContent='v'+(d.version||'—')+' · '+MN[d.mode];
    document.getElementById('ts').textContent=t('updated')+new Date().toLocaleTimeString();
  }).catch(()=>{
    errCnt++;
    if(errCnt>=3){setDot('',t('connLost'));document.getElementById('errB').style.display='block';document.getElementById('errB').textContent=t('connRetry');}
  }).finally(()=>{busy=false;});
}

setLang(L);
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
<title>Firmware Update — BoschEBike Bridge</title>
<link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><text y='.9em' font-size='90'>&#x26A1;</text></svg>">
<style>
:root{--bg:#0d1117;--sf:#161b22;--sf2:#1c2128;--bd:#30363d;--tx:#e6edf3;--mu:#8b949e;--bl:#58a6ff;--gn:#3fb950;--re:#f85149}
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--tx);font-family:system-ui,-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;font-size:14px;line-height:1.5;padding:16px}
.wrap{max-width:520px}
.back{display:inline-flex;align-items:center;gap:4px;color:var(--mu);text-decoration:none;font-size:13px;margin-bottom:16px;transition:color .15s}
.back:hover{color:var(--tx)}
.hdr{display:flex;align-items:flex-start;justify-content:space-between;gap:8px;margin-bottom:16px;flex-wrap:wrap}
h1{font-size:18px;font-weight:600;color:var(--tx)}
.lang-sw{display:flex;gap:2px;background:var(--sf);border:1px solid var(--bd);border-radius:6px;padding:2px}
.lang-btn{background:transparent;border:none;border-radius:4px;color:var(--mu);cursor:pointer;font:11px/1 system-ui,-apple-system,sans-serif;padding:3px 7px;transition:background .15s,color .15s}
.lang-btn:hover:not(.act){background:var(--bd);color:var(--tx)}
.lang-btn.act{background:var(--bl);color:#fff}
.sec{background:var(--sf);border:1px solid var(--bd);border-radius:10px;padding:16px;margin-bottom:10px}
.drop{display:block;border:2px dashed var(--bd);border-radius:8px;padding:24px;text-align:center;cursor:pointer;color:var(--mu);font-size:13px;transition:border-color .15s,color .15s}
.drop:hover,.drop.has{border-color:var(--bl);color:var(--bl)}
#fileIn{display:none}
#fname{font-size:12px;color:var(--bl);margin-top:6px;min-height:16px;text-align:center}
.ubtn{display:block;width:100%;background:#1f6feb;color:#fff;border:none;border-radius:6px;padding:10px;font:14px/1 system-ui,-apple-system,sans-serif;font-weight:500;cursor:pointer;margin-top:12px;transition:background .15s}
.ubtn:disabled{opacity:.5;cursor:not-allowed}
.ubtn:not(:disabled):hover{background:#388bfd}
.prog{display:none;margin-top:12px}
.prog-bg{background:var(--bd);border-radius:4px;height:8px;overflow:hidden}
.prog-fill{height:100%;border-radius:4px;background:var(--bl);width:0%;transition:width .2s}
#progPct{font-size:12px;color:var(--mu);text-align:right;margin-top:4px}
#result{margin-top:12px;padding:10px 12px;border-radius:6px;font-size:13px;display:none}
#result.ok{background:#3fb95033;border:1px solid var(--gn);color:var(--gn)}
#result.err{background:#f8514933;border:1px solid var(--re);color:var(--re)}
.note{font-size:12px;color:var(--mu);line-height:1.6}
.note p+p{margin-top:6px}
</style></head><body>
<div class="wrap">
<a href="/" class="back" id="backLink">&#x2190; Dashboard</a>
<div class="hdr">
  <h1 id="pageTitle">Firmware Update</h1>
  <div class="lang-sw">
    <button class="lang-btn" data-lang="en" onclick="setLang('en')">EN</button>
    <button class="lang-btn" data-lang="it" onclick="setLang('it')">IT</button>
    <button class="lang-btn" data-lang="de" onclick="setLang('de')">DE</button>
    <button class="lang-btn" data-lang="fr" onclick="setLang('fr')">FR</button>
    <button class="lang-btn" data-lang="es" onclick="setLang('es')">ES</button>
  </div>
</div>
<div class="sec">
  <label class="drop" id="dropLbl" for="fileIn">&#x1F4C2; <span id="dropTxt">Click to select firmware file (.bin)</span></label>
  <input type="file" id="fileIn" accept=".bin">
  <div id="fname"></div>
  <button class="ubtn" id="upBtn" disabled onclick="startUpload()" id="upBtn">Upload Firmware</button>
  <div class="prog" id="progWrap">
    <div class="prog-bg"><div class="prog-fill" id="progFill"></div></div>
    <div id="progPct">0%</div>
  </div>
  <div id="result"></div>
</div>
<div class="sec">
  <div class="note">
    <p id="note1">Build the firmware with PlatformIO and select the .bin file from the build output.</p>
    <p id="note2">The bridge reboots automatically after a successful update. Wait for the page to reconnect.</p>
  </div>
</div>
</div>
<script>
const TU={
en:{back:'← Dashboard',title:'Firmware Update',dropTxt:'Click to select firmware file (.bin)',upBtn:'Upload Firmware',note1:'Build the firmware with PlatformIO and select the .bin file from the build output.',note2:'The bridge reboots automatically after a successful update. Wait for the page to reconnect.',ok:'Update complete. Rebooting…',errFail:'Update failed: ',errConn:'Upload error — check connection'},
it:{back:'← Dashboard',title:'Aggiornamento firmware',dropTxt:'Clicca per selezionare il file firmware (.bin)',upBtn:'Carica firmware',note1:'Compila il firmware con PlatformIO e seleziona il file .bin dall\'output di build.',note2:'Il bridge si riavvia automaticamente dopo un aggiornamento riuscito. Attendi che la pagina si riconnetta.',ok:'Aggiornamento completato. Riavvio…',errFail:'Aggiornamento fallito: ',errConn:'Errore di upload — verifica la connessione'},
de:{back:'← Dashboard',title:'Firmware-Update',dropTxt:'Klicken, um Firmware-Datei (.bin) auszuwählen',upBtn:'Firmware hochladen',note1:'Firmware mit PlatformIO erstellen und die .bin-Datei aus dem Build-Verzeichnis auswählen.',note2:'Die Bridge startet nach einem erfolgreichen Update automatisch neu. Warten, bis die Seite sich wieder verbindet.',ok:'Update abgeschlossen. Neustart…',errFail:'Update fehlgeschlagen: ',errConn:'Upload-Fehler — Verbindung prüfen'},
fr:{back:'← Dashboard',title:'Mise à jour firmware',dropTxt:'Cliquer pour sélectionner le fichier firmware (.bin)',upBtn:'Téléverser le firmware',note1:'Compiler le firmware avec PlatformIO et sélectionner le fichier .bin depuis le dossier de build.',note2:'Le bridge redémarre automatiquement après une mise à jour réussie. Attendre que la page se reconnecte.',ok:'Mise à jour terminée. Redémarrage…',errFail:'Échec de la mise à jour : ',errConn:'Erreur d\'upload — vérifier la connexion'},
es:{back:'← Dashboard',title:'Actualización de firmware',dropTxt:'Haz clic para seleccionar el archivo firmware (.bin)',upBtn:'Subir firmware',note1:'Compila el firmware con PlatformIO y selecciona el archivo .bin del directorio de build.',note2:'El bridge se reinicia automáticamente tras una actualización exitosa. Espera a que la página se vuelva a conectar.',ok:'Actualización completada. Reiniciando…',errFail:'Error en la actualización: ',errConn:'Error de subida — verifica la conexión'}
};
let L=localStorage.getItem('lang')||'en';
if(!TU[L])L='en';

function setLang(l){
  L=l;localStorage.setItem('lang',l);
  const s=TU[l];
  document.getElementById('backLink').textContent=s.back;
  document.getElementById('pageTitle').textContent=s.title;
  document.getElementById('dropTxt').textContent=s.dropTxt;
  document.getElementById('upBtn').textContent=s.upBtn;
  document.getElementById('note1').textContent=s.note1;
  document.getElementById('note2').textContent=s.note2;
  document.querySelectorAll('.lang-btn').forEach(b=>b.classList.toggle('act',b.dataset.lang===l));
  document.title=s.title+' — BoschEBike Bridge';
}

const fileIn=document.getElementById('fileIn');
fileIn.addEventListener('change',function(){
  const lbl=document.getElementById('dropLbl');
  const fn=document.getElementById('fname');
  if(this.files.length){
    const f=this.files[0];
    lbl.classList.add('has');
    fn.textContent=f.name+' ('+(f.size/1024).toFixed(1)+' KB)';
    document.getElementById('upBtn').disabled=false;
  }else{
    lbl.classList.remove('has');
    fn.textContent='';
    document.getElementById('upBtn').disabled=true;
  }
});

function startUpload(){
  const file=fileIn.files[0];
  if(!file)return;
  const fd=new FormData();
  fd.append('firmware',file);
  document.getElementById('upBtn').disabled=true;
  document.getElementById('progWrap').style.display='block';
  const res=document.getElementById('result');
  res.style.display='none';
  const xhr=new XMLHttpRequest();
  xhr.open('POST','/update');
  xhr.upload.addEventListener('progress',function(e){
    if(e.lengthComputable){
      const p=Math.round(e.loaded/e.total*100);
      document.getElementById('progFill').style.width=p+'%';
      document.getElementById('progPct').textContent=p+'%';
    }
  });
  xhr.addEventListener('load',function(){
    res.style.display='block';
    if(xhr.status===200){res.className='ok';res.textContent=TU[L].ok;}
    else{res.className='err';res.textContent=TU[L].errFail+xhr.responseText;document.getElementById('upBtn').disabled=false;}
  });
  xhr.addEventListener('error',function(){
    res.style.display='block';res.className='err';res.textContent=TU[L].errConn;
    document.getElementById('upBtn').disabled=false;
  });
  xhr.send(fd);
}

setLang(L);
</script></body></html>)html";

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
        "{\"version\":\"" FIRMWARE_VERSION "\",\"sim\":%s,\"mode\":%d,\"base_name\":\"%s\",\"device_name\":\"%s\","
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

static void handleFieldsJson() {
    static char buf[6144];
    uint32_t ageMs = reLastPacketMs ? (millis() - reLastPacketMs) : 0xFFFFFFFFu;
    int pos = snprintf(buf, sizeof(buf), "{\"count\":%lu,\"age_ms\":%lu,\"fields\":[",
                       (unsigned long)rePacketCount, (unsigned long)ageMs);
    bool first = true;
    for (int i = 1; i < RE_MAX_FIELD && pos < (int)sizeof(buf) - 200; i++) {
        const ReField& f = reFields[i];
        if (!f.seen) continue;
        if (!first) buf[pos++] = ',';
        first = false;
        if (f.wireType == 0) {
            int64_t delta = (int64_t)f.value - (int64_t)f.prevValue;
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                "{\"n\":%d,\"wt\":0,\"v\":%llu,\"p\":%llu,\"d\":%lld,\"cnt\":%lu,\"chg\":%s}",
                i, (unsigned long long)f.value, (unsigned long long)f.prevValue,
                (long long)delta, (unsigned long)f.seenCount,
                f.changed ? "true" : "false");
        } else {
            char hexStr[33] = {};
            for (int j = 0; j < f.byteLen; j++)
                snprintf(hexStr + j * 2, sizeof(hexStr) - j * 2, "%02x", f.bytes[j]);
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                "{\"n\":%d,\"wt\":2,\"v\":%llu,\"cnt\":%lu,\"chg\":%s,\"b\":\"%s\"}",
                i, (unsigned long long)f.value, (unsigned long)f.seenCount,
                f.changed ? "true" : "false", hexStr);
        }
    }
    snprintf(buf + pos, sizeof(buf) - pos, "]}");
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "application/json", buf);
}

static const char EXPLORER_HTML[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LDI Field Explorer</title>
<style>
:root{--bg:#0d1117;--sf:#161b22;--bd:#30363d;--tx:#e6edf3;--mu:#8b949e;--bl:#58a6ff;--gn:#3fb950;--ye:#d29922;--re:#f85149;--pu:#a371f7}
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--tx);font-family:ui-monospace,'Cascadia Code',Consolas,monospace;font-size:13px;padding:16px}
.wrap{max-width:900px}
.hdr{display:flex;align-items:center;gap:16px;margin-bottom:14px;flex-wrap:wrap}
.hdr a{color:var(--mu);text-decoration:none;font-size:12px;font-family:system-ui,-apple-system,sans-serif}
.hdr a:hover{color:var(--tx)}
h1{font-size:16px;font-weight:600;color:var(--bl);font-family:system-ui,-apple-system,sans-serif}
.meta{font-size:11px;color:var(--mu);margin-bottom:10px;font-family:system-ui,-apple-system,sans-serif}
table{width:100%;border-collapse:collapse;background:var(--sf);border:1px solid var(--bd);border-radius:8px;overflow:hidden}
th{background:#1c2128;color:var(--mu);font-size:10px;letter-spacing:.6px;text-transform:uppercase;padding:8px 12px;text-align:left;border-bottom:1px solid var(--bd);cursor:pointer;user-select:none;font-family:system-ui,-apple-system,sans-serif}
th:hover{color:var(--tx)}
td{padding:6px 12px;border-bottom:1px solid #21262d;vertical-align:middle}
tr:last-child td{border-bottom:none}
tr.known td.fn{color:var(--bl);font-weight:700}
tr.unknown td.fn{color:var(--mu)}
tr.changed{background:#d2992212}
tr.changed td.delta{color:var(--ye);font-weight:700}
.badge{display:inline-block;padding:1px 6px;border-radius:10px;font-size:10px;border:1px solid;font-family:system-ui,-apple-system,sans-serif}
.badge.kn{border-color:var(--bl);color:var(--bl)}
.badge.uk{border-color:var(--mu);color:var(--mu)}
.badge.wt2{border-color:var(--pu);color:var(--pu)}
td.hex{color:var(--mu);font-size:11px}
td.name{color:var(--gn);font-family:system-ui,-apple-system,sans-serif;font-size:12px}
td.bytes{color:var(--pu);font-size:11px;word-break:break-all}
.note{margin-top:14px;font-size:11px;color:var(--mu);line-height:1.8;font-family:system-ui,-apple-system,sans-serif;background:var(--sf);border:1px solid var(--bd);border-radius:8px;padding:12px}
.note b{color:var(--tx)}
</style></head><body>
<div class="wrap">
<div class="hdr">
  <a href="/">&#x2190; Dashboard</a>
  <h1>LDI Field Explorer</h1>
</div>
<div class="meta" id="meta">Waiting for data&#x2026;</div>
<table>
<thead><tr>
  <th onclick="sort('n')">Field #</th>
  <th onclick="sort('v')">Value (dec)</th>
  <th>Hex</th>
  <th onclick="sort('p')">Prev</th>
  <th class="delta" onclick="sort('d')">Delta</th>
  <th onclick="sort('cnt')">Count</th>
  <th>Type</th>
  <th>Name / bytes</th>
</tr></thead>
<tbody id="rows"></tbody>
</table>
<div class="note">
<b>How to read this table:</b><br>
Fields highlighted in yellow changed in the last received packet &mdash; these are the interesting ones.<br>
<b>Unknown</b> fields are what we are looking for: correlate their value with what the bike is doing (pedalling harder, changing assist level, braking, etc.).<br>
Sort by <b>Delta</b> to surface the most active fields. Sort by <b>Field #</b> to see the full picture.<br><br>
<b>Known fields:</b>
1=speed&times;100 &nbsp;&bull;&nbsp; 2=cadence_rpm &nbsp;&bull;&nbsp; 5=power_W &nbsp;&bull;&nbsp; 9=ambient&times;1000
&nbsp;&bull;&nbsp; 10=battery_% &nbsp;&bull;&nbsp; 12=odometer&times;1000 &nbsp;&bull;&nbsp; 17=light(0/1/2)
&nbsp;&bull;&nbsp; 21=locked &nbsp;&bull;&nbsp; 22=charger &nbsp;&bull;&nbsp; 23=light_reserve
&nbsp;&bull;&nbsp; 24=diag &nbsp;&bull;&nbsp; 25=standstill
</div>
</div>
<script>
const KNOWN={1:'speed×100',2:'cadence_rpm',5:'power_W',9:'ambient×1000',10:'battery_%',12:'odometer×1000',17:'light(0/1/2)',21:'locked',22:'charger',23:'light_reserve',24:'diag',25:'standstill'};
let sortKey='n',sortAsc=true,lastData=null;

function sort(k){
  if(sortKey===k)sortAsc=!sortAsc;
  else{sortKey=k;sortAsc=true;}
  if(lastData)render(lastData);
}

function render(d){
  lastData=d;
  const age=d.age_ms>60000?'—':d.age_ms+'ms ago';
  document.getElementById('meta').textContent=
    'Packets received: '+d.count+' · Last packet: '+age+' · Fields seen: '+d.fields.length;
  const rows=d.fields.slice().sort((a,b)=>{
    const av=a[sortKey]??0, bv=b[sortKey]??0;
    return sortAsc?av-bv:bv-av;
  });
  document.getElementById('rows').innerHTML=rows.map(f=>{
    const known=KNOWN[f.n];
    const wt2=f.wt===2;
    const hexVal=wt2?'&mdash;':'0x'+BigInt(f.v).toString(16).toUpperCase().padStart(4,'0');
    const prevVal=wt2?'&mdash;':f.p;
    const delta=wt2?'&mdash;':(f.d>=0?'+':'')+f.d;
    const badge=wt2?'<span class="badge wt2">bytes</span>':
                (known?'<span class="badge kn">known</span>':'<span class="badge uk">unknown</span>');
    const nameTd=wt2?
      '<td class="bytes">'+f.b+'</td>':
      '<td class="name">'+(known||'')+'</td>';
    return '<tr class="'+(known?'known':'unknown')+' '+(f.chg?'changed':'')+'">'+
      '<td class="fn">'+f.n+'</td>'+
      '<td>'+(wt2?f.v+' B':f.v)+'</td>'+
      '<td class="hex">'+hexVal+'</td>'+
      '<td>'+prevVal+'</td>'+
      '<td class="delta">'+delta+'</td>'+
      '<td>'+f.cnt+'</td>'+
      '<td>'+badge+'</td>'+
      nameTd+
      '</tr>';
  }).join('');
}

function poll(){
  fetch('/fields',{cache:'no-store'})
    .then(r=>r.json()).then(render)
    .catch(()=>{document.getElementById('meta').textContent='Connection lost';});
}
poll();
setInterval(poll,1500);
</script></body></html>)html";

static void handleExplorer() {
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send_P(200, "text/html", EXPLORER_HTML);
}

static void startWebDebug() {
    IPAddress apIp(192, 168, 4, 1);
    WiFi.persistent(false);
    bool modeOk = WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    bool cfgOk = WiFi.softAPConfig(apIp, apIp, IPAddress(255, 255, 255, 0));
    bool apOk  = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, 1, false, 4);

    Serial.printf("[WiFi] AP mode=%s, config=%s, start=%s\n",
                  modeOk ? "ok" : "fail",
                  cfgOk  ? "ok" : "fail",
                  apOk   ? "ok" : "fail");
    if (apOk) {
        Serial.printf("[WiFi] SSID: %s, IP: %s, boot window: %lu ms, idle timeout: %lu ms\n",
                      WIFI_AP_SSID,
                      WiFi.softAPIP().toString().c_str(),
                      (unsigned long)WEB_BOOT_WINDOW_TIMEOUT_MS,
                      (unsigned long)WEB_IDLE_AFTER_CLIENT_TIMEOUT_MS);
    }

    webServer.on("/",        handleRoot);
    webServer.on("/data",    handleData);
    webServer.on("/status",  handleStatus);
    webServer.on("/setmode", handleSetMode);
    webServer.on("/setname", handleSetName);
    webServer.on("/setsim",    handleSetSim);
    webServer.on("/setdebug",  handleSetDebug);
    webServer.on("/log",       handleLog);
    webServer.on("/clearlog",  handleClearLog);
    webServer.on("/fields",    handleFieldsJson);
    webServer.on("/explorer",  handleExplorer);
    webServer.on("/update",  HTTP_GET,  handleUpdatePage);
    webServer.on("/update",  HTTP_POST, handleUpdateDone, handleUpdateUpload);
    webServer.begin();

    webStartedMs        = millis();
    lastWebClientSeenMs = webStartedMs;
    webHadClient        = false;
    webOk               = apOk;
    wifiActive          = apOk;
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
    uint32_t timeoutMs   = webHadClient ? WEB_IDLE_AFTER_CLIENT_TIMEOUT_MS : WEB_BOOT_WINDOW_TIMEOUT_MS;
    uint32_t referenceMs = webHadClient ? lastWebClientSeenMs : webStartedMs;
    if ((int32_t)(now - referenceMs - timeoutMs) >= 0) stopWebDebug();
}

// Setup
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

    if (BATTERY_ADC_PIN >= 0) {
        analogReadResolution(12);
        analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
    }
    updateBridgeBattery(true);

    startWebDebug();

    NimBLEDevice::init(gDeviceName);
    // The Bosch eBike requires SC + bonding (Just Works) on all modes to complete
    // its LDI encryption handshake. The original hypothesis that BOND caused
    // Suunto to disconnect was incorrect — the real causes were 128-bit UUIDs in
    // the GATT table and missing SC/CP Control Points (both now fixed).
    NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_SC | BLE_SM_PAIR_AUTHREQ_BOND);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
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

// Loop
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
