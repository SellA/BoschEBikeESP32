/*
 * BoschEBike Bridge — BLE bridge: bici Bosch → Suunto watch
 *
 * Ruoli BLE:
 *   - GAP Peripheral + GATT Client  →  verso la bici (spec LDI Bosch)
 *   - GAP Peripheral + GATT Server  →  verso Suunto (LDI service trasparente)
 *
 * Flusso normale (SIM_ENABLED = false):
 *   1. ESP32 advertise con solicitation LDI → bici si connette
 *   2. ESP32 legge notifiche LDI dalla bici (raw protobuf)
 *   3. ESP32 advertise con LDI service UUID → Suunto si connette
 *   4. ESP32 forward raw protobuf a Suunto via notify
 *
 * Flusso simulazione (SIM_ENABLED = true):
 *   1. ESP32 advertise subito con LDI service UUID → Suunto si connette
 *   2. ESP32 genera dati fake (protobuf sintetico) e li invia al Suunto
 *   (nessuna connessione alla bici reale)
 *
 * Web UI/OTA: http://192.168.4.1
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <NimBLEDevice.h>
#include "nimble/nimble/host/include/host/ble_gatt.h"
#include "nimble/nimble/host/include/host/ble_hs_mbuf.h"

// ─── Simulazione (debug) ─────────────────────────────────────────────────────
// true  → Suunto si connette subito, dati fake generati dall'ESP32
// false → flusso normale: bici prima, poi Suunto con dati reali
static const bool SIM_ENABLED = false;

#define WIFI_AP_SSID  "BoschEBike Bridge"
#define WIFI_AP_PASS  "password"

// UUID LDI Bosch (usati sia lato client bici che lato server Suunto)
#define LDI_SVC_UUID  "0000eb20-eaa2-11e9-81b4-2a2ae2dbcce4"
#define LDI_CHAR_UUID "0000eb21-eaa2-11e9-81b4-2a2ae2dbcce4"

// ─── Dati live decodificati ───────────────────────────────────────────────────
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

// ─── Decoder protobuf manuale ─────────────────────────────────────────────────
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

// ─── Encoder protobuf (per simulazione) ──────────────────────────────────────
static void notifyLdiData(const uint8_t* data, size_t len);  // forward decl

static int encodeVarint(uint8_t* buf, uint64_t val) {
    int n = 0;
    do {
        buf[n] = (uint8_t)((val & 0x7f) | (val >= 0x80 ? 0x80 : 0x00));
        val >>= 7;
        n++;
    } while (val);
    return n;
}

static int encodeField(uint8_t* buf, int fieldNum, uint64_t val) {
    int n = encodeVarint(buf, (uint64_t)(fieldNum << 3));  // tag (wire type 0)
    n += encodeVarint(buf + n, val);
    return n;
}

// Onda triangolare: oscilla tra lo e hi con periodo periodMs
static int32_t triWave(uint32_t ms, uint32_t periodMs, int32_t lo, int32_t hi) {
    uint32_t phase = ms % periodMs;
    int32_t half  = (int32_t)(periodMs / 2);
    int32_t range = hi - lo;
    return (int32_t)phase < half
        ? lo + ((int32_t)phase * range) / half
        : hi - (((int32_t)phase - half) * range) / half;
}

static uint32_t nextSimNotifyMs = 0;

static void generateAndNotifySimData() {
    uint32_t t = millis();
    uint64_t speedRaw = (uint64_t)triWave(t, 20000,  1500, 3500); // 15–35 km/h (×100)
    uint64_t cadence  = (uint64_t)triWave(t, 15000,    70,   95); // rpm
    uint64_t power    = (uint64_t)triWave(t, 12000,   120,  280); // W
    uint64_t battery  = 75;                                        // %
    uint64_t odoRaw   = 1234000;                                   // 1234 km (×1000)

    uint8_t buf[32];
    uint8_t* p = buf;
    p += encodeField(p,  1, speedRaw);
    p += encodeField(p,  2, cadence);
    p += encodeField(p,  5, power);
    p += encodeField(p, 10, battery);
    p += encodeField(p, 12, odoRaw);

    size_t len = (size_t)(p - buf);
    decodeLiveData(buf, len);   // aggiorna gData per web UI
    notifyLdiData(buf, len);
}

// ─── BLE stato ───────────────────────────────────────────────────────────────
static NimBLEServer*         pServer        = nullptr;
static NimBLECharacteristic* pLdiServerChar = nullptr;
static uint32_t              nextGattRetryMs  = 0;
static uint16_t              ebikeConnHandle  = BLE_HS_CONN_HANDLE_NONE;
static uint16_t              ldiSvcStart      = 0;
static uint16_t              ldiSvcEnd        = 0;
static uint16_t              ldiCharHandle    = 0;
static uint16_t              ldiCccdHandle    = 0;
static volatile bool         gattDiscoveryActive = false;

static volatile bool flagEbikePeripheralConn = false;
static volatile bool flagEbikeDisconn        = false;
static volatile bool flagSuuntoConn          = false;
static volatile bool flagSuuntoDisconn       = false;
static volatile bool ebikeConnected          = false;
static volatile bool ebikeGattReady          = false;
static volatile bool suuntoConnected         = false;
static volatile bool ebikeEncrypted          = false;
static volatile bool gattStartPending        = false;
static uint16_t      lastMtu                 = 0;

static NimBLEAddress ebikeAddr("");
static const ble_uuid128_t LDI_SVC_UUID_RAW = BLE_UUID128_INIT(
    0xe4, 0xcc, 0xdb, 0xe2, 0x2a, 0x2a, 0xb4, 0x81,
    0xe9, 0x11, 0xa2, 0xea, 0x20, 0xeb, 0x00, 0x00
);
static const ble_uuid128_t LDI_CHAR_UUID_RAW = BLE_UUID128_INIT(
    0xe4, 0xcc, 0xdb, 0xe2, 0x2a, 0x2a, 0xb4, 0x81,
    0xe9, 0x11, 0xa2, 0xea, 0x21, 0xeb, 0x00, 0x00
);
static const ble_uuid16_t CCCD_UUID_RAW = BLE_UUID16_INIT(0x2902);

static void startAdvertisingForEbike();
static void startAdvertisingForSuunto();
static bool openGattClient();

// ─── Forward raw LDI payload al Suunto ───────────────────────────────────────
static void notifyLdiData(const uint8_t* data, size_t len) {
    if (!suuntoConnected || !pLdiServerChar) return;
    pLdiServerChar->setValue(data, len);
    pLdiServerChar->notify();
}

static void handleLdiPayload(const uint8_t* data, size_t len) {
    decodeLiveData(data, len);
    notifyLdiData(data, len);
}

// ─── GATT client callbacks (lato bici) ───────────────────────────────────────
static int gattWriteCccdCB(uint16_t, const struct ble_gatt_error* error,
                           struct ble_gatt_attr*, void*) {
    if (error->status == 0) {
        ebikeGattReady = true;
        gattDiscoveryActive = false;
        nextGattRetryMs = 0;
        startAdvertisingForSuunto();
    } else {
        gattDiscoveryActive = false;
        nextGattRetryMs = millis() + 3000;
    }
    return 0;
}

static void writeLdiCccd(uint16_t connHandle);

static int gattDscCB(uint16_t connHandle, const struct ble_gatt_error* error,
                     uint16_t, const struct ble_gatt_dsc* dsc, void*) {
    if (error->status == 0 && dsc &&
        ble_uuid_cmp(&dsc->uuid.u, &CCCD_UUID_RAW.u) == 0) {
        ldiCccdHandle = dsc->handle;
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        if (ldiCccdHandle == 0) {
            gattDiscoveryActive = false;
            nextGattRetryMs = millis() + 3000;
            return 0;
        }
        writeLdiCccd(connHandle);
    } else if (error->status != 0) {
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
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        if (ldiCharHandle == 0) {
            gattDiscoveryActive = false;
            nextGattRetryMs = millis() + 3000;
            return 0;
        }
        int rc = ble_gattc_disc_all_dscs(connHandle, ldiCharHandle + 1,
                                         ldiSvcEnd, gattDscCB, nullptr);
        if (rc != 0) {
            if (ldiCharHandle + 1 <= ldiSvcEnd) {
                ldiCccdHandle = ldiCharHandle + 1;
                writeLdiCccd(connHandle);
            } else {
                gattDiscoveryActive = false;
                nextGattRetryMs = millis() + 3000;
            }
        }
    } else if (error->status != 0) {
        gattDiscoveryActive = false;
        nextGattRetryMs = millis() + 3000;
    }
    return 0;
}

static int gattSvcCB(uint16_t connHandle, const struct ble_gatt_error* error,
                     const struct ble_gatt_svc* service, void*) {
    if (error->status == 0 && service) {
        ldiSvcStart = service->start_handle;
        ldiSvcEnd = service->end_handle;
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        if (ldiSvcStart == 0 || ldiSvcEnd == 0) {
            gattDiscoveryActive = false;
            ebikeConnected = false;
            ebikeEncrypted = false;
            gattStartPending = false;
            ebikeConnHandle = BLE_HS_CONN_HANDLE_NONE;
            nextGattRetryMs = 0;
            startAdvertisingForEbike();
            return 0;
        }
        int rc = ble_gattc_disc_chrs_by_uuid(connHandle, ldiSvcStart, ldiSvcEnd,
                                             &LDI_CHAR_UUID_RAW.u, gattChrCB, nullptr);
        if (rc != 0) {
            gattDiscoveryActive = false;
            nextGattRetryMs = millis() + 3000;
        }
    } else if (error->status != 0) {
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

static int gattMtuCB(uint16_t connHandle, const struct ble_gatt_error* error,
                     uint16_t mtu, void*) {
    lastMtu = mtu;
    startLdiServiceDiscovery(connHandle);
    return 0;
}

static int customGapHandler(struct ble_gap_event* event, void*) {
    if (event->type == BLE_GAP_EVENT_ENC_CHANGE &&
        event->enc_change.conn_handle == ebikeConnHandle) {
        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(ebikeConnHandle, &desc);
        ebikeEncrypted = (event->enc_change.status == 0 && rc == 0 && desc.sec_state.encrypted);
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
        if (os_mbuf_copydata(event->notify_rx.om, 0, len, buf) == 0) {
            handleLdiPayload(buf, len);
        }
    }
    return 0;
}

// ─── BLE Server Callbacks ─────────────────────────────────────────────────────
class ServerCB : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, ble_gap_conn_desc* desc) override {
        // In SIM_ENABLED la bici non si connette mai: ogni connessione è Suunto
        if (!SIM_ENABLED && !ebikeConnected) {
            NimBLEAddress addr(desc->peer_ota_addr);
            ebikeAddr = addr;
            ebikeConnHandle = desc->conn_handle;
            ebikeConnected = true;
            ebikeEncrypted = desc->sec_state.encrypted;
            flagEbikePeripheralConn = true;
        } else {
            flagSuuntoConn = true;
        }
    }
    void onDisconnect(NimBLEServer*, ble_gap_conn_desc* desc) override {
        if (!SIM_ENABLED && desc->conn_handle == ebikeConnHandle) {
            ebikeConnected = false;
            ebikeGattReady = false;
            gattDiscoveryActive = false;
            ebikeEncrypted = false;
            gattStartPending = false;
            ebikeConnHandle = BLE_HS_CONN_HANDLE_NONE;
            flagEbikeDisconn = true;
        } else {
            flagSuuntoDisconn = true;
        }
    }
    bool onConfirmPIN(uint32_t) override { return true; }
};

// ─── BLE advertising ─────────────────────────────────────────────────────────
static void startAdvertisingForEbike() {
    auto* adv = NimBLEDevice::getAdvertising();
    adv->reset();

    // Solicitation LDI: la bici vede la richiesta e si connette all'ESP32
    const uint8_t advPayload[] = {
        0x02, 0x01, 0x06,
        0x11, 0x15,  // Complete list of 128-bit service solicitation UUIDs
        0xe4, 0xcc, 0xdb, 0xe2, 0x2a, 0x2a, 0xb4, 0x81,
        0xe9, 0x11, 0xa2, 0xea, 0x20, 0xeb, 0x00, 0x00
    };
    NimBLEAdvertisementData advData;
    advData.addData(std::string((const char*)advPayload, sizeof(advPayload)));
    adv->setAdvertisementData(advData);

    NimBLEAdvertisementData scanData;
    scanData.setName("BoschEBike Bridge");
    adv->setScanResponseData(scanData);
    adv->setScanResponse(true);
    adv->setMinInterval(0x0050);
    adv->setMaxInterval(0x00a0);
    adv->start();
}

static void startAdvertisingForSuunto() {
    auto* adv = NimBLEDevice::getAdvertising();
    adv->reset();

    // Espone LDI service UUID: il Suunto si connette come se fosse la bici
    const uint8_t advPayload[] = {
        0x02, 0x01, 0x06,
        0x11, 0x07,  // Complete list of 128-bit service UUIDs
        0xe4, 0xcc, 0xdb, 0xe2, 0x2a, 0x2a, 0xb4, 0x81,
        0xe9, 0x11, 0xa2, 0xea, 0x20, 0xeb, 0x00, 0x00
    };
    NimBLEAdvertisementData advData;
    advData.addData(std::string((const char*)advPayload, sizeof(advPayload)));
    adv->setAdvertisementData(advData);

    NimBLEAdvertisementData scanData;
    scanData.setName("BoschEBike");
    adv->setScanResponseData(scanData);
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

    gattStartPending = false;
    ebikeGattReady = false;
    gattDiscoveryActive = true;
    ldiSvcStart = ldiSvcEnd = ldiCharHandle = ldiCccdHandle = 0;
    NimBLEDevice::setMTU(247);
    ble_gap_set_data_len(ebikeConnHandle, 251, (251 + 14) * 8);

    int rc = ble_gattc_exchange_mtu(ebikeConnHandle, gattMtuCB, nullptr);
    if (rc != 0) startLdiServiceDiscovery(ebikeConnHandle);
    return true;
}

static void resetAndAdvertiseForEbike() {
    ebikeConnected = ebikeGattReady = suuntoConnected = false;
    gattDiscoveryActive = ebikeEncrypted = gattStartPending = false;
    ebikeConnHandle = BLE_HS_CONN_HANDLE_NONE;
    ldiSvcStart = ldiSvcEnd = ldiCharHandle = ldiCccdHandle = 0;
    flagEbikePeripheralConn = flagEbikeDisconn = flagSuuntoConn = flagSuuntoDisconn = false;
    gData = LiveData{};
    nextGattRetryMs = 0;
    startAdvertisingForEbike();
}

// ─── Web server ───────────────────────────────────────────────────────────────
static WebServer webServer(80);
static bool webOk = false;

static const char INDEX_HTML[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="it"><head><meta charset="utf-8">
<title>BoschEBike Bridge</title>
<style>
*{box-sizing:border-box}
body{background:#0d1117;color:#e6edf3;font-family:monospace;padding:16px;margin:0}
h1{color:#58a6ff;font-size:18px;margin:0 0 4px}
#st{font-size:13px;margin-bottom:14px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;max-width:500px}
.card{background:#161b22;border-radius:8px;padding:12px}
.lbl{color:#8b949e;font-size:10px;letter-spacing:.5px;margin-bottom:2px}
.val{font-size:28px;font-weight:bold}
.unit{color:#8b949e;font-size:13px}
.flags{background:#161b22;border-radius:8px;padding:10px;margin-top:8px;max-width:500px}
.f{display:inline-block;margin:3px;padding:3px 10px;border-radius:4px;font-size:12px}
.on{background:#1f6feb}.off{background:#21262d;color:#484f58}
#ts{margin-top:10px;color:#484f58;font-size:11px}
</style></head><body>
<h1>BoschEBike Bridge</h1>
<div id="st" style="color:#3fb950">Connessione...</div>
<div class="grid">
  <div class="card"><div class="lbl">VELOCITÀ</div><span class="val" id="spd">-</span> <span class="unit">km/h</span></div>
  <div class="card"><div class="lbl">CADENZA</div><span class="val" id="cad">-</span> <span class="unit">rpm</span></div>
  <div class="card"><div class="lbl">POTENZA</div><span class="val" id="pwr" style="color:#3fb950">-</span> <span class="unit">W</span></div>
  <div class="card"><div class="lbl">BATTERIA</div><span class="val" id="bat" style="color:#58a6ff">-</span> <span class="unit">%</span></div>
  <div class="card"><div class="lbl">ODOMETRO</div><span class="val" id="odo">-</span> <span class="unit">km</span></div>
  <div class="card"><div class="lbl">LUCE AMBIENTE</div><span class="val" id="lux" style="color:#d29922">-</span> <span class="unit">lux</span></div>
</div>
<div class="flags">
  <span class="f off" id="fl_lt">Luce: -</span>
  <span class="f off" id="fl_lk">Lock</span>
  <span class="f off" id="fl_ch">Charger</span>
  <span class="f off" id="fl_rv">Reserve</span>
  <span class="f off" id="fl_dg">Diag</span>
  <span class="f off" id="fl_st">Fermo</span>
  <span class="f off" id="fl_su">Suunto</span>
</div>
<div id="ts"></div>
<script>
const LT=['−','OFF','ON'];
let busy=false;
function f(id,on,txt){var e=document.getElementById(id);e.textContent=txt;e.className='f '+(on?'on':'off');}
function pollData(){
  if(busy)return; busy=true;
  fetch('/data',{cache:'no-store'}).then(r=>r.json()).then(d=>{
    var s=document.getElementById('st');
    if(d.sim){s.style.color='#a371f7';s.textContent='SIMULAZIONE | Suunto: '+(d.suunto?'connesso':'in advertising');}
    else if(!d.ebike){s.style.color='#d29922';s.textContent='BLE: in attesa bici...';}
    else if(!d.gatt){s.style.color='#d29922';s.textContent='Bici: connessa | GATT: in discovery...';}
    else{s.style.color='#3fb950';s.textContent='Bici: connessa | Suunto: '+(d.suunto?'connesso':'in advertising');}
    var v=d.valid;
    document.getElementById('spd').textContent=v?d.speed.toFixed(1):'-';
    document.getElementById('cad').textContent=v?d.cadence:'-';
    document.getElementById('pwr').textContent=v?d.power:'-';
    document.getElementById('bat').textContent=v?d.battery:'-';
    document.getElementById('odo').textContent=v?d.odometer.toFixed(1):'-';
    document.getElementById('lux').textContent=v?d.ambient.toFixed(0):'-';
    f('fl_lt',d.bike_light===2,'Luce: '+(LT[d.bike_light]||'-'));
    f('fl_lk',d.locked,'Lock');
    f('fl_ch',d.charger,'Charger');
    f('fl_rv',d.reserve,'Reserve');
    f('fl_dg',d.diag,'Diag');
    f('fl_st',d.standstill,'Fermo');
    f('fl_su',d.suunto,'Suunto');
    document.getElementById('ts').textContent='Aggiornato: '+new Date().toLocaleTimeString()+'.'+String(new Date().getMilliseconds()).padStart(3,'0');
  }).catch(()=>{document.getElementById('st').textContent='Errore connessione web';})
    .finally(()=>{busy=false;});
}
pollData();
setInterval(pollData,500);
</script></body></html>)html";

static void handleRoot() {
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send_P(200, "text/html", INDEX_HTML);
}

static void handleStatus() {
    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"ap_ip\":\"%s\",\"mode\":\"ap\",\"web\":true}",
        WiFi.softAPIP().toString().c_str()
    );
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "application/json", buf);
}

static const char UPDATE_HTML[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="it"><head><meta charset="utf-8">
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
<h1>Aggiornamento firmware</h1>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin" required>
<button type="submit">Carica firmware</button>
</form>
<p>Usa il file <code>.pio/build/esp32dev/firmware.bin</code>. L'ESP32 si riavvia automaticamente dopo un aggiornamento riuscito.</p>
</main></body></html>)html";

static void handleUpdatePage() {
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send_P(200, "text/html", UPDATE_HTML);
}

static void handleUpdateUpload() {
    HTTPUpload& upload = webServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_END) {
        if (!Update.end(true)) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
    }
}

static void handleUpdateDone() {
    bool ok = !Update.hasError();
    webServer.sendHeader("Connection", "close");
    webServer.send(ok ? 200 : 500, "text/plain",
                   ok ? "Aggiornamento completato. Riavvio..." : "Aggiornamento fallito.");
    if (ok) { delay(500); ESP.restart(); }
}

static void handleData() {
    LiveData d = gData;
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"sim\":%s,\"ebike\":%s,\"gatt\":%s,\"suunto\":%s,\"valid\":%s,"
        "\"speed\":%.2f,\"cadence\":%d,\"power\":%d,\"battery\":%d,"
        "\"odometer\":%.2f,\"ambient\":%.1f,\"bike_light\":%d,"
        "\"locked\":%s,\"charger\":%s,\"reserve\":%s,\"diag\":%s,\"standstill\":%s}",
        SIM_ENABLED     ? "true" : "false",
        ebikeConnected  ? "true" : "false",
        ebikeGattReady  ? "true" : "false",
        suuntoConnected ? "true" : "false",
        d.valid         ? "true" : "false",
        d.speedKmh, d.cadenceRpm, d.powerW, d.batterySoc,
        d.odometerKm, d.ambientLux, d.bikeLight,
        d.systemLocked ? "true" : "false",
        d.chargerConn  ? "true" : "false",
        d.lightReserve ? "true" : "false",
        d.diagActive   ? "true" : "false",
        d.notDriving   ? "true" : "false"
    );
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "application/json", buf);
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    IPAddress apIp(192, 168, 4, 1);
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    WiFi.softAPConfig(apIp, apIp, IPAddress(255, 255, 255, 0));
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, 1, false, 4);

    webServer.on("/",       handleRoot);
    webServer.on("/data",   handleData);
    webServer.on("/status", handleStatus);
    webServer.on("/update", HTTP_GET,  handleUpdatePage);
    webServer.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
    webServer.begin();
    webOk = true;

    NimBLEDevice::init("BoschEBike Bridge");
    NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_SC | BLE_SM_PAIR_AUTHREQ_BOND);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setCustomGapHandler(customGapHandler);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCB());

    // LDI GATT server: stessi UUID della bici → Suunto si connette trasparentemente
    auto* pLdiSvc = pServer->createService(NimBLEUUID(LDI_SVC_UUID));
    pLdiServerChar = pLdiSvc->createCharacteristic(
        NimBLEUUID(LDI_CHAR_UUID),
        NIMBLE_PROPERTY::NOTIFY
    );
    pLdiSvc->start();

    if (SIM_ENABLED) {
        startAdvertisingForSuunto();  // in sim mode il Suunto si connette subito
    } else {
        resetAndAdvertiseForEbike();
    }
}

// ─── Loop ────────────────────────────────────────────────────────────────────
void loop() {
    if (webOk) webServer.handleClient();

    if (!SIM_ENABLED) {
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

    if (flagSuuntoConn) {
        flagSuuntoConn = false;
        suuntoConnected = true;
        nextSimNotifyMs = 0;
    }
    if (flagSuuntoDisconn) {
        flagSuuntoDisconn = false;
        suuntoConnected = false;
        nextSimNotifyMs = 0;
        if (SIM_ENABLED || ebikeGattReady) startAdvertisingForSuunto();
    }

    // Genera dati simulati quando SIM_ENABLED e Suunto connesso (senza bici)
    if (SIM_ENABLED && suuntoConnected &&
        (int32_t)(millis() - nextSimNotifyMs) >= 0) {
        generateAndNotifySimData();
        nextSimNotifyMs = millis() + 500;
    }

    delay(1);
}
