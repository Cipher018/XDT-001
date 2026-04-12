/**
 * @file Datalink.ino
 * @brief GCU Terminal Firmware (XDT-001) — CADI v2 Multi-Vehicle Control
 * @version 0.1.0
 * @author Cipher018
 *
 * Arquitectura de 5 capas:
 *   UI Layer        → ILI9341 + táctil + LEDs + botones Cherry MX
 *   Input Layer     → HID joystick (Bluepad32)
 *   Node Manager    → Estado por nodo, failsafe, link quality
 *   Protocol Layer  → GCU_Command/GCU_Telemetry, CRC16, XXTEA
 *   RF Layer        → NRF24 conmutación CS/CE, TX/RX
 *
 * Refs: GCU_Terminal_Project.md, GCU_Implementaciones.md,
 *       CADI_Compatibility_Analysis.md
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <Bluepad32.h>
#include <Preferences.h>

// ═════════════════════════════════════════════════════════════
//  PINOUT — PCB XDT-001 (GCU_Terminal_Project.md)
// ═════════════════════════════════════════════════════════════

// ── SPI Bus (compartido) ─────────────────────────────────────
#define MOSI_PIN  23
#define MISO_PIN  19
#define SCK_PIN   18

// ── NRF24 x3 — CS y CE por nodo ──────────────────────────────
// Índice: 0=Avión, 1=Quad, 2=Carro
const uint8_t NRF_CS[3] = {25, 27, 13};
const uint8_t NRF_CE[3] = {26, 14, 12};

// ── ILI9341 Display ──────────────────────────────────────────
#define TFT_CS_PIN   4
#define TFT_RST_PIN  5
#define TFT_DC_PIN   16

// ── XPT2046 Touchscreen ─────────────────────────────────────
#define TS_CS_PIN    17
#define TS_IRQ_PIN   15

// ── Botones Cherry MX ────────────────────────────────────────
#define BTN_PREV     32
#define BTN_NEXT     33
#define BTN_ACT      34   // pull-up externo 10kΩ

// ── LEDs indicadores ─────────────────────────────────────────
const uint8_t LED_PINS[3] = {21, 22, 2};  // Avión(verde), Quad(azul), Carro(amarillo)

// ═════════════════════════════════════════════════════════════
//  PROTOCOLO CADI v2
// ═════════════════════════════════════════════════════════════

// ── MSG_TYPE values ──────────────────────────────────────────
#define MSG_CMD_MANUAL  0x02
#define MSG_CMD_GOTO    0x03
#define MSG_CMD_ORBIT   0x04
#define MSG_CMD_RTH     0x05
#define MSG_ARM         0x06
#define MSG_DISARM      0x07
#define MSG_FAILSAFE    0x08
#define MSG_CMD_VTOL    0x0A
#define MSG_CMD_STOP    0x0B
#define MSG_ERROR       0xFF

// ── Magic bytes ──────────────────────────────────────────────
const uint8_t MAGIC_CMD   = 0xAA;
const uint8_t MAGIC_TELEM = 0xBB;

// ── Node IDs ─────────────────────────────────────────────────
const uint8_t NODE_ID_GCU   = 0x00;
const uint8_t NODE_ID_PLANE = 0x01;
const uint8_t NODE_ID_QUAD  = 0x02;
const uint8_t NODE_ID_CAR   = 0x03;
const uint8_t NODE_ID[3]    = {NODE_ID_PLANE, NODE_ID_QUAD, NODE_ID_CAR};

// ── Pipe addresses — una por nodo ────────────────────────────
const byte PIPE_ADDR[3][6] = {"GCU01", "GCU02", "GCU03"};

// ── Structs del protocolo ────────────────────────────────────
// Comando Terminal → Vehículo — 28 bytes (7 words XXTEA)
struct __attribute__((packed)) GCU_Command {
    uint8_t  magic;           // 0xAA
    uint8_t  seq;             // contador incremental
    uint8_t  dst_id;          // nodo destino
    uint8_t  src_id;          // 0x00 = Terminal
    uint8_t  msg_type;        // ver MSG_TYPE
    uint8_t  _pad[3];         // padding → alineación a 4 bytes para XXTEA
    int16_t  targetYaw;       // grados/s * 10
    int16_t  targetThrottle;  // 0-1800
    int16_t  targetPitch;     // grados * 10
    int16_t  targetRoll;      // grados * 10
    int32_t  waypoint_lat;    // grados * 1e7
    int32_t  waypoint_lon;    // grados * 1e7
    int16_t  waypoint_alt;    // metros * 10
    int16_t  declinationX10;  // declinación magnética * 10
    uint16_t crc;             // CRC16-CCITT
};  // Total: 28 bytes ✓

// Telemetría Vehículo → Terminal — 32 bytes (8 words XXTEA)
struct __attribute__((packed)) GCU_Telemetry {
    uint8_t  magic;        // 0xBB
    uint8_t  seq;          // para link quality
    uint8_t  src_id;       // qué vehículo responde
    uint8_t  node_status;  // bit0=armed bit1=gps_fix bit2=bat_low bit3-4=mode
    float    latitude;     // float directo (6 decimales GPS)
    float    longitude;
    int16_t  altitude;     // metros * 10
    int16_t  heading;      // grados * 10
    int16_t  pitch;        // grados * 10
    int16_t  roll;         // grados * 10
    int16_t  gforce;       // G * 100
    int16_t  velocityX;    // m/s * 100
    int16_t  velocityY;    // m/s * 100
    int16_t  velocityZ;    // m/s * 100
    uint16_t crc;
};  // Total: 32 bytes ✓

// ── Compile-time size verification ───────────────────────────
static_assert(sizeof(GCU_Command)   == 28, "GCU_Command must be 28 bytes (7 XXTEA words)");
static_assert(sizeof(GCU_Telemetry) == 32, "GCU_Telemetry must be 32 bytes (8 XXTEA words)");

// ═════════════════════════════════════════════════════════════
//  SEGURIDAD — XXTEA + CRC16 (copia exacta de CADI_G)
// ═════════════════════════════════════════════════════════════

// Clave XXTEA — se sobreescribe con NVS si existe
uint32_t sharedKey[4] = {
    0x58444630, 0x30314B45,
    0x595F3230, 0x32362121
};  // "XDF001KEY_2026!!"

#define XXTEA_DELTA 0x9e3779b9
#define XXTEA_MX (((z>>5^y<<2) + (y>>3^z<<4)) ^ ((sum^y) + (sharedKey[(p&3)^e] ^ z)))

void btea(uint32_t *v, int n) {
    uint32_t y, z, sum;
    unsigned p, rounds, e;
    if (n > 1) {
        rounds = 6 + 52 / n;
        sum = 0;
        z = v[n - 1];
        do {
            sum += XXTEA_DELTA;
            e = (sum >> 2) & 3;
            for (p = 0; p < (unsigned)(n - 1); p++) {
                y = v[p + 1];
                z = v[p] += XXTEA_MX;
            }
            y = v[0];
            z = v[n - 1] += XXTEA_MX;
        } while (--rounds);
    } else if (n < -1) {
        n = -n;
        rounds = 6 + 52 / n;
        sum = rounds * XXTEA_DELTA;
        y = v[0];
        do {
            e = (sum >> 2) & 3;
            for (p = n - 1; p > 0; p--) {
                z = v[p - 1];
                y = v[p] -= XXTEA_MX;
            }
            z = v[n - 1];
            y = v[0] -= XXTEA_MX;
            sum -= XXTEA_DELTA;
        } while (--rounds);
    }
}

// ── CRC16-CCITT — copia exacta de CADI_G ─────────────────────
uint16_t calculateCRC16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

// ═════════════════════════════════════════════════════════════
//  NVS — Carga de clave XXTEA
// ═════════════════════════════════════════════════════════════

Preferences preferences;

void loadXXTEAKey() {
    preferences.begin("gcu_config", true);  // read-only
    if (preferences.isKey("shared_key")) {
        uint8_t nvsKey[16];
        preferences.getBytes("shared_key", nvsKey, 16);
        memcpy(sharedKey, nvsKey, 16);
        Serial.println("[NVS] Clave XXTEA cargada desde flash");
    } else {
        Serial.println("[NVS] Usando clave por defecto");
    }
    preferences.end();
}

void saveXXTEAKey(uint8_t *newKey) {
    preferences.begin("gcu_config", false);  // read-write
    preferences.putBytes("shared_key", newKey, 16);
    preferences.end();
    memcpy(sharedKey, newKey, 16);
    Serial.println("[NVS] Clave XXTEA guardada en flash");
}

// ═════════════════════════════════════════════════════════════
//  NODE MANAGER — Estado por nodo
// ═════════════════════════════════════════════════════════════

#define NODE_COUNT          3
#define FAILSAFE_TIMEOUT_MS 2000  // 2 segundos sin respuesta

struct NodeState {
    bool     connected;       // recibió telemetría recientemente
    uint32_t lastContactMs;   // millis() del último ACK válido
    uint8_t  lastSeq;         // para link quality [DL-04]
    uint32_t lossCount;       // paquetes perdidos
    uint32_t totalCount;      // paquetes totales
    uint8_t  lossRate;        // 0-100%
    GCU_Telemetry lastTelem;  // última telemetría válida recibida
    uint8_t  flightMode;      // MSG_TYPE del modo actual
};

NodeState nodes[NODE_COUNT];
uint8_t   activeNode = 0;
uint8_t   cmdSeq     = 0;

// ── Operación del nodo — modo de control por nodo ────────────
enum ControlMode { MODE_MANUAL = 0, MODE_GOTO = 1, MODE_ORBIT = 2 };
ControlMode currentMode[NODE_COUNT] = {MODE_MANUAL, MODE_MANUAL, MODE_MANUAL};

// ── Waypoint targets (para CMD_GOTO/CMD_ORBIT) ──────────────
float targetLat = 0.0f, targetLon = 0.0f, targetAlt = 0.0f;
float orbitLat  = 0.0f, orbitLon  = 0.0f, orbitAlt  = 0.0f;
float magneticDeclination = 0.0f;

// ── Inicializar estado de nodos ──────────────────────────────
void initNodeStates() {
    for (int i = 0; i < NODE_COUNT; i++) {
        nodes[i].connected     = false;
        nodes[i].lastContactMs = 0;
        nodes[i].lastSeq       = 0;
        nodes[i].lossCount     = 0;
        nodes[i].totalCount    = 0;
        nodes[i].lossRate      = 0;
        nodes[i].flightMode    = MSG_CMD_MANUAL;
        memset(&nodes[i].lastTelem, 0, sizeof(GCU_Telemetry));
    }
}

// ── Actualizar estado tras recibir telemetría válida ─────────
// [DL-04] Link quality stats por nodo — igual que CADI_G
void updateNodeState(uint8_t nodeIndex, const GCU_Telemetry &telem) {
    NodeState &n = nodes[nodeIndex];

    n.connected     = true;
    n.lastContactMs = millis();

    // ── Link quality ─────────────────────────────────────────
    n.totalCount++;
    if (n.lastSeq != 0) {
        uint8_t expected = n.lastSeq + 1;
        if (telem.seq != expected) {
            int lost = (int)telem.seq - (int)expected;
            if (lost < 0) lost += 256;
            n.lossCount  += lost;
            n.totalCount += lost;
        }
    }
    n.lastSeq  = telem.seq;
    n.lossRate = (n.totalCount > 0) ?
                 (uint8_t)((n.lossCount * 100) / n.totalCount) : 0;

    // Reset periódico — evita overflow
    if (n.totalCount > 1000) {
        n.totalCount = 0;
        n.lossCount  = 0;
    }

    // Guardar última telemetría
    memcpy(&n.lastTelem, &telem, sizeof(GCU_Telemetry));
}

// ── Verificar timeouts de todos los nodos ────────────────────
void checkNodeTimeouts() {
    uint32_t now = millis();
    for (int i = 0; i < NODE_COUNT; i++) {
        if (nodes[i].connected) {
            if (now - nodes[i].lastContactMs > FAILSAFE_TIMEOUT_MS) {
                nodes[i].connected = false;
                Serial.printf("[NODE] Nodo %d timeout — failsafe\n", i);
            }
        }
    }
}

// ── LED feedback por estado de nodo ──────────────────────────
void updateLEDs() {
    uint32_t now = millis();
    for (int i = 0; i < NODE_COUNT; i++) {
        if (nodes[i].connected) {
            digitalWrite(LED_PINS[i], HIGH);          // Encendido fijo
        } else {
            digitalWrite(LED_PINS[i], (now / 250) % 2);  // Parpadeo 2Hz
        }
    }
}

// ═════════════════════════════════════════════════════════════
//  RF LAYER — Radio NRF24 + conmutación de nodos
// ═════════════════════════════════════════════════════════════

RF24 radio(NRF_CE[0], NRF_CS[0]);

// ── Inicialización del NRF24 ─────────────────────────────────
// [DL-01] enableDynamicPayloads() REQUERIDO antes de enableAckPayload()
void initRadio() {
    radio.begin();
    radio.setChannel(108);             // Canal fuera del rango WiFi
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setRetries(8, 5);            // delay=8×250µs, 5 reintentos
    radio.enableDynamicPayloads();     // [DL-01] ← CRÍTICO para ACK payload
    radio.enableAckPayload();          // ACK con telemetría adjunta
    radio.enableDynamicAck();
    // setPayloadSize no es necesario con dynamic payloads
}

// ── Cambiar nodo activo ──────────────────────────────────────
// [DL-02] Incluye openReadingPipe(1) para recibir ACK payload
void switchActiveNode(uint8_t nodeIndex) {
    if (nodeIndex >= 3) return;

    // Desactivar nodo actual
    radio.stopListening();
    digitalWrite(NRF_CE[activeNode], LOW);
    digitalWrite(NRF_CS[activeNode], HIGH);

    // Activar nuevo nodo
    activeNode = nodeIndex;
    radio.csn(NRF_CS[activeNode]);
    radio.ce(NRF_CE[activeNode]);

    // Reconfigurar pipes para el nuevo destino
    radio.stopListening();
    radio.openWritingPipe(PIPE_ADDR[activeNode]);
    radio.openReadingPipe(1, PIPE_ADDR[activeNode]);  // [DL-02] ← CRÍTICO

    Serial.printf("[RF] Nodo activo → %d (ID=0x%02X)\n",
                  activeNode, NODE_ID[activeNode]);
}

// ═════════════════════════════════════════════════════════════
//  PROTOCOL LAYER — Empaquetado TX / Recepción RX
// ═════════════════════════════════════════════════════════════

// ── Ensamblar y transmitir un comando ────────────────────────
bool transmitCommand(GCU_Command &cmd) {
    // Asignar campos de protocolo
    cmd.magic  = MAGIC_CMD;
    cmd.seq    = cmdSeq++;
    cmd.src_id = NODE_ID_GCU;
    cmd.dst_id = NODE_ID[activeNode];

    // Paso 1: CRC sobre todo el struct excepto el campo crc
    cmd.crc = calculateCRC16((uint8_t*)&cmd, sizeof(GCU_Command) - 2);

    // Paso 2: Cifrar con XXTEA — 7 words (28 bytes)
    btea((uint32_t*)&cmd, 7);

    // Paso 3: Transmitir
    radio.stopListening();
    bool ok = radio.write(&cmd, sizeof(GCU_Command));

    return ok;
}

// ── Leer telemetría del ACK payload ──────────────────────────
// [DL-03] Verificación completa: magic + src_id + CRC
bool readAckTelemetry(uint8_t nodeIndex, GCU_Telemetry &telem) {
    if (!radio.isAckPayloadAvailable()) return false;

    uint8_t len = radio.getDynamicPayloadSize();
    if (len != sizeof(GCU_Telemetry)) {
        // Tamaño incorrecto — limpiar buffer
        uint8_t dummy[32];
        radio.read(dummy, len);
        return false;
    }

    radio.read(&telem, sizeof(GCU_Telemetry));

    // Descifrar XXTEA — 8 words (32 bytes), n negativo = descifrar
    btea((uint32_t*)&telem, -8);

    // [DL-03] Verificar MAGIC
    if (telem.magic != MAGIC_TELEM) return false;

    // [DL-03] Verificar src_id — solo aceptar del nodo esperado
    if (telem.src_id != NODE_ID[nodeIndex]) return false;

    // [DL-03] Verificar CRC
    uint16_t crcCalculado = calculateCRC16(
        (uint8_t*)&telem,
        sizeof(GCU_Telemetry) - 2
    );
    if (telem.crc != crcCalculado) return false;

    return true;
}

// ═════════════════════════════════════════════════════════════
//  INPUT LAYER — HID Joystick (Bluepad32)
// ═════════════════════════════════════════════════════════════

ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (!myControllers[i]) {
            myControllers[i] = ctl;
            Serial.printf("[HID] Controller connected slot %d\n", i);
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            Serial.printf("[HID] Controller disconnected slot %d\n", i);
            break;
        }
    }
}

// ── Estructura de comandos de control ────────────────────────
struct ControlCommands {
    float yaw;
    float throttle;
    float pitch;
    float roll;
};

// ── Leer joystick analógico ──────────────────────────────────
ControlCommands readJoystick() {
    ControlCommands cmd = {0, 0, 0, 0};

    if (myControllers[0] && myControllers[0]->isConnected()) {
        ControllerPtr c = myControllers[0];

        // Stick derecho → pitch/roll (actitud)
        cmd.pitch = -(float)c->axisRY() / 512.0f * 30.0f;   // ±30°
        cmd.roll  =  (float)c->axisRX() / 512.0f * 45.0f;   // ±45°

        // Stick izquierdo → throttle/yaw
        cmd.throttle = (-(float)c->axisY() / 512.0f + 1.0f) * 900.0f;  // 0-1800
        cmd.yaw      =  (float)c->axisX() / 512.0f * 180.0f;           // ±180°/s

        // ── Botones de modo ──────────────────────────────────
        if (c->triangle()) nodes[activeNode].flightMode = MSG_CMD_RTH;
        if (c->cross())    nodes[activeNode].flightMode = MSG_CMD_MANUAL;
        if (c->circle())   nodes[activeNode].flightMode = MSG_CMD_ORBIT;
        if (c->square())   nodes[activeNode].flightMode = MSG_CMD_GOTO;

        // ── Arm / Disarm ─────────────────────────────────────
        // L1 + R1 simultáneo = ARM, L2 + R2 = DISARM
        if (c->l1() && c->r1()) nodes[activeNode].flightMode = MSG_ARM;
        if (c->l2() && c->r2()) nodes[activeNode].flightMode = MSG_DISARM;
    }

    return cmd;
}

// ═════════════════════════════════════════════════════════════
//  UI LAYER — ILI9341 + Táctil + Botones
// ═════════════════════════════════════════════════════════════

Adafruit_ILI9341    tft(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);
XPT2046_Touchscreen touch(TS_CS_PIN, TS_IRQ_PIN);

enum UI_Page { PAGE_TELEMETRY, PAGE_MISSION, PAGE_CONFIG };
UI_Page currentPage = PAGE_TELEMETRY;

// ── Forward declarations UI ──────────────────────────────────
void drawDashboard();
void drawTelemetryPage();
void drawNodeSelector();
void handleTouchInput();
void readButtons();

// ── MSG_TYPE → nombre legible ────────────────────────────────
// [DL-05]
const char* getMsgTypeName(uint8_t msgType) {
    switch (msgType) {
        case MSG_CMD_MANUAL: return "MANUAL";
        case MSG_CMD_GOTO:   return "GOTO";
        case MSG_CMD_ORBIT:  return "ORBIT";
        case MSG_CMD_RTH:    return "RTH";
        case MSG_CMD_VTOL:   return "VTOL";
        case MSG_CMD_STOP:   return "STOP";
        case MSG_ARM:        return "ARM";
        case MSG_DISARM:     return "DISARM";
        default:             return "???";
    }
}

// ── Selector de nodos táctil ─────────────────────────────────
// [DL-06] 3 zonas de ~106px cada una en la parte superior
#define NODE_SELECTOR_Y  0
#define NODE_SELECTOR_H  28

void drawNodeSelector() {
    const char* labels[3] = {"AVION", "QUAD", "CARRO"};
    uint16_t colors[3] = {ILI9341_GREEN, ILI9341_BLUE, ILI9341_YELLOW};

    for (int i = 0; i < 3; i++) {
        uint16_t x = i * 106;
        uint16_t color = (i == activeNode) ? colors[i] : ILI9341_DARKGREY;
        tft.fillRect(x, NODE_SELECTOR_Y, 105, NODE_SELECTOR_H, color);
        tft.drawRect(x, NODE_SELECTOR_Y, 105, NODE_SELECTOR_H, ILI9341_WHITE);
        tft.setTextSize(1);
        tft.setTextColor(ILI9341_WHITE);
        tft.setCursor(x + 10, NODE_SELECTOR_Y + 10);
        tft.printf("%s %s", labels[i],
                   nodes[i].connected ? "ON" : "--");
    }
}

// ── Página de telemetría ─────────────────────────────────────
// [DL-05] Muestra datos reales del nodo activo
void drawTelemetryPage() {
    GCU_Telemetry &t = nodes[activeNode].lastTelem;
    NodeState     &n = nodes[activeNode];

    // Fondo
    tft.fillScreen(ILI9341_BLACK);

    // Selector de nodos
    drawNodeSelector();

    // Header — estado conexión + link quality
    tft.fillRect(0, 30, 320, 24, n.connected ?
                 ILI9341_DARKGREEN : ILI9341_RED);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(5, 34);
    tft.printf("NODE %d  %s  LINK:%d%%",
               activeNode + 1,
               n.connected ? "CONN" : "LOST",
               100 - n.lossRate);

    // Telemetría
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);

    tft.setCursor(10, 65);
    tft.printf("Alt: %.1fm   Vel: %.1fm/s",
               t.altitude / 10.0f,
               sqrtf(sq(t.velocityX / 100.0f) +
                     sq(t.velocityY / 100.0f)));

    tft.setCursor(10, 90);
    tft.printf("Pitch:%.1f  Roll:%.1f",
               t.pitch / 10.0f,
               t.roll  / 10.0f);

    tft.setCursor(10, 115);
    tft.printf("Hdg: %.1f   G:%.2f",
               t.heading / 10.0f,
               t.gforce  / 100.0f);

    tft.setCursor(10, 140);
    tft.printf("Lat: %.6f", t.latitude);

    tft.setCursor(10, 165);
    tft.printf("Lon: %.6f", t.longitude);

    // Comando activo
    tft.fillRect(0, 210, 320, 30, ILI9341_NAVY);
    tft.setCursor(5, 215);
    tft.printf("CMD: %s", getMsgTypeName(nodes[activeNode].flightMode));
}

// ── Dashboard dispatcher ─────────────────────────────────────
void drawDashboard() {
    switch (currentPage) {
        case PAGE_TELEMETRY:
            drawTelemetryPage();
            break;
        case PAGE_MISSION:
            tft.fillScreen(ILI9341_BLACK);
            drawNodeSelector();
            tft.fillRect(0, 30, 320, 24, ILI9341_NAVY);
            tft.setTextSize(2);
            tft.setTextColor(ILI9341_WHITE);
            tft.setCursor(5, 34);
            tft.print("MISSION");
            break;
        case PAGE_CONFIG:
            tft.fillScreen(ILI9341_BLACK);
            drawNodeSelector();
            tft.fillRect(0, 30, 320, 24, ILI9341_NAVY);
            tft.setTextSize(2);
            tft.setTextColor(ILI9341_WHITE);
            tft.setCursor(5, 34);
            tft.print("CONFIG");
            break;
    }
}

// ── Manejo de input táctil ───────────────────────────────────
// [DL-06]
void handleTouchInput() {
    if (!touch.tirqTouched() || !touch.touched()) return;

    TS_Point p = touch.getPoint();
    // Mapear coordenadas del touch a píxeles de pantalla
    int16_t tx = map(p.x, 300, 3800, 0, 320);
    int16_t ty = map(p.y, 300, 3800, 0, 240);

    // Verificar si toca el selector de nodos
    if (ty >= NODE_SELECTOR_Y && ty <= NODE_SELECTOR_Y + NODE_SELECTOR_H) {
        uint8_t tappedNode = tx / 106;
        if (tappedNode < 3 && tappedNode != activeNode) {
            switchActiveNode(tappedNode);
            drawDashboard();
        }
    }
}

// ── Lectura de botones Cherry MX ─────────────────────────────
// Debounce simple — BTN_PREV/BTN_NEXT cambian nodo, BTN_ACT cambia página
void readButtons() {
    static uint32_t lastBtnMs = 0;
    uint32_t now = millis();
    if (now - lastBtnMs < 200) return;  // debounce 200ms

    if (digitalRead(BTN_PREV) == LOW) {
        lastBtnMs = now;
        uint8_t next = (activeNode == 0) ? 2 : activeNode - 1;
        switchActiveNode(next);
        drawDashboard();
    }

    if (digitalRead(BTN_NEXT) == LOW) {
        lastBtnMs = now;
        uint8_t next = (activeNode >= 2) ? 0 : activeNode + 1;
        switchActiveNode(next);
        drawDashboard();
    }

    if (digitalRead(BTN_ACT) == LOW) {
        lastBtnMs = now;
        // Ciclar entre páginas
        currentPage = (UI_Page)((currentPage + 1) % 3);
        drawDashboard();
    }
}

// ── Inicializar display ──────────────────────────────────────
void initDisplay() {
    tft.begin();
    tft.setRotation(1);   // Landscape
    touch.begin();
    tft.fillScreen(ILI9341_BLACK);

    // Splash screen
    tft.setTextSize(3);
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(40, 80);
    tft.print("GCU Terminal");
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(80, 130);
    tft.print("XDT-001 v0.1");
    delay(1500);

    drawDashboard();
}

// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    Serial.println("[GCU] Iniciando...");

    // ── GPIO ─────────────────────────────────────────────────
    for (int i = 0; i < 3; i++) {
        pinMode(NRF_CS[i], OUTPUT);
        digitalWrite(NRF_CS[i], HIGH);
        pinMode(NRF_CE[i], OUTPUT);
        digitalWrite(NRF_CE[i], LOW);
        pinMode(LED_PINS[i], OUTPUT);
    }

    pinMode(BTN_PREV, INPUT_PULLUP);
    pinMode(BTN_NEXT, INPUT_PULLUP);
    pinMode(BTN_ACT,  INPUT);          // pull-up externo 10kΩ

    // ── NVS ──────────────────────────────────────────────────
    loadXXTEAKey();

    // ── Radio ────────────────────────────────────────────────
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
    initRadio();
    switchActiveNode(0);  // Iniciar con Avión

    // ── Node states ──────────────────────────────────────────
    initNodeStates();

    // ── Display ──────────────────────────────────────────────
    initDisplay();

    // ── HID joystick ─────────────────────────────────────────
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();

    Serial.println("[GCU] Sistema inicializado");
}

// ═════════════════════════════════════════════════════════════
//  MAIN LOOP — Ciclo completo 20Hz
// ═════════════════════════════════════════════════════════════

void loop() {
    // ── 1. Leer input HID (joystick) ─────────────────────────
    BP32.update();
    ControlCommands input = readJoystick();

    // ── 2. Armar comando según modo actual ───────────────────
    GCU_Command gcuCmd;
    memset(&gcuCmd, 0, sizeof(GCU_Command));

    switch (currentMode[activeNode]) {
        case MODE_MANUAL:
            gcuCmd.msg_type       = MSG_CMD_MANUAL;
            gcuCmd.targetYaw      = (int16_t)(input.yaw      * 10);
            gcuCmd.targetThrottle = (int16_t)(input.throttle);
            gcuCmd.targetPitch    = (int16_t)(input.pitch     * 10);
            gcuCmd.targetRoll     = (int16_t)(input.roll      * 10);
            break;

        case MODE_GOTO:
            gcuCmd.msg_type     = MSG_CMD_GOTO;
            gcuCmd.waypoint_lat = (int32_t)(targetLat * 1e7);
            gcuCmd.waypoint_lon = (int32_t)(targetLon * 1e7);
            gcuCmd.waypoint_alt = (int16_t)(targetAlt * 10);
            break;

        case MODE_ORBIT:
            gcuCmd.msg_type     = MSG_CMD_ORBIT;
            gcuCmd.waypoint_lat = (int32_t)(orbitLat * 1e7);
            gcuCmd.waypoint_lon = (int32_t)(orbitLon * 1e7);
            gcuCmd.waypoint_alt = (int16_t)(orbitAlt * 10);
            break;
    }

    // Override msg_type si el controller forzó un modo especial
    if (nodes[activeNode].flightMode != MSG_CMD_MANUAL) {
        gcuCmd.msg_type = nodes[activeNode].flightMode;
    }

    // Declinación magnética siempre incluida
    gcuCmd.declinationX10 = (int16_t)(magneticDeclination * 10);

    // ── 3. Transmitir al nodo activo ─────────────────────────
    bool ok = transmitCommand(gcuCmd);

    // ── 4. Leer telemetría del ACK ───────────────────────────
    if (ok) {
        GCU_Telemetry telem;
        if (readAckTelemetry(activeNode, telem)) {
            updateNodeState(activeNode, telem);
        }
    }

    // ── 5. Verificar timeouts de todos los nodos ─────────────
    checkNodeTimeouts();

    // ── 6. Actualizar LEDs ───────────────────────────────────
    updateLEDs();

    // ── 7. Actualizar pantalla (cada 5 ciclos = 100ms) ───────
    static uint8_t displayCounter = 0;
    if (++displayCounter >= 5) {
        displayCounter = 0;
        drawDashboard();
    }

    // ── 8. Leer touchscreen ──────────────────────────────────
    handleTouchInput();

    // ── 9. Leer botones Cherry MX ────────────────────────────
    readButtons();

    // ── 10. Mantener 20Hz ────────────────────────────────────
    delay(50);
}
