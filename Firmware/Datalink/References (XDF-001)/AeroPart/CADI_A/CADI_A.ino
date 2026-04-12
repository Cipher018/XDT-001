/**
 * @file CADI_A.ino
 * @brief XDF-001 Aero Firmware — CADI v2 (GCU Protocol)
 * @version 2.0.0
 * @author Cipher018
 *
 * Actualización CADI v1 → v2:
 *   [CA-01] SecureCommand/SecureTelemetry reemplazados por GCU_Command/GCU_Telemetry
 *   [CA-02] pushTelemetryAck() actualizado para GCU_Telemetry (32 bytes, float lat/lon)
 *   [CA-03] loop() receptor RF actualizado: GCU_Command 28 bytes, 7 words XXTEA
 *   [CA-04] Radio setup: canal 108, pipe "GCU01", enableDynamicPayloads()
 *   [CA-05] buildNodeStatus() — node_status byte con bits de estado
 *
 * Inalterado:
 *   KalmanAttitude (pitch, roll, yaw), KalmanVelocity (Vx, Vy, Vz),
 *   updateVelocityFusion(), readGPS(), readMPU(), updateTelemetry(),
 *   SlewLimiter, PIDController, mezclador de servos, Failsafe RTL,
 *   SET_KEY NVS handler, PCA9685.
 *
 * Refs: GCU_Implementaciones.md §FASE2, GCU_Terminal_Project.md,
 *       CADI_Compatibility_Analysis.md
 */

#include <mpu9250.h>
#include <Preferences.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>
#include <RF24.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <nRF24L01.h>

// ═══════════════════════════════════════════════════════
// KALMAN FILTER — ACTITUD (pitch, roll, yaw)
// Modelo de 2 estados por eje: [ángulo, bias_gyro]
//
// Predicción  (con giróscopo):
//   angle += (gyroRate - bias) * dt
//   P += Q
//
// Corrección  (con acelerómetro / magnetómetro):
//   K = P / (P + R)
//   angle += K * (medición - angle)
//   bias  += K * (medición - angle)
//   P = (1 - K) * P
//
// Q_angle: ruido del modelo — mayor → sigue al gyro más rápido
// Q_bias:  ruido del bias   — mayor → corrige bias más rápido
// R_meas:  ruido medición   — mayor → confía menos en accel/mag
// ═══════════════════════════════════════════════════════
class KalmanAttitude {
public:
  KalmanAttitude(float q_angle = 0.001f,
                 float q_bias  = 0.003f,
                 float r_meas  = 0.03f)
    : Q_angle(q_angle), Q_bias(q_bias), R_meas(r_meas)
    , angle(0.0f), bias(0.0f)
  {
    P[0][0] = 0.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 0.0f;
  }

  float update(float gyroRate, float measurement, float dt) {
    // ── Predicción ───────────────────────────────────
    float rate  = gyroRate - bias;
    angle      += rate * dt;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // ── Corrección ───────────────────────────────────
    float S  = P[0][0] + R_meas;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    float innovation = measurement - angle;
    angle += K0 * innovation;
    bias  += K1 * innovation;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;

    return angle;
  }

  void  setAngle(float a) { angle = a; }
  float getAngle()  const { return angle; }
  float getBias()   const { return bias;  }

private:
  float Q_angle, Q_bias, R_meas;
  float angle, bias;
  float P[2][2];
};

// Una instancia por eje — Yaw usa R_meas más alto (magnetómetro ruidoso)
KalmanAttitude kalmanPitch(0.001f, 0.003f, 0.03f);
KalmanAttitude kalmanRoll (0.001f, 0.003f, 0.03f);
KalmanAttitude kalmanYaw  (0.001f, 0.003f, 0.5f);

// ═══════════════════════════════════════════════════════
// ESTIMACIÓN DE VELOCIDAD — Fusión GPS + IMU + Kalman
//
// GPS_WEIGHT  : fracción GPS en la fusión (0.3 = 30% GPS, 70% IMU)
// MAX_VEL_GPS : guard de velocidad imposible (descarta fix corrupto)
// VEL_SMOOTH  : tamaño del buffer de suavizado
// ═══════════════════════════════════════════════════════
const float GPS_WEIGHT      = 0.3f;
const float MAX_VEL_GPS     = 60.0f;
const int   VEL_SMOOTH_SIZE = 3;

class KalmanVelocity {
public:
  KalmanVelocity(float q = 0.1f, float r = 1.5f)
    : _q(q), _r(r), _x(0.0f), _p(1.0f) {}

  float update(float measurement) {
    _p += _q;
    float k = _p / (_p + _r);
    _x = _x + k * (measurement - _x);
    _p = (1.0f - k) * _p;
    return _x;
  }

  void  setState(float x) { _x = x; _p = 1.0f; }
  float getState()  const { return _x; }

private:
  float _q, _r, _x, _p;
};

KalmanVelocity kfVx, kfVy, kfVz;

// ═══════════════════════════════════════════════════════
// HARDWARE
// ═══════════════════════════════════════════════════════

// --- Throttle ESC ---
Servo esc;
#define ESC_PIN 14

// --- Servo positions ---
int servo1Pos = 90, servo2Pos = 90, servo3Pos = 90, servo4Pos = 90;
int servo5Pos = 90, servo6Pos = 90, servo7Pos = 90, servo8Pos = 90;

// --- Sensors ---
TinyGPSPlus   gps;
bfs::Mpu9250  mpu;

#define GPS_SERIAL Serial2
#define GPS_RX 17
#define GPS_TX 16

// --- Radio — [CA-04] Canal 108, pipe GCU01 ---
// NOTE: CE/CSN son los pines físicos del ESP32 del avión (XDF-001)
//       Estos NO son los mismos pines del GCU terminal (XDT-001)
#define CE_PIN  5
#define CSN_PIN 4
RF24 radio(CE_PIN, CSN_PIN);
const byte PIPE_GCU01[6] = "GCU01";  // [CA-04] Pipe del protocolo GCU v2

// ═══════════════════════════════════════════════════════
// PROTOCOLO CADI v2 — GCU_Command / GCU_Telemetry
// [CA-01] Reemplaza SecureCommand (24B/6 words) y SecureTelemetry
// ═══════════════════════════════════════════════════════

// NODE_ID del avión
#define NODE_ID_GCU   0x00
#define NODE_ID_PLANE 0x01

// MSG_TYPE values
#define MSG_CMD_MANUAL  0x02
#define MSG_CMD_GOTO    0x03
#define MSG_CMD_ORBIT   0x04
#define MSG_CMD_RTH     0x05
#define MSG_ARM         0x06
#define MSG_DISARM      0x07
#define MSG_FAILSAFE    0x08
#define MSG_ERROR       0xFF

const uint8_t MAGIC_CMD   = 0xAA;
const uint8_t MAGIC_TELEM = 0xBB;

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
    uint8_t  src_id;       // 0x01 = Avión
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

// Compile-time size verification
static_assert(sizeof(GCU_Command)   == 28, "GCU_Command must be 28 bytes (7 XXTEA words)");
static_assert(sizeof(GCU_Telemetry) == 32, "GCU_Telemetry must be 32 bytes (8 XXTEA words)");

// Instancias del protocolo
GCU_Command   gcuCmd;
GCU_Telemetry gcuTelem;
uint8_t       telemSeq = 0;

// ── Target angles/throttle desde GCU ─────────────────────────
int16_t targetRoll     = 0;
int16_t targetPitch    = 0;
int16_t targetYaw      = 0;
int16_t targetThrottle = 0;
uint8_t currentMsgType = MSG_CMD_MANUAL;  // modo actual recibido

// ── Home Coordinates para RTL ─────────────────────────────────
float homeLat = 0.0f;
float homeLon = 0.0f;

// ═══════════════════════════════════════════════════════
// SEGURIDAD DE COMUNICACIONES (XXTEA + CRC16)
// Copia exacta de CADI_G — clave compartida
// ═══════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════
// HARDWARE — PCA9685 + ESC
// ═══════════════════════════════════════════════════════
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_CH_YAW     0
#define SERVO_CH_ROLL_L  1
#define SERVO_CH_ROLL_R  2
#define SERVO_CH_FLAP_L  3
#define SERVO_CH_FLAP_R  4
#define SERVO_CH_PITCH_L 5
#define SERVO_CH_PITCH_R 6
#define SERVO_PULSE_MIN  102
#define SERVO_PULSE_MAX  512
#define SERVO_FREQUENCY   50

void setServoAngle(uint8_t channel, int angle) {
    angle = constrain(angle, 0, 180);
    int pulse = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    pwm.setPWM(channel, 0, pulse);
}

// ═══════════════════════════════════════════════════════
// VARIABLES DE ESTADO
// ═══════════════════════════════════════════════════════
float latitude   = 0.0f;
float longitude  = 0.0f;
float altitudeM  = 0.0f;
float speedKmph  = 0.0f;

float pitchDeg    = 0.0f;
float rollDeg     = 0.0f;
float yawDeg      = 0.0f;
float totalGForce = 0.0f;

// Velocidad estimada en m/s (marco NED local)
float velX = 0.0f;   // Este
float velY = 0.0f;   // Norte
float velZ = 0.0f;   // Arriba

// Estado para estimación de velocidad GPS
float lastLat = 0.0f, lastLon = 0.0f, lastAlt = 0.0f;
unsigned long lastGPSVelTime = 0;
bool firstGPSFix = true;

// Timestamp IMU para dt preciso
unsigned long lastIMUTime = 0;

// Declinación magnética para corrección del Yaw
float magneticDeclinationDeg = -6.0f;

// Timestamp último paquete RF válido (para failsafe)
unsigned long lastRFTime = 0;

// Estado IMU
bool imuActive = false;

// ═══════════════════════════════════════════════════════
// FLY-BY-WIRE & PID (LAZO INTERNO)
// ═══════════════════════════════════════════════════════
class SlewLimiter {
    float current;
    float maxRate;
public:
    SlewLimiter(float rate) : current(0), maxRate(rate) {}
    float update(float target, float dt) {
        float delta = target - current;
        float maxDelta = maxRate * dt;
        if      (delta >  maxDelta) current += maxDelta;
        else if (delta < -maxDelta) current -= maxDelta;
        else                        current  = target;
        return current;
    }
};

class PIDController {
public:
    float kp, ki, kd;
    float integralMax;
    float integral;
    float lastMeasured;
    bool  firstRun;

    PIDController(float p, float i, float d, float iMax) :
        kp(p), ki(i), kd(d), integralMax(iMax),
        integral(0), lastMeasured(0), firstRun(true) {}

    float compute(float setpoint, float measured, float dt) {
        if (firstRun) { lastMeasured = measured; firstRun = false; }

        float error = setpoint - measured;
        float pOut  = kp * error;

        integral += error * dt;
        if      (integral >  integralMax) integral =  integralMax;
        else if (integral < -integralMax) integral = -integralMax;
        float iOut = ki * integral;

        float dMeasured = (measured - lastMeasured) / dt;
        float dOut = -kd * dMeasured;
        lastMeasured = measured;

        return pOut + iOut + dOut;
    }
};

// ── Controladores ─────────────────────────────────────────────
SlewLimiter   slewRoll(90.0f);
SlewLimiter   slewPitch(90.0f);
PIDController pidRoll (1.2f, 0.1f, 0.2f, 20.0f);
PIDController pidPitch(1.5f, 0.1f, 0.2f, 20.0f);

// ═══════════════════════════════════════════════════════
// [CA-05] buildNodeStatus — node_status byte
// bit0: armed  (throttle > 0)
// bit1: gps_fix (fix GPS válido)
// bit2: bat_low (reservado — lógica futura)
// bit3-4: mode  (00=manual, 01=waypoint, 10=orbit, 11=rtl)
// ═══════════════════════════════════════════════════════
uint8_t buildNodeStatus() {
    uint8_t status = 0;

    // bit0: armed — activo si el throttle supera umbral mínimo
    if (targetThrottle > 0) status |= 0x01;

    // bit1: gps_fix — válido si tenemos coordenadas no-cero
    if (latitude != 0.0f && longitude != 0.0f) status |= 0x02;

    // bit2: bat_low — reservado (TODO: implementar cuando se agregue sensor)
    // status |= 0x00;

    // bit3-4: modo de vuelo actual
    uint8_t modeField = 0x00;  // default: manual
    switch (currentMsgType) {
        case MSG_CMD_GOTO:   modeField = 0x01; break;
        case MSG_CMD_ORBIT:  modeField = 0x02; break;
        case MSG_CMD_RTH:    modeField = 0x03; break;
        default:             modeField = 0x00; break;
    }
    status |= (modeField << 3);

    return status;
}

// ═══════════════════════════════════════════════════════
// [CA-02] pushTelemetryAck — GCU_Telemetry en ACK payload
// Reemplaza pushTelemetryAck() de CADI v1 (SecureTelemetry 28B)
// ═══════════════════════════════════════════════════════
void pushTelemetryAck() {
    // Actualizar variable globales de telemetría desde sensores
    updateTelemetry();

    // Rellenar struct GCU_Telemetry
    gcuTelem.magic       = MAGIC_TELEM;
    gcuTelem.seq         = telemSeq++;
    gcuTelem.src_id      = NODE_ID_PLANE;  // 0x01 — Avión
    gcuTelem.node_status = buildNodeStatus();  // [CA-05]

    // Campos de posición y telemetría con misma escala int16 de CADI_A original
    gcuTelem.latitude  = latitude;
    gcuTelem.longitude = longitude;
    gcuTelem.altitude  = (int16_t)constrain(altitudeM   * 10.0f,  -32767, 32767);
    gcuTelem.heading   = (int16_t)constrain(yawDeg      * 10.0f,       0, 35990);
    gcuTelem.pitch     = (int16_t)constrain(pitchDeg    * 10.0f,   -9000,  9000);
    gcuTelem.roll      = (int16_t)constrain(rollDeg     * 10.0f,  -18000, 18000);
    gcuTelem.gforce    = (int16_t)constrain(totalGForce * 100.0f,      0,  3200);
    gcuTelem.velocityX = (int16_t)constrain(velX        * 100.0f,  -5000,  5000);
    gcuTelem.velocityY = (int16_t)constrain(velY        * 100.0f,  -5000,  5000);
    gcuTelem.velocityZ = (int16_t)constrain(velZ        * 100.0f,  -5000,  5000);

    // ── ORDEN CRÍTICO: NO REORDENAR ──────────────────────────
    // Paso 1: CRC sobre datos en claro (antes de cifrar)
    gcuTelem.crc = calculateCRC16(
        (uint8_t*)&gcuTelem,
        sizeof(GCU_Telemetry) - 2
    );
    // Paso 2: Cifrar paquete completo — 8 words (32 bytes)
    btea((uint32_t*)&gcuTelem, 8);
    // Paso 3: Colocar en buffer ACK
    radio.writeAckPayload(1, &gcuTelem, sizeof(GCU_Telemetry));
}

// ═══════════════════════════════════════════════════════
// SET_KEY HANDLER — NVS (igual que CADI v1)
// Formato serial: SET_KEY:16characterkey
// ═══════════════════════════════════════════════════════
static char    setKeyLineBuf[32];
static uint8_t setKeyLineIdx = 0;

void handleSetKeyCommand(const char *line) {
    if (strncmp(line, "SET_KEY:", 8) != 0) return;

    const char *keyStr = line + 8;
    size_t keyLen = strlen(keyStr);

    uint8_t keyBytes[16] = {0};
    size_t copyLen = keyLen < 16 ? keyLen : 16;
    memcpy(keyBytes, keyStr, copyLen);

    Preferences prefs;
    prefs.begin("pairing", false);
    prefs.putBytes("shared_key", keyBytes, 16);
    prefs.end();

    memcpy(sharedKey, keyBytes, 16);
    Serial.println("[SEC] New key stored in NVS. Restarting...");
    delay(200);
    ESP.restart();
}

void parseSetKeySerial() {
    while (Serial.available()) {
        char ch = (char)Serial.peek();
        if ((uint8_t)ch == MAGIC_CMD || (uint8_t)ch == MAGIC_TELEM) break;
        Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (setKeyLineIdx > 0) {
                setKeyLineBuf[setKeyLineIdx] = '\0';
                handleSetKeyCommand(setKeyLineBuf);
                setKeyLineIdx = 0;
            }
        } else if (setKeyLineIdx < 30) {
            setKeyLineBuf[setKeyLineIdx++] = ch;
        }
    }
}

// ═══════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial.println("[CADI_A] Iniciando v2.0...");

    GPS_SERIAL.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

    Wire.begin(21, 22);
    Wire.setClock(400000);

    // ── MPU9250 ─────────────────────────────────────────────
    mpu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

    int mpuRetries = 10;
    while (!mpu.Begin() && mpuRetries > 0) {
        delay(100);
        mpuRetries--;
    }
    imuActive = (mpuRetries > 0);

    if (imuActive) {
        mpu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_8G);
        mpu.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_500DPS);
        mpu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_20HZ);
        mpu.ConfigSrd(19);  // ~50Hz
        delay(1000);

        // Inicializar Kalman con primera lectura del acelerómetro
        mpu.Read();
        float ax = mpu.accel_x_mps2();
        float ay = mpu.accel_y_mps2();
        float az = mpu.accel_z_mps2();
        float initPitch = atan2f(ay, sqrtf(ax*ax + az*az)) * 57.2958f;
        float initRoll  = atan2f(-ax, az) * 57.2958f;
        kalmanPitch.setAngle(initPitch);
        kalmanRoll.setAngle(initRoll);
        kalmanYaw.setAngle(0.0f);
        lastIMUTime = millis();
    }

    // ── PCA9685 ──────────────────────────────────────────────
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQUENCY);

    // ── ESC ──────────────────────────────────────────────────
    esc.attach(ESC_PIN);

    // ── NVS — Clave XXTEA ────────────────────────────────────
    Preferences preferences;
    preferences.begin("pairing", false);
    if (preferences.isKey("shared_key")) {
        preferences.getBytes("shared_key", sharedKey, 16);
        Serial.println("[NVS] Clave XXTEA cargada desde flash");
    } else {
        Serial.println("[NVS] Usando clave por defecto");
    }
    preferences.end();

    // ── Radio — [CA-04] Canal 108 + pipe GCU01 ──────────────
    if (!radio.begin()) {
        Serial.println("[RF] ERROR: radio.begin() failed");
        while (1);
    }
    radio.setChannel(108);               // [CA-04] Canal unificado con terminal
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.setRetries(3, 5);
    radio.enableDynamicPayloads();       // [CA-04] Requerido para GCU_Telemetry
    radio.enableAckPayload();
    radio.openReadingPipe(1, PIPE_GCU01);  // [CA-04] Pipe del protocolo GCU v2
    radio.openWritingPipe(PIPE_GCU01);
    radio.startListening();

    // Pre-cargar primera telemetría en ACK buffer
    pushTelemetryAck();

    lastRFTime = millis();
    Serial.println("[CADI_A] Sistema listo");
}

// ═══════════════════════════════════════════════════════
// LOOP PRINCIPAL
// ═══════════════════════════════════════════════════════
void loop() {
    // ── Parsear comandos SET_KEY desde serial ───────────────
    parseSetKeySerial();

    // ── Leer GPS ─────────────────────────────────────────────
    readGPS();

    // ═══════════════════════════════════════════════════════
    // FAILSAFE: RTL AUTÓNOMO si no hay señal RF > 1s
    // ═══════════════════════════════════════════════════════
    if (millis() - lastRFTime > 1000) {
        if (homeLat != 0.0f && homeLon != 0.0f &&
            latitude != 0.0f && longitude != 0.0f) {
            float dLat = (homeLat - latitude) * 111320.0f;
            float dLon = (homeLon - longitude) * 111320.0f *
                         cosf(latitude * PI / 180.0f);
            float distanceToHome = sqrtf(dLat*dLat + dLon*dLon);
            float bearingToHome  = atan2f(dLon, dLat) * 180.0f / PI;

            float currentYaw = yawDeg;
            if (currentYaw > 180.0f) currentYaw -= 360.0f;

            float yawError = bearingToHome - currentYaw;
            if (yawError >  180.0f) yawError -= 360.0f;
            if (yawError < -180.0f) yawError += 360.0f;

            if (distanceToHome > 30.0f) {
                // RTL Mode: volar hacia la base
                targetRoll     = (int16_t)constrain(yawError * 0.8f, -35.0f, 35.0f);
                targetPitch    = 5;
                targetYaw      = 0;
                targetThrottle = 120;
            } else {
                // LOITER Mode: órbita autónoma alrededor del home
                static float lastRtlRadiusErr = 0.0f;
                const float RTL_ORBIT_RADIUS = 30.0f;
                const float RTL_KP_RADIUS    = 0.6f;
                const float RTL_KD_RADIUS    = 0.2f;
                const float RTL_BASE_ROLL    = 20.0f;

                float radiusErr   = distanceToHome - RTL_ORBIT_RADIUS;
                float radiusDeriv = radiusErr - lastRtlRadiusErr;
                lastRtlRadiusErr  = radiusErr;

                float rollCorrection = RTL_KP_RADIUS * radiusErr +
                                       RTL_KD_RADIUS * radiusDeriv;
                targetRoll     = (int16_t)constrain(RTL_BASE_ROLL + rollCorrection,
                                                    -35.0f, 35.0f);
                targetPitch    = 3;
                targetYaw      = 0;
                targetThrottle = 100;
            }
        } else {
            // Fallback: planeo controlado si no hay GPS/home
            targetRoll     = 0;
            targetPitch    = 5;
            targetYaw      = 0;
            targetThrottle = 0;
        }
    }

    // ── Inner Loop — Fly-By-Wire 50Hz ────────────────────────
    static unsigned long lastSensorUpdate = 0;
    unsigned long now = millis();
    if (now - lastSensorUpdate >= 20) {
        float dt = (now - lastSensorUpdate) / 1000.0f;
        if (dt <= 0) dt = 0.02f;
        lastSensorUpdate = now;

        if (imuActive) {
            readMPU();

            // 1. Suavizar setpoints de usuario
            float smoothRoll  = slewRoll.update((float)targetRoll,  dt);
            float smoothPitch = slewPitch.update((float)targetPitch, dt);

            // 2. Calcular PID
            float rollActuator  = pidRoll.compute(smoothRoll,  rollDeg,  dt);
            float pitchActuator = pidPitch.compute(smoothPitch, pitchDeg, dt);
            float yawActuator   = (float)targetYaw;

            // 3. Mezclador a Servos
            servo1Pos = targetThrottle;
            servo2Pos = map(constrain(yawActuator,   -40, 40), -40, 40,  50, 130);
            servo3Pos = map(constrain(rollActuator,  -45, 45), -45, 45,  45, 135);
            servo4Pos = map(constrain(rollActuator,  -45, 45), -45, 45, 135,  45);
            servo5Pos = 90;
            servo6Pos = 90;
            servo7Pos = map(constrain(pitchActuator, -30, 30), -30, 30,  60, 120);
            servo8Pos = map(constrain(pitchActuator, -30, 30), -30, 30,  60, 120);

            // 4. Aplicar al hardware
            int escAngle = map(servo1Pos, 0, 180, 0, 180);
            esc.write(escAngle);
            setServoAngle(SERVO_CH_YAW,     servo2Pos);
            setServoAngle(SERVO_CH_ROLL_L,  servo3Pos);
            setServoAngle(SERVO_CH_ROLL_R,  servo4Pos);
            setServoAngle(SERVO_CH_FLAP_L,  servo5Pos);
            setServoAngle(SERVO_CH_FLAP_R,  servo6Pos);
            setServoAngle(SERVO_CH_PITCH_L, servo7Pos);
            setServoAngle(SERVO_CH_PITCH_R, servo8Pos);
        }
    }

    // ── [CA-03] Receptor RF — GCU_Command 28 bytes ──────────
    if (radio.available()) {
        uint8_t len = radio.getDynamicPayloadSize();

        if (len == sizeof(GCU_Command)) {
            radio.read(&gcuCmd, sizeof(GCU_Command));

            // ── ORDEN CRÍTICO: NO REORDENAR ──────────────────
            // Paso 1: Descifrar — 7 words (28 bytes), n negativo = descifrar
            btea((uint32_t*)&gcuCmd, -7);

            // Paso 2: Verificar MAGIC
            if (gcuCmd.magic != MAGIC_CMD) goto skipCmd;

            // Paso 3: Verificar DST_ID — solo aceptar para este nodo
            if (gcuCmd.dst_id != NODE_ID_PLANE) goto skipCmd;

            // Paso 4: Verificar CRC
            {
                uint16_t crcCalc = calculateCRC16(
                    (uint8_t*)&gcuCmd,
                    sizeof(GCU_Command) - 2
                );
                if (gcuCmd.crc != crcCalc) goto skipCmd;
            }

            // Paso 5: Anti-replay — verificación de secuencia circular
            // seqDelta en [1..128] = paquete nuevo válido
            // seqDelta == 0 = duplicado (replay), >128 = del pasado
            {
                static uint8_t lastSeq = 255;
                uint8_t seqDelta = (uint8_t)(gcuCmd.seq - lastSeq);
                if (seqDelta == 0 || seqDelta > 128) goto skipCmd;
                lastSeq = gcuCmd.seq;
            }

            // ── Paquete válido ────────────────────────────────
            lastRFTime     = millis();
            currentMsgType = gcuCmd.msg_type;

            // Extraer campos de control (escalados ×10 en el protocolo)
            targetRoll     = gcuCmd.targetRoll;
            targetPitch    = gcuCmd.targetPitch;
            targetYaw      = gcuCmd.targetYaw;
            targetThrottle = gcuCmd.targetThrottle;

            // Declinación magnética
            magneticDeclinationDeg = gcuCmd.declinationX10 / 10.0f;

            // Coordenadas home desde waypoint en modo RTH
            // msg_type == MSG_CMD_RTH → waypoint_lat/lon = coordenadas del home
            if (gcuCmd.msg_type == MSG_CMD_RTH) {
                if (gcuCmd.waypoint_lat != 0 && gcuCmd.waypoint_lon != 0) {
                    homeLat = gcuCmd.waypoint_lat / 1e7f;
                    homeLon = gcuCmd.waypoint_lon / 1e7f;
                }
            }

            // Responder con telemetría en ACK payload
            pushTelemetryAck();

        } else {
            // Tamaño incorrecto — flush
            uint8_t dummy[32];
            radio.read(dummy, len);
        }

        skipCmd:;
    }
}

// ═══════════════════════════════════════════════════════
// updateVelocityFusion — GPS + IMU + Kalman
//
// Flujo:
//   1. Velocidad GPS cruda: diferencia posición / dt
//   2. Guard: descarta si supera MAX_VEL_GPS (fix corrupto)
//   3. Magnitud GPS: |V_gps|
//   4. Dirección IMU: vector unitario desde pitch/yaw del Kalman
//   5. Si |V_actual| < 2 m/s: usar magnitud GPS directamente
//   6. V_hibrida = V_imu*(1-w) + V_gps*w
//   7. Kalman 1D por eje filtra V_hibrida
//   8. Buffer de suavizado de VEL_SMOOTH_SIZE muestras
// ═══════════════════════════════════════════════════════
static float velSmBufX[VEL_SMOOTH_SIZE] = {0, 0, 0};
static float velSmBufY[VEL_SMOOTH_SIZE] = {0, 0, 0};
static float velSmBufZ[VEL_SMOOTH_SIZE] = {0, 0, 0};
static int   velSmIdx = 0;

void updateVelocityFusion(float gpsVx, float gpsVy, float gpsVz) {
    float gpsMag = sqrtf(gpsVx*gpsVx + gpsVy*gpsVy + gpsVz*gpsVz);

    float pitchRad = pitchDeg * 0.0174533f;
    float yawRad   = yawDeg   * 0.0174533f;

    float imuDirX = cosf(pitchRad) * sinf(yawRad);
    float imuDirY = cosf(pitchRad) * cosf(yawRad);
    float imuDirZ = sinf(pitchRad);

    float curMag   = sqrtf(velX*velX + velY*velY + velZ*velZ);
    float scaleMag = (curMag < 2.0f) ? gpsMag : curMag;

    float imuVx = imuDirX * scaleMag;
    float imuVy = imuDirY * scaleMag;
    float imuVz = imuDirZ * scaleMag;

    float hybX = imuVx * (1.0f - GPS_WEIGHT) + gpsVx * GPS_WEIGHT;
    float hybY = imuVy * (1.0f - GPS_WEIGHT) + gpsVy * GPS_WEIGHT;
    float hybZ = imuVz * (1.0f - GPS_WEIGHT) + gpsVz * GPS_WEIGHT;

    float filtX = kfVx.update(hybX);
    float filtY = kfVy.update(hybY);
    float filtZ = kfVz.update(hybZ);

    velSmBufX[velSmIdx] = filtX;
    velSmBufY[velSmIdx] = filtY;
    velSmBufZ[velSmIdx] = filtZ;
    velSmIdx = (velSmIdx + 1) % VEL_SMOOTH_SIZE;

    float sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < VEL_SMOOTH_SIZE; i++) {
        sumX += velSmBufX[i];
        sumY += velSmBufY[i];
        sumZ += velSmBufZ[i];
    }
    velX = sumX / VEL_SMOOTH_SIZE;
    velY = sumY / VEL_SMOOTH_SIZE;
    velZ = sumZ / VEL_SMOOTH_SIZE;
}

// ═══════════════════════════════════════════════════════
// readGPS — parseo NMEA + disparo de fusión GPS+IMU
// ═══════════════════════════════════════════════════════
void readGPS() {
    while (GPS_SERIAL.available() > 0) {
        char c = GPS_SERIAL.read();
        if (!gps.encode(c)) continue;

        if (gps.location.isValid()) {
            float lat = gps.location.lat();
            float lon = gps.location.lng();
            float alt = gps.altitude.isValid() ? gps.altitude.meters() : altitudeM;

            if (firstGPSFix) {
                lastLat = lat; lastLon = lon; lastAlt = alt;
                lastGPSVelTime = millis();
                firstGPSFix = false;
            } else {
                unsigned long now = millis();
                float dt = (now - lastGPSVelTime) / 1000.0f;

                if (dt > 0.05f) {
                    float dNorth = (lat - lastLat) * 111320.0f;
                    float dEast  = (lon - lastLon) * 111320.0f *
                                   cosf(lat * 0.0174533f);
                    float dUp    = alt - lastAlt;
                    float gpsVx  = dEast  / dt;
                    float gpsVy  = dNorth / dt;
                    float gpsVz  = dUp    / dt;

                    if (fabsf(gpsVx) < MAX_VEL_GPS &&
                        fabsf(gpsVy) < MAX_VEL_GPS &&
                        fabsf(gpsVz) < MAX_VEL_GPS) {
                        updateVelocityFusion(gpsVx, gpsVy, gpsVz);
                    }

                    lastLat = lat; lastLon = lon; lastAlt = alt;
                    lastGPSVelTime = now;
                }
            }

            latitude  = lat;
            longitude = lon;
            altitudeM = alt;
        }

        if (gps.speed.isValid()) {
            speedKmph = gps.speed.kmph();
        }
    }
}

// ═══════════════════════════════════════════════════════
// readMPU — Kalman actitud: pitch, roll, yaw
// ═══════════════════════════════════════════════════════
void readMPU() {
    if (!mpu.Read()) return;

    unsigned long now = millis();
    float dt = (now - lastIMUTime) / 1000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
    lastIMUTime = now;

    // ── Acelerómetro (m/s²) ──────────────────────────────────
    float ax = mpu.accel_x_mps2();
    float ay = mpu.accel_y_mps2();
    float az = mpu.accel_z_mps2();

    totalGForce = sqrtf(ax*ax + ay*ay + az*az) / 9.81f;

    float accelPitch = atan2f(ay, sqrtf(ax*ax + az*az)) * 57.2958f;
    float accelRoll  = atan2f(-ax, az) * 57.2958f;

    // ── Giróscopo (grados/s) ─────────────────────────────────
    float gyroPitchRate = mpu.gyro_y_radps() * 57.2958f;
    float gyroRollRate  = mpu.gyro_x_radps() * 57.2958f;
    float gyroYawRate   = mpu.gyro_z_radps() * 57.2958f;

    // ── Kalman pitch y roll (G-force adaptativo) ─────────────
    // Durante maniobras (totalGForce ≠ 1G), the accel no es confiable
    bool highG = (fabsf(totalGForce - 1.0f) > 0.3f);
    if (highG) {
        pitchDeg = kalmanPitch.update(gyroPitchRate, kalmanPitch.getAngle(), dt);
        rollDeg  = kalmanRoll.update(gyroRollRate,   kalmanRoll.getAngle(),  dt);
    } else {
        pitchDeg = kalmanPitch.update(gyroPitchRate, accelPitch, dt);
        rollDeg  = kalmanRoll.update(gyroRollRate,   accelRoll,  dt);
    }

    // ── Magnetómetro con compensación de inclinación ─────────
    float magX = mpu.mag_x_ut();
    float magY = mpu.mag_y_ut();
    float magZ = mpu.mag_z_ut();

    float pitchRad = pitchDeg * 0.0174533f;
    float rollRad  = rollDeg  * 0.0174533f;

    float magXComp =  magX * cosf(pitchRad)
                    + magY * sinf(rollRad) * sinf(pitchRad)
                    + magZ * cosf(rollRad) * sinf(pitchRad);

    float magYComp =  magY * cosf(rollRad)
                    - magZ * sinf(rollRad);

    float magYaw = atan2f(-magYComp, magXComp) * 57.2958f;
    if (magYaw < 0.0f) magYaw += 360.0f;

    // Aplicar declinación magnética
    magYaw += magneticDeclinationDeg;
    if (magYaw <    0.0f) magYaw += 360.0f;
    if (magYaw >= 360.0f) magYaw -= 360.0f;

    kalmanYaw.update(gyroYawRate, magYaw, dt);
    yawDeg = kalmanYaw.getAngle();

    // ── Filtro Complementario (Watchdog de divergencia Kalman) ─
    const float COMP_ALPHA = 0.98f;
    static float compPitch = 0.0f, compRoll = 0.0f;
    static bool  compInit  = false;

    if (!compInit) {
        compPitch = accelPitch;
        compRoll  = accelRoll;
        compInit  = true;
    }
    compPitch = COMP_ALPHA * (compPitch + gyroPitchRate * dt) +
                (1.0f - COMP_ALPHA) * accelPitch;
    compRoll  = COMP_ALPHA * (compRoll  + gyroRollRate  * dt) +
                (1.0f - COMP_ALPHA) * accelRoll;

    // Si el Kalman diverge > 15°, reset suave al complementario
    const float KALMAN_DIVERGENCE_DEG = 15.0f;
    if (fabsf(pitchDeg - compPitch) > KALMAN_DIVERGENCE_DEG) {
        kalmanPitch.setAngle(compPitch);
        pitchDeg = compPitch;
    }
    if (fabsf(rollDeg - compRoll) > KALMAN_DIVERGENCE_DEG) {
        kalmanRoll.setAngle(compRoll);
        rollDeg = compRoll;
    }

    // Normalizar yaw a [0, 360)
    if (yawDeg <    0.0f) yawDeg += 360.0f;
    if (yawDeg >= 360.0f) yawDeg -= 360.0f;
}

// ═══════════════════════════════════════════════════════
// updateTelemetry — se llama desde pushTelemetryAck()
// Nota: ahora solo actualiza variables globales (lat, lon, etc.)
//       El empaquetado final lo hace pushTelemetryAck() directamente
//       en gcuTelem, sin el struct intermedio TelemetryPacket.
// ═══════════════════════════════════════════════════════
void updateTelemetry() {
    // Las variables globales (latitude, longitude, altitudeM,
    // yawDeg, pitchDeg, rollDeg, totalGForce, velX/Y/Z)
    // ya son actualizadas por readGPS() y readMPU() en el loop.
    // Esta función se mantiene como hook por compatibilidad futura.
}
