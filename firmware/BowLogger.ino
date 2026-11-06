/*
  BowLogger.ino  —  Bow Motion Logger firmware (version 1)

  Target board : Seeed Studio XIAO nRF52840 Sense (onboard LSM6DS3TR-C 6-axis IMU)
  Framework    : Arduino, "Seeed nRF52 Boards" core (the NON-mbed one), Adafruit Bluefruit BLE stack

  WHAT IT DOES
    • Samples gyro + accel at 833 Hz (a native LSM6DS3 rate; close enough to the 800 Hz in the plan).
    • Keeps a rolling 6-second buffer in RAM.
    • Watches the accelerometer for the release spike. When it fires, it captures 1 s more,
      freezes the buffer, and streams the whole 6 s window (5 s before, 1 s after) over BLE.
    • One-time tilt calibration: hold the bow still in your normal aiming orientation and send 'c'.
      The firmware measures gravity, builds a rotation that maps the sensor frame onto the bow frame
      (X = right, Y = up the riser, Z = toward the target), measures gyro bias, and stores all of it
      in internal flash so it survives power cycles. Every sample after that is rotated and bias-corrected
      before it is buffered, so the app receives bow-frame data regardless of how the sensor is taped on.
    • Also dumps the last shot as CSV over USB serial ('p'), so you can test with no app at all.

  FLASHING (Arduino IDE 2.x)
    1. File > Preferences > "Additional boards manager URLs", add:
         https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
    2. Tools > Board > Boards Manager: install  "Seeed nRF52 Boards"
       (NOT "Seeed nRF52 mbed-enabled Boards" — this sketch uses the Bluefruit stack, which lives in the non-mbed core).
    3. Tools > Manage Libraries: install  "Seeed Arduino LSM6DS3".
       Bluefruit, Adafruit_LittleFS and InternalFileSystem ship with the board core; nothing else to install.
    4. Tools > Board > Seeed nRF52 Boards > "Seeed XIAO nRF52840 Sense". Pick the serial port.
    5. Upload. If the port vanishes or upload fails, double-tap the tiny RESET button beside the USB-C jack;
       a drive called XIAO-SENSE appears, then upload again.
    6. Tools > Serial Monitor, 115200 baud. You should see "BowLogger ready". Type a command and press Enter:
         c   calibrate: 15 s countdown to draw and settle, then 2 s of measuring at full draw
         t   test trigger (fakes a shot so you can check the pipeline without shooting)
         p   print the last shot as CSV over serial
         r   resend the last shot over BLE
         s   print status
         a   toggle auto-dump (default ON: every shot is printed as CSV over USB while a host is connected)
         L   toggle the live orientation stream over BLE (PktLive, ~50 Hz)
         z   zero the live yaw (pitch and cant are gravity-referenced and zero themselves at the calibrated aim)
         G6  set trigger threshold to 6 g (any number; default below)
       The same single-letter commands work when written to the BLE control characteristic.

  BLE INTERFACE  (device name "BowLogger")
    Service   7a1e0001-4b6f-4d6f-9c2e-6c6f67676572
      Data    7a1e0002-...  NOTIFY   shot stream, packets defined below (little-endian, packed)
      Control 7a1e0003-...  WRITE    ASCII commands as above
      Status  7a1e0004-...  READ/NOTIFY  struct StatusPkt (8 bytes)

    Shot stream = 1 header packet, N data packets, 1 end packet.
      PktHeader { u8 type=1; u16 shotId; u16 nSamples; u16 sampleRate; u16 triggerIndex;
                  f32 gyroLsbDps; f32 accelLsbG; u8 calibrated; f32 gravity[3]; u32 uptimeMs; }
      PktData   { u8 type=2; u16 shotId; u16 seq; u8 n; Sample s[n]; }     Sample = i16 gx,gy,gz,ax,ay,az
      PktEnd    { u8 type=3; u16 shotId; u16 packets; f32 peakG; f32 aimRmsDps; u16 crc16; }
      PktLive   { u8 type=4; i16 yaw, pitch, cant (0.01°); u8 state; }   streamed ~50 Hz while live mode ('L') is on
    Units: multiply gyro by gyroLsbDps to get deg/s, accel by accelLsbG to get g.
    Axes (after calibration): X = right, Y = up, Z = toward target. Yaw = rotation about Y,
    pitch about X, cant (roll) about Z. Before calibration you get raw sensor axes.

  WIRING
    Nothing beyond the board. Battery: JST-PH 2.0 LiPo into the BAT pads (check polarity: red to +).
    Optional slide switch in series with the battery + lead. USB-C charges the battery.
*/

#include <Wire.h>
#include <LSM6DS3.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

// ───────────────────────────── Tunables ─────────────────────────────
constexpr uint16_t SAMPLE_RATE_HZ  = 833;     // LSM6DS3 native ODR
constexpr float    PRE_SECONDS     = 5.0f;    // kept before the release (long enough to see the whole hold)
constexpr float    POST_SECONDS    = 1.0f;    // kept after the release
constexpr float    DEFAULT_TRIG_G  = 5.0f;    // |a| − 1 g above this = release (tune with 'G')
constexpr uint32_t REARM_MS        = 2000;    // ignore new triggers this long after a shot
constexpr float    AIM_WINDOW_S    = 1.5f;    // window for the steadiness number in the end packet
constexpr float    CAL_SECONDS     = 2.0f;    // how long you must hold still for 'c'
constexpr int      CAL_COUNTDOWN_S = 15;      // seconds between sending 'c' and the start of measuring (time to draw)
constexpr float    CAL_MAX_GYRO_DPS= 2.0f;    // calibration rejected if gyro wander (RMS about its mean) exceeds this

// LSM6DS3 full-scale settings and their LSB sizes (from the datasheet)
constexpr uint16_t GYRO_FS_DPS   = 500;                 // ±500 dps
constexpr uint16_t ACCEL_FS_G    = 16;                  // ±16 g (release spike can exceed 8 g)
constexpr float    GYRO_LSB_DPS  = 17.50f / 1000.0f;    // 17.5 mdps/LSB at ±500 dps
constexpr float    ACCEL_LSB_G   = 0.488f / 1000.0f;    // 0.488 mg/LSB at ±16 g

constexpr uint16_t PRE_SAMPLES   = (uint16_t)(PRE_SECONDS  * SAMPLE_RATE_HZ);   // 2499
constexpr uint16_t POST_SAMPLES  = (uint16_t)(POST_SECONDS * SAMPLE_RATE_HZ);   // 833
constexpr uint16_t BUF_SAMPLES   = PRE_SAMPLES + POST_SAMPLES;                   // 3332  (~40 KB)
constexpr uint16_t AIM_SAMPLES   = (uint16_t)(AIM_WINDOW_S * SAMPLE_RATE_HZ);

// LSM6DS3 registers we touch directly
constexpr uint8_t REG_STATUS   = 0x1E;   // bit0 XLDA (accel ready), bit1 GDA (gyro ready)
constexpr uint8_t REG_OUTX_L_G = 0x22;   // start of 12-byte gyro+accel output block

// XIAO nRF52840 battery sense (1 M / 510 k divider on P0.31, enabled by pulling P0.14 low)
#ifndef PIN_VBAT
  #define PIN_VBAT 32
#endif
#ifndef VBAT_ENABLE
  #define VBAT_ENABLE 14
#endif

// ───────────────────────────── Types ─────────────────────────────
struct __attribute__((packed)) Sample { int16_t gx, gy, gz, ax, ay, az; };

struct __attribute__((packed)) PktHeader {
  uint8_t  type;          // 1
  uint16_t shotId;
  uint16_t nSamples;
  uint16_t sampleRate;
  uint16_t triggerIndex;  // index of the triggering sample within the stream
  float    gyroLsbDps;
  float    accelLsbG;
  uint8_t  calibrated;
  float    gravity[3];    // unit gravity in SENSOR frame at calibration (0,0,0 if uncalibrated)
  uint32_t uptimeMs;
};

constexpr uint8_t SAMPLES_PER_PKT = 19;   // 6 + 19*12 = 234 bytes, fits a 247-byte MTU
struct __attribute__((packed)) PktData {
  uint8_t  type;          // 2
  uint16_t shotId;
  uint16_t seq;
  uint8_t  n;
  Sample   s[SAMPLES_PER_PKT];
};

struct __attribute__((packed)) PktEnd {
  uint8_t  type;          // 3
  uint16_t shotId;
  uint16_t packets;       // number of data packets sent
  float    peakG;         // peak |a| in the post window
  float    aimRmsDps;     // RMS gyro magnitude over the last AIM_WINDOW_S before release
  uint16_t crc16;         // CRC-16/CCITT over all sample bytes, in stream order
};

struct __attribute__((packed)) StatusPkt {
  uint8_t  state;         // 0 idle/filling, 1 armed, 2 post-capture, 3 sending, 4 calibrating
  uint8_t  calibrated;
  uint16_t shotCount;
  uint16_t batteryMv;
  uint16_t sampleRate;
};

struct Calibration {
  uint32_t magic;         // 0xB0A7CA1B when valid
  float    R[9];          // row-major rotation, bow = R * sensor
  float    gyroBias[3];   // raw LSB
  float    gravity[3];    // unit vector, sensor frame
};
constexpr uint32_t CAL_MAGIC = 0xB0A7CA1B;
constexpr const char* CAL_PATH = "/bowcal.bin";

enum State : uint8_t { ST_FILLING = 0, ST_ARMED = 1, ST_POST = 2, ST_SENDING = 3, ST_CAL = 4 };

// ───────────────────────────── Globals ─────────────────────────────
LSM6DS3 imu(I2C_MODE, 0x6A);

static Sample   ring[BUF_SAMPLES];   // live rolling buffer
static Sample   shot[BUF_SAMPLES];   // last completed shot, oldest → newest (kept until the next shot)
static uint16_t head = 0;            // next write position in ring
static uint32_t written = 0;         // total samples written since boot (for the fill check)
static State    state = ST_FILLING;
static uint16_t postRemaining = 0;
static uint32_t lastShotMs = 0;
static uint16_t shotCount = 0;
static bool     haveShot = false;    // ring holds a frozen, complete shot
static float    triggerG = DEFAULT_TRIG_G;
static float    postPeakG = 0;
static bool     autoDumpSerial = true;   // print each shot as CSV over USB when a host is listening ('a' toggles)

// Live orientation stream ('L' toggles). Angles in degrees, bow frame, relative to the calibrated aim.
// Pitch and cant: complementary filter (gyro + gravity), so they don't drift. Yaw has no gravity reference,
// so it is gyro-only with a slow leak back to zero; 'z' zeroes it instantly.
static bool     liveOn = false;
static float    liveYaw = 0, livePitch = 0, liveCant = 0;
static uint16_t liveDiv = 0;
constexpr uint16_t LIVE_EVERY_N  = 17;      // 833 / 17 ≈ 49 packets per second
constexpr float    LIVE_ACC_TAU  = 1.0f;    // s: how fast gravity pulls pitch/cant back
constexpr float    LIVE_YAW_TAU  = 20.0f;   // s: yaw recentering time constant

struct __attribute__((packed)) PktLive {
  uint8_t type;          // 4
  int16_t yaw, pitch, cant;   // 0.01 degree units
  uint8_t state;
};

static Calibration cal = {};
static bool calibrated = false;

static volatile char pendingCmd = 0; // set by BLE write or serial, handled in loop()
static float pendingArg = 0;

BLEService        svc     ("7a1e0001-4b6f-4d6f-9c2e-6c6f67676572");
BLECharacteristic dataChr ("7a1e0002-4b6f-4d6f-9c2e-6c6f67676572");
BLECharacteristic ctrlChr ("7a1e0003-4b6f-4d6f-9c2e-6c6f67676572");
BLECharacteristic statChr ("7a1e0004-4b6f-4d6f-9c2e-6c6f67676572");

// ───────────────────────────── Small helpers ─────────────────────────────
static inline void ledSet(bool r, bool g, bool b) {   // XIAO RGB LED is active-low
  digitalWrite(LED_RED,   r ? LOW : HIGH);
  digitalWrite(LED_GREEN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE,  b ? LOW : HIGH);
}

static uint16_t readBatteryMv() {
  analogReadResolution(12);
  analogReference(AR_INTERNAL_2_4);
  uint32_t raw = 0;
  for (int i = 0; i < 8; i++) raw += analogRead(PIN_VBAT);
  raw /= 8;
  // 2.4 V ref over 12 bits, divider gain (1000k+510k)/510k
  return (uint16_t)(raw * 2400.0f / 4095.0f * (1510.0f / 510.0f));
}

static uint16_t crc16_ccitt(uint16_t crc, const uint8_t* p, size_t n) {
  while (n--) {
    crc ^= (uint16_t)(*p++) << 8;
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

static void publishStatus() {
  StatusPkt s;
  s.state      = state;
  s.calibrated = calibrated;
  s.shotCount  = shotCount;
  s.batteryMv  = readBatteryMv();
  s.sampleRate = SAMPLE_RATE_HZ;
  statChr.write(&s, sizeof s);
  if (Bluefruit.connected() && statChr.notifyEnabled()) statChr.notify(&s, sizeof s);
}

static void printStatus() {
  Serial.printf("state=%u calibrated=%u shots=%u battery=%u mV trigger=%.1f g connected=%u\n",
                state, calibrated, shotCount, readBatteryMv(), triggerG, Bluefruit.connected());
}

// ───────────────────────────── Calibration ─────────────────────────────
// Build R so that R * a = b, with a = unit gravity (sensor frame) and b = (0,-1,0) (bow frame "down").
// Rodrigues' formula; the minimal rotation, so the sensor's yaw about the riser is preserved.
static void rotationFromGravity(const float a[3], float R[9]) {
  const float b[3] = { 0.f, -1.f, 0.f };
  float v[3] = { a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0] };
  float c  = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
  float s2 = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
  if (s2 < 1e-9f) {
    // Already aligned (c>0) or exactly opposite (c<0): identity, or 180° about Z.
    float I[9]  = {1,0,0, 0,1,0, 0,0,1};
    float Pi[9] = {-1,0,0, 0,-1,0, 0,0,1};
    memcpy(R, c > 0 ? I : Pi, sizeof I);
    return;
  }
  float k = (1.f - c) / s2;
  // K = skew(v);  R = I + K + K^2 * k
  R[0] = 1 + k * (-v[2]*v[2] - v[1]*v[1]);
  R[1] =    -v[2] + k * (v[0]*v[1]);
  R[2] =     v[1] + k * (v[0]*v[2]);
  R[3] =     v[2] + k * (v[0]*v[1]);
  R[4] = 1 + k * (-v[2]*v[2] - v[0]*v[0]);
  R[5] =    -v[0] + k * (v[1]*v[2]);
  R[6] =    -v[1] + k * (v[0]*v[2]);
  R[7] =     v[0] + k * (v[1]*v[2]);
  R[8] = 1 + k * (-v[1]*v[1] - v[0]*v[0]);
}

static bool loadCalibration() {
  File f(InternalFS);
  if (!f.open(CAL_PATH, FILE_O_READ)) return false;
  Calibration tmp;
  int n = f.read((uint8_t*)&tmp, sizeof tmp);
  f.close();
  if (n != (int)sizeof tmp || tmp.magic != CAL_MAGIC) return false;
  cal = tmp;
  return true;
}

static void saveCalibration() {
  InternalFS.remove(CAL_PATH);
  File f(InternalFS);
  if (f.open(CAL_PATH, FILE_O_WRITE)) { f.write((uint8_t*)&cal, sizeof cal); f.close(); }
}

static bool readRaw(Sample& s);  // forward

static void runCalibration() {
  State prev = state;
  state = ST_CAL;
  publishStatus();
  // Countdown so you can send 'c' from the page, then draw, anchor and settle before it measures.
  // LED: slow blue blink during the countdown, solid blue while measuring, green x2 when done, red x3 if rejected.
  Serial.printf("Calibrating in %d s: draw, anchor, aim and hold still...\n", CAL_COUNTDOWN_S);
  for (int i = CAL_COUNTDOWN_S; i > 0; i--) {
    Serial.printf("  %d\n", i);
    // slow blue blink, speeding up to a fast blink for the last 3 seconds so you know it's about to measure
    if (i > 3) { ledSet(0, 0, 1); delay(150); ledSet(0, 0, 0); delay(850); }
    else { for (int k = 0; k < 4; k++) { ledSet(0, 0, 1); delay(100); ledSet(0, 0, 0); delay(150); } }
  }
  Serial.println("Measuring: hold still");
  ledSet(0, 0, 1);

  const uint32_t want = (uint32_t)(CAL_SECONDS * SAMPLE_RATE_HZ);
  // Stillness is judged by how much the gyro WANDERS around its mean, not by the mean itself:
  // the raw gyro has a constant offset of a few dps even on a table, and measuring that offset
  // is the point of this step.
  double sg[3] = {0,0,0}, sa[3] = {0,0,0}, sgsq[3] = {0,0,0};
  uint32_t n = 0;
  uint32_t t0 = millis();
  while (n < want && millis() - t0 < (uint32_t)(CAL_SECONDS * 1000 + 1500)) {
    Sample s;
    if (!readRaw(s)) continue;
    sg[0] += s.gx; sg[1] += s.gy; sg[2] += s.gz;
    sgsq[0] += (double)s.gx*s.gx; sgsq[1] += (double)s.gy*s.gy; sgsq[2] += (double)s.gz*s.gz;
    sa[0] += s.ax; sa[1] += s.ay; sa[2] += s.az;
    n++;
  }
  if (n < want / 2) { Serial.println("Calibration failed: IMU not responding"); goto done; }

  float wander;   // RMS deviation from the mean, all axes, in dps
  {
    double var = 0;
    for (int i = 0; i < 3; i++) { double m = sg[i] / n; var += sgsq[i] / n - m * m; }
    wander = sqrtf((float)fmax(var, 0.0)) * GYRO_LSB_DPS;
  }
  if (wander > CAL_MAX_GYRO_DPS) {
    Serial.printf("Calibration rejected: you moved (%.1f dps wander). Try again.\n", wander);
    for (int i = 0; i < 3; i++) { ledSet(1,0,0); delay(120); ledSet(0,0,0); delay(120); }
    goto done;
  }
  {
    float g[3] = { (float)(sa[0]/n), (float)(sa[1]/n), (float)(sa[2]/n) };
    float gn = sqrtf(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
    Calibration c;
    c.magic = CAL_MAGIC;
    for (int i = 0; i < 3; i++) { c.gravity[i] = g[i] / gn; c.gyroBias[i] = (float)(sg[i] / n); }
    rotationFromGravity(c.gravity, c.R);
    cal = c;
    calibrated = true;
    saveCalibration();
    Serial.printf("Calibrated. gravity(sensor)=[%.3f %.3f %.3f] = %.3f g, gyro bias=[%.1f %.1f %.1f] LSB\n",
                  c.gravity[0], c.gravity[1], c.gravity[2], gn * ACCEL_LSB_G,
                  c.gyroBias[0], c.gyroBias[1], c.gyroBias[2]);
    for (int i = 0; i < 2; i++) { ledSet(0,1,0); delay(150); ledSet(0,0,0); delay(150); }
  }
done:
  // Calibration changes the frame, so the live buffer is no longer comparable. Refill.
  written = 0;
  state = ST_FILLING; (void)prev;
  publishStatus();
}

// ───────────────────────────── IMU ─────────────────────────────
static bool imuBegin() {
#ifdef PIN_LSM6DS3_POWER
  pinMode(PIN_LSM6DS3_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3_POWER, HIGH);
  delay(10);
#endif
  imu.settings.gyroEnabled      = 1;
  imu.settings.gyroRange        = GYRO_FS_DPS;
  imu.settings.gyroSampleRate   = SAMPLE_RATE_HZ;
  imu.settings.gyroBandWidth    = 400;
  imu.settings.gyroFifoEnabled  = 0;
  imu.settings.accelEnabled     = 1;
  imu.settings.accelRange       = ACCEL_FS_G;
  imu.settings.accelSampleRate  = SAMPLE_RATE_HZ;
  imu.settings.accelBandWidth   = 400;
  imu.settings.accelODROff      = 1;    // bandwidth from the setting above, not tied to ODR
  imu.settings.accelFifoEnabled = 0;
  imu.settings.fifoModeWord     = 0;
  imu.settings.tempEnabled      = 0;
  if (imu.begin() != IMU_SUCCESS) return false;
  Wire.setClock(400000);                // 12-byte burst at 833 Hz needs fast-mode I2C
  return true;
}

// Reads one new raw sample if both gyro and accel have fresh data. Returns false if nothing new.
static bool readRaw(Sample& s) {
  uint8_t st = 0;
  if (imu.readRegister(&st, REG_STATUS) != IMU_SUCCESS) return false;
  if ((st & 0x03) != 0x03) return false;
  uint8_t b[12];
  if (imu.readRegisterRegion(b, REG_OUTX_L_G, 12) != IMU_SUCCESS) return false;
  s.gx = (int16_t)(b[0]  | (b[1]  << 8));
  s.gy = (int16_t)(b[2]  | (b[3]  << 8));
  s.gz = (int16_t)(b[4]  | (b[5]  << 8));
  s.ax = (int16_t)(b[6]  | (b[7]  << 8));
  s.ay = (int16_t)(b[8]  | (b[9]  << 8));
  s.az = (int16_t)(b[10] | (b[11] << 8));
  return true;
}

static inline int16_t clamp16(float v) {
  if (v >  32767.f) return  32767;
  if (v < -32768.f) return -32768;
  return (int16_t)lrintf(v);
}

// Applies gyro bias removal and the calibration rotation. No-op when uncalibrated.
static inline void toBowFrame(Sample& s) {
  if (!calibrated) return;
  const float* R = cal.R;
  float g[3] = { s.gx - cal.gyroBias[0], s.gy - cal.gyroBias[1], s.gz - cal.gyroBias[2] };
  float a[3] = { (float)s.ax, (float)s.ay, (float)s.az };
  s.gx = clamp16(R[0]*g[0] + R[1]*g[1] + R[2]*g[2]);
  s.gy = clamp16(R[3]*g[0] + R[4]*g[1] + R[5]*g[2]);
  s.gz = clamp16(R[6]*g[0] + R[7]*g[1] + R[8]*g[2]);
  s.ax = clamp16(R[0]*a[0] + R[1]*a[1] + R[2]*a[2]);
  s.ay = clamp16(R[3]*a[0] + R[4]*a[1] + R[5]*a[2]);
  s.az = clamp16(R[6]*a[0] + R[7]*a[1] + R[8]*a[2]);
}

// ───────────────────────────── Shot transmission ─────────────────────────────
static bool bleSend(const void* p, uint16_t len) {
  if (!Bluefruit.connected() || !dataChr.notifyEnabled()) return false;
  uint32_t t0 = millis();
  while (!dataChr.notify(p, len)) {           // notify() waits for a TX buffer; false = link problem
    if (millis() - t0 > 3000 || !Bluefruit.connected()) return false;
    delay(2);
  }
  return true;
}

// Copies the frozen ring (oldest → newest, starting at `head`) into the shot buffer.
static void freezeShot() {
  for (uint16_t i = 0; i < BUF_SAMPLES; i++) shot[i] = ring[(head + i) % BUF_SAMPLES];
}
static inline const Sample& streamSample(uint16_t i) { return shot[i]; }

static void computeShotStats(float& peakG, float& aimRms) {
  const uint16_t trig = PRE_SAMPLES - 1;
  peakG = 0;
  for (uint16_t i = trig; i < BUF_SAMPLES; i++) {
    const Sample& s = streamSample(i);
    float m = sqrtf((float)s.ax*s.ax + (float)s.ay*s.ay + (float)s.az*s.az) * ACCEL_LSB_G;
    if (m > peakG) peakG = m;
  }
  double acc = 0; uint16_t n = 0;
  for (int32_t i = (int32_t)trig - AIM_SAMPLES; i < (int32_t)trig; i++) {
    if (i < 0) continue;
    const Sample& s = streamSample((uint16_t)i);
    float gm2 = ((float)s.gx*s.gx + (float)s.gy*s.gy + (float)s.gz*s.gz);
    acc += gm2; n++;
  }
  aimRms = n ? sqrtf((float)(acc / n)) * GYRO_LSB_DPS : 0;
}

static bool sendShotBLE() {
  if (!haveShot) { Serial.println("No shot stored yet"); return false; }
  if (!Bluefruit.connected() || !dataChr.notifyEnabled()) {
    Serial.println("Not connected / notifications off; shot kept in memory ('r' to resend)");
    return false;
  }
  State prev = state; state = ST_SENDING; publishStatus();
  ledSet(0, 0, 1);

  PktHeader h;
  h.type = 1; h.shotId = shotCount; h.nSamples = BUF_SAMPLES; h.sampleRate = SAMPLE_RATE_HZ;
  h.triggerIndex = PRE_SAMPLES - 1; h.gyroLsbDps = GYRO_LSB_DPS; h.accelLsbG = ACCEL_LSB_G;
  h.calibrated = calibrated;
  for (int i = 0; i < 3; i++) h.gravity[i] = calibrated ? cal.gravity[i] : 0.f;
  h.uptimeMs = millis();
  bool ok = bleSend(&h, sizeof h);

  uint16_t crc = 0xFFFF, seq = 0;
  for (uint16_t i = 0; ok && i < BUF_SAMPLES; i += SAMPLES_PER_PKT) {
    PktData d;
    d.type = 2; d.shotId = shotCount; d.seq = seq++;
    uint16_t left = BUF_SAMPLES - i;
    d.n = (uint8_t)(left < SAMPLES_PER_PKT ? left : SAMPLES_PER_PKT);
    for (uint8_t k = 0; k < d.n; k++) d.s[k] = streamSample(i + k);
    crc = crc16_ccitt(crc, (const uint8_t*)d.s, d.n * sizeof(Sample));
    ok = bleSend(&d, 6 + d.n * sizeof(Sample));
  }

  if (ok) {
    PktEnd e; e.type = 3; e.shotId = shotCount; e.packets = seq;
    float pk, rms; computeShotStats(pk, rms); e.peakG = pk; e.aimRmsDps = rms; e.crc16 = crc;
    ok = bleSend(&e, sizeof e);
    Serial.printf("Shot %u sent: %u packets, peak %.1f g, aim RMS %.2f dps\n", shotCount, seq, e.peakG, e.aimRmsDps);
  } else {
    Serial.println("BLE send interrupted; 'r' to resend");
  }
  state = prev == ST_SENDING ? ST_FILLING : prev;
  return ok;
}

static void printShotCSV() {
  if (!haveShot) { Serial.println("No shot stored yet"); return; }
  float peakG, aimRms; computeShotStats(peakG, aimRms);
  Serial.printf("# shot=%u rate=%u trigger_index=%u calibrated=%u peak_g=%.2f aim_rms_dps=%.2f\n",
                shotCount, SAMPLE_RATE_HZ, PRE_SAMPLES - 1, calibrated, peakG, aimRms);
  Serial.println("t_ms,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g");
  for (uint16_t i = 0; i < BUF_SAMPLES; i++) {
    const Sample& s = streamSample(i);
    float t = ((int32_t)i - (int32_t)(PRE_SAMPLES - 1)) * 1000.0f / SAMPLE_RATE_HZ;  // 0 at release
    Serial.printf("%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n", t,
                  s.gx * GYRO_LSB_DPS, s.gy * GYRO_LSB_DPS, s.gz * GYRO_LSB_DPS,
                  s.ax * ACCEL_LSB_G,  s.ay * ACCEL_LSB_G,  s.az * ACCEL_LSB_G);
    if ((i & 63) == 0) yield();
  }
  Serial.println("# end");
}

// ───────────────────────────── Commands ─────────────────────────────
static void queueCommand(const char* txt, size_t len) {
  if (len == 0) return;
  char c = txt[0];
  if (c == 'g') c = 'G';
  else if (c == 'l') c = 'L';
  else if (c >= 'A' && c <= 'Z' && c != 'G' && c != 'L') c = (char)(c - 'A' + 'a');
  pendingArg = (len > 1) ? atof(txt + 1) : 0;
  pendingCmd = c;
}

static void handlePendingCommand() {
  char c = pendingCmd; if (!c) return; pendingCmd = 0;
  switch (c) {
    case 'c': runCalibration(); break;
    case 't':
      if (state == ST_ARMED || state == ST_FILLING) {
        Serial.println("Test trigger");
        postRemaining = POST_SAMPLES; postPeakG = 0; state = ST_POST; ledSet(1,0,0);
      }
      break;
    case 'p': printShotCSV(); break;
    case 'r': sendShotBLE(); break;
    case 's': printStatus(); publishStatus(); break;
    case 'a': autoDumpSerial = !autoDumpSerial; Serial.printf("Auto-dump over USB: %s\n", autoDumpSerial ? "on" : "off"); break;
    case 'L': liveOn = !liveOn; liveYaw = 0; Serial.printf("Live stream: %s\n", liveOn ? "on" : "off"); break;
    case 'z': liveYaw = 0; Serial.println("Yaw zeroed"); break;
    case 'G':
      if (pendingArg >= 1.0f && pendingArg <= 15.0f) { triggerG = pendingArg; Serial.printf("Trigger threshold = %.1f g\n", triggerG); }
      else Serial.println("Trigger threshold must be 1..15 g, e.g. G6");
      break;
    default: Serial.printf("Unknown command '%c'\n", c);
  }
}

static void onCtrlWrite(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
  queueCommand((const char*)data, len);
}

static void onConnect(uint16_t conn_hdl) {
  BLEConnection* conn = Bluefruit.Connection(conn_hdl);
  conn->requestMtuExchange(247);        // 234-byte data packets need the big MTU
  conn->requestDataLengthUpdate();
  conn->requestPHY();                   // 2M PHY if the phone supports it
  Serial.println("BLE connected");
  publishStatus();
}

static void onDisconnect(uint16_t, uint8_t reason) {
  Serial.printf("BLE disconnected (0x%02X)\n", reason);
}

static void bleBegin() {
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("BowLogger");
  Bluefruit.Periph.setConnInterval(6, 12);      // 7.5–15 ms, keeps the shot upload short
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  svc.begin();

  dataChr.setProperties(CHR_PROPS_NOTIFY);
  dataChr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  dataChr.setMaxLen(244);
  dataChr.begin();

  ctrlChr.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  ctrlChr.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  ctrlChr.setMaxLen(20);
  ctrlChr.setWriteCallback(onCtrlWrite);
  ctrlChr.begin();

  statChr.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  statChr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  statChr.setFixedLen(sizeof(StatusPkt));
  statChr.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(svc);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);   // fast for 30 s, then slow
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);               // 0 = never stop
}

// ───────────────────────────── Arduino entry points ─────────────────────────────
void setup() {
  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT); pinMode(LED_BLUE, OUTPUT);
  ledSet(0, 0, 0);
  pinMode(VBAT_ENABLE, OUTPUT); digitalWrite(VBAT_ENABLE, LOW);   // enable the battery divider

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // give the monitor a moment, but don't wait on battery

  InternalFS.begin();
  calibrated = loadCalibration();

  if (!imuBegin()) {
    Serial.println("IMU init failed. Check board selection (XIAO nRF52840 Sense) and library.");
    while (true) { ledSet(1,0,0); delay(200); ledSet(0,0,0); delay(200); }
  }
  bleBegin();

  Serial.println("BowLogger ready");
  Serial.printf("  %u Hz, %u-sample buffer (%.1f s pre / %.1f s post), trigger %.1f g\n",
                SAMPLE_RATE_HZ, BUF_SAMPLES, PRE_SECONDS, POST_SECONDS, triggerG);
  Serial.printf("  calibration: %s\n", calibrated ? "loaded from flash" : "NONE - send 'c' with the bow held still");
  Serial.println("  commands: c=calibrate  t=test trigger  p=print CSV  r=resend  s=status  a=auto-dump on/off  G<n>=threshold");
  Serial.println("  auto-dump is ON: each shot prints as CSV over USB while a host is connected");
  publishStatus();
}

void loop() {
  // Serial command line
  static char line[24]; static uint8_t ll = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') { if (ll) { line[ll] = 0; queueCommand(line, ll); ll = 0; } }
    else if (ll < sizeof(line) - 1) line[ll++] = ch;
  }
  handlePendingCommand();

  // Housekeeping LED + status heartbeat
  static uint32_t lastStatus = 0;
  uint32_t now = millis();
  if (state == ST_ARMED || state == ST_FILLING) {
    bool conn = Bluefruit.connected();
    bool on = (now % 2000) < (conn ? 40u : 120u);   // short green tick if connected, blue if not
    ledSet(0, conn && on, !conn && on);
  }
  if (now - lastStatus > 5000) { lastStatus = now; publishStatus(); }

  // Sampling (runs whenever the IMU has a fresh pair)
  if (state == ST_SENDING || state == ST_CAL) return;
  Sample s;
  if (!readRaw(s)) return;
  toBowFrame(s);
  ring[head] = s;
  head = (uint16_t)((head + 1) % BUF_SAMPLES);
  written++;

  float aMag = sqrtf((float)s.ax*s.ax + (float)s.ay*s.ay + (float)s.az*s.az) * ACCEL_LSB_G;

  // Live orientation estimate (cheap enough to run every sample).
  {
    constexpr float dt = 1.0f / SAMPLE_RATE_HZ;
    const float gx = s.gx * GYRO_LSB_DPS, gy = s.gy * GYRO_LSB_DPS, gz = s.gz * GYRO_LSB_DPS;
    livePitch += gx * dt; liveYaw += gy * dt; liveCant += gz * dt;
    liveYaw -= liveYaw * (dt / LIVE_YAW_TAU);
    // Gravity reference for pitch and cant, trusted only when the bow isn't being shocked (|a| near 1 g).
    if (fabsf(aMag - 1.0f) < 0.15f) {
      const float ax = s.ax, ay = s.ay, az = s.az;
      const float pAcc = atan2f(az, -ay) * 57.2958f;    // bow frame: Y up, Z toward target
      const float cAcc = atan2f(-ax, -ay) * 57.2958f;
      const float k = dt / LIVE_ACC_TAU;
      livePitch += (pAcc - livePitch) * k;
      liveCant  += (cAcc - liveCant)  * k;
    }
    if (liveOn && ++liveDiv >= LIVE_EVERY_N) {
      liveDiv = 0;
      if (Bluefruit.connected() && dataChr.notifyEnabled()) {
        PktLive p; p.type = 4;
        p.yaw = (int16_t)lrintf(fmaxf(-320.f, fminf(320.f, liveYaw)) * 100.f);
        p.pitch = (int16_t)lrintf(fmaxf(-320.f, fminf(320.f, livePitch)) * 100.f);
        p.cant = (int16_t)lrintf(fmaxf(-320.f, fminf(320.f, liveCant)) * 100.f);
        p.state = state;
        dataChr.notify(&p, sizeof p);
      }
    }
  }

  switch (state) {
    case ST_FILLING:
      if (written >= BUF_SAMPLES) { state = ST_ARMED; Serial.println("Armed"); publishStatus(); }
      break;

    case ST_ARMED:
      if (fabsf(aMag - 1.0f) > triggerG && now - lastShotMs > REARM_MS) {
        state = ST_POST; postRemaining = POST_SAMPLES; postPeakG = aMag; lastShotMs = now;
        ledSet(1, 0, 0);
      }
      break;

    case ST_POST:
      if (aMag > postPeakG) postPeakG = aMag;
      if (--postRemaining == 0) {
        // Ring is now exactly [3 s before trigger ... 1 s after]; `head` points at the oldest sample.
        freezeShot();
        shotCount++; haveShot = true;
        Serial.printf("Shot %u captured (peak %.1f g)\n", shotCount, postPeakG);
        sendShotBLE();                 // no-op if not connected; shot[] stays until the next shot ('r' resends)
        if (autoDumpSerial && Serial) printShotCSV();   // USB host attached: push the CSV without being asked
        written = 0; state = ST_FILLING;   // refill the ring before re-arming (also enforces REARM_MS)
        publishStatus();
      }
      break;

    default: break;
  }
}
