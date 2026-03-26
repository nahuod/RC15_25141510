#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

#include <Adafruit_VL53L0X.h>
#include <ChainableLED.h>
#include <Adafruit_BNO08x.h>

// ====== GPS (Air530) ======
#include <TinyGPS++.h>
TinyGPSPlus gps;

// ============================================================
// 1) IMU (BNO085) SPI pins
// ============================================================
const uint8_t BNO_CS  = 8;
const uint8_t BNO_INT = 9;
const uint8_t BNO_RST = 7;

// ============================================================
// 2) SD card SPI CS
// ============================================================
const uint8_t SD_CS = 4;
const char* LOG_NAME = "four1.CSV";

// ============================================================
// 3) ToF (VL53L0X) + I2C MUX
// ============================================================
const int NUM_SENSORS = 4;

const int TOF_MAX_MM         = 2000;
const int TOF_THRESH_MM      = 1200;
const int TOF_BLINK_START_MM = 500;

const int TOF_BLINK_PERIOD_NEAR_MS = 40;
const int TOF_BLINK_PERIOD_FAR_MS  = 200;

const float TOF_DIM_LEVEL = 0.10f;

const uint8_t MUX_ADDR = 0x70;

const int LED_TOF_CLK   = 5;
const int LED_TOF_DATA  = 6;
const int LED_TOF_COUNT = 6;

// ============================================================
// 4) GSR
// ============================================================
const int GSR_PIN = A0;

const int LED_GSR_CLK   = 2;
const int LED_GSR_DATA  = 3;
const int LED_GSR_COUNT = 2;

const uint32_t GSR_SENSOR_INTERVAL_MS = 30;
const uint32_t GSR_LED_INTERVAL_MS    = 30;

const float GSR_ALPHA = 0.10f;

const float GSR_MIN_RISE_RATE = 0.002f;
const float GSR_MAX_FALL_RATE = 0.002f;
const int   GSR_MIN_SPAN      = 80;

// ============================================================
// 5) Scheduling / logging
// ============================================================
const uint32_t TOF_SENSOR_INTERVAL_MS = 60;
const uint32_t TOF_LED_INTERVAL_MS    = 25;

const uint32_t LOG_INTERVAL_MS = 1000;   // 1s 写一行
const uint32_t FLUSH_EVERY_N_LINES = 10;

// ============================================================
// Objects / state
// ============================================================
// ToF
Adafruit_VL53L0X lox[NUM_SENSORS];
ChainableLED ledsToF(LED_TOF_CLK, LED_TOF_DATA, LED_TOF_COUNT);

int dToF[NUM_SENSORS] = {-1, -1, -1, -1};
int minD = -1;
uint32_t tofBlinkPeriod = 0;
bool tofBlinkOn = true;

// GSR
ChainableLED ledsGSR(LED_GSR_CLK, LED_GSR_DATA, LED_GSR_COUNT);
int   gsrRaw = 0;
float gsrFilt = 0;

float gsrMinDyn = 0;
float gsrMaxDyn = 0;
bool  gsrRangeInited = false;

// IMU
Adafruit_BNO08x bno08x(BNO_RST);
sh2_SensorValue_t sensorValue;

bool hasAccel=false, hasGyro=false, hasQuat=false;
float ax=0, ay=0, az=0;
float gx=0, gy=0, gz=0;
float qi=0, qj=0, qk=0, qr=1;

float bax=0, bay=0, baz=0;
float bgx=0, bgy=0, bgz=0;

// GPS cached (给日志用：每秒写一次“最新值”)
double gps_lat = 0.0;
double gps_lng = 0.0;
double gps_speed_kmph = 0.0;
uint32_t gps_date = 0;   // yyyymmdd
uint32_t gps_time = 0;   // hhmmsscc
uint32_t gps_sats = 0;
bool gps_fix = false;

// SD
File logFile;
uint32_t logLineCount = 0;

// Timers
uint32_t lastToFSensorMs = 0;
uint32_t lastToFLedMs    = 0;
uint32_t lastGSRSensorMs = 0;
uint32_t lastGSRLedMs    = 0;
uint32_t lastLogMs       = 0;

// ============================================================
// Utility functions
// ============================================================
void muxSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

bool readOneToF(int ch, int &mm) {
  muxSelect((uint8_t)ch);
  VL53L0X_RangingMeasurementData_t m;
  lox[ch].rangingTest(&m, false);

  if (m.RangeStatus == 4) { mm = -1; return false; }
  mm = (int)m.RangeMilliMeter;
  if (mm <= 0 || mm > TOF_MAX_MM) { mm = -1; return false; }
  return true;
}

void setStripRGB(ChainableLED &strip, int count, uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < count; i++) strip.setColorRGB(i, r, g, b);
}
void setStripOff(ChainableLED &strip, int count) {
  setStripRGB(strip, count, 0, 0, 0);
}

void blueToRed_ToF_Linear(int mm, uint8_t &r, uint8_t &g, uint8_t &b) {
  int x = mm;
  if (x < 0) x = 0;
  if (x > TOF_THRESH_MM) x = TOF_THRESH_MM;

  float t = (float)x / (float)TOF_THRESH_MM;
  r = (uint8_t)(255.0f * (1.0f - t));
  g = 0;
  b = (uint8_t)(255.0f * t);
}

uint32_t tofBlinkPeriodFromDistance(int mm) {
  int x = mm;
  if (x < 0) x = 0;
  if (x > TOF_BLINK_START_MM) x = TOF_BLINK_START_MM;

  float t = (float)x / (float)TOF_BLINK_START_MM;
  t = t * t;
  float p = (float)TOF_BLINK_PERIOD_NEAR_MS + t * (float)(TOF_BLINK_PERIOD_FAR_MS - TOF_BLINK_PERIOD_NEAR_MS);
  return (uint32_t)(p + 0.5f);
}

void updateGsrDynamicRange(float v) {
  if (!gsrRangeInited) {
    gsrMinDyn = v;
    gsrMaxDyn = v;
    gsrRangeInited = true;
    return;
  }

  if (v < gsrMinDyn) gsrMinDyn = v;
  if (v > gsrMaxDyn) gsrMaxDyn = v;

  if (v > gsrMinDyn) gsrMinDyn += (v - gsrMinDyn) * GSR_MIN_RISE_RATE;
  if (v < gsrMaxDyn) gsrMaxDyn += (v - gsrMaxDyn) * GSR_MAX_FALL_RATE;

  if ((gsrMaxDyn - gsrMinDyn) < (float)GSR_MIN_SPAN) {
    float mid = 0.5f * (gsrMaxDyn + gsrMinDyn);
    gsrMinDyn = mid - 0.5f * (float)GSR_MIN_SPAN;
    gsrMaxDyn = mid + 0.5f * (float)GSR_MIN_SPAN;
  }
}

void blueToRed_GSR_Linear_Dyn(float gsrValue, uint8_t &r, uint8_t &g, uint8_t &b) {
  float minV = gsrMinDyn;
  float maxV = gsrMaxDyn;

  float x = gsrValue;
  if (x < minV) x = minV;
  if (x > maxV) x = maxV;

  float t = 0.0f;
  if (maxV > minV) t = (x - minV) / (maxV - minV);

  r = (uint8_t)(255.0f * (1.0f - t));
  g = 0;
  b = (uint8_t)(255.0f * t);
}

void enableImuReports() {
  bno08x.enableReport(SH2_ACCELEROMETER,        20000);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000);
  bno08x.enableReport(SH2_ROTATION_VECTOR,      20000);
}

// z=前后， y=左右，x=上下
static inline void sensorToBody(float sx, float sy, float sz, float &bx, float &by, float &bz) {
  bx = sz;  // 前后
  by = sy;  // 左右
  bz = sx;  // 上下
}

// ====== GPS helper: 持续喂 TinyGPS++，并缓存最新值 ======
void gpsPoll() {
  // Uno R4 WiFi / 具备 Serial1 的板子：Air530 接 UART 通常用 Serial1
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // 更新缓存：有新数据就刷新（不影响你 1s 记录节奏）
  if (gps.location.isUpdated()) {
    gps_fix = gps.location.isValid();
    if (gps_fix) {
      gps_lat = gps.location.lat();
      gps_lng = gps.location.lng();
      gps_speed_kmph = gps.speed.kmph();
    } else {
      gps_lat = 0; gps_lng = 0; gps_speed_kmph = 0;
    }
  }

  // 时间日期/卫星数也尽量更新（即使没定位也可能有）
  if (gps.date.isUpdated()) gps_date = gps.date.isValid() ? gps.date.value() : 0;
  if (gps.time.isUpdated()) gps_time = gps.time.isValid() ? gps.time.value() : 0;
  if (gps.satellites.isUpdated()) gps_sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
}

// ============================================================
// setup
// ============================================================
void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }

  Wire.begin();

  // ===== GPS 串口：Air530 默认 9600 =====
  Serial1.begin(9600);

  // （可选但推荐）避免 SPI 设备干扰：把 CS 拉高
  pinMode(SD_CS, OUTPUT);  digitalWrite(SD_CS, HIGH);
  pinMode(BNO_CS, OUTPUT); digitalWrite(BNO_CS, HIGH);

  // ---------- Init ToF ----------
  for (int ch = 0; ch < NUM_SENSORS; ch++) {
    muxSelect((uint8_t)ch);
    if (!lox[ch].begin()) {
      Serial.print("ToF init fail CH"); Serial.println(ch);
      while (1) { delay(100); }
    }
    delay(10);
  }

  // ---------- LEDs off ----------
  setStripOff(ledsToF, LED_TOF_COUNT);
  setStripOff(ledsGSR, LED_GSR_COUNT);

  // ---------- Init GSR ----------
  gsrRaw = analogRead(GSR_PIN);
  gsrFilt = (float)gsrRaw;
  updateGsrDynamicRange(gsrFilt);

  // ---------- Init SD ----------
  Serial.println("Init SD...");
  if (!SD.begin(SD_CS)) {
    Serial.println("SD.begin FAILED. Check wiring/card.");
    while (1) { delay(100); }
  }

  logFile = SD.open(LOG_NAME, FILE_WRITE);
  if (!logFile) {
    Serial.println("SD.open FAILED.");
    while (1) { delay(100); }
  }

  if (logFile.size() == 0) {
    // 加入 GPS 列：gps_fix,lat,lng,speed_kmph,date,time,sats
    logFile.println(
      "ms,"
      "d0,d1,d2,d3,minD,"
      "gsr_raw,gsr_filt,gsr_minDyn,gsr_maxDyn,"
      "ax,ay,az,gx,gy,gz,qi,qj,qk,qr,"
      "bax,bay,baz,bgx,bgy,bgz,"
      "gps_fix,lat,lng,speed_kmph,date,time,sats"
    );
    logFile.flush();
  } else {
    logFile.println("---- new session ----");
    logFile.flush();
  }

  // ---------- Init IMU ----------
  Serial.println("Starting BNO085 (SPI)...");
  if (!bno08x.begin_SPI(BNO_CS, BNO_INT)) {
    Serial.println("BNO08x not detected! Check wiring.");
    while (1) { delay(100); }
  }
  enableImuReports();

  Serial.println("All systems started.");
}

// ============================================================
// loop
// ============================================================
void loop() {
  uint32_t now = millis();

  // ===== 一直读 GPS（不会改变你的刷新频率）=====
  gpsPoll();

  // ---------- IMU ----------
  while (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER:
        ax = sensorValue.un.accelerometer.x;
        ay = sensorValue.un.accelerometer.y;
        az = sensorValue.un.accelerometer.z;
        hasAccel = true;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        gx = sensorValue.un.gyroscope.x;
        gy = sensorValue.un.gyroscope.y;
        gz = sensorValue.un.gyroscope.z;
        hasGyro = true;
        break;

      case SH2_ROTATION_VECTOR:
        qi = sensorValue.un.rotationVector.i;
        qj = sensorValue.un.rotationVector.j;
        qk = sensorValue.un.rotationVector.k;
        qr = sensorValue.un.rotationVector.real;
        hasQuat = true;
        break;

      default:
        break;
    }
  }

  sensorToBody(ax, ay, az, bax, bay, baz);
  sensorToBody(gx, gy, gz, bgx, bgy, bgz);

  // ---------- ToF sample ----------
  if (now - lastToFSensorMs >= TOF_SENSOR_INTERVAL_MS) {
    lastToFSensorMs = now;

    int tmpMin = 999999;
    for (int ch = 0; ch < NUM_SENSORS; ch++) {
      int mm;
      bool ok = readOneToF(ch, mm);
      dToF[ch] = ok ? mm : -1;
      if (ok && mm < tmpMin) tmpMin = mm;
    }
    minD = (tmpMin == 999999) ? -1 : tmpMin;

    if (minD > 0 && minD <= TOF_BLINK_START_MM) {
      tofBlinkPeriod = tofBlinkPeriodFromDistance(minD);
    } else {
      tofBlinkPeriod = 0;
      tofBlinkOn = true;
    }
  }

  // ---------- ToF LED ----------
  if (now - lastToFLedMs >= TOF_LED_INTERVAL_MS) {
    lastToFLedMs = now;

    if (minD < 0) {
      setStripOff(ledsToF, LED_TOF_COUNT);
    } else if (minD >= TOF_THRESH_MM) {
      setStripRGB(ledsToF, LED_TOF_COUNT, 255, 255, 255);
    } else {
      uint8_t r, g, b;
      blueToRed_ToF_Linear(minD, r, g, b);

      if (minD <= TOF_BLINK_START_MM && tofBlinkPeriod > 0) {
        tofBlinkOn = ((now / tofBlinkPeriod) % 2UL) == 0UL;
        if (tofBlinkOn) {
          setStripRGB(ledsToF, LED_TOF_COUNT, r, g, b);
        } else {
          setStripRGB(ledsToF, LED_TOF_COUNT,
                      (uint8_t)(r * TOF_DIM_LEVEL),
                      (uint8_t)(g * TOF_DIM_LEVEL),
                      (uint8_t)(b * TOF_DIM_LEVEL));
        }
      } else {
        setStripRGB(ledsToF, LED_TOF_COUNT, r, g, b);
      }
    }
  }

  // ---------- GSR sample ----------
  if (now - lastGSRSensorMs >= GSR_SENSOR_INTERVAL_MS) {
    lastGSRSensorMs = now;

    gsrRaw  = analogRead(GSR_PIN);
    gsrFilt = (1.0f - GSR_ALPHA) * gsrFilt + GSR_ALPHA * (float)gsrRaw;
    updateGsrDynamicRange(gsrFilt);
  }

  // ---------- GSR LED ----------
  if (now - lastGSRLedMs >= GSR_LED_INTERVAL_MS) {
    lastGSRLedMs = now;

    uint8_t r, g, b;
    blueToRed_GSR_Linear_Dyn(gsrFilt, r, g, b);
    setStripRGB(ledsGSR, LED_GSR_COUNT, r, g, b);
  }

  // ---------- Logging (single CSV, aligned by ms) ----------
  if (now - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = now;

    if (logFile) {
      logFile.print(now); logFile.print(",");

      // ToF
      logFile.print(dToF[0]); logFile.print(",");
      logFile.print(dToF[1]); logFile.print(",");
      logFile.print(dToF[2]); logFile.print(",");
      logFile.print(dToF[3]); logFile.print(",");
      logFile.print(minD);    logFile.print(",");

      // GSR
      logFile.print(gsrRaw);        logFile.print(",");
      logFile.print(gsrFilt, 2);    logFile.print(",");
      logFile.print(gsrMinDyn, 2);  logFile.print(",");
      logFile.print(gsrMaxDyn, 2);  logFile.print(",");

      // IMU (sensor frame)
      logFile.print(ax, 6); logFile.print(",");
      logFile.print(ay, 6); logFile.print(",");
      logFile.print(az, 6); logFile.print(",");
      logFile.print(gx, 6); logFile.print(",");
      logFile.print(gy, 6); logFile.print(",");
      logFile.print(gz, 6); logFile.print(",");
      logFile.print(qi, 6); logFile.print(",");
      logFile.print(qj, 6); logFile.print(",");
      logFile.print(qk, 6); logFile.print(",");
      logFile.print(qr, 6); logFile.print(",");

      // IMU (body frame)
      logFile.print(bax, 6); logFile.print(",");
      logFile.print(bay, 6); logFile.print(",");
      logFile.print(baz, 6); logFile.print(",");
      logFile.print(bgx, 6); logFile.print(",");
      logFile.print(bgy, 6); logFile.print(",");
      logFile.print(bgz, 6); logFile.print(",");

      // GPS：没定位则 0（按你的要求）
      if (gps_fix) {
        logFile.print(1); logFile.print(",");
        logFile.print(gps_lat, 6); logFile.print(",");
        logFile.print(gps_lng, 6); logFile.print(",");
        logFile.print(gps_speed_kmph, 2); logFile.print(",");
        logFile.print(gps_date); logFile.print(",");
        logFile.print(gps_time); logFile.print(",");
        logFile.println(gps_sats);
      } else {
        logFile.print(0); logFile.print(",");
        logFile.print(0); logFile.print(",");
        logFile.print(0); logFile.print(",");
        logFile.print(0); logFile.print(",");
        logFile.print(0); logFile.print(",");
        logFile.print(0); logFile.print(",");
        logFile.println(gps_sats); // 卫星数如果有就写出来，没有就 0
      }

      logLineCount++;
      if (logLineCount % FLUSH_EVERY_N_LINES == 0) {
        logFile.flush();
      }
    }
  }
}
