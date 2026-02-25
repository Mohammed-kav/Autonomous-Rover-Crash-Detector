// =====================================================
// 4WD Car + IR Obstacle Avoidance + ADXL335 Crash Detect + GPS (TinyGPS++)
// Board: Arduino MEGA
// Pins:
//   ADXL335: X->A0, Y->A1, Z->A2  (AREF wired to 3.3V if USE_AREF_3V3 = 1)
//   IR sensors: as per your working car code
//   GPS: NEO-6M TX -> RX3 (pin 15), RX -> TX3 (pin 14), baud 9600
// =====================================================

#include <Arduino.h>
#include <math.h>
#include <TinyGPSPlus.h>

// =====================================================
// ------------------ IR + MOTOR SECTION ----------------
// =====================================================

// ======================== PINS ========================
#define IR_TOP_RIGHT 52     // front-right
#define IR_TOP_LEFT 25      // front-left
#define IR_BOTTOM_LEFT 8    // rear-left
#define IR_BOTTOM_RIGHT 22  // rear-right
#define IR_TOP_MIDDLE 31    // front-middle (priority)

// Motor Driver Pins (L298N or similar)
#define ENA 3
#define IN1 53
#define IN2 51
#define ENB 2
#define IN3 29
#define IN4 27

// ======================== TIMING ======================
const unsigned long LOGIC_TICK_MS = 25;  // fast tick for sensor + action, 40Hz ticks
const unsigned long DEBOUNCE_MS   = 120; // sensor stability time, to avoid false triggers due to noise

// Tuned pulses 
const unsigned int TURN_L_MS = 600;
const unsigned int TURN_R_MS = 600;
const unsigned int FWD_MS    = 300;
const unsigned int BACK_MS   = 300;
const unsigned int HOLD_STOP = 200;

// ======================== STATE =======================
// Debounced flags (true = obstacle present)
bool FL_blocked = false;  // front-left
bool FM_blocked = false;  // front-middle
bool FR_blocked = false;  // front-right
bool RL_blocked = false;  // rear-left
bool RR_blocked = false;  // rear-right

// Debounce bookkeeping: //////////what is this
int FL_lastRaw, FM_lastRaw, FR_lastRaw, RL_lastRaw, RR_lastRaw;
unsigned long FL_lastEdge, FM_lastEdge, FR_lastEdge, RL_lastEdge, RR_lastEdge;

// Action scheduler (non-blocking): to allow GPS feeding + crash detection + sensors to keep running continuously
enum MotionType : uint8_t {
  ACT_NONE = 0,
  ACT_FORWARD_CONT,  // continuous forward (no deadline)
  ACT_BACK_PULSE,
  ACT_TURN_LEFT_PULSE,
  ACT_TURN_RIGHT_PULSE,
  ACT_ROTATE180_SEQUENCE,
  ACT_ROTATE360_SEQUENCE,
  ACT_PIVOT_RIGHT_BACK_SEQUENCE,  // pivot RIGHT then back
  ACT_PIVOT_LEFT_BACK_SEQUENCE    // pivot LEFT then back
};

/**************non-blocking motion engine**************/
MotionType currentAct = ACT_NONE; // what motion sequence is active
uint8_t     actStep    = 0; // which step inside the sequence
unsigned long actDeadline = 0;
unsigned long lastTick    = 0;
/**************non-blocking motion engine**************/

// Forward declarations (IR/motion)
void updateSensors();
void checkAndPlan();  // decides next motion when idle and GPS locked
void beginMotion(MotionType t);
void issueStep();
void cancelMotion();
void moveForward();
void moveBackward();
void moveStop();
void turnRight();
void turnLeft();
void rotate360();
void rotate180();

// =====================================================
// ---------------- ADXL335 CRASH SECTION ---------------
// =====================================================

// ---------- Pins ----------
const uint8_t AX = A0, AY = A1, AZ = A2;

// ---------- Reference ----------
#define USE_AREF_3V3 1 
const float VREF = USE_AREF_3V3 ? 3.3f : 5.0f;

// ADXL335 is ratiometric; using 3.3V reference matches sensor supply and gives more consistent conversion.
//ratiometric means output depends on (is proportional to) its supply voltage.
const float SENS_V_PER_G = 0.330f * (VREF / 3.3f);
const float ADC_LSB_V    = VREF / 1023.0f;
const float COUNTS_PER_G = SENS_V_PER_G / ADC_LSB_V;
const int   MID          = 512;     // 10-bit midscale

// ---------- Sampling & filter ----------
const unsigned long SAMPLE_US = 16000; // ~62.5 Hz
const float ALPHA_G = 0.10f;           // g-vector for LPF a compromise: stable orientation but still responsive enough for detection

// Mapping from your calibration
const int MAP_X = 0;  // Body X (forward) <- Sensor X
const int MAP_Y = 2;  // Body Y (up)      <- Sensor Z
const int MAP_Z = 1;  // Body Z (right)   <- Sensor Y
const int SGN_X = +1; // make nose-up => +pitch
const int SGN_Y = -1; // up = -Zsensor
const int SGN_Z = -1; // right = +Ysensor

// Crash-detect thresholds
const float A_SPIKE_G        = 2.0f;   // |a| above this = impact
const float DELTA_A_G        = 0.50f;  // |a| deviation from baseline
const float JERK_THR_G_PER_S = 8.0f;   // |Δg| / dt threshold (g per second)

// Direction-specific tilt thresholds from 'Processing' calibration (deg)
const float TILT_FWD_DEG   = 20.3f;  // Forward avg |pitch|
const float TILT_BACK_DEG  = 22.3f;  // Backward avg |pitch|
const float TILT_RIGHT_DEG = 37.5f;  // Right avg |roll|
const float TILT_LEFT_DEG  = 35.2f;  // Left avg |roll|

const unsigned long TILT_HOLD_MS   = 400;
const unsigned long ALERT_LOCK_MS  = 4000;

// =====================================================
// ---------------- ADXL335 CRASH SECTION ---------------
// =====================================================


/******* After-crash stop + builtin LED countdown **********/
const unsigned long CRASH_STOP_MS = 8000UL;
static unsigned long crashStopUntilMs   = 0;
static unsigned long crashLedSecMarkMs  = 0;
static unsigned long crashLedPulseOffMs = 0;
static uint8_t       crashLedSecLeft    = 0;
const unsigned long LED_PULSE_MS = 150UL;
/******* After-crash stop + builtin LED countdown **********/

/********************** Helpers *****************************/
int median5(int a, int b, int c, int d, int e) { // median kills spikes and noise
  int v[5] = {a,b,c,d,e};
  for (int i=1;i<5;i++){
    int t=v[i], j=i-1;
    while (j>=0 && v[j]>t){ v[j+1]=v[j]; j--; }
    v[j+1]=t;
  }
  return v[2];
}

int readMedian(uint8_t pin){
  int r0=analogRead(pin), r1=analogRead(pin), r2=analogRead(pin);
  int r3=analogRead(pin), r4=analogRead(pin);
  return median5(r0,r1,r2,r3,r4);
}
/********************** Helpers *****************************/

/******************* Crash state ****************************/
unsigned long nextT_us = 0;
float gxF=0, gyF=0, gzF=0;
float prevBX=0, prevBY=0, prevBZ=0;
unsigned long prevUs = 0;
float aBaseline = 1.0f;
const float BASE_ALPHA = 0.005f;
unsigned long tiltStartMs   = 0;
unsigned long alertLockUntil= 0;
/******************* Crash state ****************************/

// Forward declaration
void updateCrashDetection();

// =====================================================
// -------------------- GPS SECTION ---------------------
// =====================================================

TinyGPSPlus gps;
const uint32_t GPSBaud = 9600;   // NEO-6M default
bool gpsLocked = false;          // car must NOT move until this is true

// ======================================================================
// ========================= ESP32 UART2 BRIDGE =========================
// ======================================================================
#define ESP32_BRIDGE_BAUD 115200UL

// Cache last valid GPS-based values so crash report always has data.
static int32_t  g_last_lat_e7    = 0;
static int32_t  g_last_lon_e7    = 0;
static uint16_t g_last_speed_cms = 0;
static uint32_t g_last_time_s    = 0;   // GPS UTC seconds since midnight (0..86399)

// GPS time-of-day helper (GPS-based only;)
static void updateLastGpsTime(void)
{
  if (gps.time.isValid()) {
    g_last_time_s =
      (uint32_t)gps.time.hour()   * 3600UL +
      (uint32_t)gps.time.minute() * 60UL +
      (uint32_t)gps.time.second();
  }
}

static void sendCrashToESP32(uint16_t speed_cms, int32_t lat_e7, int32_t lon_e7, uint32_t time_s)
{
  // Format: CRASH,1,speed_cms,lat_e7,lon_e7,time_s\n, Simple CSV protocol, easy to parse on ESP32 side.
  Serial2.print(F("CRASH,1,"));
  Serial2.print(speed_cms);
  Serial2.print(',');
  Serial2.print(lat_e7);
  Serial2.print(',');
  Serial2.print(lon_e7);
  Serial2.print(',');
  Serial2.print(time_s);
  Serial2.print('\n');
}

void feedGPS() {
  while (Serial3.available()) {
    char c = Serial3.read();
    gps.encode(c);
  }

  // One-shot lock: once we get a valid recent fix(younger than 2s), enable movement
  if (!gpsLocked && gps.location.isValid() && gps.location.age() < 2000) {
    gpsLocked = true;
    Serial.println(F("GPS LOCK ACQUIRED → Car obstacle avoidance enabled."));
  }

  // Keep last valid fix for crash report, cache last good reading So when a crash happens, even if GPS temporarily becomes invalid, you still send the last known valid location/speed/time.
  if (gps.location.isValid()) {
    g_last_lat_e7 = (int32_t)(gps.location.lat() * 10000000.0);
    g_last_lon_e7 = (int32_t)(gps.location.lng() * 10000000.0);
  }
  if (gps.speed.isValid()) {
    // cm/s = m/s * 100
    double cms = gps.speed.mps() * 100.0;
    if (cms < 0) cms = 0;
    if (cms > 65535.0) cms = 65535.0;
    g_last_speed_cms = (uint16_t)(cms + 0.5);
  }
  updateLastGpsTime();
}

// Small helper to print GPS info when crash occurs
void printGpsForCrash() {
  if (gps.location.isValid()) {
    Serial.print(F(" LAT="));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(" LON="));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F(" LAT=NA LON=NA"));
  }

  if (gps.speed.isValid()) {
    Serial.print(F(" SPEED_KMPH="));
    Serial.print(gps.speed.kmph(), 1);
  }

  if (gps.date.isValid() && gps.time.isValid()) {
    Serial.print(F(" UTC="));
    Serial.print(gps.date.year());
    Serial.print('-');
    Serial.print(gps.date.month());
    Serial.print('-');
    Serial.print(gps.date.day());
    Serial.print('T');
    Serial.print(gps.time.hour());
    Serial.print(':');
    Serial.print(gps.time.minute());
    Serial.print(':');
    Serial.print(gps.time.second());
  }
}

// =====================================================
// ------------------------ SETUP -----------------------
// =====================================================

void setup() {
  // IR + motors
  pinMode(IR_TOP_RIGHT,   INPUT);
  pinMode(IR_TOP_LEFT,    INPUT);
  pinMode(IR_BOTTOM_LEFT, INPUT);
  pinMode(IR_BOTTOM_RIGHT,INPUT);
  pinMode(IR_TOP_MIDDLE,  INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // ADXL reference
  if (USE_AREF_3V3) analogReference(EXTERNAL); // AREF pin wired to 3.3V

  Serial.begin(9600);      // debug
  Serial3.begin(GPSBaud);  // GPS on Serial3 (pins 14/15)
  Serial2.begin(ESP32_BRIDGE_BAUD);  // UART2 to ESP32 (TX2=16, RX2=17)

  delay(50);

  // Init debounce state
  unsigned long now = millis();
  FL_lastRaw = digitalRead(IR_TOP_LEFT);    FL_lastEdge = now; FL_blocked = (FL_lastRaw == LOW);
  FM_lastRaw = digitalRead(IR_TOP_MIDDLE);  FM_lastEdge = now; FM_blocked = (FM_lastRaw == LOW);
  FR_lastRaw = digitalRead(IR_TOP_RIGHT);   FR_lastEdge = now; FR_blocked = (FR_lastRaw == LOW);
  RL_lastRaw = digitalRead(IR_BOTTOM_LEFT); RL_lastEdge = now; RL_blocked = (RL_lastRaw == LOW);
  RR_lastRaw = digitalRead(IR_BOTTOM_RIGHT);RR_lastEdge = now; RR_blocked = (RR_lastRaw == LOW);

  moveStop();
  currentAct = ACT_NONE;
  actStep    = 0;
  actDeadline= 0;

  // ADXL dummy reads & crash timer init
  analogRead(AX); analogRead(AY); analogRead(AZ);
  nextT_us = micros();

  Serial.println(F("System booting..."));
  Serial.println(F("Waiting for GPS lock. Car will NOT move until GPS is locked."));
  alertLockUntil = millis() + 5000UL;   // ignore crash triggers for first 5s to avoid wiring/power noises
}

// =====================================================
// ------------------------- LOOP ----------------------
// =====================================================

void loop() {
  // 1) Always feed GPS (non-blocking)
  feedGPS();

  // 2) Crash detection (ADXL) at ~62.5 Hz
  updateCrashDetection();

  // 3) IR obstacle avoidance & motion scheduler (only if GPS locked)
  unsigned long now = millis();

  if (now - lastTick >= LOGIC_TICK_MS) {
    lastTick = now;

    // Always keep IR sensors updated
    updateSensors();

    // If we are in the 8s post-crash stop window, FORCE STOP and run LED countdown
    if (crashStopUntilMs != 0) {
      if ((long)(crashStopUntilMs - now) > 0) {
        if (currentAct != ACT_NONE) {
          cancelMotion();
        } else {
          moveStop();
        }

        // Turn off LED after the short pulse
        if (crashLedPulseOffMs != 0 && (long)(now - crashLedPulseOffMs) >= 0) {
          digitalWrite(LED_BUILTIN, LOW);
          crashLedPulseOffMs = 0;
        }

        // Every 1s: decrement remaining seconds and pulse LED once
        if ((long)(now - crashLedSecMarkMs) >= 1000) {
          crashLedSecMarkMs += 1000UL;
          if (crashLedSecLeft > 0) crashLedSecLeft--;

          if (crashLedSecLeft > 0) {
            digitalWrite(LED_BUILTIN, HIGH);
            crashLedPulseOffMs = now + LED_PULSE_MS;
          } else {
            digitalWrite(LED_BUILTIN, LOW);
          }
        }

        return;
      } else {
        crashStopUntilMs = 0;
        crashLedSecMarkMs = 0;
        crashLedPulseOffMs = 0;
        crashLedSecLeft = 0;
        digitalWrite(LED_BUILTIN, LOW);
      }
    }

    // If GPS is not locked, FORCE STOP and do nothing else
    if (!gpsLocked) {
      if (currentAct != ACT_NONE) {
        cancelMotion();
      } else {
        moveStop();
      }
      return;
    }

    // Once GPS is locked, normal obstacle avoidance:

    // Preempt continuous forward if hazard appears
    if (currentAct == ACT_FORWARD_CONT) {
      bool hazardFront = (FM_blocked || FL_blocked || FR_blocked);
      if (hazardFront) {
        cancelMotion();
      }
    }

    // Drive scheduler
    if (currentAct != ACT_NONE && currentAct != ACT_FORWARD_CONT) {
      if (now >= actDeadline) {
        actStep++;
        issueStep();  // may clear to ACT_NONE
      }
    } else if (currentAct == ACT_FORWARD_CONT) {
      // keep going forward until preempted by hazard
    } else {
      // If idle and GPS locked, decide next action
      checkAndPlan();
    }
  }
}

// =====================================================
// ----------- CRASH DETECTION ROUTINE -----------------
// =====================================================

void updateCrashDetection() {
  unsigned long nowUs = micros();
  if ((long)(nowUs - nextT_us) < 0) return;  // not yet time
  nextT_us += SAMPLE_US;

  // 1) Sensor-frame g from centered counts
  int rx = readMedian(AX);
  int ry = readMedian(AY);
  int rz = readMedian(AZ);

  float gx = (rx - MID) / COUNTS_PER_G;
  float gy = (ry - MID) / COUNTS_PER_G;
  float gz = (rz - MID) / COUNTS_PER_G;

  // 2) Low-pass filter on g
  gxF = (1.0f - ALPHA_G)*gxF + ALPHA_G*gx;
  gyF = (1.0f - ALPHA_G)*gyF + ALPHA_G*gy;
  gzF = (1.0f - ALPHA_G)*gzF + ALPHA_G*gz;

  // 3) Map to BODY frame (X fwd, Y up, Z right)
  float s[3] = { gxF, gyF, gzF };
  float bX = SGN_X * s[MAP_X];
  float bY = SGN_Y * s[MAP_Y];
  float bZ = SGN_Z * s[MAP_Z];

  // 4) Roll/Pitch
  float xP = bX;    // forward
  float yP = bZ;    // right
  float zP = -bY;   // down (≈ +1g at rest)

  float rollDeg  = -atan2f(yP, zP) * 180.0f/PI;
  float pitchDeg =  atan2f(-xP, sqrtf(yP*yP + zP*zP)) * 180.0f/PI;
  float amag     = sqrtf(bX*bX + bY*bY + bZ*bZ);

  // 5) Baseline |a|
  aBaseline = (1.0f - BASE_ALPHA)*aBaseline + BASE_ALPHA*amag;
  float deltaA = fabsf(amag - aBaseline);

  // Jerk
  float jerk = 0.0f;
  if (prevUs != 0) {
    float dt = (nowUs - prevUs) / 1e6f;
    if (dt > 0.0005f) {
      float dBx = bX - prevBX;
      float dBy = bY - prevBY;
      float dBz = bZ - prevBZ;
      float dMag = sqrtf(dBx*dBx + dBy*dBy + dBz*dBz);
      jerk = dMag / dt;
    }
  }
  prevUs = nowUs;
  prevBX = bX;
  prevBY = bY;
  prevBZ = bZ;

  bool impact = (amag >= A_SPIKE_G) ||
                (deltaA >= DELTA_A_G && jerk >= JERK_THR_G_PER_S);

  // Tilt classification
  unsigned long nowMs = nowUs / 1000UL;

  bool tiltForward  = (pitchDeg <= -TILT_FWD_DEG);
  bool tiltBackward = (pitchDeg >=  TILT_BACK_DEG);
  bool tiltRight    = (rollDeg  >=  TILT_RIGHT_DEG);
  bool tiltLeft     = (rollDeg  <= -TILT_LEFT_DEG);

  bool bigTilt = tiltForward || tiltBackward || tiltRight || tiltLeft;

  if (bigTilt) {
    if (tiltStartMs == 0) tiltStartMs = nowMs;
  } else {
    tiltStartMs = 0;
  }

  bool rollover = (tiltStartMs != 0) &&
                  ((nowMs - tiltStartMs) >= TILT_HOLD_MS);

  if ((impact || rollover) && (nowMs >= alertLockUntil)) {
    // Direction
    const char* dir = "unknown";
    if      (tiltForward)  dir = "nose_up";
    else if (tiltBackward) dir = "nose_down";
    else if (tiltRight)    dir = "right_roll";
    else if (tiltLeft)     dir = "left_roll";
    else {
      if (fabsf(pitchDeg) >= fabsf(rollDeg))
        dir = (pitchDeg >= 0.0f) ? "nose_down" : "nose_up";
      else
        dir = (rollDeg  >= 0.0f) ? "right_roll" : "left_roll";
    }

    // Cause
    const char* cause = "impact_only";
    if (impact && rollover)       cause = "impact_plus_tilt";
    else if (!impact && rollover) cause = "tilt_only";

    Serial.print(F("CRASH "));
    Serial.print(cause);
    Serial.print(F(" "));
    Serial.print(dir);

    // Append GPS info if available
    printGpsForCrash();
    Serial.println();

    // Send crash packet to ESP32 over UART2 (GPS-based cached values)
    sendCrashToESP32(g_last_speed_cms, g_last_lat_e7, g_last_lon_e7, g_last_time_s);

    // Start 8s stop + LED countdown (1 blink per second)
    unsigned long t0 = millis();
    crashStopUntilMs   = t0 + CRASH_STOP_MS;
    crashLedSecMarkMs  = t0;
    crashLedSecLeft    = (uint8_t)(CRASH_STOP_MS / 1000UL); // 8
    digitalWrite(LED_BUILTIN, HIGH);
    crashLedPulseOffMs = t0 + LED_PULSE_MS;

    // Stop motors immediately
    cancelMotion();
    moveStop();

    // Lockout at least through the stop window (prevents repeated crash triggers)
    alertLockUntil = nowMs + (CRASH_STOP_MS > ALERT_LOCK_MS ? CRASH_STOP_MS : ALERT_LOCK_MS);
    tiltStartMs    = 0;
  }
}

// =====================================================
// ---------- IR SENSOR UPDATE & DECISION --------------
// =====================================================

inline void debUpdate(int pin, int &lastRaw, unsigned long &lastEdge, bool &debFlag) {
  unsigned long now = millis();
  int raw = digitalRead(pin);
  if (raw != lastRaw) {
    lastRaw = raw;
    lastEdge = now;
  }
  if ((now - lastEdge) >= DEBOUNCE_MS) {
    debFlag = (lastRaw == LOW);  // active-LOW -> blocked
  }
}

void updateSensors() {
  debUpdate(IR_TOP_LEFT,  FL_lastRaw, FL_lastEdge, FL_blocked);
  debUpdate(IR_TOP_MIDDLE,FM_lastRaw, FM_lastEdge, FM_blocked);
  debUpdate(IR_TOP_RIGHT, FR_lastRaw, FR_lastEdge, FR_blocked);
  debUpdate(IR_BOTTOM_LEFT, RL_lastRaw, RL_lastEdge, RL_blocked);
  debUpdate(IR_BOTTOM_RIGHT,RR_lastRaw, RR_lastEdge, RR_blocked);
}

void checkAndPlan() {
  if (FL_blocked && !FR_blocked) { beginMotion(ACT_TURN_RIGHT_PULSE); return; }
  if (FR_blocked && !FL_blocked) { beginMotion(ACT_TURN_LEFT_PULSE);  return; }

  if (!FM_blocked && !FL_blocked && !FR_blocked) { beginMotion(ACT_FORWARD_CONT); return; }

  if (FM_blocked && !FL_blocked && !FR_blocked) { beginMotion(ACT_TURN_RIGHT_PULSE); return; }

  if (FM_blocked && FL_blocked && FR_blocked) {
    bool rearBothClear   = (!RL_blocked && !RR_blocked);
    bool rearBothBlocked = ( RL_blocked &&  RR_blocked);
    bool rearOneBlocked  = ( RL_blocked ^   RR_blocked);

    if (rearBothClear)   { beginMotion(ACT_ROTATE180_SEQUENCE); return; }
    if (rearBothBlocked) { beginMotion(ACT_ROTATE360_SEQUENCE); return; }
    if (rearOneBlocked)  {
      if (RL_blocked) beginMotion(ACT_PIVOT_RIGHT_BACK_SEQUENCE);
      else            beginMotion(ACT_PIVOT_LEFT_BACK_SEQUENCE);
      return;
    }
  }

  if (!FM_blocked && FL_blocked && FR_blocked) { beginMotion(ACT_TURN_LEFT_PULSE); return; }

  beginMotion(ACT_ROTATE360_SEQUENCE);
}

// =====================================================
// ---------------- ACTION SCHEDULER -------------------
// =====================================================

void beginMotion(MotionType t) {
  if (t == ACT_FORWARD_CONT) {
    moveForward();
    currentAct = ACT_FORWARD_CONT;
    actStep    = 0;
    actDeadline= 0;
    return;
  }
  currentAct = t;
  actStep    = 0;
  issueStep();
}

void cancelMotion() {
  moveStop();
  currentAct = ACT_NONE;
  actStep    = 0;
  actDeadline= 0;
}

void issueStep() {
  unsigned long now = millis();

  switch (currentAct) {
    case ACT_BACK_PULSE:
      if (actStep == 0) { moveBackward(); actDeadline = now + BACK_MS; }
      else if (actStep == 1) { moveStop(); actDeadline = now + HOLD_STOP; }
      else { currentAct = ACT_NONE; }
      break;

    case ACT_TURN_LEFT_PULSE:
      if (actStep == 0) { turnLeft(); actDeadline = now + TURN_L_MS; }
      else if (actStep == 1) { moveStop(); actDeadline = now + HOLD_STOP; }
      else { currentAct = ACT_NONE; }
      break;

    case ACT_TURN_RIGHT_PULSE:
      if (actStep == 0) { turnRight(); actDeadline = now + TURN_R_MS; }
      else if (actStep == 1) { moveStop(); actDeadline = now + HOLD_STOP; }
      else { currentAct = ACT_NONE; }
      break;

    case ACT_ROTATE180_SEQUENCE:
      if (actStep == 0) { rotate180(); actDeadline = now + (2UL * TURN_L_MS); }
      else if (actStep == 1) { moveStop(); actDeadline = now + HOLD_STOP; }
      else if (actStep == 2) { beginMotion(ACT_FORWARD_CONT); }
      else { currentAct = ACT_NONE; }
      break;

    case ACT_ROTATE360_SEQUENCE:
      if (actStep == 0) { rotate360(); actDeadline = now + (4UL * TURN_L_MS); }
      else if (actStep == 1) { moveStop(); actDeadline = now + HOLD_STOP; }
      else { currentAct = ACT_NONE; }
      break;

    case ACT_PIVOT_RIGHT_BACK_SEQUENCE:
      if (actStep == 0) { turnRight(); actDeadline = now + TURN_R_MS; }
      else if (actStep == 1) { moveStop(); actDeadline = now + HOLD_STOP; }
      else if (actStep == 2) { moveBackward(); actDeadline = now + BACK_MS; }
      else if (actStep == 3) { moveStop(); actDeadline = now + HOLD_STOP; }
      else { currentAct = ACT_NONE; }
      break;

    case ACT_PIVOT_LEFT_BACK_SEQUENCE:
      if (actStep == 0) { turnLeft(); actDeadline = now + TURN_L_MS; }
      else if (actStep == 1) { moveStop(); actDeadline = now + HOLD_STOP; }
      else if (actStep == 2) { moveBackward(); actDeadline = now + BACK_MS; }
      else if (actStep == 3) { moveStop(); actDeadline = now + HOLD_STOP; }
      else { currentAct = ACT_NONE; }
      break;

    default:
      currentAct = ACT_NONE;
      break;
  }
}

// =====================================================
// ---------------- MOTOR PRIMITIVES -------------------
// =====================================================

void moveForward() {
  analogWrite(ENA, 140);
  analogWrite(ENB, 140);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println(F("MOVE forward"));
}

void moveBackward() {
  analogWrite(ENA, 140);
  analogWrite(ENB, 140);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println(F("MOVE backward"));
}

void moveStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  analogWrite(ENA, 155);
  analogWrite(ENB, 155);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println(F("MOVE turn_right"));
}

void turnLeft() {
  analogWrite(ENA, 140);
  analogWrite(ENB, 140);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println(F("MOVE turn_left"));
}

void rotate360() {
  analogWrite(ENA, 140);
  analogWrite(ENB, 140);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println(F("MOVE rotate_in_place"));
}

void rotate180() {
  analogWrite(ENA, 140);
  analogWrite(ENB, 140);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println(F("MOVE rotate_180_in_place"));
}
