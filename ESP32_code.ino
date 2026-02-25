#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define CRASH_HOLD_MS          10000UL   // keep crash_val=1 for 10s
#define CRASH_REFRESH_MS        1000UL   // re-notify crash_val=1 while active
#define UART_BAUD              115200UL

// ESP32 UART pins to Arduino MEGA Serial2:
//   MEGA TX2 (pin 16) -> ESP32 RX (through voltage divider)
//   MEGA RX2 (pin 17) <- ESP32 TX 
#define MEGA_UART_RX_PIN 16
#define MEGA_UART_TX_PIN 17

static const char *DEVICE_NAME  = "ESP32 crash report";

// Service UUID
static const char *SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab";

// Characteristics UUIDs
static const char *CRASH_UUID   = "12345678-1234-1234-1234-1234567890ad"; // uint8 (READ+NOTIFY, CCCD)
static const char *SPEED_UUID   = "12345678-1234-1234-1234-1234567890b1"; // uint16 (cm/s)
static const char *LAT_UUID     = "12345678-1234-1234-1234-1234567890b2"; // int32  (deg * 1e7)
static const char *LON_UUID     = "12345678-1234-1234-1234-1234567890b3"; // int32  (deg * 1e7)
static const char *TIME_UUID    = "12345678-1234-1234-1234-1234567890b4"; // uint32 (GPS seconds since midnight)

static BLECharacteristic *g_crashChar = nullptr;
static BLECharacteristic *g_speedChar = nullptr;
static BLECharacteristic *g_latChar   = nullptr;
static BLECharacteristic *g_lonChar   = nullptr;
static BLECharacteristic *g_timeChar  = nullptr;

// Latest values (what Automate reads)
static uint16_t g_speed_cms = 0;
static int32_t  g_lat_e7    = 0;
static int32_t  g_lon_e7    = 0;
static uint32_t g_time_s    = 0;

static bool g_crashActive = false;
static unsigned long g_crashEndMs = 0;
static unsigned long g_lastCrashRefreshMs = 0;

static HardwareSerial MegaSerial(2);

// ----------------- helpers to write GATT values -----------------
static void setU8(BLECharacteristic *ch, uint8_t v) {
  if (!ch) return;
  ch->setValue(&v, 1);
}

static void setU16(BLECharacteristic *ch, uint16_t v) {
  if (!ch) return;
  uint8_t b[2];
  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
  ch->setValue(b, 2);
}

static void setI32(BLECharacteristic *ch, int32_t v) {
  if (!ch) return;
  uint8_t b[4];
  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
  b[2] = (uint8_t)((v >> 16) & 0xFF);
  b[3] = (uint8_t)((v >> 24) & 0xFF);
  ch->setValue(b, 4);
}

static void setU32(BLECharacteristic *ch, uint32_t v) {
  if (!ch) return;
  uint8_t b[4];
  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
  b[2] = (uint8_t)((v >> 16) & 0xFF);
  b[3] = (uint8_t)((v >> 24) & 0xFF);
  ch->setValue(b, 4);
}

static void crashNotify(uint8_t v) {
  setU8(g_crashChar, v);
  if (g_crashChar) g_crashChar->notify();
}

static void applyTelemetryToGatt(void) {
  setU16(g_speedChar, g_speed_cms);
  setI32(g_latChar,   g_lat_e7);
  setI32(g_lonChar,   g_lon_e7);
  setU32(g_timeChar,  g_time_s);
}

// ----------------- UART line parsing -----------------
static bool parseCrashLine(char *line, uint16_t *speed, int32_t *lat, int32_t *lon, uint32_t *t) {
  // Expected: CRASH,1,speed_cms,lat_e7,lon_e7,time_s
  // Example : CRASH,1,123,320000000,350000000,86399
  char *save = nullptr;

  char *tok = strtok_r(line, ",", &save);
  if (!tok || strcmp(tok, "CRASH") != 0) return false;

  tok = strtok_r(nullptr, ",", &save); // "1"
  if (!tok) return false;

  tok = strtok_r(nullptr, ",", &save);
  if (!tok) return false;
  long sp = strtol(tok, nullptr, 10);

  tok = strtok_r(nullptr, ",", &save);
  if (!tok) return false;
  long la = strtol(tok, nullptr, 10);

  tok = strtok_r(nullptr, ",", &save);
  if (!tok) return false;
  long lo = strtol(tok, nullptr, 10);

  tok = strtok_r(nullptr, ",", &save);
  if (!tok) return false;
  unsigned long ts = strtoul(tok, nullptr, 10);

  if (sp < 0) sp = 0;
  if (sp > 65535) sp = 65535;

  *speed = (uint16_t)sp;
  *lat   = (int32_t)la;
  *lon   = (int32_t)lo;
  *t     = (uint32_t)ts;
  return true;
}

static bool uartReadLine(char *out, size_t outLen) {
  static size_t i = 0;

  while (MegaSerial.available()) {
    int c = MegaSerial.read();
    if (c < 0) break;

    if (c == '\r') continue;

    if (c == '\n') {
      out[i] = '\0';
      i = 0;
      return true;
    }

    if (i + 1 < outLen) {
      out[i++] = (char)c;
    } else {
      // overflow: drop line
      i = 0;
    }
  }
  return false;
}

// ----------------- crash state machine -----------------
static void startCrash(unsigned long nowMs) {
  g_crashActive = true;
  g_crashEndMs = nowMs + CRASH_HOLD_MS;
  g_lastCrashRefreshMs = 0;

  crashNotify(1);

  Serial.print("[CRASH] START speed=");
  Serial.print(g_speed_cms);
  Serial.print(" lat_e7=");
  Serial.print(g_lat_e7);
  Serial.print(" lon_e7=");
  Serial.print(g_lon_e7);
  Serial.print(" time_s=");
  Serial.println(g_time_s);
}

static void stopCrash(unsigned long nowMs) {
  (void)nowMs;
  crashNotify(0);
  g_crashActive = false;
  Serial.println("[CRASH] STOP");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("[INIT] Starting BLE...");
  BLEDevice::init(DEVICE_NAME);

  BLEServer *server = BLEDevice::createServer();
  BLEService *svc = server->createService(SERVICE_UUID);

  // Crash characteristic: READ + NOTIFY + CCCD
  g_crashChar = svc->createCharacteristic(
    CRASH_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  g_crashChar->addDescriptor(new BLE2902());

  // Telemetry: READ-only (Automate reads)
  g_speedChar = svc->createCharacteristic(SPEED_UUID, BLECharacteristic::PROPERTY_READ);
  g_latChar   = svc->createCharacteristic(LAT_UUID,   BLECharacteristic::PROPERTY_READ);
  g_lonChar   = svc->createCharacteristic(LON_UUID,   BLECharacteristic::PROPERTY_READ);
  g_timeChar  = svc->createCharacteristic(TIME_UUID,  BLECharacteristic::PROPERTY_READ);

  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();

  // Init GATT values with correct lengths
  setU8(g_crashChar, 0);
  applyTelemetryToGatt();

  // Start UART2 to MEGA
  MegaSerial.begin(UART_BAUD, SERIAL_8N1, MEGA_UART_RX_PIN, MEGA_UART_TX_PIN);
  Serial.println("[INIT] Advertising started; UART2 ready (waiting CRASH lines)");
}

void loop() {
  unsigned long nowMs = millis();

  // 1) Read UART lines from Arduino
  char line[96];
  if (uartReadLine(line, sizeof(line))) {
    uint16_t sp;
    int32_t la, lo;
    uint32_t ts;

    // Make a copy because parseCrashLine uses strtok (modifies buffer)
    char tmp[96];
    strncpy(tmp, line, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';

    if (parseCrashLine(tmp, &sp, &la, &lo, &ts)) {
      // Update telemetry values from Arduino
      g_speed_cms = sp;
      g_lat_e7 = la;
      g_lon_e7 = lo;
      g_time_s = ts;

      applyTelemetryToGatt();

      // Trigger crash window
      if (!g_crashActive) {
        startCrash(nowMs);
      } else {
        // If already active and you get another CRASH line, extend the window
        g_crashEndMs = nowMs + CRASH_HOLD_MS;
      }
    } else {
        //debug other lines
       Serial.print("[UART] ");
       Serial.println(line);
    }
  }

  // 2) While crash is active: refresh notify and clear after hold time
  if (g_crashActive) {
    if (g_lastCrashRefreshMs == 0 || (nowMs - g_lastCrashRefreshMs >= CRASH_REFRESH_MS)) {
      g_lastCrashRefreshMs = nowMs;
      crashNotify(1);
    }

    if ((long)(nowMs - g_crashEndMs) >= 0) {
      stopCrash(nowMs);
    }
  }

  delay(2);
}
