// Step 3: parking floor tracker + Bluetooth (BLE). ESP32-C3 + BMP280.
//
// Same floor logic as step 2 (floor_tracker.ino), plus:
//  - BLE device "ParkFloor" sends the floor label ("B2", "1F", ...) every second.
//  - The phone page (phone/index.html) saves the last label + GPS. When the car is
//    turned off, the device loses power, BLE disconnects, and the last saved label stays.
//
// Library (Library Manager): "Adafruit BMP280 Library". BLE is built into the esp32 board package.
// Board: ESP32C3 Dev Module, Tools > USB CDC On Boot: Enabled. Serial 115200.

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <BLE.h>                    // esp32 core 4.x BLE API
#include <Preferences.h>            // flash storage that survives power-off
#include <Adafruit_SSD1306.h>       // optional 0.96" OLED on the same I2C pins; runs fine without it

const int SDA_PIN = 8;              // ESP32-C3 SuperMini I2C
const int SCL_PIN = 9;
const float FLOOR_HEIGHT_M = 3.0;   // tune per parking lot (usually 2.8-3.5 m)
const float MOVE_M = 0.4;           // height change within the window that counts as a ramp
const int MOVE_WINDOW_S = 10;
const int LOOKBACK_S = 180;         // shortcut: fixed 3 min window, idling longer than this resets to 1F; upgrade with GPS speed from the phone

// Must match phone/index.html. Lowercase (Web Bluetooth requires it).
static const BLEUUID SERVICE_UUID("6e1f0001-5a3c-4b7e-9d2a-1c0f7a3b8e01");
static const BLEUUID FLOOR_CHAR_UUID("6e1f0002-5a3c-4b7e-9d2a-1c0f7a3b8e01");
static const BLEUUID ROAD_CHAR_UUID("6e1f0003-5a3c-4b7e-9d2a-1c0f7a3b8e01");  // phone writes here when GPS speed > 25 km/h

Adafruit_BMP280 bmp;
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool hasOled = false;

BLECharacteristic floorChar, roadChar;
bool bleConnected = false;

// Road signal from the phone: driving fast means ground level (1F). Fixes mountain roads and hills.
volatile bool roadSignal = false;
bool gpsSeen = false;       // shortcut: once set, stays for this power-on; hills after the phone page closes are not reset
float hSinceRoad = 0;       // ramp height since the last road signal
bool skipEpisode = false;   // climb that was under way at the road signal is road, not a ramp: skip it until it ends

float rawDh[LOOKBACK_S];   // height change each second
float accDh[LOOKBACK_S];   // same, but 0 while still
int idx = 0;
bool moving = false;
float prevP;

// Remembered parking floor (0 = 1F, -2 = B2). Power is cut without warning, so the floor
// is written to flash whenever it has been steady for STORE_AFTER_S while still.
const int STORE_AFTER_S = 5;
Preferences prefs;
int offsetFloor = 0;        // added to the lookback sum; starts as the floor where the car was turned off
long offsetUntilMs = -1;    // -1 = keep until the first ramp after power-on ends
bool wasMoving = false;
int storedFloor = 0, steadyFloor = 0, steadySecs = 0;

// Trip log, kept in flash so it survives the power cut at parking: one record per ramp segment
// (raw height change, how much of it was counted as floors, seconds since the last road signal),
// plus power-on and floor-saved records. Read it over USB: send 'L' on the serial port (also printed at boot).
struct LogRec { uint32_t t; uint16_t dur; int16_t rawCm; int16_t cntCm; uint16_t sinceRoad; uint8_t flags; };
enum : uint8_t { LF_GPS = 1, LF_SKIP = 2, LF_STORE = 64, LF_BOOT = 128 };
const int LOG_N = 32;
LogRec logBuf[LOG_N];
uint8_t logHead = 0, logCount = 0;          // ring buffer: next write slot, number of valid records
float rawSum = 0;                           // every height change since power-on (not filtered)
unsigned long lastRoadMs = 0, epStartMs = 0;
bool roadEver = false, epSkipped = false;
float epRawStart = 0, epCntStart = 0;

void setupBle() {
  if (!BLE.begin("ParkFloor")) {
    Serial.println("BLE init failed, running without Bluetooth.");
    return;
  }
  BLEServer server = BLE.createServer();
  server.onConnect([](BLEServer, const BLEConnInfo&) { bleConnected = true; });
  server.onDisconnect([](BLEServer, const BLEConnInfo&, uint8_t) {
    bleConnected = false;
    BLE.startAdvertising();         // let the phone reconnect
  });
  BLEService service = server.createService(SERVICE_UUID);
  floorChar = service.createCharacteristic(FLOOR_CHAR_UUID, BLEProperty::Read | BLEProperty::Notify,
                                           BLEPermission::ReadOpen);
  roadChar = service.createCharacteristic(ROAD_CHAR_UUID, BLEProperty::Write | BLEProperty::WriteNR,
                                          BLEPermission::WriteOpen);
  roadChar.onWrite([](BLECharacteristic, const BLEConnInfo&) { roadSignal = true; });
  server.start();
  BLEAdvertising adv = BLE.getAdvertising();
  adv.addServiceUUID(SERVICE_UUID);
  adv.start();
}

void addLog(uint16_t dur, float rawM, float cntM, uint8_t flags) {
  uint16_t since = roadEver ? (uint16_t)min((millis() - lastRoadMs) / 1000UL, 65534UL) : 65535;
  logBuf[logHead] = { (uint32_t)(millis() / 1000), dur, (int16_t)lroundf(rawM * 100), (int16_t)lroundf(cntM * 100), since, flags };
  logHead = (logHead + 1) % LOG_N;
  if (logCount < LOG_N) logCount++;
  prefs.putBytes("log", logBuf, sizeof(logBuf));
  prefs.putUChar("logHead", logHead);
  prefs.putUChar("logCount", logCount);
}

void printLog() {
  Serial.printf("---- trip log (%d records, oldest first) ----\n", logCount);
  for (int k = 0; k < logCount; k++) {
    const LogRec& r = logBuf[(logHead - logCount + k + LOG_N) % LOG_N];
    String road = r.sinceRoad == 65535 ? String("no road signal yet") : String(r.sinceRoad) + "s after road signal";
    if (r.flags & LF_BOOT)       Serial.printf("POWER ON   saved floor %s\n", floorLabel(r.rawCm).c_str());
    else if (r.flags & LF_STORE) Serial.printf("  t=%5lus  SAVED %s at %+.2fm (%s)\n", (unsigned long)r.t, floorLabel(r.rawCm).c_str(), r.cntCm / 100.0, road.c_str());
    else Serial.printf("  t=%5lus  ramp %3us  raw %+.2fm  counted %+.2fm  %s%s%s\n", (unsigned long)r.t, r.dur,
                       r.rawCm / 100.0, r.cntCm / 100.0, road.c_str(), (r.flags & LF_GPS) ? " [GPS]" : "", (r.flags & LF_SKIP) ? " [skipped]" : "");
  }
  Serial.println("---- end of log ----");
}

void publish(const String& label) {
  if (!floorChar) return;
  floorChar.setValue(label);        // String overload; a const char* would hit the template and send pointer bytes
  if (floorChar.getSubscribedCount() > 0) floorChar.notify();
}

// GY-BMP280 boards answer at 0x76 or 0x77. Some are really BME280 (chip id 0x60);
// its pressure part is the same, so accept it too.
bool beginSensor() {
  for (uint8_t addr : {0x76, 0x77})
    for (uint8_t id : {BMP280_CHIPID, 0x60})
      if (bmp.begin(addr, id)) return true;
  return false;
}

float readMedianHpa() {
  float v[5];
  for (int i = 0; i < 5; i++) {
    v[i] = bmp.readPressure() / 100.0;
    delay(180);
  }
  for (int i = 1; i < 5; i++)
    for (int j = i; j > 0 && v[j] < v[j - 1]; j--) { float t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
  return v[2];
}

// Height of pressure p relative to pRef (m). Up = positive.
float heightDiff(float p, float pRef) {
  return 44330.0 * (1.0 - pow(p / pRef, 1.0 / 5.255));
}

// Feed one height change per second. Returns height (m) moved on ramps in the last LOOKBACK_S.
float stepTracker(float dh) {
  rawDh[idx] = dh;
  float windowSum = 0;
  for (int k = 0; k < MOVE_WINDOW_S; k++) windowSum += rawDh[(idx - k + LOOKBACK_S) % LOOKBACK_S];
  bool nowMoving = fabs(windowSum) > MOVE_M;
  if (!nowMoving) skipEpisode = false;
  if (nowMoving && !moving) {
    // movement started somewhere inside the window: keep those seconds too
    for (int k = 1; k < MOVE_WINDOW_S; k++) {
      int j = (idx - k + LOOKBACK_S) % LOOKBACK_S;
      accDh[j] = rawDh[j];
      hSinceRoad += rawDh[j];
    }
  }
  moving = nowMoving;
  accDh[idx] = moving ? dh : 0;
  if (moving && !skipEpisode) hSinceRoad += dh;
  idx = (idx + 1) % LOOKBACK_S;

  float h = 0;
  for (int k = 0; k < LOOKBACK_S; k++) h += accDh[k];
  return h;
}

// 0 -> "1F", -1 -> "B1", +1 -> "2F"
String floorLabel(int f) {
  return f < 0 ? "B" + String(-f) : String(f + 1) + "F";
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!beginSensor()) {
    Serial.println("BMP280 not found. Check wiring (VCC=3.3V, GND, SDA=8, SCL=9).");
    while (true) delay(1000);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,    // temperature
                  Adafruit_BMP280::SAMPLING_X16,   // pressure
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_1);
  delay(500);                       // let the filter settle

  hasOled = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!hasOled) Serial.println("No OLED found, continuing without screen.");

  prefs.begin("park", false);
  storedFloor = steadyFloor = offsetFloor = prefs.getChar("floor", 0);
  Serial.printf("Starting from saved floor %s\n", floorLabel(offsetFloor).c_str());
  if (prefs.getBytesLength("log") == sizeof(logBuf)) {
    prefs.getBytes("log", logBuf, sizeof(logBuf));
    logHead = prefs.getUChar("logHead", 0) % LOG_N;
    logCount = min((int)prefs.getUChar("logCount", 0), LOG_N);
  }
  addLog(0, offsetFloor, 0, LF_BOOT);       // power-on marker (rawCm holds the saved floor number)
  printLog();

  setupBle();
  prevP = readMedianHpa();
}

void loop() {
  float p = readMedianHpa();        // takes ~1 s, sets the loop rate
  float dh = heightDiff(p, prevP);
  float hRoadBefore = hSinceRoad;
  float h = stepTracker(dh);
  prevP = p;
  rawSum += dh;
  if (Serial.available() && Serial.read() == 'L') printLog();

  // ramp segment start/end, for the trip log
  if (moving && !wasMoving) {
    float win = 0;                  // the segment really started up to MOVE_WINDOW_S ago (stepTracker backfills it)
    for (int k = 1; k <= MOVE_WINDOW_S; k++) win += rawDh[(idx - k + LOOKBACK_S) % LOOKBACK_S];
    epStartMs = millis() - MOVE_WINDOW_S * 1000UL;
    epRawStart = rawSum - win;
    epCntStart = hRoadBefore;
    epSkipped = false;
  }
  if (moving) epSkipped |= skipEpisode;
  if (!moving && wasMoving)
    addLog((millis() - epStartMs) / 1000, rawSum - epRawStart, hSinceRoad - epCntStart,
           (gpsSeen ? LF_GPS : 0) | (epSkipped ? LF_SKIP : 0));

  if (wasMoving && !moving && offsetUntilMs < 0) {
    // First ramp after power-on is over (e.g. drove up out of the home garage):
    // fold it into the offset and restart the lookback from here.
    offsetFloor += (int)round(h / FLOOR_HEIGHT_M);
    memset(accDh, 0, sizeof(accDh));
    h = 0;
    offsetUntilMs = millis() + LOOKBACK_S * 1000L;
  }
  wasMoving = moving;
  if (offsetUntilMs >= 0 && (long)(millis() - offsetUntilMs) > 0) offsetFloor = 0;

  if (roadSignal) {
    roadSignal = false;
    gpsSeen = true;
    hSinceRoad = 0;
    skipEpisode = moving;
    roadEver = true;
    lastRoadMs = millis();
    if (moving) epCntStart = 0;     // counting restarts mid-segment
  }
  // With GPS: count ramps since the last fast driving. Without: 3-minute lookback + saved floor.
  int floorNum = gpsSeen ? (int)round(hSinceRoad / FLOOR_HEIGHT_M)
                         : (int)round(h / FLOOR_HEIGHT_M) + offsetFloor;
  // height the floor is computed from, shown on screen so the floor height can be calibrated
  float usedH = gpsSeen ? hSinceRoad : h + offsetFloor * FLOOR_HEIGHT_M;
  steadySecs = (floorNum == steadyFloor && !moving) ? steadySecs + 1 : 0;
  steadyFloor = floorNum;
  if (steadySecs >= STORE_AFTER_S && floorNum != storedFloor) {
    prefs.putChar("floor", floorNum);
    storedFloor = floorNum;
    addLog(0, floorNum, usedH, LF_STORE | (gpsSeen ? LF_GPS : 0));  // rawCm holds the floor number
  }

  String label = floorLabel(floorNum);
  Serial.printf("floor %s | height %+.2f m | %s | BLE %s | GPS %s\n", label.c_str(), usedH,
                moving ? "MOVING" : "still", bleConnected ? "on" : "off", gpsSeen ? "on" : "off");
  publish(label);
  if (hasOled) showOled(label, usedH);
}

// Big floor label, small status line: BT = phone connected, GPS = road signals received, height in m.
void showOled(const String& label, float height) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(5);
  oled.setCursor((128 - 30 * label.length()) / 2, 6);
  oled.print(label);
  oled.setTextSize(1);
  oled.setCursor(0, 56);
  oled.printf("%s %s %-6s %+.1fm", bleConnected ? "BT" : "  ", gpsSeen ? "GPS" : "   ", moving ? "MOVING" : "", height);
  oled.display();
}
