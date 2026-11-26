#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <HX711.h>
#include <Preferences.h>
#include <math.h>

// ======================================================
// WIFI AP CONFIG
// ======================================================

const char* AP_SSID     = "PickToLight-Edge";
const char* AP_PASSWORD = "assembly123";

IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GW(192, 168, 4, 1);
IPAddress AP_SUB(255, 255, 255, 0);

DNSServer dnsServer;
WebServer server(80);

// ======================================================
// LED CONFIG — two strips, 6 bins split across them
// ======================================================
//
// FIX (was broken): the old single-strip lightBin() referenced
// leds[]/BIN_COLORS[] which no longer exist after switching to two
// strips. Every LED function below now consistently uses ledsA/ledsB
// via the BIN_MAP lookup table — nothing else references the old
// single-strip arrays anymore.

#define LED_PIN_A       13
#define LED_PIN_B       12

#define NUM_BINS        6

#define LEDS_STRIP_A    34
#define LEDS_STRIP_B    34

CRGB ledsA[LEDS_STRIP_A];
CRGB ledsB[LEDS_STRIP_B];
#define LED_BRIGHTNESS  60

// Status colors — requested mapping:
//   Yellow = pick from this bin now
//   Green  = correct quantity picked
//   Orange = correct bin, wrong quantity
//   Red    = wrong bin picked
//   Off    = inactive bin
const CRGB COLOR_PICK      = CRGB(255, 255, 0);   // Yellow
const CRGB COLOR_CORRECT   = CRGB(0, 255, 0);     // Green
const CRGB COLOR_WRONG_QTY = CRGB(255, 128, 0);   // Orange
const CRGB COLOR_WRONG_BIN = CRGB(255, 0, 0);     // Red
const CRGB COLOR_OFF       = CRGB::Black;

struct BinMap {
  bool    stripA;   // true = strip A, false = strip B
  uint8_t start;     // starting LED index on that strip
};

const uint8_t LEDS_PER_BIN = 9;

const BinMap BIN_MAP[NUM_BINS] = {
  { true,  0  },   // Bin 1 -> Strip A, LEDs 0-8
  { true,  12 },   // Bin 2 -> Strip A, LEDs 12-20
  { true,  24 },   // Bin 3 -> Strip A, LEDs 24-32
  { false, 0  },   // Bin 4 -> Strip B, LEDs 0-8
  { false, 12 },   // Bin 5 -> Strip B, LEDs 12-20
  { false, 24 },   // Bin 6 -> Strip B, LEDs 24-32
};

// ======================================================
// HX711 CONFIG — ONE MODULE PER BIN, each with its OWN clock line
// ======================================================
//
// FIX (was broken): bins 1-3 previously used GPIO 35/36/37, which on
// the N16R8 are physically wired to the Octal PSRAM/Flash and are not
// available as general-purpose IO — using them corrupts memory access
// or hangs the board, even if the sketch compiles. All 6 SCK lines
// also previously shared GPIO 45 (a strapping pin); each channel now
// gets its own independent DT+SCK pair on confirmed-safe GPIOs.

struct PinPair { uint8_t dt; uint8_t sck; };

const PinPair HX_PINS[NUM_BINS] = {
  // ESP32-S3 N16R8 safe GPIOs. GPIO 35/36/37 are used by Octal PSRAM,
  // and GPIO 45 is a strapping pin, so none of those pins are used here.
  // IMPORTANT: wire each HX711 to the DT/SCK pair shown below.
  {  4, 18},  
  {  5, 3 },   
  {  7, 17 },  
  {  6, 8 },  
  { 9, 10 },   
  { 16, 11 },   
};

#define DEFAULT_CALIBRATION_FACTOR  400.5439f

// Measured calibration factors per channel (from Calibration_factor.txt).
// Order is Bin 1 -> Bin 6. Used as the fallback default in
// loadCalibrationFromNVS() whenever NVS has no stored value for that
// bin yet (e.g. first boot, or after an NVS erase) — it does not
// override a value you've already saved via the serial 'y' confirm
// step, so your existing calibration in NVS still wins as-is.
const float FACTORY_CALIBRATION_FACTOR[NUM_BINS] = {
  427.6983f,  // Bin 1
  402.1431f,  // Bin 2
  405.6976f,  // Bin 3
  308.0060f,  // Bin 4
  417.0097f,  // Bin 5
  428.4901f   // Bin 6
};

#define DRIFT_RECAL_PCT          3.0f
#define MIN_PICK_WEIGHT_G        2.0f

// Low-stock alert threshold — bins below this % of their configured
// full-bin capacity are flagged in /api/lowstock and /api/status.
#define LOW_STOCK_THRESHOLD_PCT  20.0f

HX711 scales[NUM_BINS];
bool  scaleReady[NUM_BINS]      = { false };

float calibFactor[NUM_BINS];
float baselineWeight[NUM_BINS];
float currentWeight[NUM_BINS];

// Full-bin capacity in grams — "100% stock" reference per bin. Configured
// via sequences.json's top-level "binCapacities" array, or defaults to 0
// (capacity unknown -> low-stock check skipped for that bin).
float binCapacityGrams[NUM_BINS] = { 0.0f };

// ======================================================
// CALIBRATION STATE (NVS-backed, PER CHANNEL)
// ======================================================

enum CalState {
  CAL_IDLE = 0, CAL_WAIT_EMPTY, CAL_WAIT_TARE, CAL_WAIT_KNOWN,
  CAL_WAIT_PLACE, CAL_CONFIRM, CAL_WAIT_PART,
};

CalState calState        = CAL_IDLE;
int      calTargetBin    = -1;
float    calKnownG[NUM_BINS];
float    calNewFactor    = 0.0f;
float    calVerifyG      = 0.0f;
float    calErrorPct     = 0.0f;
bool     driftWarning[NUM_BINS]  = { false };
float    storedPartWeight[NUM_BINS] = { 0.0f };

Preferences prefs;
static const char* NVS_NS = "ptl";

String nvsKey(const char* base, int binIdx) {
  return String(base) + String(binIdx);
}

// ======================================================
// LED STATUS STATE — drives the wrong-bin / wrong-qty colors
// ======================================================
//
// New: tracks a per-bin "status" so any bin can independently show
// pick/correct/wrong-qty/wrong-bin/off, instead of only ever lighting
// the single active bin yellow. confirmStep() sets these after every
// weight check; resetSession()/session start clears them.

enum BinLedStatus { LED_OFF = 0, LED_PICK, LED_CORRECT, LED_WRONG_QTY, LED_WRONG_BIN };
BinLedStatus binLedStatus[NUM_BINS] = { LED_OFF };
CRGB colorForStatus(BinLedStatus status);

// Briefly hold a wrong-bin/wrong-qty color before reverting to the
// correct "pick" indicator, so the operator sees the alert rather
// than it being instantly overwritten by the next render call.
#define ALERT_HOLD_MS 2500
unsigned long alertHoldUntil[NUM_BINS] = { 0 };

// ======================================================
// EMPLOYEE ROSTER (fixed list, loaded from LittleFS)
// ======================================================
//
// employees.json on LittleFS:
// { "employees": [ { "id":"EMP001", "name":"Alice", "pin":"1234",
//   "department":"Assembly", "designation":"Operator", "shift":"Shift A",
//   "status":"Active", "lastUpdated":"2026-06-20T13:00:00Z" }, ... ] }
//
// This is shop-floor identification, NOT cryptographic security — PINs
// are short and stored in plaintext on the device. Session tokens are simple
// random strings with an eight-hour idle expiry. That's an
// appropriate tradeoff for a device with no internet and no real attacker
// model beyond "make sure the right name shows up in the log".

#define MAX_EMPLOYEES 20
#define MAX_ACTIVE_TOKENS 6   // matches NUM_BINS as a sane operator ceiling
#define TOKEN_TTL_MS (8UL * 60UL * 60UL * 1000UL)

struct Employee {
  String id;
  String name;
  String pin;
  String department;
  String designation;
  String shift;
  String status;
  String lastUpdated;
};

Employee employees[MAX_EMPLOYEES];
int totalEmployees = 0;

// FIX (was broken): this struct had a default member initializer
// (`bool inUse = false;`), which makes it a non-aggregate under the
// Arduino toolchain's C++ dialect, so brace-init like
// `activeTokens[slot] = { token, id, name, true };` fails to compile.
// Removed the default initializer; inUse is now explicitly set to
// false for every slot in setup() instead.
struct ActiveSessionToken {
  String token;
  String employeeId;
  String employeeName;
  bool   inUse;
  unsigned long expiresAtMs;
};

ActiveSessionToken activeTokens[MAX_ACTIVE_TOKENS];

int parseCsvLine(const String& line, String* fields, int maxFields) {
  int count = 0;
  String current;
  bool quoted = false;
  for (unsigned int i = 0; i < line.length(); i++) {
    char c = line[i];
    if (c == '"') {
      if (quoted && i + 1 < line.length() && line[i + 1] == '"') {
        current += '"';
        i++;
      } else {
        quoted = !quoted;
      }
    } else if (c == ',' && !quoted) {
      if (count < maxFields) fields[count++] = current;
      current = "";
    } else {
      current += c;
    }
  }
  if (count < maxFields) fields[count++] = current;
  return count;
}

bool loadEmployees() {
  // Preferred workflow: edit the supplied workbook, export the Employees sheet
  // as employees.csv, then upload the data folder to LittleFS.
  if (LittleFS.exists("/employees.csv")) {
    File csv = LittleFS.open("/employees.csv", "r");
    if (!csv) { Serial.println("[AUTH] Cannot open employees.csv"); return false; }
    totalEmployees = 0;
    bool firstLine = true;
    while (csv.available() && totalEmployees < MAX_EMPLOYEES) {
      String line = csv.readStringUntil('\n');
      line.replace("\r", "");
      if (firstLine) { firstLine = false; continue; }
      if (line.length() == 0) continue;
      String column[8];
      int columns = parseCsvLine(line, column, 8);
      if (columns < 7) continue;
      Employee& employee = employees[totalEmployees];
      employee.id          = column[0];
      employee.name        = column[1];
      employee.department  = column[2];
      employee.designation = column[3];
      employee.shift       = column[4];
      employee.status      = column[5];
      employee.pin         = column[6];
      employee.lastUpdated = columns > 7 ? column[7] : "";
      employee.id.trim(); employee.id.toUpperCase();
      employee.name.trim(); employee.pin.trim(); employee.status.trim();
      if (employee.id.length() > 0 && employee.name.length() > 0 &&
          employee.pin.length() > 0) totalEmployees++;
    }
    csv.close();
    Serial.printf("[AUTH] Loaded %d employees from CSV\n", totalEmployees);
    return totalEmployees > 0;
  }

  if (!LittleFS.exists("/employees.json")) {
    Serial.println("[AUTH] employees.csv/employees.json missing — login will always fail");
    return false;
  }
  File f = LittleFS.open("/employees.json", "r");
  if (!f) { Serial.println("[AUTH] Cannot open employees.json"); return false; }

  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { Serial.printf("[AUTH] JSON error: %s\n", err.c_str()); return false; }

  JsonArray arr = doc["employees"].as<JsonArray>();
  if (arr.isNull()) { Serial.println("[AUTH] employees array missing"); return false; }

  totalEmployees = 0;
  for (JsonObject e : arr) {
    if (totalEmployees >= MAX_EMPLOYEES) break;
    employees[totalEmployees].id          = e["id"]          | "";
    employees[totalEmployees].name        = e["name"]        | "";
    if (employees[totalEmployees].id.length() == 0)
      employees[totalEmployees].id = e["employeeId"] | "";
    if (employees[totalEmployees].name.length() == 0)
      employees[totalEmployees].name = e["employeeName"] | "Unknown";
    employees[totalEmployees].pin         = e["pin"]         | "";
    employees[totalEmployees].department  = e["department"]  | "Assembly";
    employees[totalEmployees].designation = e["designation"] | "Operator";
    employees[totalEmployees].shift       = e["shift"]       | "General";
    employees[totalEmployees].status      = e["status"]      | "Active";
    employees[totalEmployees].lastUpdated = e["lastUpdated"] | "";
    employees[totalEmployees].id.trim();
    employees[totalEmployees].id.toUpperCase();
    if (employees[totalEmployees].id.length() > 0 &&
        employees[totalEmployees].pin.length() > 0) totalEmployees++;
  }
  Serial.printf("[AUTH] Loaded %d employees\n", totalEmployees);
  return totalEmployees > 0;
}

// Simple pseudo-random token — sufficient for a shop-floor session tag,
// not intended to resist a determined attacker.
String generateToken() {
  String t = "";
  const char chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
  for (int i = 0; i < 16; i++) t += chars[random(0, 62)];
  return t;
}

int findEmployeeByCreds(const String& id, const String& pin) {
  String normalizedId = id;
  normalizedId.trim();
  normalizedId.toUpperCase();
  for (int i = 0; i < totalEmployees; i++) {
    if (employees[i].id == normalizedId && employees[i].pin == pin &&
        employees[i].status.equalsIgnoreCase("Active")) return i;
  }
  return -1;
}

int findFreeTokenSlot() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_ACTIVE_TOKENS; i++) {
    if (activeTokens[i].inUse &&
        (long)(activeTokens[i].expiresAtMs - now) <= 0) activeTokens[i].inUse = false;
    if (!activeTokens[i].inUse) return i;
  }
  return -1;
}

int findTokenSlot(const String& token) {
  unsigned long now = millis();
  for (int i = 0; i < MAX_ACTIVE_TOKENS; i++) {
    if (activeTokens[i].inUse &&
        (long)(activeTokens[i].expiresAtMs - now) <= 0) activeTokens[i].inUse = false;
    if (activeTokens[i].inUse && activeTokens[i].token == token) {
      activeTokens[i].expiresAtMs = now + TOKEN_TTL_MS;
      return i;
    }
  }
  return -1;
}

String tokenFromCookie() {
  if (!server.hasHeader("Cookie")) return "";
  String cookie = server.header("Cookie");
  const String key = "PTLSESSION=";
  int start = cookie.indexOf(key);
  if (start < 0) return "";
  int end = cookie.indexOf(';', start + key.length());
  if (end < 0) end = cookie.length();
  return cookie.substring(start + key.length(), end);
}

String requestToken(JsonDocument& request) {
  String token = request["token"] | "";
  if (token.length() == 0) token = tokenFromCookie();
  return token;
}

// Looks up the operator name/id for a token, used when starting a
// session so the assembly log records who's running it. Falls back
// to "Unknown" if the token is missing/expired rather than blocking
// the session — a stale token shouldn't stop the assembly line.
void resolveOperator(const String& token, String& outId, String& outName) {
  int slot = findTokenSlot(token);
  if (slot >= 0) {
    outId   = activeTokens[slot].employeeId;
    outName = activeTokens[slot].employeeName;
  } else {
    outId   = "—";
    outName = "Unknown";
  }
}

// ======================================================
// ASSEMBLY LOG (LittleFS-persisted, ring-buffered)
// ======================================================
//
// assembly_log.json on LittleFS:
// { "logs": [ { assemblyId, operatorId, operatorName, modelName,
//               startedAtSec, completedAtSec, durationSec, status }, ... ] }
//
// Capped at MAX_LOG_ENTRIES to avoid unbounded flash growth — oldest
// entries are dropped once the cap is hit (ring-buffer behavior).

#define MAX_LOG_ENTRIES 200

struct LogEntry {
  String assemblyId;
  String operatorId;
  String operatorName;
  String modelName;
  unsigned long startedAtSec;
  long   completedAtSec;   // -1 if still in progress
  long   durationSec;      // -1 if still in progress
  String status;           // "in_progress" | "completed" | "aborted"
  String startTimestamp;   // ISO-8601 timestamp supplied by the dashboard
  String completionTimestamp;
};

void appendLogEntry(const LogEntry& entry);

LogEntry assemblyLog[MAX_LOG_ENTRIES];
int logCount = 0;          // how many valid entries (grows to MAX then wraps)
int logHead  = 0;          // index where the NEXT entry will be written
int nextAssemblyNum = 1;

int currentLogIndex = -1;  // index of the in-progress log entry, or -1

String makeAssemblyId() {
  char buf[12];
  snprintf(buf, sizeof(buf), "A%04d", nextAssemblyNum++);
  return String(buf);
}

void appendLogEntry(const LogEntry& entry) {
  assemblyLog[logHead] = entry;
  currentLogIndex = logHead;
  logHead = (logHead + 1) % MAX_LOG_ENTRIES;
  if (logCount < MAX_LOG_ENTRIES) logCount++;
}

void updateCurrentLogEntry(const String& status, long completedAtSec, long durationSec,
                           const String& completionTimestamp) {
  if (currentLogIndex < 0) return;
  assemblyLog[currentLogIndex].status        = status;
  assemblyLog[currentLogIndex].completedAtSec = completedAtSec;
  assemblyLog[currentLogIndex].durationSec    = durationSec;
  assemblyLog[currentLogIndex].completionTimestamp = completionTimestamp;
}

bool saveLogToLittleFS() {
  // Capacity covers 200 records including ISO timestamps on the ESP32-S3.
  DynamicJsonDocument doc(65536);
  JsonArray arr = doc.createNestedArray("logs");

  int start = (logCount < MAX_LOG_ENTRIES) ? 0 : logHead;
  for (int n = 0; n < logCount; n++) {
    int idx = (start + n) % MAX_LOG_ENTRIES;
    LogEntry& e = assemblyLog[idx];
    JsonObject o = arr.createNestedObject();
    o["assemblyId"]     = e.assemblyId;
    o["operatorId"]     = e.operatorId;
    o["operatorName"]   = e.operatorName;
    o["modelName"]      = e.modelName;
    o["startedAtSec"]   = e.startedAtSec;
    o["completedAtSec"] = e.completedAtSec;
    o["durationSec"]    = e.durationSec;
    o["status"]         = e.status;
    o["startTimestamp"] = e.startTimestamp;
    o["completionTimestamp"] = e.completionTimestamp;
  }

  File f = LittleFS.open("/assembly_log.json", "w");
  if (!f) { Serial.println("[LOG] Failed to open assembly_log.json for write"); return false; }
  serializeJson(doc, f);
  f.close();
  return true;
}

void loadLogFromLittleFS() {
  if (!LittleFS.exists("/assembly_log.json")) {
    Serial.println("[LOG] No existing assembly_log.json — starting fresh");
    return;
  }
  File f = LittleFS.open("/assembly_log.json", "r");
  if (!f) return;

  DynamicJsonDocument doc(65536);
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { Serial.printf("[LOG] Parse error: %s — starting fresh\n", err.c_str()); return; }

  JsonArray arr = doc["logs"].as<JsonArray>();
  if (arr.isNull()) return;

  logCount = 0; logHead = 0;
  int maxNum = 0;
  for (JsonObject o : arr) {
    if (logCount >= MAX_LOG_ENTRIES) break;
    LogEntry& e = assemblyLog[logCount];
    e.assemblyId     = o["assemblyId"]   | "";
    e.operatorId     = o["operatorId"]   | "";
    e.operatorName   = o["operatorName"] | "";
    e.modelName      = o["modelName"]    | "";
    e.startedAtSec   = o["startedAtSec"]   | 0;
    e.completedAtSec = o["completedAtSec"] | -1;
    e.durationSec    = o["durationSec"]    | -1;
    e.status         = o["status"]       | "completed";
    e.startTimestamp = o["startTimestamp"] | "";
    e.completionTimestamp = o["completionTimestamp"] | "";
    logCount++;

    int num = e.assemblyId.substring(1).toInt();
    if (num > maxNum) maxNum = num;
  }
  logHead = logCount % MAX_LOG_ENTRIES;
  nextAssemblyNum = maxNum + 1;
  Serial.printf("[LOG] Restored %d log entries (next ID: A%04d)\n", logCount, nextAssemblyNum);
}

int findLogIndexByAssemblyId(const String& assemblyId) {
  if (assemblyId.length() == 0) return -1;
  for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
    if (assemblyLog[i].assemblyId == assemblyId) return i;
  }
  return -1;
}

String historyStatus(const String& storedStatus) {
  if (storedStatus == "completed" || storedStatus == "Completed") return "Completed";
  if (storedStatus == "aborted" || storedStatus == "cancelled" ||
      storedStatus == "Cancelled") return "Cancelled";
  return "In Progress";
}

String durationLabel(long totalSeconds) {
  if (totalSeconds < 0) return "—";
  long hours = totalSeconds / 3600;
  long minutes = (totalSeconds % 3600) / 60;
  long seconds = totalSeconds % 60;
  String result;
  if (hours > 0) result += String(hours) + "h ";
  if (minutes > 0 || hours > 0) result += String(minutes) + "m ";
  result += String(seconds) + "s";
  return result;
}

String csvEscape(String value) {
  value.replace("\r", " ");
  value.replace("\n", " ");
  if (value.indexOf(',') >= 0 || value.indexOf('"') >= 0) {
    value.replace("\"", "\"\"");
    return "\"" + value + "\"";
  }
  return value;
}

// ======================================================
// DATA STRUCTURES — ASSEMBLY MODELS
// ======================================================

struct Step {
  int    step;
  String part;
  int    bin;
  int    qty;
  float  unitWeight;
};

struct Model {
  int    id;
  String name;
  String image;
  int    totalSteps;
  Step   steps[20];
};

Model models[10];
int   totalModels = 0;

// ======================================================
// SESSION VARIABLES
// ======================================================

bool   sessionActive     = false;
int    activeModelIndex  = -1;
int    currentStepIndex  = 0;
bool   baselinesCaptured = false;
String sessionOperatorId, sessionOperatorName;
unsigned long sessionStartedAtSec = 0;

// ======================================================
// FORWARD DECLARATIONS
// ======================================================

bool   loadJSON();
void   initLoadCells();
bool   captureBaseline(int binIdx);
float  readWeight(int binIdx, uint8_t samples = 8);
float  readMedian(int binIdx, int n = 7);
float  getStepUnitWeight(const Step& step);
float  lookupPartUnitWeight(const String& part);
void   clearLEDs();
void   lightBin(int binIdx);
void   showBinColor(int binIdx, CRGB color);
void   renderAllBinLeds();
void   addCORSHeaders();
void   handleOptions();
void   sendJson(int code, const String& body);
int    findModelIndexById(int modelId);
String getMimeType(const String& path);
void   loadCalibrationFromNVS(int binIdx);
void   saveCalibrationToNVS(int binIdx);
void   startCalibration(int binIdx);
void   handleCalCommand(const String& cmd);
void   calPrintState();
String calStatusJson();

// ======================================================
// MIME TYPES
// ======================================================

String getMimeType(const String& path) {
  if (path.endsWith(".html")) return "text/html";
  if (path.endsWith(".css"))  return "text/css";
  if (path.endsWith(".js"))   return "application/javascript";
  if (path.endsWith(".json")) return "application/json";
  if (path.endsWith(".jpg") || path.endsWith(".jpeg")) return "image/jpeg";
  if (path.endsWith(".png"))  return "image/png";
  if (path.endsWith(".gif"))  return "image/gif";
  if (path.endsWith(".ico"))  return "image/x-icon";
  if (path.endsWith(".svg"))  return "image/svg+xml";
  return "application/octet-stream";
}

// ======================================================
// NVS — LOAD / SAVE CALIBRATION (PER CHANNEL)
// ======================================================

void loadCalibrationFromNVS(int binIdx) {
  prefs.begin(NVS_NS, true);
  float stored = prefs.getFloat(nvsKey("calf", binIdx).c_str(), 0.0f);
  calKnownG[binIdx]        = prefs.getFloat(nvsKey("know", binIdx).c_str(), 0.0f);
  storedPartWeight[binIdx] = prefs.getFloat(nvsKey("part", binIdx).c_str(), 0.0f);
  prefs.end();

  if (stored != 0.0f) {
    calibFactor[binIdx] = stored;
  } else {
    calibFactor[binIdx] = FACTORY_CALIBRATION_FACTOR[binIdx];
  }
  if (scaleReady[binIdx]) scales[binIdx].set_scale(calibFactor[binIdx]);
}

void saveCalibrationToNVS(int binIdx) {
  prefs.begin(NVS_NS, false);
  prefs.putFloat(nvsKey("calf", binIdx).c_str(), calibFactor[binIdx]);
  prefs.putFloat(nvsKey("know", binIdx).c_str(), calKnownG[binIdx]);
  prefs.putFloat(nvsKey("part", binIdx).c_str(), storedPartWeight[binIdx]);
  prefs.end();
}

// ======================================================
// HX711 HELPERS (PER CHANNEL)
// ======================================================

float readMedian(int binIdx, int n) {
  if (!scaleReady[binIdx] || !scales[binIdx].is_ready()) return NAN;
  float buf[15];
  int   cnt = min(n, 15);
  for (int i = 0; i < cnt; i++) { buf[i] = scales[binIdx].get_units(1); delay(10); }
  for (int i = 1; i < cnt; i++) {
    float key = buf[i]; int j = i - 1;
    while (j >= 0 && buf[j] > key) { buf[j + 1] = buf[j]; j--; }
    buf[j + 1] = key;
  }
  return buf[cnt / 2];
}

void initLoadCells() {
  for (int b = 0; b < NUM_BINS; b++) {
    scales[b].begin(HX_PINS[b].dt, HX_PINS[b].sck);
    unsigned long t = millis();
    while (!scales[b].is_ready() && millis() - t < 1500) delay(50);

    if (!scales[b].is_ready()) {
      scaleReady[b] = false;
      Serial.printf("[HX711] Bin %d — not responding (DT=%d SCK=%d)\n",
                    b + 1, HX_PINS[b].dt, HX_PINS[b].sck);
      continue;
    }
    scaleReady[b] = true;
    loadCalibrationFromNVS(b);
    scales[b].tare(10);
    baselineWeight[b] = 0.0f;
    currentWeight[b]  = 0.0f;
    Serial.printf("[HX711] Bin %d ready — Factor=%.4f\n", b + 1, calibFactor[b]);
  }
}

float readWeight(int binIdx, uint8_t samples) {
  if (binIdx < 0 || binIdx >= NUM_BINS || !scaleReady[binIdx]) return 0.0f;
  if (!scales[binIdx].is_ready()) return currentWeight[binIdx];
  return fabsf(scales[binIdx].get_units(samples));
}

bool captureBaseline(int binIdx) {
  if (binIdx < 0 || binIdx >= NUM_BINS || !scaleReady[binIdx]) return false;
  baselineWeight[binIdx] = readWeight(binIdx, 10);
  currentWeight[binIdx]  = baselineWeight[binIdx];
  Serial.printf("[HX711] Bin %d baseline = %.2f g\n", binIdx + 1, baselineWeight[binIdx]);
  return true;
}

// ======================================================
// LOW STOCK CALCULATION
// ======================================================

struct LowStockResult {
  int   bin;
  float percentRemaining;
  float currentGrams;
  float capacityGrams;
};

int computeLowStock(LowStockResult* outResults, int maxResults);

int computeLowStock(LowStockResult* outResults, int maxResults) {
  int count = 0;
  for (int b = 0; b < NUM_BINS && count < maxResults; b++) {
    if (!scaleReady[b] || binCapacityGrams[b] <= 0.0f) continue;

    float live = readWeight(b, 4);
    float pct  = (live / binCapacityGrams[b]) * 100.0f;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    if (pct < LOW_STOCK_THRESHOLD_PCT) {
      outResults[count].bin              = b + 1;
      outResults[count].percentRemaining = pct;
      outResults[count].currentGrams     = live;
      outResults[count].capacityGrams    = binCapacityGrams[b];
      count++;
    }
  }
  return count;
}

// Finds the part name currently assigned to a bin, for friendlier
// low-stock alert labels. Looks across ALL models' steps, returns the
// first match — bins are usually dedicated to one part type anyway.
String findPartLabelForBin(int binNumber) {
  for (int m = 0; m < totalModels; m++) {
    for (int s = 0; s < models[m].totalSteps; s++) {
      if (models[m].steps[s].bin == binNumber) return models[m].steps[s].part;
    }
  }
  return "";
}

// ======================================================
// CALIBRATION STATE MACHINE (per-channel, unchanged logic)
// ======================================================

void startCalibration(int binIdx) {
  if (binIdx < 0 || binIdx >= NUM_BINS) { Serial.println("[CAL] Invalid bin index"); return; }
  if (!scaleReady[binIdx]) { Serial.printf("[CAL] Bin %d sensor not detected\n", binIdx + 1); return; }
  calTargetBin = binIdx;
  calState     = CAL_WAIT_EMPTY;
  calPrintState();
}

void calPrintState() {
  Serial.println();
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.printf("  CALIBRATING BIN %d\n", calTargetBin + 1);
  switch (calState) {
    case CAL_WAIT_EMPTY:
      Serial.println("  STEP 1 — EMPTY THIS BIN'S SCALE, press Enter when stable");
      break;
    case CAL_WAIT_KNOWN:
      Serial.println("  STEP 2 — type known reference weight (g) and Enter");
      break;
    case CAL_WAIT_PLACE:
      Serial.printf("  Place %.1f g reference, press Enter when stable\n", calKnownG[calTargetBin]);
      break;
    case CAL_CONFIRM:
      Serial.printf("  Factor=%.4f  Verify=%.2fg (exp %.2fg)  Error=%.1f%%\n",
                    calNewFactor, calVerifyG, calKnownG[calTargetBin], calErrorPct);
      Serial.println("  'y' to SAVE | 'r' to REDO | 'n' to cancel");
      break;
    case CAL_WAIT_PART:
      Serial.println("  STEP 3 — place ONE part, send 'p' to capture, or 'skip'");
      break;
    default: break;
  }
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

String calStatusJson() {
  DynamicJsonDocument doc(768);
  doc["state"]     = (int)calState;
  doc["targetBin"] = calTargetBin + 1;
  doc["newFactor"]   = calNewFactor;
  doc["verifyGrams"] = calVerifyG;
  doc["errorPct"]    = calErrorPct;
  JsonArray channels = doc.createNestedArray("channels");
  for (int b = 0; b < NUM_BINS; b++) {
    JsonObject c = channels.createNestedObject();
    c["bin"] = b + 1; c["ready"] = scaleReady[b];
    c["calibFactor"] = calibFactor[b]; c["driftWarning"] = driftWarning[b];
  }
  String out; serializeJson(doc, out); return out;
}

void handleCalCommand(const String& cmdIn) {
  String cmd = cmdIn;

  if (cmd.startsWith("cal ") || cmd.startsWith("CAL ")) {
    int bin = cmd.substring(4).toInt();
    if (bin < 1 || bin > NUM_BINS) { Serial.printf("[CAL] Usage: cal <1-%d>\n", NUM_BINS); return; }
    if (sessionActive) { Serial.println("[CAL] Cannot calibrate during active session!"); return; }
    startCalibration(bin - 1);
    return;
  }

  if (cmd.startsWith("check ")) {
    int bin = cmd.substring(6).toInt();
    if (bin < 1 || bin > NUM_BINS) return;
    int b = bin - 1;
    if (calKnownG[b] <= 0) { Serial.printf("[CHECK] Bin %d has no reference weight.\n", bin); return; }
    delay(2000);
    float measured = fabsf(readMedian(b, 15));
    float errPct = (fabsf(measured - calKnownG[b]) / calKnownG[b]) * 100.0f;
    driftWarning[b] = (errPct > DRIFT_RECAL_PCT);
    Serial.printf("[CHECK] Bin %d — %.2fg vs ref %.2fg (%.1f%% err)\n", bin, measured, calKnownG[b], errPct);
    return;
  }

  if (calState == CAL_IDLE) return;
  int b = calTargetBin;

  if (cmd == "r" || cmd == "R") {
    scales[b].set_scale(1.0f); calNewFactor = 0; calState = CAL_WAIT_EMPTY; calPrintState(); return;
  }
  if (cmd == "t" || cmd == "T" || cmd == "tare" || (calState == CAL_WAIT_EMPTY && cmd == "")) {
    if (sessionActive) { Serial.println("[TARE] Session active — stop it first."); return; }
    scales[b].tare(15);
    if (calState == CAL_WAIT_EMPTY) { calState = CAL_WAIT_KNOWN; calPrintState(); }
    return;
  }
  if (calState == CAL_WAIT_KNOWN) {
    float kg = cmd.toFloat();
    if (kg <= 0) { Serial.println("[?] Enter weight in grams"); return; }
    calKnownG[b] = kg; calState = CAL_WAIT_PLACE; calPrintState(); return;
  }
  if (calState == CAL_WAIT_PLACE) {
    if (cmd == "" || cmd == "go") {
      delay(800);
      scales[b].set_scale(1.0f);
      float rawSum = 0;
      for (int i = 0; i < 3; i++) { rawSum += scales[b].get_units(10); delay(200); }
      calNewFactor = (rawSum / 3.0f) / calKnownG[b];
      scales[b].set_scale(calNewFactor);
      delay(500);
      calVerifyG  = fabsf(readMedian(b, 15));
      calErrorPct = (fabsf(calVerifyG - calKnownG[b]) / calKnownG[b]) * 100.0f;
      calState = CAL_CONFIRM; calPrintState();
    }
    return;
  }
  if (calState == CAL_CONFIRM) {
    if (cmd == "y" || cmd == "Y") {
      calibFactor[b] = calNewFactor; scales[b].set_scale(calibFactor[b]);
      driftWarning[b] = false; saveCalibrationToNVS(b);
      delay(300); scales[b].tare(15);
      calState = CAL_WAIT_PART; calPrintState();
    } else if (cmd == "r" || cmd == "R") {
      calState = CAL_WAIT_EMPTY; scales[b].set_scale(1.0f); calPrintState();
    } else if (cmd == "n" || cmd == "N") {
      scales[b].set_scale(calibFactor[b]); calState = CAL_IDLE; calTargetBin = -1;
    }
    return;
  }
  if (calState == CAL_WAIT_PART) {
    if (cmd == "skip") { calState = CAL_IDLE; calTargetBin = -1; return; }
    if (cmd == "p" || cmd == "P") {
      delay(1500);
      float w = fabsf(readMedian(b, 15));
      if (!isnan(w) && w > 0.01f) {
        storedPartWeight[b] = w; saveCalibrationToNVS(b);
        Serial.printf("[PART] Bin %d unit weight = %.4f g\n", b + 1, w);
        calState = CAL_IDLE; calTargetBin = -1;
      } else {
        Serial.println("[PART] Nothing detected — try 'p' again.");
      }
    }
    return;
  }
}

// ======================================================
// SETUP
// ======================================================

void setup() {
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && millis() - t < 3000) delay(10);
  delay(500);
  randomSeed(esp_random());

  Serial.println();
  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║   ESP32-S3 Pick-To-Light  v6.1       ║");
  Serial.println("║   Fixed pins + wrong-bin/qty LEDs    ║");
  Serial.println("╚══════════════════════════════════════╝");
  Serial.println();

  // Explicitly init every token slot — required now that
  // ActiveSessionToken has no default member initializer.
  for (int i = 0; i < MAX_ACTIVE_TOKENS; i++) {
    activeTokens[i].inUse = false;
    activeTokens[i].expiresAtMs = 0;
  }

  // Start the access point before initializing external hardware. This makes
  // Wi-Fi failures visible immediately and prevents a slow/miswired HX711 from
  // delaying AP startup.
  WiFi.mode(WIFI_OFF);
  delay(100);
  bool modeOK   = WiFi.mode(WIFI_AP);
  bool configOK = WiFi.softAPConfig(AP_IP, AP_GW, AP_SUB);
  bool apOK     = WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.println();
  Serial.printf("[WiFi] mode=%s config=%s softAP=%s\n",
                modeOK ? "OK" : "FAIL",
                configOK ? "OK" : "FAIL",
                apOK ? "OK" : "FAIL");
  if (apOK) {
    dnsServer.start(53, "*", WiFi.softAPIP());
    Serial.print("[WiFi] SSID: ");
    Serial.println(AP_SSID);
    Serial.print("[WiFi] IP:   ");
    Serial.println(WiFi.softAPIP());
    Serial.print("[WiFi] MAC:  ");
    Serial.println(WiFi.softAPmacAddress());
  } else {
    Serial.println("[WiFi] ERROR: access point could not be started");
  }

  FastLED.addLeds<WS2812B, LED_PIN_A, GRB>(ledsA, LEDS_STRIP_A);
  FastLED.addLeds<WS2812B, LED_PIN_B, GRB>(ledsB, LEDS_STRIP_B);
  FastLED.setBrightness(LED_BRIGHTNESS);
  clearLEDs();

  initLoadCells();

  if (!LittleFS.begin(true)) {
    Serial.println("[FS] Mount Failed");
  } else {
    Serial.println("[FS] Mounted");
  }

  if (loadJSON())      Serial.printf("[DB] Loaded %d models\n", totalModels);
  else                 Serial.println("[DB] JSON load failed");

  if (loadEmployees()) Serial.println("[AUTH] Employee roster ready");
  loadLogFromLittleFS();

  // WebServer only exposes requested headers. Cookie collection lets the
  // dashboard authenticate subsequent API calls without storing PIN/token data.
  const char* collectedHeaders[] = { "Cookie" };
  server.collectHeaders(collectedHeaders, 1);

  // ── HTTP ROUTES ──────────────────────────────────────

  const char* apiRoutes[] = {
    "/api/models", "/api/sequence", "/api/session/start",
    "/api/session/confirm", "/api/session/reset",
    "/api/status", "/api/weight", "/api/tare", "/api/lowstock", "/api/logs",
    "/api/employees", "/api/history", "/api/history/export",
    "/api/login", "/api/logout",
    "/api/calibrate/status", "/api/calibrate/start",
    "/api/calibrate/command", "/api/calibrate/check"
  };
  for (auto r : apiRoutes) server.on(r, HTTP_OPTIONS, handleOptions);

  server.on("/", HTTP_GET, []() {
    File f = LittleFS.open("/index.html", "r");
    if (!f) { server.send(404, "text/plain", "index.html missing"); return; }
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.streamFile(f, "text/html");
    f.close();
  });

  // ── AUTH ──────────────────────────────────────────────

  server.on("/api/login", HTTP_POST, []() {
    if (!server.hasArg("plain")) { sendJson(400, "{\"error\":\"No body\"}"); return; }
    DynamicJsonDocument req(256);
    if (deserializeJson(req, server.arg("plain"))) { sendJson(400, "{\"error\":\"Bad JSON\"}"); return; }

    String employeeId = req["employeeId"] | "";
    String pin         = req["pin"] | "";
    if (pin.length() == 0) pin = req["password"] | "";

    int idx = findEmployeeByCreds(employeeId, pin);
    if (idx < 0) {
      sendJson(401, "{\"error\":\"Invalid employee ID or PIN\"}");
      Serial.printf("[AUTH] Failed login attempt: %s\n", employeeId.c_str());
      return;
    }

    int slot = findFreeTokenSlot();
    if (slot < 0) {
      sendJson(503, "{\"error\":\"Too many active sessions on this device\"}");
      return;
    }

    String token = generateToken();
    // FIX: explicit field assignment instead of brace-init, since the
    // struct no longer supports aggregate initialization the same way
    // with the default member initializer removed. This form always
    // works regardless of aggregate rules.
    activeTokens[slot].token        = token;
    activeTokens[slot].employeeId   = employees[idx].id;
    activeTokens[slot].employeeName = employees[idx].name;
    activeTokens[slot].inUse        = true;
    activeTokens[slot].expiresAtMs   = millis() + TOKEN_TTL_MS;

    DynamicJsonDocument res(768);
    res["id"]    = employees[idx].id;
    res["name"]  = employees[idx].name;
    res["token"] = token;
    JsonObject employee = res.createNestedObject("employee");
    employee["id"]          = employees[idx].id;
    employee["name"]        = employees[idx].name;
    employee["department"]  = employees[idx].department;
    employee["designation"] = employees[idx].designation;
    employee["shift"]       = employees[idx].shift;
    employee["status"]      = employees[idx].status;
    server.sendHeader("Set-Cookie",
                      "PTLSESSION=" + token +
                      "; Path=/; HttpOnly; SameSite=Strict; Max-Age=28800");
    String out; serializeJson(res, out); sendJson(200, out);
    Serial.printf("[AUTH] %s (%s) signed in\n", employees[idx].name.c_str(), employees[idx].id.c_str());
  });

  server.on("/api/logout", HTTP_POST, []() {
    String token = tokenFromCookie();
    if (server.hasArg("plain")) {
      DynamicJsonDocument req(128);
      if (!deserializeJson(req, server.arg("plain"))) {
        String bodyToken = req["token"] | "";
        if (bodyToken.length() > 0) token = bodyToken;
      }
    }
    int slot = findTokenSlot(token);
    if (slot >= 0) {
      Serial.printf("[AUTH] %s signed out\n", activeTokens[slot].employeeName.c_str());
      activeTokens[slot].inUse = false;
    }
    server.sendHeader("Set-Cookie",
                      "PTLSESSION=; Path=/; HttpOnly; SameSite=Strict; Max-Age=0");
    sendJson(200, "{\"status\":\"signed_out\"}");
  });

  server.on("/api/employees", HTTP_GET, []() {
    int tokenSlot = findTokenSlot(tokenFromCookie());
    if (tokenSlot < 0) { sendJson(401, "{\"error\":\"Authentication required\"}"); return; }

    DynamicJsonDocument doc(6144);
    JsonArray arr = doc.createNestedArray("employees");
    for (int i = 0; i < totalEmployees; i++) {
      JsonObject employee = arr.createNestedObject();
      employee["id"]          = employees[i].id;
      employee["name"]        = employees[i].name;
      employee["department"]  = employees[i].department;
      employee["designation"] = employees[i].designation;
      employee["shift"]       = employees[i].shift;
      employee["status"]      = employees[i].status;
      employee["lastUpdated"] = employees[i].lastUpdated;
    }
    String out; serializeJson(doc, out); sendJson(200, out);
  });

  // ── MODELS / SEQUENCE ────────────────────────────────

  server.on("/api/models", HTTP_GET, []() {
    DynamicJsonDocument doc(2048);
    JsonArray arr = doc.createNestedArray("models");
    for (int i = 0; i < totalModels; i++) {
      JsonObject m = arr.createNestedObject();
      m["id"] = models[i].id; m["name"] = models[i].name;
      m["image"] = models[i].image; m["img"] = models[i].image;
      m["totalSteps"] = models[i].totalSteps;
    }
    String out; serializeJson(doc, out); sendJson(200, out);
  });

  server.on("/api/sequence", HTTP_GET, []() {
    int idx = findModelIndexById(server.arg("modelId").toInt());
    if (idx < 0) { sendJson(400, "{\"error\":\"Invalid modelId\"}"); return; }
    Model& m = models[idx];
    DynamicJsonDocument doc(8192);
    doc["modelId"] = m.id; doc["modelName"] = m.name;
    doc["image"] = m.image; doc["totalSteps"] = m.totalSteps;
    JsonArray steps = doc.createNestedArray("steps");
    for (int i = 0; i < m.totalSteps; i++) {
      JsonObject s = steps.createNestedObject();
      s["step"] = m.steps[i].step; s["part"] = m.steps[i].part;
      s["bin"]  = m.steps[i].bin;  s["qty"]  = m.steps[i].qty;
      s["unitWeight"] = m.steps[i].unitWeight;
    }
    String out; serializeJson(doc, out); sendJson(200, out);
  });

  // ── SESSION ───────────────────────────────────────────

  server.on("/api/session/start", HTTP_POST, []() {
      if (!server.hasArg("plain")) { sendJson(400, "{\"error\":\"No body\"}"); return; }
      DynamicJsonDocument req(256);
      if (deserializeJson(req, server.arg("plain"))) { sendJson(400, "{\"error\":\"Bad JSON\"}"); return; }

      int idx = findModelIndexById(req["modelId"] | -1);
      if (idx < 0) { sendJson(400, "{\"error\":\"Invalid modelId\"}"); return; }
      if (models[idx].totalSteps <= 0) { sendJson(400, "{\"error\":\"No steps\"}"); return; }

      String token = requestToken(req);
      int tokenSlot = findTokenSlot(token);
      if (tokenSlot < 0) {
        sendJson(401, "{\"error\":\"Please sign in again before starting an assembly\"}");
        return;
      }

      Step& first = models[idx].steps[0];
      int firstBin = first.bin - 1;
      if (firstBin < 0 || firstBin >= NUM_BINS) {
        sendJson(400, "{\"error\":\"Invalid bin configured for first step\"}");
        return;
      }

      // FIX: don't block session start just because the first step's
      // load cell isn't ready — fall back to button-confirm-only for
      // that bin, same as every other step already does in
      // /api/session/confirm and /api/weight. A missing/unwired scale
      // on bin 1 shouldn't stop the whole assembly from running.
      if (scaleReady[firstBin]) {
        captureBaseline(firstBin);
      } else {
        baselineWeight[firstBin] = 0.0f;
        currentWeight[firstBin]  = 0.0f;
        Serial.printf("[SESSION] Bin %d has no ready load cell — starting without weight verification\n", firstBin + 1);
      }

      activeModelIndex = idx; currentStepIndex = 0;
      sessionActive = true; baselinesCaptured = true;

    // Reset every bin's LED status, then mark only the first step's
    // bin as "pick now" (yellow); everything else goes dark.
    for (int b = 0; b < NUM_BINS; b++) { binLedStatus[b] = LED_OFF; alertHoldUntil[b] = 0; }
    binLedStatus[firstBin] = LED_PICK;
    renderAllBinLeds();

    sessionOperatorId   = activeTokens[tokenSlot].employeeId;
    sessionOperatorName = activeTokens[tokenSlot].employeeName;
    sessionStartedAtSec = millis() / 1000;

    LogEntry entry;
    entry.assemblyId     = makeAssemblyId();
    entry.operatorId     = sessionOperatorId;
    entry.operatorName   = sessionOperatorName;
    entry.modelName       = models[idx].name;
    entry.startedAtSec   = sessionStartedAtSec;
    entry.completedAtSec = -1;
    entry.durationSec    = -1;
    entry.status         = "in_progress";
    entry.startTimestamp = req["startTimestamp"] | "";
    entry.completionTimestamp = "";
    appendLogEntry(entry);
    saveLogToLittleFS();

    DynamicJsonDocument res(256);
    res["status"] = "started"; res["modelName"] = models[idx].name;
    res["step"] = first.step; res["part"] = first.part;
    res["bin"]  = first.bin;  res["qty"]  = first.qty;
    res["baselineGrams"] = baselineWeight[firstBin];
    res["assemblyId"]    = entry.assemblyId;
    res["operatorName"]  = sessionOperatorName;
    res["startTimestamp"] = entry.startTimestamp;
    String out; serializeJson(res, out); sendJson(200, out);
    Serial.printf("[SESSION] %s started by %s (%s)\n",
                  entry.assemblyId.c_str(), sessionOperatorName.c_str(), sessionOperatorId.c_str());
  });

  // FIX / NEW: confirm now expects the operator to specify WHICH bin
  // they picked from (binPicked), so a wrong-bin pick can be detected
  // and shown in red — previously the firmware only ever checked the
  // weight on the bin the current step expected, with no way to tell
  // if the operator actually picked from a different bin entirely.
  // If the dashboard doesn't send binPicked, it defaults to the
  // expected bin (so existing front-end code keeps working) — this
  // makes wrong-bin detection an additive feature, not a breaking change.
  server.on("/api/session/confirm", HTTP_POST, []() {
    if (!sessionActive || activeModelIndex < 0 || activeModelIndex >= totalModels) {
      sendJson(400, "{\"error\":\"No active session\"}"); return;
    }
    Model& m    = models[activeModelIndex];
    Step&  step = m.steps[currentStepIndex];
    int    expectedBin = step.bin - 1;

    DynamicJsonDocument req(128);
    int pickedBin = expectedBin;   // default: assume correct bin if not specified
    if (server.hasArg("plain") && !deserializeJson(req, server.arg("plain"))) {
      int reportedBin = req["binPicked"] | (step.bin);
      pickedBin = reportedBin - 1;
    }

    // ── WRONG BIN ─────────────────────────────────────
    if (pickedBin != expectedBin && pickedBin >= 0 && pickedBin < NUM_BINS) {
      binLedStatus[pickedBin] = LED_WRONG_BIN;
      alertHoldUntil[pickedBin] = millis() + ALERT_HOLD_MS;
      renderAllBinLeds();

      DynamicJsonDocument e(256);
      e["error"] = "Wrong bin"; e["status"] = "wrong_bin";
      e["expectedBin"] = step.bin; e["pickedBin"] = pickedBin + 1;
      String out; serializeJson(e, out); sendJson(400, out);
      Serial.printf("[WRONG BIN] Expected Bin %d, got Bin %d\n", step.bin, pickedBin + 1);
      return;
    }

    int bin = expectedBin;
    bool isSensorBin = (bin >= 0 && bin < NUM_BINS && scaleReady[bin]);

    if (isSensorBin) {
      currentWeight[bin] = readWeight(bin, 10);
      float removed = baselineWeight[bin] - currentWeight[bin];
      if (removed < 0) removed = 0;
      float unitWeight = getStepUnitWeight(step);
      if (unitWeight <= 0.0f) {
        DynamicJsonDocument e(256);
        e["error"] = "Unit weight not configured"; e["part"] = step.part;
        String out; serializeJson(e, out); sendJson(422, out); return;
      }
      int estQty = (int)lroundf(removed / unitWeight);

      // ── CORRECT BIN, WRONG QTY ──────────────────────
      if (estQty != step.qty) {
        binLedStatus[bin] = LED_WRONG_QTY;
        alertHoldUntil[bin] = millis() + ALERT_HOLD_MS;
        renderAllBinLeds();

        DynamicJsonDocument e(384);
        e["error"] = "Quantity mismatch"; e["status"] = "wrong_quantity";
        e["part"] = step.part;
        e["expectedQty"] = step.qty; e["estimatedQty"] = estQty;
        e["removedGrams"] = removed; e["unitWeight"] = unitWeight;
        e["bin"] = step.bin;
        String out; serializeJson(e, out); sendJson(400, out);
        Serial.printf("[WRONG QTY] Bin %d expected=%d got=%d\n", step.bin, step.qty, estQty);
        return;
      }
      baselineWeight[bin] = currentWeight[bin];
    }

    // ── CORRECT BIN, CORRECT QTY ───────────────────────
    binLedStatus[bin] = LED_CORRECT;
    alertHoldUntil[bin] = millis() + ALERT_HOLD_MS;
    renderAllBinLeds();

    DynamicJsonDocument res(256);
    if (currentStepIndex >= m.totalSteps - 1) {
      long completedAtSec = millis() / 1000;
      long durationSec    = completedAtSec - (long)sessionStartedAtSec;
      String completionTimestamp = req["completionTimestamp"] | "";
      updateCurrentLogEntry("completed", completedAtSec, durationSec, completionTimestamp);
      saveLogToLittleFS();

      res["status"] = "complete";
      res["durationSec"] = durationSec;
      Serial.printf("[SESSION] Complete — duration %lds\n", durationSec);

      sessionActive = false;
      activeModelIndex = -1; currentStepIndex = 0;

      // Hold the final "correct" green briefly, then go dark.
      delay(400);
      for (int b = 0; b < NUM_BINS; b++) binLedStatus[b] = LED_OFF;
      clearLEDs();
    } else {
      currentStepIndex++;
      Step& next = m.steps[currentStepIndex];
      int nextBin = next.bin - 1;
      if (nextBin >= 0 && nextBin < NUM_BINS && scaleReady[nextBin]) captureBaseline(nextBin);
      if (nextBin >= 0 && nextBin < NUM_BINS && nextBin != bin) {
        binLedStatus[nextBin] = LED_PICK;
        renderAllBinLeds();
      }

      res["status"] = "advanced"; res["nextStep"] = next.step;
      res["nextPart"] = next.part; res["nextBin"] = next.bin; res["nextQty"] = next.qty;
    }
    String out; serializeJson(res, out); sendJson(200, out);
  });

  server.on("/api/session/reset", HTTP_POST, []() {
    String completionTimestamp = "";
    if (server.hasArg("plain")) {
      DynamicJsonDocument req(192);
      if (!deserializeJson(req, server.arg("plain")))
        completionTimestamp = req["completionTimestamp"] | "";
    }
    if (sessionActive) {
      long completedAtSec = millis() / 1000;
      long durationSec    = completedAtSec - (long)sessionStartedAtSec;
      updateCurrentLogEntry("aborted", completedAtSec, durationSec, completionTimestamp);
      saveLogToLittleFS();
      Serial.println("[SESSION] Aborted by reset");
    }
    sessionActive = false; activeModelIndex = -1;
    currentStepIndex = 0; baselinesCaptured = false;
    for (int b = 0; b < NUM_BINS; b++) {
      baselineWeight[b] = 0.0f;
      binLedStatus[b]    = LED_OFF;
      alertHoldUntil[b]  = 0;
    }
    clearLEDs();
    sendJson(200, "{\"status\":\"reset\"}");
  });

  // ── STATUS / WEIGHT ──────────────────────────────────

  server.on("/api/status", HTTP_GET, []() {
    DynamicJsonDocument doc(768);
    doc["online"] = true; doc["totalModels"] = totalModels;
    doc["totalEmployees"] = totalEmployees;
    doc["sessionActive"] = sessionActive;

    bool anyDrift = false;
    JsonArray channels = doc.createNestedArray("channels");
    for (int b = 0; b < NUM_BINS; b++) {
      JsonObject c = channels.createNestedObject();
      c["bin"] = b + 1; c["ready"] = scaleReady[b];
      c["driftWarning"] = driftWarning[b];
      if (driftWarning[b]) anyDrift = true;
    }
    doc["driftWarning"] = anyDrift;

    LowStockResult lowStock[NUM_BINS];
    int lowCount = computeLowStock(lowStock, NUM_BINS);
    doc["lowStockCount"] = lowCount;

    if (sessionActive && activeModelIndex >= 0) {
      doc["modelId"]   = models[activeModelIndex].id;
      doc["modelName"] = models[activeModelIndex].name;
      doc["stepIndex"] = currentStepIndex;
      doc["operatorName"] = sessionOperatorName;
      doc["assemblyId"]   = (currentLogIndex >= 0) ? assemblyLog[currentLogIndex].assemblyId : "";
    }
    String out; serializeJson(doc, out); sendJson(200, out);
  });

  server.on("/api/weight", HTTP_GET, []() {
    if (!sessionActive || activeModelIndex < 0) {
      sendJson(400, "{\"error\":\"No active session\"}"); return;
    }
    Step& step = models[activeModelIndex].steps[currentStepIndex];
    int bin = step.bin - 1;

    if (bin < 0 || bin >= NUM_BINS || !scaleReady[bin]) {
      DynamicJsonDocument doc(256);
      doc["error"] = "No sensor on this bin"; doc["bin"] = step.bin;
      String out; serializeJson(doc, out); sendJson(200, out);
      return;
    }

    currentWeight[bin] = readWeight(bin, 6);
    float removed = baselineWeight[bin] - currentWeight[bin];
    if (removed < 0) removed = 0;
    float unitWeight = getStepUnitWeight(step);
    int estQty = (unitWeight > 0) ? (int)lroundf(removed / unitWeight) : 0;

    DynamicJsonDocument doc(256);
    doc["bin"] = step.bin;
    doc["currentGrams"]  = currentWeight[bin];
    doc["baselineGrams"] = baselineWeight[bin];
    doc["removedGrams"]  = removed;
    doc["unitWeight"]    = unitWeight;
    doc["estimatedQty"]  = estQty;
    doc["expectedQty"]   = step.qty;
    String out; serializeJson(doc, out); sendJson(200, out);
  });

  // ── LOW STOCK ─────────────────────────────────────────

  server.on("/api/tare", HTTP_POST, []() {
    if (!server.hasArg("plain")) { sendJson(400, "{\"error\":\"No body\"}"); return; }

    DynamicJsonDocument req(192);
    if (deserializeJson(req, server.arg("plain"))) {
      sendJson(400, "{\"error\":\"Bad JSON\"}"); return;
    }

    bool tareAll = req["all"] | false;
    int requestedBin = req["bin"] | 0;
    int startBin = tareAll ? 0 : requestedBin - 1;
    int endBin   = tareAll ? NUM_BINS - 1 : requestedBin - 1;

    if (!tareAll && (requestedBin < 1 || requestedBin > NUM_BINS)) {
      sendJson(400, "{\"error\":\"Invalid bin\"}"); return;
    }

    DynamicJsonDocument doc(1024);
    JsonArray bins = doc.createNestedArray("bins");
    int taredCount = 0;

    for (int b = startBin; b <= endBin; b++) {
      JsonObject item = bins.createNestedObject();
      item["bin"] = b + 1;
      item["ready"] = scaleReady[b];

      if (!scaleReady[b] || !scales[b].is_ready()) {
        item["tared"] = false;
        continue;
      }

      scales[b].tare(15);
      baselineWeight[b] = 0.0f;
      currentWeight[b]  = 0.0f;
      item["tared"] = true;
      taredCount++;
      Serial.printf("[HX711] Bin %d tared from dashboard\n", b + 1);
    }

    doc["status"] = (taredCount > 0) ? "tared" : "no_ready_scales";
    doc["count"] = taredCount;
    String out; serializeJson(doc, out); sendJson(200, out);
  });

  server.on("/api/lowstock", HTTP_GET, []() {
    LowStockResult results[NUM_BINS];
    int count = computeLowStock(results, NUM_BINS);

    DynamicJsonDocument doc(1024);
    JsonArray arr = doc.createNestedArray("bins");
    for (int i = 0; i < count; i++) {
      JsonObject o = arr.createNestedObject();
      o["bin"]              = results[i].bin;
      o["percentRemaining"] = results[i].percentRemaining;
      o["currentGrams"]     = results[i].currentGrams;
      o["capacityGrams"]    = results[i].capacityGrams;
      o["label"]            = findPartLabelForBin(results[i].bin);
    }
    doc["threshold"] = LOW_STOCK_THRESHOLD_PCT;
    String out; serializeJson(doc, out); sendJson(200, out);
  });

  // ── ASSEMBLY LOG / STATUS TAB ────────────────────────

  server.on("/api/logs", HTTP_GET, []() {
    addCORSHeaders();
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "");
    server.sendContent("{\"logs\":[");
    bool firstRecord = true;
    int start = (logCount < MAX_LOG_ENTRIES) ? 0 : logHead;
    for (int n = logCount - 1; n >= 0; n--) {
      int idx = (start + n) % MAX_LOG_ENTRIES;
      LogEntry& e = assemblyLog[idx];
      StaticJsonDocument<1024> row;
      JsonObject o = row.to<JsonObject>();
      o["assemblyId"]     = e.assemblyId;
      o["operatorId"]     = e.operatorId;
      o["operatorName"]   = e.operatorName;
      o["modelName"]      = e.modelName;
      o["startedAtSec"]   = e.startedAtSec;
      o["completedAtSec"] = e.completedAtSec;
      o["durationSec"]    = e.durationSec;
      o["status"]         = e.status;
      o["startTimestamp"] = e.startTimestamp;
      o["completionTimestamp"] = e.completionTimestamp;
      String chunk;
      serializeJson(row, chunk);
      if (!firstRecord) server.sendContent(",");
      firstRecord = false;
      server.sendContent(chunk);
    }
    server.sendContent("]}");
    server.sendContent("");
  });

  // Dashboard-compatible history contract. The GET response is derived from
  // the same ring buffer as /api/logs; POST upserts the browser's ISO timestamps.
  server.on("/api/history", HTTP_GET, []() {
    int tokenSlot = findTokenSlot(tokenFromCookie());
    if (tokenSlot < 0) { sendJson(401, "{\"error\":\"Authentication required\"}"); return; }

    addCORSHeaders();
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "");
    server.sendContent("{\"history\":[");
    bool firstRecord = true;
    int start = (logCount < MAX_LOG_ENTRIES) ? 0 : logHead;
    for (int n = logCount - 1; n >= 0; n--) {
      int idx = (start + n) % MAX_LOG_ENTRIES;
      LogEntry& entry = assemblyLog[idx];
      StaticJsonDocument<1024> row;
      row["employeeId"]          = entry.operatorId;
      row["employeeName"]        = entry.operatorName;
      row["assemblyId"]          = entry.assemblyId;
      row["modelName"]           = entry.modelName;
      row["startTimestamp"]      = entry.startTimestamp;
      row["completionTimestamp"] = entry.completionTimestamp;
      row["durationSeconds"]     = entry.durationSec;
      row["duration"]            = durationLabel(entry.durationSec);
      row["status"]              = historyStatus(entry.status);
      String chunk;
      serializeJson(row, chunk);
      if (!firstRecord) server.sendContent(",");
      firstRecord = false;
      server.sendContent(chunk);
    }
    server.sendContent("]}");
    server.sendContent("");
  });

  server.on("/api/history", HTTP_POST, []() {
    int tokenSlot = findTokenSlot(tokenFromCookie());
    if (tokenSlot < 0) { sendJson(401, "{\"error\":\"Authentication required\"}"); return; }
    if (!server.hasArg("plain")) { sendJson(400, "{\"error\":\"No body\"}"); return; }

    DynamicJsonDocument req(1536);
    if (deserializeJson(req, server.arg("plain"))) {
      sendJson(400, "{\"error\":\"Bad JSON\"}"); return;
    }

    String assemblyId = req["assemblyId"] | "";
    String modelName  = req["modelName"]  | "";
    String status     = req["status"]     | "Completed";
    if (assemblyId.length() == 0 || modelName.length() == 0) {
      sendJson(400, "{\"error\":\"assemblyId and modelName are required\"}"); return;
    }

    int idx = findLogIndexByAssemblyId(assemblyId);
    if (idx < 0) {
      if (sessionActive) {
        sendJson(409, "{\"error\":\"Cannot import a different record during an active session\"}");
        return;
      }
      LogEntry imported;
      imported.assemblyId = assemblyId;
      imported.operatorId = activeTokens[tokenSlot].employeeId;
      imported.operatorName = activeTokens[tokenSlot].employeeName;
      imported.modelName = modelName;
      imported.startedAtSec = 0;
      imported.completedAtSec = 0;
      imported.durationSec = req["durationSeconds"] | 0;
      imported.status = status.equalsIgnoreCase("Cancelled") ? "aborted" : "completed";
      imported.startTimestamp = req["startTimestamp"] | "";
      imported.completionTimestamp = req["completionTimestamp"] | "";
      appendLogEntry(imported);
      idx = currentLogIndex;
    } else {
      LogEntry& entry = assemblyLog[idx];
      entry.operatorId = activeTokens[tokenSlot].employeeId;
      entry.operatorName = activeTokens[tokenSlot].employeeName;
      entry.modelName = modelName;
      String startTimestamp = req["startTimestamp"] | "";
      String completionTimestamp = req["completionTimestamp"] | "";
      if (startTimestamp.length() > 0) entry.startTimestamp = startTimestamp;
      if (completionTimestamp.length() > 0)
        entry.completionTimestamp = completionTimestamp;
      entry.durationSec = req["durationSeconds"] | entry.durationSec;
      entry.status = status.equalsIgnoreCase("Cancelled") ? "aborted" : "completed";
    }
    saveLogToLittleFS();
    sendJson(200, "{\"ok\":true}");
  });

  server.on("/api/history/export", HTTP_GET, []() {
    int tokenSlot = findTokenSlot(tokenFromCookie());
    if (tokenSlot < 0) { sendJson(401, "{\"error\":\"Authentication required\"}"); return; }

    server.sendHeader("Content-Disposition", "attachment; filename=assembly_history.csv");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/csv", "");
    server.sendContent("employeeId,employeeName,assemblyId,modelName,startTimestamp,completionTimestamp,durationSeconds,duration,status\n");
    int start = (logCount < MAX_LOG_ENTRIES) ? 0 : logHead;
    for (int n = 0; n < logCount; n++) {
      int idx = (start + n) % MAX_LOG_ENTRIES;
      LogEntry& entry = assemblyLog[idx];
      String line = csvEscape(entry.operatorId) + "," +
                    csvEscape(entry.operatorName) + "," +
                    csvEscape(entry.assemblyId) + "," +
                    csvEscape(entry.modelName) + "," +
                    csvEscape(entry.startTimestamp) + "," +
                    csvEscape(entry.completionTimestamp) + "," +
                    String(entry.durationSec) + "," +
                    csvEscape(durationLabel(entry.durationSec)) + "," +
                    historyStatus(entry.status) + "\n";
      server.sendContent(line);
    }
    server.sendContent("");
  });

  // ── CALIBRATION API ───────────────────────────────────

  server.on("/api/calibrate/status", HTTP_GET, []() { sendJson(200, calStatusJson()); });

  server.on("/api/calibrate/start", HTTP_POST, []() {
    if (sessionActive) { sendJson(409, "{\"error\":\"Cannot calibrate during active session\"}"); return; }
    if (!server.hasArg("plain")) { sendJson(400, "{\"error\":\"No body\"}"); return; }
    DynamicJsonDocument req(128);
    if (deserializeJson(req, server.arg("plain"))) { sendJson(400, "{\"error\":\"Bad JSON\"}"); return; }
    int bin = req["bin"] | -1;
    if (bin < 1 || bin > NUM_BINS) { sendJson(400, "{\"error\":\"Invalid bin\"}"); return; }
    startCalibration(bin - 1);
    sendJson(200, calStatusJson());
  });

  server.on("/api/calibrate/command", HTTP_POST, []() {
    if (!server.hasArg("plain")) { sendJson(400, "{\"error\":\"No body\"}"); return; }
    DynamicJsonDocument req(128);
    if (deserializeJson(req, server.arg("plain"))) { sendJson(400, "{\"error\":\"Bad JSON\"}"); return; }
    String cmd = req["cmd"] | ""; cmd.trim();
    handleCalCommand(cmd);
    sendJson(200, calStatusJson());
  });

  server.on("/api/calibrate/check", HTTP_POST, []() {
    if (!server.hasArg("plain")) { sendJson(400, "{\"error\":\"No body\"}"); return; }
    DynamicJsonDocument req(128);
    if (deserializeJson(req, server.arg("plain"))) { sendJson(400, "{\"error\":\"Bad JSON\"}"); return; }
    int bin = req["bin"] | -1;
    if (bin < 1 || bin > NUM_BINS) { sendJson(400, "{\"error\":\"Invalid bin\"}"); return; }
    int b = bin - 1;
    if (calKnownG[b] <= 0) { sendJson(422, "{\"error\":\"No reference weight stored for this bin\"}"); return; }
    delay(1500);
    float measured = fabsf(readMedian(b, 15));
    float errPct   = (fabsf(measured - calKnownG[b]) / calKnownG[b]) * 100.0f;
    driftWarning[b] = (errPct > DRIFT_RECAL_PCT);
    DynamicJsonDocument doc(256);
    doc["bin"] = bin; doc["measured"] = measured; doc["reference"] = calKnownG[b];
    doc["errorPct"] = errPct; doc["driftWarning"] = driftWarning[b];
    String out; serializeJson(doc, out); sendJson(200, out);
  });

  // ── STATIC FILE FALLBACK ──────────────────────────────

  server.onNotFound([]() {
    String path = server.uri();
    int q = path.indexOf('?');
    if (q != -1) path = path.substring(0, q);
    if (!path.startsWith("/")) path = "/" + path;

    if (LittleFS.exists(path)) {
      File f = LittleFS.open(path, "r");
      server.streamFile(f, getMimeType(path));
      f.close();
      return;
    }
    bool looksLikeAsset =
      path.endsWith(".jpg") || path.endsWith(".jpeg") || path.endsWith(".png") ||
      path.endsWith(".gif") || path.endsWith(".svg")  || path.endsWith(".css") ||
      path.endsWith(".js")  || path.endsWith(".ico")  || path.endsWith(".json");

    if (!looksLikeAsset) {
      server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
      server.send(302, "text/plain", "");
      return;
    }
    server.send(404, "text/plain", "File not found");
  });

  server.begin();
  Serial.println("[HTTP] Server started");
  Serial.println();
  Serial.println("Connect to WiFi:  PickToLight-Edge");
  Serial.println("Open browser:     http://192.168.4.1");
}

// ======================================================
// LOOP
// ======================================================

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    handleCalCommand(input);
  }

  // Once an alert's hold time expires, revert that bin back to its
  // "pick now" yellow if it's still the active step's bin, otherwise
  // off — so red/orange flashes don't stay lit forever.
  static unsigned long lastLedCheck = 0;
  if (millis() - lastLedCheck > 250) {
    lastLedCheck = millis();
    bool changed = false;
    for (int b = 0; b < NUM_BINS; b++) {
      if (alertHoldUntil[b] != 0 && millis() > alertHoldUntil[b]) {
        alertHoldUntil[b] = 0;
        bool isActiveStepBin = sessionActive && activeModelIndex >= 0 &&
                                (models[activeModelIndex].steps[currentStepIndex].bin - 1 == b);
        binLedStatus[b] = isActiveStepBin ? LED_PICK : LED_OFF;
        changed = true;
      }
    }
    if (changed) renderAllBinLeds();
  }

  static unsigned long lastCalPrint = 0;
  if (calState != CAL_IDLE && millis() - lastCalPrint > 700) {
    lastCalPrint = millis();
    int b = calTargetBin;
    if (calState == CAL_WAIT_EMPTY && b >= 0 && scales[b].is_ready()) {
      long raw = scales[b].read_median(7);
      Serial.printf("  Bin %d RAW: %ld\n", b + 1, raw);
    }
    if (calState == CAL_WAIT_PLACE && b >= 0) {
      Serial.printf("  Bin %d reading: %.2f g\n", b + 1, fabsf(readMedian(b, 5)));
    }
    if (calState == CAL_WAIT_PART && b >= 0) {
      Serial.printf("  Bin %d on scale: %.2f g\n", b + 1, readWeight(b, 5));
    }
  }

  delay(1);
}

// ======================================================
// JSON LOADER
// ======================================================

bool loadJSON() {
  if (!LittleFS.exists("/sequences.json")) {
    Serial.println("[FS] sequences.json missing"); return false;
  }
  File f = LittleFS.open("/sequences.json", "r");
  if (!f) { Serial.println("[FS] Cannot open sequences.json"); return false; }

  DynamicJsonDocument doc(16384);
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { Serial.printf("[FS] JSON error: %s\n", err.c_str()); return false; }

  JsonArray caps = doc["binCapacities"].as<JsonArray>();
  if (!caps.isNull()) {
    int i = 0;
    for (JsonVariant v : caps) {
      if (i >= NUM_BINS) break;
      binCapacityGrams[i] = v.as<float>();
      i++;
    }
    Serial.printf("[DB] Loaded bin capacities for %d bins\n", i);
  }

  JsonArray arr = doc["models"].as<JsonArray>();
  if (arr.isNull()) { Serial.println("[JSON] models array missing"); return false; }

  totalModels = 0;
  for (JsonObject m : arr) {
    if (totalModels >= 10) break;
    Model& model  = models[totalModels];
    model.id      = m["id"] | totalModels;
    model.name    = m["name"] | "Unknown";
    model.image   = m["image"] | m["img"] | "";

    JsonArray sa = m["steps"].as<JsonArray>();
    int si = 0;
    if (!sa.isNull()) {
      for (JsonObject s : sa) {
        if (si >= 20) break;
        model.steps[si].step       = s["step"] | (si + 1);
        model.steps[si].part       = s["part"] | "Part";
        model.steps[si].bin        = s["bin"]  | 1;
        model.steps[si].qty        = s["qty"]  | 1;
        model.steps[si].unitWeight = s["unitWeight"] | 0.0f;
        si++;
      }
    }
    model.totalSteps = si;
    totalModels++;
  }
  return totalModels > 0;
}

// ======================================================
// PART WEIGHT LOOKUP
// ======================================================

float getStepUnitWeight(const Step& step) {
  if (step.unitWeight > 0.0f) return step.unitWeight;
  return lookupPartUnitWeight(step.part);
}

float lookupPartUnitWeight(const String& part) {
  String p = part; p.toLowerCase();
  // Measured part weights (Part-wise measured weight + tolerance data).
  // Checked before the older generic entries below so "Bearing A"/"Bearing B"
  // don't fall through to the generic "ball bearing" line.
  if (p.indexOf("bearing a")     >= 0) return 131.3f;  // Big Bearing
  if (p.indexOf("bearing b")     >= 0) return 107.5f;  // Small Bearing
  if (p.indexOf("big bearing")   >= 0) return 131.3f;
  if (p.indexOf("small bearing") >= 0) return 107.5f;
  if (p.indexOf("coupler")       >= 0) return 179.0f;
  if (p.indexOf("coupling")      >= 0) return 179.0f;
  if (p == "bolt" || p.indexOf("bolt") == 0) return 59.0f;
  if (p == "nut"  || p.indexOf("nut")  == 0) return 21.3f;
  if (p == "washer" || p.indexOf("washer") == 0) return 13.0f;
  if (p == "gasket" || p.indexOf("gasket") == 0) return 14.0f;
  if (p.indexOf("bolt m6")        >= 0) return 2.0f;
  if (p.indexOf("washer 6mm")     >= 0) return 0.5f;
  if (p.indexOf("nut m6")         >= 0) return 1.0f;
  if (p.indexOf("spring pin")     >= 0) return 0.8f;
  if (p.indexOf("gasket a")       >= 0) return 2.5f;
  if (p.indexOf("gasket b")       >= 0) return 2.5f;
  if (p.indexOf("seal ring")      >= 0) return 1.2f;
  if (p.indexOf("bolt m8")        >= 0) return 4.0f;
  if (p.indexOf("flat washer")    >= 0) return 0.6f;
  if (p.indexOf("lock nut m8")    >= 0) return 1.8f;
  if (p.indexOf("hex bolt m10")   >= 0) return 7.0f;
  if (p.indexOf("retainer clip")  >= 0) return 0.5f;
  if (p.indexOf("motor housing")  >= 0) return 120.0f;
  if (p.indexOf("stator")         >= 0) return 250.0f;
  if (p.indexOf("winding")        >= 0) return 85.0f;
  if (p.indexOf("rotor")          >= 0) return 150.0f;
  if (p.indexOf("ball bearing")   >= 0) return 15.0f;
  if (p.indexOf("end bell")       >= 0) return 40.0f;
  if (p.indexOf("bolt m5")        >= 0) return 1.2f;
  if (p.indexOf("carbon brush")   >= 0) return 6.0f;
  if (p.indexOf("cooling fan")    >= 0) return 20.0f;
  if (p.indexOf("fan cover clip") >= 0) return 0.3f;
  if (p.indexOf("fan cover")      >= 0) return 25.0f;
  if (p.indexOf("cable grommet")  >= 0) return 1.0f;
  if (p.indexOf("bare pcb")       >= 0) return 20.0f;
  if (p.indexOf("resistors")      >= 0) return 0.01f;
  if (p.indexOf("capacitors")     >= 0) return 0.02f;
  if (p.indexOf("electrolytic cap") >= 0) return 0.7f;
  if (p.indexOf("diode")          >= 0) return 0.03f;
  if (p.indexOf("ic socket")      >= 0) return 1.5f;
  if (p.indexOf("microcontroller")>= 0) return 0.6f;
  if (p.indexOf("pin header")     >= 0) return 1.0f;
  if (p.indexOf("jst connector")  >= 0) return 0.4f;
  if (p.indexOf("crystal")        >= 0) return 0.4f;
  if (p.indexOf("led")            >= 0) return 0.2f;
  if (p.indexOf("fuse holder")    >= 0) return 1.2f;
  if (p.indexOf("terminal block") >= 0) return 1.5f;
  return 0.0f;
}

// ======================================================
// LED HELPERS
// ======================================================

// Lights a single bin a single color, leaving all other bins as they
// currently are (does NOT clear the whole strip first) — used by
// renderAllBinLeds() to paint each bin according to binLedStatus[].
void showBinColor(int binIdx, CRGB color) {
  if (binIdx < 0 || binIdx >= NUM_BINS) return;
  const BinMap& map = BIN_MAP[binIdx];

  if (map.stripA) {
    for (int i = map.start; i < map.start + LEDS_PER_BIN && i < LEDS_STRIP_A; i++)
      ledsA[i] = color;
  } else {
    for (int i = map.start; i < map.start + LEDS_PER_BIN && i < LEDS_STRIP_B; i++)
      ledsB[i] = color;
  }
}

CRGB colorForStatus(BinLedStatus status) {
  switch (status) {
    case LED_PICK:      return COLOR_PICK;
    case LED_CORRECT:   return COLOR_CORRECT;
    case LED_WRONG_QTY: return COLOR_WRONG_QTY;
    case LED_WRONG_BIN: return COLOR_WRONG_BIN;
    default:            return COLOR_OFF;
  }
}

// Repaints every bin according to binLedStatus[], then pushes the
// frame to both strips in one FastLED.show() call. This replaces the
// old lightBin()-only model so multiple bins can show different
// colors simultaneously (e.g. Bin 2 red for wrong-bin while Bin 1
// stays yellow for the actual expected pick).
void renderAllBinLeds() {
  for (int b = 0; b < NUM_BINS; b++) {
    showBinColor(b, colorForStatus(binLedStatus[b]));
  }
  FastLED.show();
}

// Kept for backward compatibility / simple single-bin highlighting
// (e.g. at session start before any status logic is needed).
void lightBin(int binIdx) {
  for (int b = 0; b < NUM_BINS; b++) binLedStatus[b] = LED_OFF;
  if (binIdx >= 0 && binIdx < NUM_BINS) binLedStatus[binIdx] = LED_PICK;
  renderAllBinLeds();
}

void clearLEDs() {
  fill_solid(ledsA, LEDS_STRIP_A, CRGB::Black);
  fill_solid(ledsB, LEDS_STRIP_B, CRGB::Black);
  FastLED.show();
}

// ======================================================
// HTTP HELPERS
// ======================================================

int findModelIndexById(int modelId) {
  for (int i = 0; i < totalModels; i++)
    if (models[i].id == modelId) return i;
  return -1;
}

void sendJson(int code, const String& body) {
  addCORSHeaders();
  server.send(code, "application/json", body);
}

void handleOptions() {
  addCORSHeaders();
  server.send(204, "text/plain", "");
}

void addCORSHeaders() {
  server.sendHeader("Access-Control-Allow-Origin",  "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}
