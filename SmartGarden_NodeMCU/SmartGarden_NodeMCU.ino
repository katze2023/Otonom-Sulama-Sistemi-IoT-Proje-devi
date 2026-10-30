/**
 * ============================================================
 *  IoT TABANLI OTOMATİK BAHÇE SULAMA VE VERİ TAKİP SİSTEMİ
 *  NodeMCU ESP8266 V3 - WiFi Gateway & Cloud Bridge
 * ============================================================
 *
 *  Mimari   : Master-Slave Hibrit IoT Mimarisi
 *  Görev    : UART Veri Alma | Paket Parse | Firebase Aktarım | OTA Hazır
 *  Platform : NodeMCU V3 (ESP8266 - 80MHz)
 *  IDE      : Arduino IDE 2.x
 *
 *  Yazar    : Fatih Kaya - G231210072
 *  Ders     : Nesnelerin İnterneti ve Uygulamaları
 *  Tarih    : 2025
 *
 * ============================================================
 *  KATMANLI MİMARİ:
 *  [1] HAL  - Hardware Abstraction Layer  (Pin & donanım)
 *  [2] CPL  - Communication Parse Layer  (UART okuma & parse)
 *  [3] CLL  - Cloud Layer                (Firebase yazma)
 *  [4] SHL  - System Health Layer        (Bağlantı izleme)
 *  [5] DBG  - Debug Layer                (Serial çıktı)
 *
 * ============================================================
 *  KULLANILAN KÜTÜPHANELEr:
 *  - ESP8266WiFi.h          : WiFi bağlantısı (Built-in)
 *  - FirebaseESP8266.h      : Firebase Realtime DB
 *    → Kütüphane Adı: "Firebase Arduino Client Library for ESP8266 and ESP32"
 *    → Yazar: Mobizt
 *    → Arduino IDE Library Manager'dan yükle
 *  - SoftwareSerial.h       : Arduino UART (Built-in)
 *  - ArduinoJson.h          : JSON oluşturma (v6)
 *    → Arduino IDE Library Manager'dan yükle
 *
 * ============================================================
 *  PIN YAPISI:
 *  D5 (GPIO14) → Arduino TX'ten gelen RX (voltaj bölücü ile!)
 *  D6 (GPIO12) → Arduino RX'e giden TX
 *  D4 (GPIO2)  → Dahili LED (WiFi durum göstergesi)
 *
 * ============================================================
 *  UART PAKET FORMATI (Arduino'dan gelen):
 *  <ID:ARD-GW-01,PKT:00042,T:24.5,H:58.0,SOIL:752,RAIN:830,
 *   WATER:460,LDR:795,MQ2:180,PUMP:1,FAULT:0x00,STATE:MONITORING>
 * ============================================================
 */

#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <FirebaseESP8266.h>
#include <ArduinoJson.h>

// ============================================================
//  BÖLÜM 1: KONFİGÜRASYON — KENDİ BİLGİLERİNİ GİR
// ============================================================

// --- WiFi Bilgileri ---
#define WIFI_SSID           "WIFI_ADINIZ"
#define WIFI_PASSWORD       "WIFI_SIFRENIZ"

// --- Firebase Bilgileri ---
// Firebase Console → Project Settings → General → Web API Key
#define FIREBASE_HOST       "PROJE_ADINIZ-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH       "FIREBASE_DATABASE_SECRET_KEY"

// --- Cihaz Kimliği ---
#define DEVICE_ID           "NODE-GW-01"
#define FIRMWARE_VERSION    "1.0.0"

// --- Pin Tanımları ---
#define PIN_SOFT_RX         14   // D5 → Arduino TX'ten gelir (voltaj bölücü!)
#define PIN_SOFT_TX         12   // D6 → Arduino RX'e gider
#define PIN_STATUS_LED      2    // D4 → Dahili LED (LOW = açık)

// --- Zamanlama ---
#define INTERVAL_FIREBASE   10000UL   // Firebase yazma periyodu (10 sn)
#define INTERVAL_WIFI_CHECK 30000UL   // WiFi kontrol periyodu
#define INTERVAL_DEBUG      5000UL    // Debug çıktı periyodu
#define WIFI_TIMEOUT        20000UL   // WiFi bağlantı timeout
#define UART_TIMEOUT        100UL     // UART okuma timeout

// --- Firebase Yolları ---
#define FB_PATH_SENSORS     "/garden/sensors"
#define FB_PATH_PUMP        "/garden/pump"
#define FB_PATH_SYSTEM      "/garden/system"
#define FB_PATH_ALERTS      "/garden/alerts"
#define FB_PATH_HISTORY     "/garden/history"

// ============================================================
//  BÖLÜM 2: VERİ TİPLERİ
// ============================================================

/**
 * @enum WiFiState
 * @brief WiFi bağlantı durumu
 */
enum WiFiState {
  WIFI_DISCONNECTED = 0,
  WIFI_CONNECTING,
  WIFI_CONNECTED,
  WIFI_FAILED
};

/**
 * @enum FirebaseState
 * @brief Firebase bağlantı durumu
 */
enum FirebaseState {
  FB_DISCONNECTED = 0,
  FB_CONNECTING,
  FB_CONNECTED,
  FB_ERROR
};

/**
 * @enum ParseResult
 * @brief UART paket parse sonucu
 */
enum ParseResult {
  PARSE_OK = 0,
  PARSE_NO_DATA,
  PARSE_INVALID_FORMAT,
  PARSE_CHECKSUM_FAIL,
  PARSE_INCOMPLETE
};

/**
 * @struct SensorPayload
 * @brief Arduino'dan gelen ayrıştırılmış sensör verisi
 */
struct SensorPayload {
  // Kimlik
  char     deviceID[16];
  uint32_t packetID;

  // Sensör değerleri
  float    temperature;
  float    humidity;
  int      soilMoisture;
  int      rainSensor;
  int      waterLevel;
  int      ldr;
  int      mq2;

  // Sistem durumu
  bool     pumpActive;
  uint8_t  faultMask;
  char     systemState[16];

  // Meta
  bool     valid;
  unsigned long receivedAt;
};

/**
 * @struct SystemStatus
 * @brief NodeMCU sistem durumu
 */
struct SystemStatus {
  WiFiState    wifiState;
  FirebaseState fbState;
  uint32_t     totalPacketsReceived;
  uint32_t     totalPacketsFailed;
  uint32_t     totalFirebaseWrites;
  uint32_t     totalFirebaseFails;
  unsigned long lastPacketTime;
  unsigned long lastFirebaseTime;
  unsigned long uptime;
  float        successRate;
};

/**
 * @struct AlertSystem
 * @brief Uyarı/alarm yönetimi
 */
struct AlertSystem {
  bool gasAlert;
  bool waterLowAlert;
  bool sensorFaultAlert;
  bool pumpOvertimeAlert;
  bool wifiLostAlert;
  unsigned long lastAlertTime;
};

// ============================================================
//  BÖLÜM 3: GLOBAL NESNELER
// ============================================================

// --- Donanım ---
SoftwareSerial arduinoSerial(PIN_SOFT_RX, PIN_SOFT_TX);

// --- Firebase ---
FirebaseData   fbData;
FirebaseAuth   fbAuth;
FirebaseConfig fbConfig;

// --- Global Yapılar ---
SensorPayload  payload;
SystemStatus   sysStatus;
AlertSystem    alertSys;

// --- UART Tampon ---
String uartBuffer = "";
bool   packetStarted = false;

// --- Zamanlama ---
unsigned long lastFirebaseTime  = 0;
unsigned long lastWifiCheckTime = 0;
unsigned long lastDebugTime     = 0;
unsigned long bootTime          = 0;

// ============================================================
//  BÖLÜM 4: FONKSİYON PROTOTİPLERİ
// ============================================================
// [HAL]
void     hal_initPins();
void     hal_setLED(bool state);
void     hal_blinkLED(int count, int delayMs);

// [NET] Network Layer
bool     net_connectWiFi();
void     net_checkWiFiConnection();
bool     net_initFirebase();
void     net_reconnectIfNeeded();

// [CPL] Communication Parse Layer
ParseResult cpl_readUART();
ParseResult cpl_parsePacket(const String& raw);
bool        cpl_extractField(const String& src, const char* key, String& out);
int         cpl_extractInt(const String& src, const char* key, int defaultVal = 0);
float       cpl_extractFloat(const String& src, const char* key, float defaultVal = 0.0);
bool        cpl_extractBool(const String& src, const char* key);
void        cpl_extractString(const String& src, const char* key, char* out, size_t maxLen);

// [CLL] Cloud Layer
bool     cll_writeToFirebase();
bool     cll_writeSensorData();
bool     cll_writePumpStatus();
bool     cll_writeSystemStatus();
bool     cll_writeAlerts();
bool     cll_writeHistory();
void     cll_evaluateAlerts();

// [SHL] System Health Layer
void     shl_updateSystemStatus();
void     shl_checkAlerts();

// [DBG] Debug Layer
void     dbg_printBanner();
void     dbg_printPayload();
void     dbg_printSystemStatus();
void     dbg_printParseResult(ParseResult r);

// [UTL] Utility
const char* utl_wifiStateToStr(WiFiState s);
const char* utl_fbStateToStr(FirebaseState s);
String      utl_buildTimestamp();
uint8_t     utl_faultBitCount(uint8_t mask);

// ============================================================
//  BÖLÜM 5: SETUP
// ============================================================

void setup() {
  Serial.begin(9600);
  arduinoSerial.begin(9600);

  hal_initPins();
  memset(&payload,   0, sizeof(payload));
  memset(&sysStatus, 0, sizeof(sysStatus));
  memset(&alertSys,  0, sizeof(alertSys));

  bootTime = millis();

  dbg_printBanner();

  // WiFi bağlantısı
  Serial.println(F("[NET] WiFi bağlantısı kuruluyor..."));
  if (net_connectWiFi()) {
    sysStatus.wifiState = WIFI_CONNECTED;
    hal_blinkLED(3, 200);
    Serial.println(F("[NET] WiFi bağlandı!"));

    // Firebase başlat
    if (net_initFirebase()) {
      sysStatus.fbState = FB_CONNECTED;
      Serial.println(F("[CLL] Firebase bağlantısı kuruldu!"));
    } else {
      sysStatus.fbState = FB_ERROR;
      Serial.println(F("[CLL] Firebase bağlantısı BAŞARISIZ!"));
    }
  } else {
    sysStatus.wifiState = WIFI_FAILED;
    Serial.println(F("[NET] WiFi bağlantısı BAŞARISIZ! Offline modda devam."));
  }

  Serial.println(F("[SYS] NodeMCU hazır. Veri bekleniyor..."));
}

// ============================================================
//  BÖLÜM 6: MAIN LOOP (Non-Blocking)
// ============================================================

void loop() {
  unsigned long now = millis();

  // [TASK-1] UART'tan veri oku ve parse et
  ParseResult result = cpl_readUART();
  if (result == PARSE_OK) {
    sysStatus.totalPacketsReceived++;
    sysStatus.lastPacketTime = now;
    cll_evaluateAlerts();
    dbg_printPayload();
  } else if (result == PARSE_INVALID_FORMAT) {
    sysStatus.totalPacketsFailed++;
  }

  // [TASK-2] Firebase'e yaz
  if (now - lastFirebaseTime >= INTERVAL_FIREBASE) {
    lastFirebaseTime = now;
    if (payload.valid && sysStatus.wifiState == WIFI_CONNECTED) {
      if (cll_writeToFirebase()) {
        sysStatus.totalFirebaseWrites++;
        sysStatus.lastFirebaseTime = now;
        hal_blinkLED(1, 100);
      } else {
        sysStatus.totalFirebaseFails++;
      }
    }
  }

  // [TASK-3] WiFi kontrol
  if (now - lastWifiCheckTime >= INTERVAL_WIFI_CHECK) {
    lastWifiCheckTime = now;
    net_reconnectIfNeeded();
  }

  // [TASK-4] Debug çıktı
  if (now - lastDebugTime >= INTERVAL_DEBUG) {
    lastDebugTime = now;
    shl_updateSystemStatus();
    dbg_printSystemStatus();
  }
}

// ============================================================
//  BÖLÜM 7: HAL - DONANIM SOYUTLAMA
// ============================================================

void hal_initPins() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, HIGH); // HIGH = kapalı (aktif LOW)
}

void hal_setLED(bool state) {
  digitalWrite(PIN_STATUS_LED, state ? LOW : HIGH);
}

void hal_blinkLED(int count, int delayMs) {
  for (int i = 0; i < count; i++) {
    hal_setLED(true);
    delay(delayMs);
    hal_setLED(false);
    delay(delayMs);
  }
}

// ============================================================
//  BÖLÜM 8: NET - AĞ KATMANI
// ============================================================

/**
 * @brief WiFi'ye bağlanır, timeout ile
 */
bool net_connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startTime = millis();
  Serial.print(F("[NET] Bağlanıyor"));
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startTime > WIFI_TIMEOUT) {
      Serial.println(F("\n[NET] Timeout!"));
      return false;
    }
    delay(500);
    Serial.print(F("."));
    hal_setLED(!digitalRead(PIN_STATUS_LED)); // Bağlanırken yanıp söner
  }
  Serial.println();
  Serial.print(F("[NET] IP: "));
  Serial.println(WiFi.localIP());
  hal_setLED(true); // Bağlandı — LED sabit yanar
  return true;
}

/**
 * @brief Firebase bağlantısını başlatır
 */
bool net_initFirebase() {
  fbConfig.host           = FIREBASE_HOST;
  fbConfig.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);

  // Test yazması
  if (Firebase.setString(fbData, "/garden/system/status", "ONLINE")) {
    return true;
  }
  Serial.print(F("[CLL] Firebase test hatası: "));
  Serial.println(fbData.errorReason());
  return false;
}

/**
 * @brief WiFi bağlantısını kontrol eder, gerekirse yeniden bağlanır
 */
void net_reconnectIfNeeded() {
  if (WiFi.status() != WL_CONNECTED) {
    sysStatus.wifiState  = WIFI_DISCONNECTED;
    alertSys.wifiLostAlert = true;
    Serial.println(F("[NET] WiFi kesildi! Yeniden bağlanılıyor..."));
    hal_setLED(false);
    if (net_connectWiFi()) {
      sysStatus.wifiState    = WIFI_CONNECTED;
      alertSys.wifiLostAlert = false;
    }
  } else {
    sysStatus.wifiState = WIFI_CONNECTED;
    alertSys.wifiLostAlert = false;
  }
}

// ============================================================
//  BÖLÜM 9: CPL - UART PARSE KATMANI
// ============================================================

/**
 * @brief SoftwareSerial'dan karakter karakter okur, tam paket gelince parse eder
 *
 *  Paket formatı: <...veri...>
 *  '<' ile başlar, '>' ile biter.
 */
ParseResult cpl_readUART() {
  while (arduinoSerial.available()) {
    char c = arduinoSerial.read();

    if (c == '<') {
      uartBuffer    = "";
      packetStarted = true;
      uartBuffer   += c;
    } else if (c == '>' && packetStarted) {
      uartBuffer   += c;
      packetStarted = false;
      // Tam paket alındı — parse et
      ParseResult result = cpl_parsePacket(uartBuffer);
      uartBuffer = "";
      return result;
    } else if (packetStarted) {
      uartBuffer += c;
      // Tampon taşmasını önle
      if (uartBuffer.length() > 250) {
        uartBuffer    = "";
        packetStarted = false;
        return PARSE_INVALID_FORMAT;
      }
    }
  }
  return PARSE_NO_DATA;
}

/**
 * @brief Ham UART stringini SensorPayload yapısına ayrıştırır
 *
 *  Örnek giriş:
 *  <ID:ARD-GW-01,PKT:00042,T:24.5,H:58.0,SOIL:752,RAIN:830,
 *   WATER:460,LDR:795,MQ2:180,PUMP:1,FAULT:0x00,STATE:MONITORING>
 */
ParseResult cpl_parsePacket(const String& raw) {
  // Format kontrolü
  if (!raw.startsWith("<") || !raw.endsWith(">")) {
    Serial.println(F("[CPL] HATA: Geçersiz paket formatı!"));
    return PARSE_INVALID_FORMAT;
  }

  // '<' ve '>' karakterlerini çıkar
  String content = raw.substring(1, raw.length() - 1);

  // Zorunlu alanları kontrol et
  if (content.indexOf("T:") < 0 || content.indexOf("SOIL:") < 0) {
    Serial.println(F("[CPL] HATA: Eksik alan!"));
    return PARSE_INCOMPLETE;
  }

  // Ayrıştırma
  cpl_extractString(content, "ID",    payload.deviceID,   sizeof(payload.deviceID));
  cpl_extractString(content, "STATE", payload.systemState, sizeof(payload.systemState));

  payload.packetID     = (uint32_t)cpl_extractInt(content, "PKT");
  payload.temperature  = cpl_extractFloat(content, "T");
  payload.humidity     = cpl_extractFloat(content, "H");
  payload.soilMoisture = cpl_extractInt(content, "SOIL");
  payload.rainSensor   = cpl_extractInt(content, "RAIN");
  payload.waterLevel   = cpl_extractInt(content, "WATER");
  payload.ldr          = cpl_extractInt(content, "LDR");
  payload.mq2          = cpl_extractInt(content, "MQ2");
  payload.pumpActive   = (cpl_extractInt(content, "PUMP") == 1);

  // FAULT alanını hex olarak parse et
  String faultStr = "";
  cpl_extractField(content, "FAULT", faultStr);
  payload.faultMask = (uint8_t)strtol(faultStr.c_str(), nullptr, 16);

  payload.valid      = true;
  payload.receivedAt = millis();

  Serial.print(F("[CPL] Paket ayrıştırıldı. PKT#"));
  Serial.println(payload.packetID);
  return PARSE_OK;
}

/**
 * @brief Virgülle ayrılmış key:value formatından değer çıkarır
 * @param src  Kaynak string
 * @param key  Aranacak anahtar
 * @param out  Çıktı string referansı
 * @return     Bulundu mu?
 */
bool cpl_extractField(const String& src, const char* key, String& out) {
  String searchKey = String(key) + ":";
  int startIdx = src.indexOf(searchKey);
  if (startIdx < 0) return false;

  startIdx += searchKey.length();
  int endIdx = src.indexOf(',', startIdx);
  if (endIdx < 0) endIdx = src.length();

  out = src.substring(startIdx, endIdx);
  out.trim();
  return true;
}

int cpl_extractInt(const String& src, const char* key, int defaultVal) {
  String val;
  if (!cpl_extractField(src, key, val)) return defaultVal;
  return val.toInt();
}

float cpl_extractFloat(const String& src, const char* key, float defaultVal) {
  String val;
  if (!cpl_extractField(src, key, val)) return defaultVal;
  return val.toFloat();
}

bool cpl_extractBool(const String& src, const char* key) {
  return (cpl_extractInt(src, key) == 1);
}

void cpl_extractString(const String& src, const char* key, char* out, size_t maxLen) {
  String val;
  if (!cpl_extractField(src, key, val)) {
    strncpy(out, "UNKNOWN", maxLen);
    return;
  }
  strncpy(out, val.c_str(), maxLen - 1);
  out[maxLen - 1] = '\0';
}

// ============================================================
//  BÖLÜM 10: CLL - CLOUD (FİREBASE) KATMANI
// ============================================================

/**
 * @brief Tüm Firebase yazma işlemlerini koordine eder
 */
bool cll_writeToFirebase() {
  if (WiFi.status() != WL_CONNECTED) return false;

  bool ok = true;
  ok &= cll_writeSensorData();
  ok &= cll_writePumpStatus();
  ok &= cll_writeSystemStatus();
  ok &= cll_writeAlerts();
  ok &= cll_writeHistory();

  return ok;
}

/**
 * @brief Sensör verilerini Firebase'e JSON olarak yazar
 *
 *  Yol: /garden/sensors/
 *  {
 *    "temperature": 24.5,
 *    "humidity": 58.0,
 *    "soilMoisture": 752,
 *    "rainSensor": 830,
 *    "waterLevel": 460,
 *    "ldr": 795,
 *    "mq2": 180,
 *    "timestamp": 1234567890
 *  }
 */
bool cll_writeSensorData() {
  FirebaseJson json;
  json.set("temperature",  payload.temperature);
  json.set("humidity",     payload.humidity);
  json.set("soilMoisture", payload.soilMoisture);
  json.set("rainSensor",   payload.rainSensor);
  json.set("waterLevel",   payload.waterLevel);
  json.set("ldr",          payload.ldr);
  json.set("mq2",          payload.mq2);
  json.set("packetID",     (int)payload.packetID);
  json.set("deviceID",     payload.deviceID);
  json.set("timestamp",    (int)(millis() / 1000));

  if (Firebase.updateNode(fbData, FB_PATH_SENSORS, json)) {
    Serial.println(F("[CLL] Sensör verisi Firebase'e yazıldı."));
    return true;
  }
  Serial.print(F("[CLL] HATA - Sensör yazma: "));
  Serial.println(fbData.errorReason());
  return false;
}

/**
 * @brief Pompa durumunu Firebase'e yazar
 *
 *  Yol: /garden/pump/
 */
bool cll_writePumpStatus() {
  FirebaseJson json;
  json.set("active",      payload.pumpActive);
  json.set("systemState", payload.systemState);
  json.set("faultMask",   payload.faultMask);
  json.set("faultCount",  utl_faultBitCount(payload.faultMask));
  json.set("timestamp",   (int)(millis() / 1000));

  if (Firebase.updateNode(fbData, FB_PATH_PUMP, json)) {
    Serial.println(F("[CLL] Pompa durumu Firebase'e yazıldı."));
    return true;
  }
  Serial.print(F("[CLL] HATA - Pompa yazma: "));
  Serial.println(fbData.errorReason());
  return false;
}

/**
 * @brief NodeMCU sistem durumunu Firebase'e yazar
 *
 *  Yol: /garden/system/
 */
bool cll_writeSystemStatus() {
  FirebaseJson json;
  json.set("status",              "ONLINE");
  json.set("nodeID",              DEVICE_ID);
  json.set("firmwareVersion",     FIRMWARE_VERSION);
  json.set("wifiSSID",            WIFI_SSID);
  json.set("wifiRSSI",            (int)WiFi.RSSI());
  json.set("uptimeSeconds",       (int)(millis() / 1000));
  json.set("totalPackets",        (int)sysStatus.totalPacketsReceived);
  json.set("failedPackets",       (int)sysStatus.totalPacketsFailed);
  json.set("firebaseWrites",      (int)sysStatus.totalFirebaseWrites);
  json.set("successRate",         sysStatus.successRate);
  json.set("freeHeap",            (int)ESP.getFreeHeap());
  json.set("timestamp",           (int)(millis() / 1000));

  if (Firebase.updateNode(fbData, FB_PATH_SYSTEM, json)) {
    Serial.println(F("[CLL] Sistem durumu Firebase'e yazıldı."));
    return true;
  }
  Serial.print(F("[CLL] HATA - Sistem yazma: "));
  Serial.println(fbData.errorReason());
  return false;
}

/**
 * @brief Uyarıları Firebase'e yazar
 *
 *  Yol: /garden/alerts/
 */
bool cll_writeAlerts() {
  FirebaseJson json;
  json.set("gasAlert",         alertSys.gasAlert);
  json.set("waterLowAlert",    alertSys.waterLowAlert);
  json.set("sensorFaultAlert", alertSys.sensorFaultAlert);
  json.set("wifiLostAlert",    alertSys.wifiLostAlert);
  json.set("anyAlert",         (
    alertSys.gasAlert         ||
    alertSys.waterLowAlert    ||
    alertSys.sensorFaultAlert ||
    alertSys.wifiLostAlert
  ));
  json.set("timestamp", (int)(millis() / 1000));

  if (Firebase.updateNode(fbData, FB_PATH_ALERTS, json)) {
    return true;
  }
  Serial.print(F("[CLL] HATA - Alert yazma: "));
  Serial.println(fbData.errorReason());
  return false;
}

/**
 * @brief Geçmiş veri kaydı (push ile zaman damgalı)
 *
 *  Yol: /garden/history/{auto-id}/
 *  Her Firebase yazmasında yeni bir kayıt oluşturur.
 *  Maksimum kayıt sayısını Firebase kurallarıyla sınırlayabilirsin.
 */
bool cll_writeHistory() {
  FirebaseJson json;
  json.set("temperature",  payload.temperature);
  json.set("humidity",     payload.humidity);
  json.set("soilMoisture", payload.soilMoisture);
  json.set("waterLevel",   payload.waterLevel);
  json.set("pumpActive",   payload.pumpActive);
  json.set("faultMask",    payload.faultMask);
  json.set("state",        payload.systemState);
  json.set("timestamp",    (int)(millis() / 1000));

  if (Firebase.pushJSON(fbData, FB_PATH_HISTORY, json)) {
    Serial.println(F("[CLL] Geçmiş kaydı oluşturuldu."));
    return true;
  }
  Serial.print(F("[CLL] HATA - History yazma: "));
  Serial.println(fbData.errorReason());
  return false;
}

/**
 * @brief Gelen payload'a göre uyarıları değerlendirir
 */
void cll_evaluateAlerts() {
  // Gaz uyarısı (bit 7 = 0x80)
  alertSys.gasAlert = (payload.faultMask & 0x80) != 0;

  // Su seviyesi düşük
  alertSys.waterLowAlert = (payload.waterLevel < 200);

  // Sensör arızası (herhangi bir sensör biti set ise)
  alertSys.sensorFaultAlert = (payload.faultMask & 0x3F) != 0;

  // Uyarı varsa logla
  if (alertSys.gasAlert) {
    Serial.println(F("[ALT] ⚠️  GAZ UYARISI! Ortamda gaz/duman tespit edildi!"));
  }
  if (alertSys.waterLowAlert) {
    Serial.println(F("[ALT] ⚠️  SU SEVİYESİ DÜŞÜK! Depo kontrol edilmeli!"));
  }
  if (alertSys.sensorFaultAlert) {
    Serial.print(F("[ALT] ⚠️  SENSÖR ARIZASI! Mask: 0x"));
    Serial.println(payload.faultMask, HEX);
  }
}

// ============================================================
//  BÖLÜM 11: SHL - SİSTEM SAĞLIK KATMANI
// ============================================================

void shl_updateSystemStatus() {
  sysStatus.uptime = millis() / 1000;

  uint32_t total = sysStatus.totalPacketsReceived + sysStatus.totalPacketsFailed;
  if (total > 0) {
    sysStatus.successRate = (float)sysStatus.totalPacketsReceived / total * 100.0;
  }

  // WiFi durumu güncelle
  sysStatus.wifiState = (WiFi.status() == WL_CONNECTED) ? WIFI_CONNECTED : WIFI_DISCONNECTED;
}

// ============================================================
//  BÖLÜM 12: DBG - DEBUG KATMANI
// ============================================================

void dbg_printBanner() {
  Serial.println(F("\n╔══════════════════════════════════════════╗"));
  Serial.println(F("║   IoT Akıllı Bahçe Sulama Sistemi       ║"));
  Serial.println(F("║   NodeMCU ESP8266 - WiFi Gateway         ║"));
  Serial.print  (F("║   Firmware v")); Serial.print(FIRMWARE_VERSION);
  Serial.println(F("                        ║"));
  Serial.println(F("╚══════════════════════════════════════════╝"));
}

void dbg_printPayload() {
  Serial.println(F("\n--- [CPL] Ayrıştırılan Veri ---"));
  Serial.print(F("  Cihaz ID  : ")); Serial.println(payload.deviceID);
  Serial.print(F("  Paket No  : ")); Serial.println(payload.packetID);
  Serial.print(F("  Sıcaklık  : ")); Serial.print(payload.temperature); Serial.println(F(" °C"));
  Serial.print(F("  Nem       : ")); Serial.print(payload.humidity);    Serial.println(F(" %"));
  Serial.print(F("  Toprak    : ")); Serial.println(payload.soilMoisture);
  Serial.print(F("  Yağmur    : ")); Serial.println(payload.rainSensor);
  Serial.print(F("  Su Sev.   : ")); Serial.println(payload.waterLevel);
  Serial.print(F("  LDR       : ")); Serial.println(payload.ldr);
  Serial.print(F("  MQ-2      : ")); Serial.println(payload.mq2);
  Serial.print(F("  Pompa     : ")); Serial.println(payload.pumpActive ? "AKTİF" : "PASİF");
  Serial.print(F("  Durum     : ")); Serial.println(payload.systemState);
  Serial.print(F("  Hata Mask : 0x")); Serial.println(payload.faultMask, HEX);
}

void dbg_printSystemStatus() {
  Serial.println(F("\n--- [SHL] Sistem Durumu ---"));
  Serial.print(F("  WiFi      : ")); Serial.println(utl_wifiStateToStr(sysStatus.wifiState));
  Serial.print(F("  Firebase  : ")); Serial.println(utl_fbStateToStr(sysStatus.fbState));
  Serial.print(F("  RSSI      : ")); Serial.print(WiFi.RSSI()); Serial.println(F(" dBm"));
  Serial.print(F("  Uptime    : ")); Serial.print(sysStatus.uptime); Serial.println(F(" sn"));
  Serial.print(F("  Paketler  : ")); Serial.print(sysStatus.totalPacketsReceived);
  Serial.print(F(" OK / "));        Serial.print(sysStatus.totalPacketsFailed);
  Serial.println(F(" FAIL"));
  Serial.print(F("  Başarı    : %")); Serial.println(sysStatus.successRate, 1);
  Serial.print(F("  FB Yazma  : ")); Serial.println(sysStatus.totalFirebaseWrites);
  Serial.print(F("  Free Heap : ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
}

// ============================================================
//  BÖLÜM 13: UTİLİTY FONKSİYONLARI
// ============================================================

const char* utl_wifiStateToStr(WiFiState s) {
  switch (s) {
    case WIFI_DISCONNECTED: return "DISCONNECTED";
    case WIFI_CONNECTING:   return "CONNECTING";
    case WIFI_CONNECTED:    return "CONNECTED";
    case WIFI_FAILED:       return "FAILED";
    default:                return "UNKNOWN";
  }
}

const char* utl_fbStateToStr(FirebaseState s) {
  switch (s) {
    case FB_DISCONNECTED: return "DISCONNECTED";
    case FB_CONNECTING:   return "CONNECTING";
    case FB_CONNECTED:    return "CONNECTED";
    case FB_ERROR:        return "ERROR";
    default:              return "UNKNOWN";
  }
}

/**
 * @brief Hata maskesindeki set bit sayısını döner
 */
uint8_t utl_faultBitCount(uint8_t mask) {
  uint8_t count = 0;
  while (mask) {
    count += (mask & 1);
    mask >>= 1;
  }
  return count;
}
