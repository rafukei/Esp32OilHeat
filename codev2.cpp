/**
 * ÖLJYLÄMMITYSJÄRJESTELMÄ - FreeRTOS + WiFi
 * 
 * Tämä ohjelma hallinnoi öljylämmitysjärjestelmää ESP32:lla.
 * Ominaisuudet:
 * - Lämpötilan mittaus DS18B20-antureilla
 * - Automaattinen säätö hystereesillä
 * - Web-käyttöliittymä ohjaamiseen
 * - WiFi-asetuksien hallinta (oma verkko tai yhteys olemassa olevaan verkkoon)
 * - Lämpötilakäyrän säätö
 * - Turvatoimet anturivirheille
 * - Watchdog-valvonta
 * - Asetusten tallennus flash-muistiin
 */

#include <WiFi.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <esp_task_wdt.h>

#define DEBUG_TEMP 1  // Debug-tulostus lämpötiloille

// ==================== KONFIGURAATIO ====================

// GPIO-pinnien määrittely
#define OUTSIDE_SENSOR_PIN 23    // Ulkolämpötilan anturi
#define WATER_SENSOR_PIN 22      // Menoveden lämpötilan anturi
#define RELAY_PIN 2              // Releen ohjaus

// Säätöparametrit
#define HYSTERESIS 4.0           // Hystereesi lämpötilasäädössä (°C)

// Anturivirheiden tunnistus
#define SENSOR_FAULT_TEMP -50.0         // Alle tämän lämpötilan = virhe
#define SENSOR_FAULT_TIMEOUT 30000      // 30 sekuntia ennen vakavaa virhettä
#define MAX_CONSECUTIVE_FAILURES 3      // Sallittu peräkkäisten virheiden määrä
#define TEMP_READ_INTERVAL_MS 5000      // Lämpötilan lukemisväli (ms)

// WiFi-yhteysparametrit
#define WIFI_CONNECT_TIMEOUT 10000      // 10 sekuntia yhteydenottoaikaa
#define WIFI_RECONNECT_INTERVAL 60000   // 60 sekuntia uudelleenyhdistämisväli

// Task-prioriteetit (korkeampi = tärkeämpi)
#define TASK_PRIORITY_HIGH 3            // Kriittiset taskit
#define TASK_PRIORITY_MEDIUM 2          // Normaalit taskit
#define TASK_PRIORITY_LOW 1             // Taustatehtävät

// Task-stackin koot
#define TASK_STACK_SMALL 2048           // Pienet taskit
#define TASK_STACK_MEDIUM 3072          // Keskisuuret taskit
#define TASK_STACK_LARGE 4096           // Suuret taskit (web-palvelin)

// Event Group -bitit järjestelmän tiloille
#define BIT_SENSOR_FAULT_OUTSIDE (1 << 0)  // Ulkoanturin vakava virhe
#define BIT_SENSOR_FAULT_WATER   (1 << 1)  // Vesianturin vakava virhe
#define BIT_HEATING_DISABLED     (1 << 2)  // Lämmitys estetty (liian lämmin)
#define BIT_TEMP_TASK_ALIVE      (1 << 3)  // Lämpötilatask elossa (watchdog)

// ==================== GLOBAALIT RESURSSIT ====================

// Mutexit (suojatut resurssit)
SemaphoreHandle_t xSensorStatusMutex;   // Anturien tilatiedot
SemaphoreHandle_t xTemperatureMutex;    // Lämpötilamittaukset
SemaphoreHandle_t xBurnerStateMutex;    // Polttimen tila
SemaphoreHandle_t xSettingsMutex;       // Asetukset ja käyrät
SemaphoreHandle_t xWifiConfigMutex;     // WiFi-asetukset

// Jono releen ohjauskomentoille
QueueHandle_t xRelayControlQueue;

// Event Group järjestelmän tiloille
EventGroupHandle_t xSystemEvents;

// Task-handlet (viittaukset säikeisiin)
TaskHandle_t xTemperatureTaskHandle = NULL;  // Lämpötilanlukija
TaskHandle_t xControlTaskHandle = NULL;      // Säätölogiikka
TaskHandle_t xWebServerTaskHandle = NULL;    // Web-palvelin
TaskHandle_t xRelayTaskHandle = NULL;        // Releen ohjaus
TaskHandle_t xWatchdogTaskHandle = NULL;     // Watchdog-valvoja
TaskHandle_t xFlashTaskHandle = NULL;        // Flash-tallennus

// Anturioliot DS18B20-sensoreille
OneWire oneWireOutside(OUTSIDE_SENSOR_PIN);
OneWire oneWireWater(WATER_SENSOR_PIN);
DallasTemperature sensorsOutside(&oneWireOutside);
DallasTemperature sensorsWater(&oneWireWater);

// Web-palvelin olio portissa 80
WebServer server(80);

// Pysyvän muistin (flash) käsittely
Preferences preferences;

// HTML-sivu (määritellään myöhemmin)
extern const char* htmlPage;

// ==================== DATATIENOT ====================

/**
 * WiFi-asetukset
 * Tallentaa sekä oman verkon että mahdollisen ulkoisen verkon tiedot
 */
struct WifiConfig {
  char ap_ssid[32] = "OilHeaterAP";      // Oman verkon nimi
  char ap_password[32] = "enmuista";     // Oman verkon salasana
  char sta_ssid[32] = "";                // Ulkoisen verkon nimi
  char sta_password[32] = "";            // Ulkoisen verkon salasana
  bool use_sta = false;                  // Käytetäänkö ulkoista verkkoa?
  bool connected = false;                // Onko yhteys aktiivinen?
  bool ap_mode = false;                  // Ollaanko AP-tilassa?
};

/**
 * Lämpötilasäädyskäyrän pisteet
 * 4 pistettä lineaariselle interpoloinnille
 */
struct CurvePoints {
  float points[4][2] = {
    {-20.0, 75.0},   // -20°C ulkona -> 75°C vettä
    {-10.0, 65.0},   // -10°C ulkona -> 65°C vettä
    {0.0, 55.0},     // 0°C ulkona -> 55°C vettä
    {10.0, 45.0}     // 10°C ulkona -> 45°C vettä
  };
};

/**
 * Anturin lämpötiladata
 * Sisältää nykyisen ja historiatiedot
 */
struct SensorData {
  float currentTemp = 0.0;               // Viimeisin mitattu lämpötila
  float lastValidTemp = 0.0;             // Viimeisin hyväksytty lämpötila
  unsigned long lastValidReadTime = 0;   // Viimeisestä hyvästä lukemisesta
  
  // 10 arvon liukuvat keskiarvot virhetilanteita varten
  float lastValidValues[10] = {0};
  int validValueIndex = 0;
  int validValueCount = 0;
  
  /**
   * Lisää uuden hyväksytyn arvon historiaan
   * @param value Hyväksytty lämpötila
   */
  void addValidValue(float value) {
    lastValidValues[validValueIndex] = value;
    validValueIndex = (validValueIndex + 1) % 10;
    if (validValueCount < 10) validValueCount++;
  }
  
  /**
   * Laskee historiatietojen keskiarvon
   * @return Lämpötilan keskiarvo
   */
  float getAverageValidValue() {
    if (validValueCount == 0) return 0.0;
    float sum = 0.0;
    for (int i = 0; i < validValueCount; i++) {
      sum += lastValidValues[i];
    }
    return sum / validValueCount;
  }
};

/**
 * Anturin tilatiedot
 * Seuraa virheiden määrää ja kestoa
 */
struct SensorStatus {
  bool fault = false;                    // Vakava vika
  bool temporaryFault = false;           // Tilapäinen häiriö
  unsigned long faultStartTime = 0;      // Vian alkamisaika
  int consecutiveFailures = 0;           // Peräkkäiset virheet
};

/**
 * Järjestelmän kokonaistila
 * Kaikki alidatat suojattu omilla mutexeillaan
 */
struct SystemState {
  // Lämpötiladata (suojaa: xTemperatureMutex)
  float targetWaterTemp = 0.0;           // Laskettu tavoitelämpötila
  bool heatingDisabled = false;          // Lämmitys estetty?
  
  // Polttimen tila (suojaa: xBurnerStateMutex)
  bool burnerState = false;              // Poltin päällä/pois
  bool burnerManualOverride = false;     // Manuaalitila käytössä?
  
  // Anturidata (suojaa: xSensorStatusMutex)
  SensorData outsideData;                // Ulkoanturin data
  SensorData waterData;                  // Vesianturin data
  SensorStatus outsideStatus;            // Ulkoanturin tila
  SensorStatus waterStatus;              // Vesianturin tila
  
  // Asetukset (suojaa: xSettingsMutex)
  CurvePoints curve;                     // Lämpötilakäyrä
  
  // WiFi-asetukset (suojaa: xWifiConfigMutex)
  WifiConfig wifiConfig;                 // WiFi-konfiguraatio
  
  // Flash-tallennuksen hallinta
  bool settingsDirty = false;            // Asetuksia muutettu?
  unsigned long lastSettingsChange = 0;  // Viimeinen muutos
};

// Globaali järjestelmän tila
SystemState systemState;

/**
 * Releen ohjausviesti
 * Välitetään jonon kautta releen ohjaustaskille
 */
struct RelayCommand {
  enum CommandType {
    SET_STATE,      // Normaali tilan asetus
    FORCE_OFF,      // Pakotettu sammutus
    EMERGENCY_OFF   // Hätäsammutus
  };
  
  CommandType type;  // Komennon tyyppi
  bool state;        // Haluttu tila (SET_STATE:lle)
};

// ==================== WIFI HALLINTA ====================

/**
 * Alustaa WiFi-yhteyden
 * - Lataa asetukset flashista
 * - Yrittää yhdistää ulkoiseen verkkoon jos STA käytössä
 * - Luo oman verkon (AP) jos STA ei käytössä tai yhteys epäonnistuu
 * @return true jos WiFi alustettu onnistuneesti
 */
bool initWiFi() {
  Serial.println("Alustetaan WiFi...");
  
  // Lataa WiFi-asetukset flash-muistista
  preferences.begin("oilheater", true);
  preferences.getString("ap_ssid", systemState.wifiConfig.ap_ssid, 32);
  preferences.getString("ap_password", systemState.wifiConfig.ap_password, 32);
  preferences.getString("sta_ssid", systemState.wifiConfig.sta_ssid, 32);
  preferences.getString("sta_password", systemState.wifiConfig.sta_password, 32);
  systemState.wifiConfig.use_sta = preferences.getBool("use_sta", false);
  preferences.end();
  
  Serial.printf("AP SSID: %s\n", systemState.wifiConfig.ap_ssid);
  Serial.printf("STA käytössä: %s\n", systemState.wifiConfig.use_sta ? "KYLLÄ" : "EI");
  
  bool connected = false;
  
  // Yritä yhdistää ulkoiseen verkkoon jos STA käytössä
  if (systemState.wifiConfig.use_sta && strlen(systemState.wifiConfig.sta_ssid) > 0) {
    Serial.printf("Yritetään yhdistää verkkoon: %s\n", systemState.wifiConfig.sta_ssid);
    
    // Sammuta mahdollinen vanha yhteys
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    
    // Yritä yhdistää
    WiFi.begin(systemState.wifiConfig.sta_ssid, systemState.wifiConfig.sta_password);
    
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_CONNECT_TIMEOUT) {
      delay(500);
      Serial.print(".");
    }
    
    // Tarkista onnistuiko yhdistäminen
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      systemState.wifiConfig.connected = true;
      systemState.wifiConfig.ap_mode = false;
      Serial.println("\nYhdistetty verkkoon!");
      Serial.printf("IP-osoite: %s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("\nYhteys verkkoon epäonnistui");
      WiFi.disconnect(true);
    }
  }
  
  // Jos STA ei käytössä tai yhteys epäonnistui, luo oma verkko (AP)
  if (!connected) {
    Serial.println("Luodaan oma verkko (Access Point)...");
    
    // Sammuta WiFi täysin
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    
    // Varmista että AP-asetukset ovat oikein
    if (strlen(systemState.wifiConfig.ap_ssid) == 0) {
      strcpy(systemState.wifiConfig.ap_ssid, "OilHeaterAP");
    }
    if (strlen(systemState.wifiConfig.ap_password) == 0) {
      strcpy(systemState.wifiConfig.ap_password, "enmuista");
    }
    
    // Aseta staattinen IP-osoite AP:lle
    IPAddress local_ip(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    
    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.softAP(systemState.wifiConfig.ap_ssid, systemState.wifiConfig.ap_password);
    
    // Päivitä tilatiedot
    systemState.wifiConfig.connected = true;
    systemState.wifiConfig.ap_mode = true;
    
    Serial.println("Oma verkko luotu onnistuneesti!");
    Serial.printf("SSID: %s\n", systemState.wifiConfig.ap_ssid);
    Serial.printf("IP-osoite: %s\n", WiFi.softAPIP().toString().c_str());
  }
  
  return true;
}

/**
 * Tallentaa WiFi-asetukset flash-muistiin
 */
void saveWifiConfig() {
  preferences.begin("oilheater", false);
  
  if (xSemaphoreTake(xWifiConfigMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    preferences.putString("ap_ssid", systemState.wifiConfig.ap_ssid);
    preferences.putString("ap_password", systemState.wifiConfig.ap_password);
    preferences.putString("sta_ssid", systemState.wifiConfig.sta_ssid);
    preferences.putString("sta_password", systemState.wifiConfig.sta_password);
    preferences.putBool("use_sta", systemState.wifiConfig.use_sta);
    xSemaphoreGive(xWifiConfigMutex);
  }
  
  preferences.end();
  Serial.println("WiFi-asetukset tallennettu flash-muistiin");
}

/**
 * Yhdistää verkkoon uudelleen
 * Käytetään kun WiFi-asetuksia muutetaan tai yhteys katkeaa
 */
void reconnectWiFi() {
  Serial.println("Yhdistetään verkkoon uudelleen...");
  
  // Sammuta nykyinen yhteys
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  
  // Alusta WiFi uudelleen
  initWiFi();
  
  // Jos web-palvelin on käynnissä, se toimii edelleen uudella IP-osoitteella
  Serial.println("Yhteys uudelleen alustettu");
}

// ==================== APUFUNKTIOT ====================

/**
 * Lukitsee sensorStatus ja temperature mutexit tiukassa järjestyksessä
 * @return true jos mutexit saatiin, false jos timeout
 */
bool lockSensorAndTemperature() {
  if (xSemaphoreTake(xSensorStatusMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (xSemaphoreTake(xTemperatureMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      return true;
    }
    xSemaphoreGive(xSensorStatusMutex);
  }
  return false;
}

/**
 * Vapauttaa sensorStatus ja temperature mutexit
 */
void unlockSensorAndTemperature() {
  xSemaphoreGive(xTemperatureMutex);
  xSemaphoreGive(xSensorStatusMutex);
}

/**
 * Tarkistaa onko lämpötilan arvo virheellinen
 * @param temperature Tarkistettava lämpötila
 * @param isWaterSensor True jos vesianturi
 * @return true jos virheellinen, false jos ok
 */
bool isTemperatureFaulty(float temperature, bool isWaterSensor) {
  // DS18B20 palauttaa tämän kun anturia ei löydy
  if (temperature == DEVICE_DISCONNECTED_C) {
    return true;
  }
  
  // Liian matala lämpötila
  if (temperature < SENSOR_FAULT_TEMP) {
    return true;
  }
  
  // Liian korkeat lämpötilat (anturityyppikohtaiset)
  if (isWaterSensor) {
    if (temperature > 120.0) return true; // Vesi ei saisi olla yli 120°C
  } else {
    if (temperature > 100.0) return true; // Ulkona ei saisi olla yli 100°C
  }
  
  return false;
}

/**
 * Päivittää anturin tilatiedot uuden mittauksen perusteella
 * @param status Anturin tilatietojen rakenne
 * @param data Anturin lämpötiladata
 * @param newValue Uusi mitattu lämpötila
 * @param isWaterSensor True jos vesianturi
 */
void updateSensorStatus(SensorStatus& status, SensorData& data, 
                       float newValue, bool isWaterSensor) {
  unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
  bool isFaulty = isTemperatureFaulty(newValue, isWaterSensor);
  
  if (isFaulty) {
    // VIRHEELLINEN LUKEMA
    status.consecutiveFailures++;
    
    // Ensimmäinen virhe -> aloita ajastin
    if (status.consecutiveFailures == 1) {
      status.faultStartTime = currentTime;
    }
    
    // Saavutettu maksimivirhemäärä -> tilapäinen häiriö
    if (status.consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
      status.temporaryFault = true;
    }
    
    // Virhe kestänyt yli 30s -> vakava vika
    if (currentTime - status.faultStartTime > SENSOR_FAULT_TIMEOUT && 
        status.consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
      status.fault = true;
      status.temporaryFault = false;
      
      // Aseta Event Groupiin vastaava bitti
      EventBits_t bits = BIT_SENSOR_FAULT_OUTSIDE;
      if (isWaterSensor) bits = BIT_SENSOR_FAULT_WATER;
      xEventGroupSetBits(xSystemEvents, bits);
    }
  } else {
    // HYVÄKSYTTY LUKEMA
    data.currentTemp = newValue;
    data.lastValidTemp = newValue;
    data.lastValidReadTime = currentTime;
    data.addValidValue(newValue);

    // Nollaa kaikki virheet
    status.consecutiveFailures = 0;
    status.temporaryFault = false;
    if (status.fault) {
      status.fault = false;
      EventBits_t bits = isWaterSensor ? BIT_SENSOR_FAULT_WATER : BIT_SENSOR_FAULT_OUTSIDE;
      xEventGroupClearBits(xSystemEvents, bits);
    }
  }
}

/**
 * Laskee tavoitelämpötilan menovedelle annetun ulkolämpötilan perusteella
 * Käyttää 4 pisteen lineaarista käyrää
 * @param outsideTemp Nykyinen ulkolämpötila
 * @param curve Lämpötilakäyrä
 * @return Tavoitelämpötila vedelle, 0.0 jos lämmitys estetty
 */
float calculateTargetTemperature(float outsideTemp, const CurvePoints& curve) {
  // Jos ulkolämpötila yli lämpimimmän pisteen (10°C)
  if (outsideTemp >= curve.points[3][0]) {
    return 0.0;  // Lämmitys pois päältä
  }
  
  // Jos ulkolämpötila alle kylmimmän pisteen (-20°C)
  if (outsideTemp <= curve.points[0][0]) {
    return curve.points[0][1];  // Maksimilämpötila
  }
  
  // Etsi oikea segmentti käyrältä
  for (int i = 0; i < 3; i++) {
    if (outsideTemp >= curve.points[i][0] && outsideTemp <= curve.points[i+1][0]) {
      // Lineaarinen interpolaatio kahden pisteen välillä
      float ratio = (outsideTemp - curve.points[i][0]) / 
                   (curve.points[i+1][0] - curve.points[i][0]);
      return curve.points[i][1] + ratio * (curve.points[i+1][1] - curve.points[i][1]);
    }
  }
  
  return 45.0; // Oletusarvo
}

// ==================== TASKIT ====================

/**
 * Lämpötilan lukemis-task
 * Lukee DS18B20-anturit ei-blokkaavasti tilakoneella
 * Päivittää anturien tilat ja laskee tavoitelämpötilan
 */
void temperatureTask(void *parameter) {
  Serial.println("Lämpötilatask aloitettu");
  
  // Ilmoita watchdog-taskille että olemme elossa
  xEventGroupSetBits(xSystemEvents, BIT_TEMP_TASK_ALIVE);
  
  // Alusta DS18B20-anturit
  sensorsOutside.begin();
  sensorsWater.begin();
  sensorsOutside.setResolution(12);     // 12-bittinen tarkkuus (0.0625°C)
  sensorsWater.setResolution(12);
  sensorsOutside.setWaitForConversion(false);  // Ei-blokkaava lukeminen
  sensorsWater.setWaitForConversion(false);
  
  // Tilakone lämpötilan lukemiselle
  enum ReadState { 
    IDLE,            // Odota lukemisväliä
    REQUEST_OUTSIDE, // Lähetä pyyntö ulkoanturille
    WAIT_OUTSIDE,    // Odota ulkoanturin mittauksen valmistumista
    READ_OUTSIDE,    // Lue ulkoanturin tulos
    REQUEST_WATER,   // Lähetä pyyntö vesianturille
    WAIT_WATER,      // Odota vesianturin mittauksen valmistumista
    READ_WATER       // Lue vesianturin tulos
  };
  
  ReadState readState = IDLE;
  unsigned long requestTime = 0;
  
  while (1) {
    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    switch (readState) {
      case IDLE:
        // Odota määritetty aika ennen seuraavaa lukemista
        vTaskDelay(pdMS_TO_TICKS(TEMP_READ_INTERVAL_MS));
        
        // Ilmoita watchdogille että olemme edelleen elossa
        xEventGroupSetBits(xSystemEvents, BIT_TEMP_TASK_ALIVE);
        
        // Tarkista löytyykö anturit
        if (sensorsOutside.getDeviceCount() == 0) {
          Serial.println("VAROITUS: Ulkoanturia ei löydy!");
          sensorsOutside.begin();
        }
        if (sensorsWater.getDeviceCount() == 0) {
          Serial.println("VAROITUS: Vesianturia ei löydy!");
          sensorsWater.begin();
        }
        
        // Aloita uusi mittaussykli
        sensorsOutside.requestTemperatures();
        readState = REQUEST_OUTSIDE;
        requestTime = currentTime;
        break;
        
      case REQUEST_OUTSIDE:
        // Odota että pyyntö on käsitelty
        if (currentTime - requestTime > 100) {
          readState = WAIT_OUTSIDE;
        }
        break;
        
      case WAIT_OUTSIDE:
        // Odota että DS18B20 mittaus valmistuu (max 750ms 12-bitille)
        if (currentTime - requestTime > 750) {
          float outsideTemp = sensorsOutside.getTempCByIndex(0);
          
          #if DEBUG_TEMP
          Serial.printf("[TEMP] OUTSIDE: %.2f °C\n", outsideTemp);
          #endif
          
          // Päivitä ulkoanturin tila suojatusti
          if (xSemaphoreTake(xSensorStatusMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            updateSensorStatus(systemState.outsideStatus, systemState.outsideData, 
                              outsideTemp, false);
            xSemaphoreGive(xSensorStatusMutex);
          }
          
          // Aloita vesianturin mittaus
          sensorsWater.requestTemperatures();
          readState = REQUEST_WATER;
          requestTime = currentTime;
        }
        break;
        
      case REQUEST_WATER:
        if (currentTime - requestTime > 100) {
          readState = WAIT_WATER;
        }
        break;
        
      case WAIT_WATER:
        if (currentTime - requestTime > 750) {
          float waterTemp = sensorsWater.getTempCByIndex(0);
          
          #if DEBUG_TEMP
          Serial.printf("[TEMP] Water: %.2f °C\n", waterTemp);
          #endif
          
          // Päivitä vesianturin tila suojatusti
          if (xSemaphoreTake(xSensorStatusMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            updateSensorStatus(systemState.waterStatus, systemState.waterData, 
                              waterTemp, true);
            xSemaphoreGive(xSensorStatusMutex);
          }
          
          // Laske tavoitelämpötila suojatusti
          if (lockSensorAndTemperature()) {
            // Valitse käytettävä ulkolämpötila (keskiarvo jos vika)
            float outsideTempToUse = systemState.outsideStatus.fault ? 
                                    systemState.outsideData.getAverageValidValue() : 
                                    systemState.outsideData.currentTemp;
            
            // Hae käyräpisteet ja laske tavoite
            if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
              float targetTemp = calculateTargetTemperature(outsideTempToUse, systemState.curve);
              
              // Päivitä järjestelmän tila
              systemState.targetWaterTemp = targetTemp;
              systemState.heatingDisabled = (outsideTempToUse >= systemState.curve.points[3][0]);
              
              // Aseta/tyhjää Event Groupin bitti lämmityksen tilalle
              if (systemState.heatingDisabled) {
                xEventGroupSetBits(xSystemEvents, BIT_HEATING_DISABLED);
              } else {
                xEventGroupClearBits(xSystemEvents, BIT_HEATING_DISABLED);
              }
              
              xSemaphoreGive(xSettingsMutex);
            }
            
            unlockSensorAndTemperature();
          }
          
          // Palataan odotustilaan
          readState = IDLE;
        }
        break;
    }
    
    // Anna muille säikeille ajoaikaa
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
 * Säätölogiikan task
 * Suorittaa polttimen automaattisen säätölogiikan
 * Tarkistaa lämpötilat ja päivittää polttimen tilan hystereesin mukaan
 */
void controlTask(void *parameter) {
  Serial.println("Säätötask aloitettu");
  
  unsigned long lastControlTime = 0;
  
  while (1) {
    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Suorita säätölogiikkaa sekunnin välein
    if (currentTime - lastControlTime >= 1000) {
      lastControlTime = currentTime;
      
      // Alustetaan säätöehdot
      bool canControl = true;
      float waterTempToUse = 0.0;
      float targetTemp = 0.0;
      bool heatingDisabled = false;
      
      // Tarkista Event Group:ista systeemin tilat
      EventBits_t events = xEventGroupGetBits(xSystemEvents);
      bool anyFault = (events & (BIT_SENSOR_FAULT_OUTSIDE | BIT_SENSOR_FAULT_WATER)) != 0;
      heatingDisabled = (events & BIT_HEATING_DISABLED) != 0;
      
      // Jos vikoja tai lämmitys estetty, ei automaattista säätöä
      if (anyFault || heatingDisabled) {
        canControl = false;
      }
      
      // Hae tarvittavat lämpötilat suojatusti
      if (lockSensorAndTemperature()) {
        targetTemp = systemState.targetWaterTemp;
        
        // Valitse käytettävä vesilämpötila virhetilanteiden perusteella
        if (systemState.waterStatus.fault) {
          waterTempToUse = systemState.waterData.getAverageValidValue();
        } else if (systemState.waterStatus.temporaryFault) {
          waterTempToUse = systemState.waterData.lastValidTemp;
        } else {
          waterTempToUse = systemState.waterData.currentTemp;
        }
        
        // Tarkista onko manuaalitila päällä
        bool manualMode = false;
        if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          manualMode = systemState.burnerManualOverride;
          xSemaphoreGive(xBurnerStateMutex);
        }
        
        // Manuaalitila estää automaattisen säätön
        if (manualMode) {
          canControl = false;
        }
        
        unlockSensorAndTemperature();
      }
      
      // Suorita automaattinen säätö jos ehdot täyttyvät JA targetTemp > 0
      if (canControl && targetTemp > 0) {
        bool currentBurnerState = false;
        
        // Hae nykyinen polttimen tila
        if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          currentBurnerState = systemState.burnerState;
          xSemaphoreGive(xBurnerStateMutex);
        }
        
        bool newBurnerState = currentBurnerState;
        
        // HYSTEREESILLÄ VARUSTETTU SÄÄTÖLOGIIKKA
        if (!currentBurnerState && waterTempToUse < (targetTemp - HYSTERESIS)) {
          // Poltin pois, vesi liian kylmä -> käynnistä
          newBurnerState = true;
        } 
        else if (currentBurnerState && waterTempToUse > (targetTemp + HYSTERESIS)) {
          // Poltin päällä, vesi liian kuuma -> sammuta
          newBurnerState = false;
        }
        
        // Jos tila muuttui, päivitä se ja lähetä releelle komento
        if (newBurnerState != currentBurnerState) {
          // Päivitä polttimen tila
          if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            systemState.burnerState = newBurnerState;
            xSemaphoreGive(xBurnerStateMutex);
          }
          
          // Lähetä komento releen ohjaustaskille
          RelayCommand cmd;
          cmd.type = RelayCommand::SET_STATE;
          cmd.state = newBurnerState;
          xQueueSend(xRelayControlQueue, &cmd, 0);
        }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/**
 * Releen ohjauksen task
 * Vastaanottaa relekomentoja jonosta ja ohjaa relettä turvallisesti
 * Tarkistaa turvaehdot ennen releen kytkemistä
 */
void relayTask(void *parameter) {
  Serial.println("Releetask aloitettu");
  
  // Alusta rele turvalliseen tilaan
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // LOW = rele auki = poltin pois = TURVALLINEN TILA
  
  RelayCommand cmd;
  
  while (1) {
    // Odota komentoa jonosta (blokkaava odotus)
    if (xQueueReceive(xRelayControlQueue, &cmd, portMAX_DELAY)) {
      
      // Tarkista Event Group:ista systeemin tilat
      EventBits_t events = xEventGroupGetBits(xSystemEvents);
      bool anyFault = (events & (BIT_SENSOR_FAULT_OUTSIDE | BIT_SENSOR_FAULT_WATER)) != 0;
      bool heatingDisabled = (events & BIT_HEATING_DISABLED) != 0;
      
      bool shouldTurnOn = false;
      
      // Käsittele komento tyypin mukaan
      switch (cmd.type) {
        case RelayCommand::SET_STATE:
          // Normaali tilan asetus - tarkista turvaehdot
          if (!anyFault && !heatingDisabled) {
            shouldTurnOn = cmd.state;
          } else {
            shouldTurnOn = false;
          }
          break;
          
        case RelayCommand::FORCE_OFF:
          // Pakotettu sammutus
          shouldTurnOn = false;
          break;
          
        case RelayCommand::EMERGENCY_OFF:
          // Hätäsammutus - ohita kaikki logiikat
          shouldTurnOn = false;
          break;
      }
      
      // Erikoistarkistus: manuaalitila voi ohittaa virhetilanteen
      if (!shouldTurnOn && cmd.type == RelayCommand::SET_STATE) {
        bool manualMode = false;
        if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          manualMode = systemState.burnerManualOverride;
          xSemaphoreGive(xBurnerStateMutex);
        }
        
        // Manuaalitilassa käyttäjä voi käynnistää vaikka olisi virhe
        if (manualMode && cmd.state) {
          shouldTurnOn = true;
        }
      }
      
      // Ohjaa relettä
      digitalWrite(RELAY_PIN, shouldTurnOn ? HIGH : LOW);
      
      // Päivitä polttimen tila muistiin
      if (cmd.type == RelayCommand::SET_STATE) {
        if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          systemState.burnerState = shouldTurnOn;
          xSemaphoreGive(xBurnerStateMutex);
        }
      }
    }
  }
}

/**
 * Watchdog-task
 * Valvoo muita taskeja ja suorittaa hätätoimenpiteitä
 * jos jokin task lakkaa vastaamasta
 */
void watchdogTask(void *parameter) {
  Serial.println("Watchdog-task aloitettu");
  
  // Alusta ESP32:n hardware watchdog
  esp_task_wdt_init(30, true);  // 30 sekunnin timeout, panic reset
  esp_task_wdt_add(NULL);       // Lisää tämä task watchdogin valvontaan
  
  unsigned long lastTempTaskAlive = xTaskGetTickCount() * portTICK_PERIOD_MS;
  
  while (1) {
    // Tarkista onko lämpötilatask elossa
    EventBits_t events = xEventGroupGetBits(xSystemEvents);
    if (events & BIT_TEMP_TASK_ALIVE) {
      // Task on elossa, päivitä aikaleima
      lastTempTaskAlive = xTaskGetTickCount() * portTICK_PERIOD_MS;
      xEventGroupClearBits(xSystemEvents, BIT_TEMP_TASK_ALIVE);
    }
    
    // Jos lämpötilatask ei ole ollut elossa 60 sekuntiin
    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (currentTime - lastTempTaskAlive > 60000) {
      Serial.println("WATCHDOG: Lämpötilatask ei vastaa - hätäsammutus!");
      
      // Lähetä hätäsammutuskomento releelle
      RelayCommand cmd;
      cmd.type = RelayCommand::EMERGENCY_OFF;
      xQueueSend(xRelayControlQueue, &cmd, 0);
      
      // Nollaa aikaleima
      lastTempTaskAlive = currentTime;
    }
    
    // Resetoi hardware watchdog
    esp_task_wdt_reset();
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
 * Flash-tallennuksen task
 * Tallentaa asetukset flash-muistiin viiveellä
 * Estää liian useat flash-kirjoitukset, mikä pidentää flashin elinikää
 */
void flashTask(void *parameter) {
  Serial.println("Flash-task aloitettu");
  
  while (1) {
    bool shouldSave = false;
    
    // Tarkista onko asetuksia tallennettava
    if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (systemState.settingsDirty) {
        unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Odota 5 sekuntia viimeisestä muutoksesta
        if (currentTime - systemState.lastSettingsChange > 5000) {
          shouldSave = true;
          systemState.settingsDirty = false;
        }
      }
      xSemaphoreGive(xSettingsMutex);
    }
    
    // Tallenna flash-muistiin
    if (shouldSave) {
      preferences.begin("oilheater", false);
      
      if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < 4; i++) {
          preferences.putFloat(("out" + String(i)).c_str(), systemState.curve.points[i][0]);
          preferences.putFloat(("water" + String(i)).c_str(), systemState.curve.points[i][1]);
        }
        xSemaphoreGive(xSettingsMutex);
      }
      
      preferences.end();
      Serial.println("Asetukset tallennettu flash-muistiin");
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ==================== HTML SIVU ====================

const char* htmlPage = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Öljylämmityksen ohjaus</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 20px;
      background-color: #f5f5f5;
    }
    .container {
      max-width: 900px;
      margin: auto;
      background: white;
      padding: 20px;
      border-radius: 10px;
      box-shadow: 0 0 10px rgba(0,0,0,0.1);
    }
    .sensor-box {
      background: #e3f2fd;
      padding: 15px;
      margin: 10px 0;
      border-radius: 5px;
    }
    .control-box {
      background: #f3e5f5;
      padding: 15px;
      margin: 10px 0;
      border-radius: 5px;
    }
    .config-box {
      background: #fff3e0;
      padding: 15px;
      margin: 10px 0;
      border-radius: 5px;
    }
    .curve-box {
      background: #e8f5e9;
      padding: 15px;
      margin: 10px 0;
      border-radius: 5px;
    }
    .fault {
      color: #f44336;
      font-weight: bold;
    }
    .warning {
      color: #ff9800;
      font-weight: bold;
    }
    .ok {
      color: #4CAF50;
      font-weight: bold;
    }
    button {
      background-color: #4CAF50;
      color: white;
      padding: 10px 15px;
      border: none;
      border-radius: 5px;
      cursor: pointer;
      margin: 5px;
    }
    button:hover {
      background-color: #45a049;
    }
    button.danger {
      background-color: #f44336;
    }
    button.danger:hover {
      background-color: #d32f2f;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      margin: 10px 0;
    }
    th, td {
      border: 1px solid #ddd;
      padding: 8px;
      text-align: center;
    }
    th {
      background-color: #4CAF50;
      color: white;
    }
    input[type=text], input[type=password], input[type=number] {
      width: 200px;
      padding: 8px;
      margin: 5px 0;
      border: 1px solid #ddd;
      border-radius: 4px;
    }
    .tab {
      overflow: hidden;
      border: 1px solid #ccc;
      background-color: #f1f1f1;
      border-radius: 5px 5px 0 0;
    }
    .tab button {
      background-color: inherit;
      float: left;
      border: none;
      outline: none;
      cursor: pointer;
      padding: 14px 16px;
      transition: 0.3s;
      color: black;
    }
    .tab button:hover {
      background-color: #ddd;
    }
    .tab button.active {
      background-color: #4CAF50;
      color: white;
    }
    .tabcontent {
      display: none;
      padding: 20px;
      border: 1px solid #ccc;
      border-top: none;
      border-radius: 0 0 5px 5px;
    }
  </style>
  <script>
    let currentTab = 'control';
    
    // Vaihda välilehteä
    function openTab(evt, tabName) {
      var i, tabcontent, tablinks;
      tabcontent = document.getElementsByClassName("tabcontent");
      for (i = 0; i < tabcontent.length; i++) {
        tabcontent[i].style.display = "none";
      }
      tablinks = document.getElementsByClassName("tablink");
      for (i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" active", "");
      }
      document.getElementById(tabName).style.display = "block";
      evt.currentTarget.className += " active";
      currentTab = tabName;
    }
    
    // Päivitä järjestelmän data
    function updateData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          // Lämpötilat
          document.getElementById('outsideTemp').innerText = data.outsideTemp.toFixed(1) + ' °C';
          document.getElementById('waterTemp').innerText = data.waterTemp.toFixed(1) + ' °C';
          document.getElementById('targetTemp').innerText = data.targetTemp.toFixed(1) + ' °C';
          
          // Anturien tilat
          document.getElementById('outsideStatus').innerText = data.outsideStatus;
          document.getElementById('outsideStatus').className = data.outsideStatus === 'VIRHE' ? 'fault' : 
                                                              data.outsideStatus === 'VAROITUS' ? 'warning' : 'ok';
          document.getElementById('waterStatus').innerText = data.waterStatus;
          document.getElementById('waterStatus').className = data.waterStatus === 'VIRHE' ? 'fault' : 
                                                            data.waterStatus === 'VAROITUS' ? 'warning' : 'ok';
          
          // Poltin
          document.getElementById('burnerState').innerText = data.burnerState ? 'PÄÄLLÄ' : 'POIS';
          document.getElementById('burnerMode').innerText = data.manualMode ? 'Manuaali' : 'Automaatti';
          document.getElementById('heatingDisabled').innerText = data.heatingDisabled ? 'KYLLÄ' : 'EI';
          
          // WiFi
          document.getElementById('wifiMode').innerText = data.wifiMode;
          document.getElementById('wifiIP').innerText = data.wifiIP;
        })
        .catch(error => {
          console.error('Virhe datan haussa:', error);
        });
    }
    
    // Aseta manuaali/automaatti-tila
    function setManualState(state) {
      fetch('/control?manual=' + (state ? '1' : '0'))
        .then(() => updateData());
    }
    
    // Aseta polttimen tila (vain manuaalitilassa)
    function setBurnerState(state) {
      fetch('/control?burner=' + (state ? '1' : '0'))
        .then(() => updateData());
    }
    
    // Hätäsammutus
    function emergencyStop() {
      if (confirm('Haluatko varmasti suorittaa hätäsammutuksen?')) {
        fetch('/emergency')
          .then(() => {
            alert('Hätäsammutus suoritettu!');
            updateData();
          });
      }
    }
    
    // Tallenna lämpötilakäyrä
    function saveCurve() {
      const curveData = [];
      for(let i = 0; i < 4; i++) {
        const outside = document.getElementById('editOutside' + i).value;
        const water = document.getElementById('editWater' + i).value;
        curveData.push(outside + ',' + water);
      }
      
      fetch('/setcurve?' + curveData.map((d, i) => `p${i}=${d}`).join('&'))
        .then(() => {
          alert('Käyrä tallennettu!');
          updateData();
        });
    }
    
    // Lataa lämpötilakäyrän nykyiset arvot
    function loadCurve() {
      fetch('/getcurve')
        .then(response => response.json())
        .then(data => {
          for(let i = 0; i < 4; i++) {
            document.getElementById('editOutside' + i).value = data.points[i][0];
            document.getElementById('editWater' + i).value = data.points[i][1];
          }
        });
    }
    
    // Tallenna WiFi-asetukset
    function saveWifiConfig() {
      const apSsid = document.getElementById('apSsid').value;
      const apPassword = document.getElementById('apPassword').value;
      const useSta = document.getElementById('useSta').checked;
      const staSsid = document.getElementById('staSsid').value;
      const staPassword = document.getElementById('staPassword').value;
      
      if (apSsid.length < 1) {
        alert('AP-verkon nimi ei voi olla tyhjä!');
        return;
      }
      
      if (useSta && staSsid.length < 1) {
        alert('Verkon nimi ei voi olla tyhjä kun STA-tila on käytössä!');
        return;
      }
      
      const params = new URLSearchParams({
        ap_ssid: apSsid,
        ap_password: apPassword,
        use_sta: useSta ? '1' : '0',
        sta_ssid: staSsid,
        sta_password: staPassword
      });
      
      if (confirm('Tallennetaanko WiFi-asetukset?\nLaite käynnistyy uudelleen.')) {
        fetch('/setwifi?' + params.toString())
          .then(response => response.text())
          .then(result => {
            alert('WiFi-asetukset tallennettu!\n' + result);
          });
      }
    }
    
    // Lataa WiFi-asetukset
    function loadWifiConfig() {
      fetch('/getwifi')
        .then(response => response.json())
        .then(data => {
          document.getElementById('apSsid').value = data.ap_ssid || '';
          document.getElementById('apPassword').value = data.ap_password || '';
          document.getElementById('useSta').checked = data.use_sta || false;
          document.getElementById('staSsid').value = data.sta_ssid || '';
          document.getElementById('staPassword').value = data.sta_password || '';
          
          // Päivitä STA-kenttien näkyvyys
          toggleStaFields();
        })
        .catch(error => {
          console.error('Virhe WiFi-asetusten lataamisessa:', error);
        });
    }
    
    // Näytä/piilota STA-kentät
    function toggleStaFields() {
      const useSta = document.getElementById('useSta').checked;
      document.getElementById('staFields').style.display = useSta ? 'block' : 'none';
    }
    
    // Päivitä dataa automaattisesti 5 sekunnin välein
    setInterval(updateData, 5000);
    
    // Alustus sivun latautuessa
    window.onload = function() {
      // Avaa ensimmäinen välilehti
      document.getElementsByClassName("tablink")[0].click();
      
      // Lataa asetukset
      loadWifiConfig();
      loadCurve();
      
      // Päivitä data
      updateData();
    };
  </script>
</head>
<body>
  <div class="container">
    <h1>Öljylämmityksen ohjaus</h1>
    
    <div class="tab">
      <button class="tablink" onclick="openTab(event, 'control')">Ohjaus</button>
      <button class="tablink" onclick="openTab(event, 'wifi')">WiFi-asetukset</button>
      <button class="tablink" onclick="openTab(event, 'curve')">Lämpötilakäyrä</button>
    </div>
    
    <!-- OHJAUS-VÄLILEHTI -->
    <div id="control" class="tabcontent">
      <div class="sensor-box">
        <h2>Lämpötilat</h2>
        <p><strong>Ulkolämpötila:</strong> <span id="outsideTemp">--</span></p>
        <p><strong>Menoveden lämpötila:</strong> <span id="waterTemp">--</span></p>
        <p><strong>Tavoitelämpötila:</strong> <span id="targetTemp">--</span></p>
      </div>
      
      <div class="status-box">
        <h2>Järjestelmän tila</h2>
        <p><strong>Ulkoanturi:</strong> <span id="outsideStatus" class="ok">OK</span></p>
        <p><strong>Vesianturi:</strong> <span id="waterStatus" class="ok">OK</span></p>
        <p><strong>Lämmitys estetty (liian lämmin):</strong> <span id="heatingDisabled">--</span></p>
      </div>
      
      <div class="control-box">
        <h2>Öljypoltin</h2>
        <p><strong>Tila:</strong> <span id="burnerState">--</span></p>
        <p><strong>Toimintatila:</strong> <span id="burnerMode">--</span></p>
        
        <button onclick="setManualState(false)">Automaattitila</button>
        <button onclick="setManualState(true)">Manuaalitila</button>
        <br>
        <button onclick="setBurnerState(true)">Käynnistä poltin</button>
        <button onclick="setBurnerState(false)">Sammuta poltin</button>
        <br>
        <button onclick="emergencyStop()" class="danger">Hätäsammutus</button>
      </div>
      
      <div class="sensor-box">
        <h2>WiFi-tila</h2>
        <p><strong>Tila:</strong> <span id="wifiMode">--</span></p>
        <p><strong>IP-osoite:</strong> <span id="wifiIP">--</span></p>
      </div>
    </div>
    
    <!-- WIFI-ASETUKSET VÄLILEHTI -->
    <div id="wifi" class="tabcontent">
      <div class="config-box">
        <h2>WiFi-asetukset</h2>
        
        <h3>Oma verkko (Access Point)</h3>
        <p>Laite luo oman WiFi-verkon, johon voit yhdistää suoraan.</p>
        <p><strong>Verkon nimi (SSID):</strong><br>
        <input type="text" id="apSsid" placeholder="AP-verkon nimi"></p>
        <p><strong>Salasana:</strong><br>
        <input type="password" id="apPassword" placeholder="AP-salasana"></p>
        
        <h3>Yhteys olemassa olevaan verkkoon (valinnainen)</h3>
        <p><input type="checkbox" id="useSta" onclick="toggleStaFields()">
        <label for="useSta">Yhdistä olemassa olevaan verkkoon</label></p>
        
        <div id="staFields" style="display: none;">
          <p><strong>Verkon nimi (SSID):</strong><br>
          <input type="text" id="staSsid" placeholder="Verkon nimi"></p>
          <p><strong>Salasana:</strong><br>
          <input type="password" id="staPassword" placeholder="Verkon salasana"></p>
          <p><em>Huom: Jos yhteys epäonnistuu, laite luo automaattisesti oman verkon.</em></p>
        </div>
        
        <button onclick="saveWifiConfig()">Tallenna WiFi-asetukset</button>
        <button onclick="loadWifiConfig()">Lataa nykyiset asetukset</button>
        
        <p><em>Huom: Tallennuksen jälkeen laite käynnistyy uudelleen automaattisesti.</em></p>
      </div>
    </div>
    
    <!-- LÄMPÖTILAKÄYRÄ VÄLILEHTI -->
    <div id="curve" class="tabcontent">
      <div class="curve-box">
        <h2>Lämpötilakäyrän säätö</h2>
        <p>Aseta 4 pistettä lineaariselle lämpötilakäyrälle.</p>
        
        <table>
          <tr>
            <th>Piste</th>
            <th>Ulkolämpötila (°C)</th>
            <th>Vesilämpötila (°C)</th>
          </tr>
          <tr>
            <td>1 (kylmin)</td>
            <td><input type="number" id="editOutside0" step="0.1" value="-20.0"></td>
            <td><input type="number" id="editWater0" step="0.1" value="75.0"></td>
          </tr>
          <tr>
            <td>2</td>
            <td><input type="number" id="editOutside1" step="0.1" value="-10.0"></td>
            <td><input type="number" id="editWater1" step="0.1" value="65.0"></td>
          </tr>
          <tr>
            <td>3</td>
            <td><input type="number" id="editOutside2" step="0.1" value="0.0"></td>
            <td><input type="number" id="editWater2" step="0.1" value="55.0"></td>
          </tr>
          <tr>
            <td>4 (lämpimin)</td>
            <td><input type="number" id="editOutside3" step="0.1" value="10.0"></td>
            <td><input type="number" id="editWater3" step="0.1" value="45.0"></td>
          </tr>
        </table>
        
        <button onclick="saveCurve()">Tallenna käyrä</button>
        <button onclick="loadCurve()">Lataa nykyinen käyrä</button>
        
        <p><em>Käyrä tallennetaan automaattisesti 5 sekunnin viiveellä.</em></p>
      </div>
    </div>
  </div>
</body>
</html>
)rawliteral";

/**
 * Web-palvelin task
 * Tarjoaa web-käyttöliittymän järjestelmän ohjaamiseen
 */
void webServerTask(void *parameter) {
  Serial.println("Web-palvelintask aloitettu");
  
  // Odota että WiFi on täysin alustettu
  delay(2000);
  
  /**
   * HTTP-reitit
   */
  
  // Pääsivu
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", htmlPage);
  });
  
  // JSON-data
  server.on("/data", HTTP_GET, []() {
    // Kerää data suojatusti
    float outsideTemp = 0.0, waterTemp = 0.0, targetTemp = 0.0;
    bool heatingDisabled = false;
    String outsideStatus = "OK", waterStatus = "OK";
    String wifiMode = "--", wifiIP = "--";
    
    // Hae lämpötilat
    if (lockSensorAndTemperature()) {
      outsideTemp = systemState.outsideStatus.fault ? 
                   systemState.outsideData.getAverageValidValue() : 
                   systemState.outsideData.currentTemp;
      
      waterTemp = systemState.waterStatus.fault ? 
                  systemState.waterData.getAverageValidValue() : 
                  systemState.waterData.currentTemp;
      
      if (systemState.outsideStatus.fault) {
        outsideStatus = "VIRHE";
      } else if (systemState.outsideStatus.temporaryFault) {
        outsideStatus = "VAROITUS";
      }
      
      if (systemState.waterStatus.fault) {
        waterStatus = "VIRHE";
      } else if (systemState.waterStatus.temporaryFault) {
        waterStatus = "VAROITUS";
      }
      
      targetTemp = systemState.targetWaterTemp;
      heatingDisabled = systemState.heatingDisabled;
      
      unlockSensorAndTemperature();
    }
    
    // Hae polttimen tila
    bool burnerState = false, manualMode = false;
    if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      burnerState = systemState.burnerState;
      manualMode = systemState.burnerManualOverride;
      xSemaphoreGive(xBurnerStateMutex);
    }
    
    // Hae WiFi-tila
    if (systemState.wifiConfig.ap_mode) {
      wifiMode = "AP (Oma verkko)";
      wifiIP = WiFi.softAPIP().toString();
    } else if (systemState.wifiConfig.connected) {
      wifiMode = "STA (Verkko)";
      wifiIP = WiFi.localIP().toString();
    }
    
    // Muodosta JSON
    String json = "{";
    json += "\"outsideTemp\":" + String(outsideTemp, 1);
    json += ",\"waterTemp\":" + String(waterTemp, 1);
    json += ",\"targetTemp\":" + String(targetTemp, 1);
    json += ",\"burnerState\":" + String(burnerState ? "true" : "false");
    json += ",\"manualMode\":" + String(manualMode ? "true" : "false");
    json += ",\"outsideStatus\":\"" + outsideStatus + "\"";
    json += ",\"waterStatus\":\"" + waterStatus + "\"";
    json += ",\"heatingDisabled\":" + String(heatingDisabled ? "true" : "false");
    json += ",\"wifiMode\":\"" + wifiMode + "\"";
    json += ",\"wifiIP\":\"" + wifiIP + "\"";
    json += "}";
    
    server.send(200, "application/json", json);
  });
  
  // Ohjauskomennot
  server.on("/control", HTTP_GET, []() {
    if (server.hasArg("manual")) {
      bool manual = server.arg("manual").toInt() == 1;
      if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        systemState.burnerManualOverride = manual;
        xSemaphoreGive(xBurnerStateMutex);
      }
      Serial.printf("Manuaalitila asetettu: %s\n", manual ? "PÄÄLLÄ" : "POIS");
    }
    
    if (server.hasArg("burner")) {
      bool burner = server.arg("burner").toInt() == 1;
      bool manualMode = false;
      
      if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        manualMode = systemState.burnerManualOverride;
        if (manualMode) {
          systemState.burnerState = burner;
          
          RelayCommand cmd;
          cmd.type = RelayCommand::SET_STATE;
          cmd.state = burner;
          xQueueSend(xRelayControlQueue, &cmd, 0);
        }
        xSemaphoreGive(xBurnerStateMutex);
      }
      
      if (manualMode) {
        Serial.printf("Poltin asetettu manuaalisesti: %s\n", burner ? "PÄÄLLÄ" : "POIS");
      }
    }
    
    server.send(200, "text/plain", "OK");
  });
  
  // Lämpötilakäyrän asetus
  server.on("/setcurve", HTTP_GET, []() {
    for (int i = 0; i < 4; i++) {
      if (server.hasArg("p" + String(i))) {
        String values = server.arg("p" + String(i));
        int commaPos = values.indexOf(',');
        if (commaPos > 0) {
          float outTemp = values.substring(0, commaPos).toFloat();
          float waterTemp = values.substring(commaPos + 1).toFloat();
          
          if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            systemState.curve.points[i][0] = outTemp;
            systemState.curve.points[i][1] = waterTemp;
            systemState.settingsDirty = true;
            systemState.lastSettingsChange = xTaskGetTickCount() * portTICK_PERIOD_MS;
            xSemaphoreGive(xSettingsMutex);
          }
        }
      }
    }
    
    server.send(200, "text/plain", "Käyräpisteet vastaanotettu (tallennetaan flashiin viiveellä)");
  });
  
  // Lämpötilakäyrän hakeminen
  server.on("/getcurve", HTTP_GET, []() {
    String json = "[";
    
    if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      for (int i = 0; i < 4; i++) {
        json += "[";
        json += String(systemState.curve.points[i][0], 1);
        json += ",";
        json += String(systemState.curve.points[i][1], 1);
        json += "]";
        if (i < 3) json += ",";
      }
      xSemaphoreGive(xSettingsMutex);
    }
    
    json += "]";
    server.send(200, "application/json", json);
  });
  
  // Hätäsammutus
  server.on("/emergency", HTTP_GET, []() {
    RelayCommand cmd;
    cmd.type = RelayCommand::EMERGENCY_OFF;
    xQueueSend(xRelayControlQueue, &cmd, 0);
    
    Serial.println("Hätäsammutus suoritettu käyttäjän pyynnöstä");
    server.send(200, "text/plain", "Hätäsammutus suoritettu");
  });
  
  // WiFi-asetusten hakeminen
  server.on("/getwifi", HTTP_GET, []() {
    String json = "{";
    
    if (xSemaphoreTake(xWifiConfigMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      json += "\"ap_ssid\":\"" + String(systemState.wifiConfig.ap_ssid) + "\"";
      json += ",\"ap_password\":\"" + String(systemState.wifiConfig.ap_password) + "\"";
      json += ",\"use_sta\":" + String(systemState.wifiConfig.use_sta ? "true" : "false");
      json += ",\"sta_ssid\":\"" + String(systemState.wifiConfig.sta_ssid) + "\"";
      json += ",\"sta_password\":\"" + String(systemState.wifiConfig.sta_password) + "\"";
      xSemaphoreGive(xWifiConfigMutex);
    }
    
    json += "}";
    server.send(200, "application/json", json);
  });
  
  // WiFi-asetusten tallentaminen
  server.on("/setwifi", HTTP_GET, []() {
    bool changed = false;
    
    if (xSemaphoreTake(xWifiConfigMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (server.hasArg("ap_ssid")) {
        strncpy(systemState.wifiConfig.ap_ssid, server.arg("ap_ssid").c_str(), 31);
        changed = true;
      }
      if (server.hasArg("ap_password")) {
        strncpy(systemState.wifiConfig.ap_password, server.arg("ap_password").c_str(), 31);
        changed = true;
      }
      if (server.hasArg("use_sta")) {
        systemState.wifiConfig.use_sta = (server.arg("use_sta").toInt() == 1);
        changed = true;
      }
      if (server.hasArg("sta_ssid")) {
        strncpy(systemState.wifiConfig.sta_ssid, server.arg("sta_ssid").c_str(), 31);
        changed = true;
      }
      if (server.hasArg("sta_password")) {
        strncpy(systemState.wifiConfig.sta_password, server.arg("sta_password").c_str(), 31);
        changed = true;
      }
      xSemaphoreGive(xWifiConfigMutex);
    }
    
    if (changed) {
      saveWifiConfig();
      server.send(200, "text/plain", "WiFi-asetukset tallennettu. Laite käynnistyy uudelleen...");
      delay(1000);
      ESP.restart();
    } else {
      server.send(200, "text/plain", "Ei muutoksia WiFi-asetuksiin.");
    }
  });
  
  // Sivua ei löydy
  server.onNotFound([]() {
    server.send(404, "text/plain", "Sivua ei löydy");
  });
  
  // Käynnistä web-palvelin
  server.begin();
  Serial.println("HTTP-palvelin käynnistetty portissa 80");
  
  // Tulosta yhteystiedot
  if (systemState.wifiConfig.ap_mode) {
    Serial.println("\n=== WEB-PALVELIN VALMIS ===");
    Serial.println("Yhdistä omaan verkkoon:");
    Serial.println("  SSID: " + String(systemState.wifiConfig.ap_ssid));
    Serial.println("  Salasana: " + String(systemState.wifiConfig.ap_password));
    Serial.println("  Avaa selaimessa: http://" + WiFi.softAPIP().toString());
  } else {
    Serial.println("\n=== WEB-PALVELIN VALMIS ===");
    Serial.println("Yhdistetty verkkoon:");
    Serial.println("  Avaa selaimessa: http://" + WiFi.localIP().toString());
  }
  
  // Web-palvelimen pääsilmukka
  while (1) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==================== SETUP ====================

/**
 * Arduino setup()-funktio
 * Suoritetaan kerran käynnistyessä
 * Alustaa kaikki järjestelmän komponentit ja käynnistää säikeet
 */
void setup() {
  // Alusta sarjamonitori debug-tulostusta varten
  Serial.begin(115200);
  delay(2000);  // Odota että sarjamonitori on valmis
  
  Serial.println("\n\n=== ÖLJYLÄMMITYSJÄRJESTELMÄ - FreeRTOS ===\n");
  Serial.println("Järjestelmä käynnistyy...");
  
  /**
   * 1. LUO MUTEXIT
   * Deadlockin välttämiseksi käytetään aina samaa järjestystä
   */
  xSensorStatusMutex = xSemaphoreCreateMutex();
  xTemperatureMutex = xSemaphoreCreateMutex();
  xBurnerStateMutex = xSemaphoreCreateMutex();
  xSettingsMutex = xSemaphoreCreateMutex();
  xWifiConfigMutex = xSemaphoreCreateMutex();
  
  if (xSensorStatusMutex == NULL || xTemperatureMutex == NULL || 
      xBurnerStateMutex == NULL || xSettingsMutex == NULL || xWifiConfigMutex == NULL) {
    Serial.println("VIRHE: Mutexien luonti epäonnistui!");
    while(1);
  }
  
  /**
   * 2. LUO EVENT GROUP
   * Käytetään tehokkaaseen tilanvaihtojen ilmoittamiseen
   */
  xSystemEvents = xEventGroupCreate();
  if (xSystemEvents == NULL) {
    Serial.println("VIRHE: Event Groupin luonti epäonnistui!");
    while(1);
  }
  
  /**
   * 3. LUO JONO
   * Releen ohjauskommunikaatioon säikeiden välillä
   */
  xRelayControlQueue = xQueueCreate(10, sizeof(RelayCommand));
  if (xRelayControlQueue == NULL) {
    Serial.println("VIRHE: Jonon luonti epäonnistui!");
    while(1);
  }
  
  /**
   * 4. LATAA ASETUKSET FLASH-MUISTISTA
   * Ladataan lämpötilakäyrä (WiFi-asetukset ladataan myöhemmin)
   */
  preferences.begin("oilheater", true);
  for (int i = 0; i < 4; i++) {
    systemState.curve.points[i][0] = preferences.getFloat(
      ("out" + String(i)).c_str(), 
      systemState.curve.points[i][0]
    );
    systemState.curve.points[i][1] = preferences.getFloat(
      ("water" + String(i)).c_str(), 
      systemState.curve.points[i][1]
    );
  }
  preferences.end();
  Serial.println("Asetukset ladattu flash-muistista");
  
  /**
   * 5. ALUSTA WIFI
   * Lataa WiFi-asetukset ja yhdistä verkkoon tai luo oma verkko
   */
  initWiFi();
  
  /**
   * 6. LUO SÄIKEET (TASKS)
   * Jokaiselle toiminnolle oma säie oikealla prioriteetilla
   */
  
  // Watchdog-task (korkein prioriteetti)
  xTaskCreatePinnedToCore(
    watchdogTask,
    "WatchdogTask",
    TASK_STACK_SMALL,
    NULL,
    TASK_PRIORITY_HIGH + 1,
    &xWatchdogTaskHandle,
    0  // Core 0
  );
  
  // Rele-task (korkea prioriteetti - turvallisuuskriittinen)
  xTaskCreatePinnedToCore(
    relayTask,
    "RelayTask",
    TASK_STACK_SMALL,
    NULL,
    TASK_PRIORITY_HIGH,
    &xRelayTaskHandle,
    1  // Core 1
  );
  
  // Lämpötilatask
  xTaskCreatePinnedToCore(
    temperatureTask,
    "TempTask",
    TASK_STACK_MEDIUM,
    NULL,
    TASK_PRIORITY_HIGH,
    &xTemperatureTaskHandle,
    1
  );
  
  // Säätötask
  xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    TASK_STACK_MEDIUM,
    NULL,
    TASK_PRIORITY_MEDIUM,
    &xControlTaskHandle,
    1
  );
  
  // Flash-task (taustatehtävä)
  xTaskCreatePinnedToCore(
    flashTask,
    "FlashTask",
    TASK_STACK_SMALL,
    NULL,
    TASK_PRIORITY_LOW,
    &xFlashTaskHandle,
    1
  );
  
  // Odota että järjestelmä on stabiloitunut
  delay(3000);
  
  // Web-palvelintask (ei-kriittinen)
  xTaskCreatePinnedToCore(
    webServerTask,
    "WebServerTask",
    TASK_STACK_LARGE,
    NULL,
    TASK_PRIORITY_LOW,
    &xWebServerTaskHandle,
    0  // Core 0 (sama kuin WiFi)
  );
  
  Serial.println("\nKaikki säikeet luotu onnistuneesti");
  Serial.println("Järjestelmä valmiina käyttöön!");
}

// ==================== LOOP ====================

/**
 * Arduino loop()-funktio
 * Perinteinen Arduino-silmukka, joka ajetaan core 0:lla
 * Tarkistaa WiFi-yhteyden ja suorittaa muita ei-kriittisiä tehtäviä
 */
void loop() {
  static unsigned long lastWifiCheck = 0;
  unsigned long currentTime = millis();
  
  // Tarkista WiFi-yhteys 30 sekunnin välein (vain STA-tilassa)
  if (currentTime - lastWifiCheck > WIFI_RECONNECT_INTERVAL) {
    lastWifiCheck = currentTime;
    
    if (systemState.wifiConfig.use_sta && !systemState.wifiConfig.ap_mode) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi-yhteys katkesi, yritetään yhdistää uudelleen...");
        reconnectWiFi();
      }
    }
  }
  
  // Vapauta ajoaikaa muille säikeille
  vTaskDelay(pdMS_TO_TICKS(1000));
}
