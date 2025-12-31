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

// ==================== KONFIGURAATIO ====================
#define OUTSIDE_SENSOR_PIN 4    // GPIO-pinni ulkolämpötilan anturille
#define WATER_SENSOR_PIN 5      // GPIO-pinni menoveden lämpötilan anturille
#define RELAY_PIN 2             // GPIO-pinni releen ohjaukseen
#define HYSTERESIS 4.0          // Hystereesi lämpötilasäädössä (°C)

// Anturivirheen parametrit
#define SENSOR_FAULT_TEMP -50.0         // Lämpötila, jota alempi arvo on virhe
#define SENSOR_FAULT_TIMEOUT 30000      // 30 sekuntia ennen vakavaa virhettä
#define MAX_CONSECUTIVE_FAILURES 3      // Sallittu peräkkäisten virheiden määrä
#define TEMP_READ_INTERVAL_MS 5000      // Lämpötilan lukemisen väli (ms)

// Task prioriteetit (korkeampi numero = korkeampi prioriteetti)
#define TASK_PRIORITY_HIGH 3            // Kriittiset taskit (rele, watchdog)
#define TASK_PRIORITY_MEDIUM 2          // Normaalit taskit (säätö, lämpötilat)
#define TASK_PRIORITY_LOW 1             // Tausta-taskit (web, flash)

// Task stack koot (muisti byteina)
#define TASK_STACK_SMALL 2048           // Pienille taskeille
#define TASK_STACK_MEDIUM 3072          // Keskikokoisille taskeille
#define TASK_STACK_LARGE 4096           // ISOille taskeille (esim. web-palvelin)

// Event Group bitit eri järjestelmätiloille
#define BIT_SENSOR_FAULT_OUTSIDE (1 << 0)  // Ulkoanturin vakava virhe
#define BIT_SENSOR_FAULT_WATER   (1 << 1)  // Vesianturin vakava virhe
#define BIT_HEATING_DISABLED     (1 << 2)  // Lämmitys pois päältä (liian lämmin)
#define BIT_TEMP_TASK_ALIVE      (1 << 3)  // Lämpötilatask elossa (watchdog)

// WIFI asetukset
const char* ssid = "OilHeaterAP";      // WiFi-verkon nimi (SSID)
const char* password = "enmuista";     // WiFi-verkon salasana

// ==================== GLOBAALIT RESURSSIT ====================
/**
 * MUTEX-JÄRJESTYS (kriittinen deadlockin välttämiseksi):
 * 1. xSensorStatusMutex  - Anturien tilatiedot
 * 2. xTemperatureMutex   - Lämpötilamittaukset
 * 3. xBurnerStateMutex   - Polttimen tila
 * 4. xSettingsMutex      - Asetukset ja käyrät
 * 
 * HUOM: Aina otetaan samassa järjestyksessä kaikissa taskeissa!
 */
SemaphoreHandle_t xSensorStatusMutex;   // Suojaa anturien tilatietoja
SemaphoreHandle_t xTemperatureMutex;    // Suojaa lämpötilamittauksia
SemaphoreHandle_t xBurnerStateMutex;    // Suojaa polttimen tilaa
SemaphoreHandle_t xSettingsMutex;       // Suojaa asetuksia

// Jono releen ohjauskomentoille
QueueHandle_t xRelayControlQueue;

// Event Group järjestelmän tiloille ja virheille
EventGroupHandle_t xSystemEvents;

// Task handles säikeiden hallintaan
TaskHandle_t xTemperatureTaskHandle = NULL;  // Lämpötilanlukija
TaskHandle_t xControlTaskHandle = NULL;      // Säätölogiikka
TaskHandle_t xWebServerTaskHandle = NULL;    // Web-palvelin
TaskHandle_t xRelayTaskHandle = NULL;        // Releen ohjaus
TaskHandle_t xWatchdogTaskHandle = NULL;     // Watchdog-valvoja
TaskHandle_t xFlashTaskHandle = NULL;        // Flash-tallennus

// Anturioliot DS18B20-sensoreille
OneWire oneWireOutside(OUTSIDE_SENSOR_PIN);    // Ulkoanturin OneWire-väylä
OneWire oneWireWater(WATER_SENSOR_PIN);        // Vesianturin OneWire-väylä
DallasTemperature sensorsOutside(&oneWireOutside);  // Ulkoanturin DallasTemperature-olio
DallasTemperature sensorsWater(&oneWireWater);      // Vesianturin DallasTemperature-olio

// Web-palvelin olio portissa 80
WebServer server(80);

// Pysyvän muistin (flash) käsittelyyn
Preferences preferences;

// ==================== DATATIENOT ====================

  const char* htmlPage = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
  <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta http-equiv="Content-Type" content="text/html; charset=utf-8">
    <title>Öljylämmityksen ohjaus</title>
    <style>
      body {
        font-family: Arial, sans-serif;
        margin: 20px;
        background-color: #f5f5f5;
      }
      .container {
        max-width: 800px;
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
      .status-box {
        background: #e8f5e8;
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
      input[type=number] {
        width: 80px;
        padding: 5px;
      }
    </style>
    <script>

      function updateData() {
        fetch('/data')
          .then(response => response.json())
          .then(data => {
            // Päivitä lämpötilat
            document.getElementById('outsideTemp').innerText = data.outsideTemp.toFixed(1) + ' °C';
            document.getElementById('waterTemp').innerText = data.waterTemp.toFixed(1) + ' °C';
            document.getElementById('targetTemp').innerText = data.targetTemp.toFixed(1) + ' °C';
            
            // Päivitä anturien tilat
            document.getElementById('outsideStatus').innerText = data.outsideStatus;
            document.getElementById('outsideStatus').className = data.outsideStatus === 'VIRHE' ? 'fault' : 
                                                                data.outsideStatus === 'VAROITUS' ? 'warning' : 'ok';
            document.getElementById('waterStatus').innerText = data.waterStatus;
            document.getElementById('waterStatus').className = data.waterStatus === 'VIRHE' ? 'fault' : 
                                                              data.waterStatus === 'VAROITUS' ? 'warning' : 'ok';
            
            // Päivitä polttimen tila
            document.getElementById('burnerState').innerText = data.burnerState ? 'PÄÄLLÄ' : 'POIS';
            document.getElementById('burnerMode').innerText = data.manualMode ? 'Manuaali' : 'Automaatti';
            document.getElementById('heatingDisabled').innerText = data.heatingDisabled ? 'KYLLÄ' : 'EI';
          })
          .catch(error => {
            console.error('Virhe datan haussa:', error);
          });
      }
      

      function setManualState(state) {
        fetch('/control?manual=' + (state ? '1' : '0'))
          .then(() => updateData());
      }
      
   
      function setBurnerState(state) {
        fetch('/control?burner=' + (state ? '1' : '0'))
          .then(() => updateData());
      }
      

      function emergencyStop() {
        if (confirm('Haluatko varmasti suorittaa hätäsammutuksen?')) {
          fetch('/emergency')
            .then(() => {
              alert('Hätäsammutus suoritettu!');
              updateData();
            });
        }
      }
      
 
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
      
      // Päivitä dataa automaattisesti 5 sekunnin välein
      setInterval(updateData, 5000);
      
      // Päivitä data heti sivun latautuessa
      window.onload = updateData;
    </script>
  </head>
  <body>
    <div class="container">
      <h1>Öljylämmityksen ohjaus</h1>
      
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
        <button onclick="emergencyStop()" style="background-color: #f44336;">Hätäsammutus</button>
      </div>
      
      <div class="sensor-box">
        <h2>Lämpötilakäyrän säätö</h2>
        <table>
          <tr>
            <th>Piste</th>
            <th>Ulkolämpötila (°C)</th>
            <th>Vesilämpötila (°C)</th>
          </tr>
          <tr>
            <td>1</td>
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
            <td>4</td>
            <td><input type="number" id="editOutside3" step="0.1" value="10.0"></td>
            <td><input type="number" id="editWater3" step="0.1" value="45.0"></td>
          </tr>
        </table>
        <button onclick="saveCurve()">Tallenna käyrä</button>
      </div>
    </div>
  </body>
  </html>
  )rawliteral";
/**
 * Lämpötilasäädyskäyrän pisteet.
 * Jokainen rivi edustaa yhtä pistettä käyrällä muodossa:
 * [ulkolämpötila, menoveden tavoitelämpötila]
 * 
 * Käyttää lineaarista interpolaatiota pisteiden välillä.
 */
struct CurvePoints {
  float points[4][2] = {
    {-20.0, 75.0},   // Piste 1: -20°C ulkona -> 75°C vettä
    {-10.0, 65.0},   // Piste 2: -10°C ulkona -> 65°C vettä
    {0.0, 55.0},     // Piste 3: 0°C ulkona -> 55°C vettä
    {10.0, 45.0}     // Piste 4: 10°C ulkona -> 45°C vettä
  };
};

/**
 * Anturin lämpötiladata.
 * Sisältää nykyisen lämpötilan, historian ja viimeiset hyväksytyt arvot.
 * 
 * Käyttää 10 arvon liukuvana keskiarvona tilapäisissä virhetilanteissa.
 */
struct SensorData {
  float currentTemp = 0.0;               // Viimeisin mitattu lämpötila
  float lastValidTemp = 0.0;             // Viimeisin hyväksytty lämpötila
  unsigned long lastValidReadTime = 0;   // Aikaleima viimeisestä hyvästä lukemasta
  
  // Viimeiset 10 hyvää arvoa liukuvan keskiarvon laskemiseen
  float lastValidValues[10] = {0};       // Arvohistoria
  int validValueIndex = 0;               // Nykyinen indeksi historiataulukossa
  int validValueCount = 0;               // Kuinka monta arvoa historiassa on
  
  /**
   * Lisää uuden hyväksytyn lämpötilan historiaan.
   * Ylläpitää 10 viimeisintä arvoa kiertävänä puskurina.
   * 
   * @param value Hyväksytty lämpötilan arvo
   */
  void addValidValue(float value) {
    lastValidValues[validValueIndex] = value;
    validValueIndex = (validValueIndex + 1) % 10;
    if (validValueCount < 10) validValueCount++;
  }
  
  /**
   * Laskee historiatietojen perusteella liukuvan keskiarvon.
   * Käytetään kun anturi on vikana, jotta saadaan realistinen arvio.
   * 
   * @return Lämpötilan keskiarvo historiasta
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
 * Anturin tilatiedot ja virhehistoria.
 * Seuraa anturin vikatilanteita ja niiden kestoa.
 */
struct SensorStatus {
  bool fault = false;                    // Vakava vika (yli 30s jatkunut)
  bool temporaryFault = false;           // Tilapäinen häiriö (alle 30s)
  unsigned long faultStartTime = 0;      // Aikaleima vian alkamisesta
  int consecutiveFailures = 0;           // Peräkkäisten virheellisten lukemien määrä
};

/**
 * Järjestelmän kokonaistila.
 * Kaikki alidatat on suojattu omilla mutexeillaan.
 */
struct SystemState {
  // Lämpötiladata (suojaa: xTemperatureMutex)
  float targetWaterTemp = 0.0;           // Laskettu tavoitelämpötila vedelle
  bool heatingDisabled = false;          // True jos ulkolämpötila yli maksimipisteen
  
  // Polttimen tila (suojaa: xBurnerStateMutex)
  bool burnerState = false;              // Polttimen nykyinen tila (true = päällä)
  bool burnerManualOverride = false;     // Manuaalitila päällä/pois
  
  // Anturidata (suojaa: xSensorStatusMutex)
  SensorData outsideData;                // Ulkoanturin lämpötiladata
  SensorData waterData;                  // Vesianturin lämpötiladata
  SensorStatus outsideStatus;            // Ulkoanturin tilatiedot
  SensorStatus waterStatus;              // Vesianturin tilatiedot
  
  // Asetukset (suojaa: xSettingsMutex)
  CurvePoints curve;                     // Lämpötilakäyrän pisteet
  
  // Flash-tallennuksen hallinta
  bool settingsDirty = false;            // True jos asetuksia on muutettu
  unsigned long lastSettingsChange = 0;  // Viimeisen muutoksen aikaleima
};

// Globaali järjestelmän tila (jaettu resurssi kaikkien säikeiden kesken)
SystemState systemState;

/**
 * Releen ohjausviesti.
 * Välitetään jonon kautta releen ohjaustaskille.
 */
struct RelayCommand {
  enum CommandType {
    SET_STATE,      // Normaali tilan asetus (käytä säätölogiikkaa)
    FORCE_OFF,      // Pakota rele pois (turvatoimi)
    EMERGENCY_OFF   // Hätäsammutus (ohita kaikki logiikat)
  };
  
  CommandType type;  // Komennon tyyppi
  bool state;        // Haluttu tila (käytössä vain SET_STATE:lle)
};

// ==================== TURVALLISET APUFUNKTIOT ====================

/**
 * Lukitsee sensorStatus ja temperature mutexit tiukassa järjestyksessä.
 * DEADLOCKIN VÄLTTÄMISEKSI: Aina sama järjestys kaikkialla!
 * 
 * @return true jos molemmat mutexit saatiin, false jos timeout
 */
bool lockSensorAndTemperature() {
  // 1. Ota ensin xSensorStatusMutex
  if (xSemaphoreTake(xSensorStatusMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // 2. Sitten xTemperatureMutex
    if (xSemaphoreTake(xTemperatureMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      return true;
    }
    // Jos toinen mutex epäonnistui, vapauta ensimmäinen
    xSemaphoreGive(xSensorStatusMutex);
  }
  return false;
}

/**
 * Vapauttaa sensorStatus ja temperature mutexit käänteisessä järjestyksessä.
 */
void unlockSensorAndTemperature() {
  xSemaphoreGive(xTemperatureMutex);
  xSemaphoreGive(xSensorStatusMutex);
}

/**
 * Lukitsee temperature ja burnerState mutexit tiukassa järjestyksessä.
 * 
 * @return true jos molemmat mutexit saatiin, false jos timeout
 */
bool lockTemperatureAndBurner() {
  // 1. Ota ensin xTemperatureMutex
  if (xSemaphoreTake(xTemperatureMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // 2. Sitten xBurnerStateMutex
    if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      return true;
    }
    xSemaphoreGive(xTemperatureMutex);
  }
  return false;
}

/**
 * Vapauttaa temperature ja burnerState mutexit käänteisessä järjestyksessä.
 */
void unlockTemperatureAndBurner() {
  xSemaphoreGive(xBurnerStateMutex);
  xSemaphoreGive(xTemperatureMutex);
}

/**
 * Tarkistaa onko lämpötilan arvo virheellinen.
 * Tarkistaa sekä DS18B20-spesifiset virhekoodit että epärealistiset arvot.
 * 
 * @param temperature Tarkistettava lämpötila
 * @param isWaterSensor True jos tarkistetaan vesianturia, false jos ulkoanturia
 * @return True jos lämpötila on virheellinen, false jos ok
 */
bool isTemperatureFaulty(float temperature, bool isWaterSensor) {
  // DS18B20 palauttaa DEVICE_DISCONNECTED_C jos anturia ei löydy
  if (temperature == DEVICE_DISCONNECTED_C) {
    return true;
  }
  
  // Liian matala lämpötila (alle -50°C)
  if (temperature < SENSOR_FAULT_TEMP) {
    return true;
  }
  
  // Liian korkeat lämpötilat (riippuu anturityypistä)
  if (isWaterSensor) {
    if (temperature > 120.0) return true; // Vesi ei saisi olla yli 120°C
  } else {
    if (temperature > 100.0) return true; // Ulkona ei saisi olla yli 100°C
  }
  
  return false;
}

/**
 * Päivittää anturin tilatiedot uuden mittauksen perusteella.
 * Hallitsee virheiden laskentaa ja siirtymisiä eri tiloihin.
 * 
 * @param status Anturin tilatietojen rakenne (päivitettävä)
 * @param data Anturin lämpötiladata (päivitettävä)
 * @param newValue Uusi mitattu lämpötilan arvo
 * @param isWaterSensor True jos vesianturi, false jos ulkoanturi
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
    // HYVÄKSYTTÄVÄ LUKEMA
    data.currentTemp = newValue;
    data.lastValidTemp = newValue;
    data.lastValidReadTime = currentTime;
    data.addValidValue(newValue);
    
    // Jos oli tilapäinen häiriö, nollaa laskuri
    if (status.temporaryFault) {
      status.consecutiveFailures = 0;
      status.temporaryFault = false;
      
      // Tyhjää Event Groupin bitti
      EventBits_t bits = BIT_SENSOR_FAULT_OUTSIDE;
      if (isWaterSensor) bits = BIT_SENSOR_FAULT_WATER;
      xEventGroupClearBits(xSystemEvents, bits);
    }
    
    // Jos oli pieni määrä virheitä (alle MAX), nollaa ne
    if (status.consecutiveFailures > 0 && status.consecutiveFailures < MAX_CONSECUTIVE_FAILURES) {
      status.consecutiveFailures = 0;
    }
  }
}

/**
 * Laskee tavoitelämpötilan menovedelle annetun ulkolämpötilan perusteella.
 * Käyttää 4 pisteen lineaarista käyrää interpolointiin.
 * 
 * @param outsideTekijä Nykyinen ulkolämpötila
 * @param curve Lämpötilakäyrän pisteet
 * @return Tavoitelämpötila vedelle, 0.0 jos lämmitys estetty
 */
float calculateTargetTemperature(float outsideTemp, const CurvePoints& curve) {
  // Jos ulkolämpötila yli lämpimimmän pisteen (10°C oletuksena)
  if (outsideTemp >= curve.points[3][0]) {
    return 0.0;  // Lämmitys pois päältä
  }
  
  // Jos ulkolämpötila alle kylmimmän pisteen (-20°C oletuksena)
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
  
  return 45.0; // Oletusarvo jos jokin meni pieleen
}

// ==================== TASKIT ====================

/**
 * Lämpötilan lukemis-task.
 * 
 * Tämä task lukee DS18B20-anturit EI-BLOKKAAVASTI käyttäen tilakonetta.
 * Se päivittää anturien tilat ja laskee tavoitelämpötilan.
 * 
 * Prioriteetti: Korkea (TASK_PRIORITY_HIGH)
 * Core: 1 (vapaa core)
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
  sensorsOutside.setWaitForConversion(false);  // EI-BLOKKAAVA lukeminen
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
  unsigned long requestTime = 0;  // Aikaleima viimeisestä pyynnöstä
  
  while (1) {
    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    switch (readState) {
      case IDLE:
        // Odota määritetty aika ennen seuraavaa lukemista
        vTaskDelay(pdMS_TO_TICKS(TEMP_READ_INTERVAL_MS));
        
        // Ilmoita watchdogille että olemme edelleen elossa
        xEventGroupSetBits(xSystemEvents, BIT_TEMP_TASK_ALIVE);
        
        // Aloita uusi mittaussykli
        sensorsOutside.requestTemperatures();
        readState = REQUEST_OUTSIDE;
        requestTime = currentTime;
        break;
        
      case REQUEST_OUTSIDE:
        // Odota että pyyntö on käsitelty (lyhyt viive)
        if (currentTime - requestTime > 100) {
          readState = WAIT_OUTSIDE;
        }
        break;
        
      case WAIT_OUTSIDE:
        // Odota että DS18B20 mittaus valmistuu (max 750ms 12-bitille)
        if (currentTime - requestTime > 750) {
          float outsideTemp = sensorsOutside.getTempCByIndex(0);
          
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
 * Säätölogiikan task.
 * 
 * Tämä task suorittaa polttimen automaattisen säätölogiikan.
 * Se tarkistaa lämpötilat ja päivittää polttimen tilan hystereesin mukaan.
 * 
 * Prioriteetti: Keskitaso (TASK_PRIORITY_MEDIUM)
 * Core: 1 (vapaa core)
 */
void controlTask(void *parameter) {
  Serial.println("Säätötask aloitettu");
  
  unsigned long lastControlTime = 0;  // Viimeisen säätösuorituksen aikaleima
  
  while (1) {
    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Suorita säätölogiikkaa sekunnin välein
    if (currentTime - lastControlTime >= 1000) {
      lastControlTime = currentTime;
      
      // Alustetaan säätöehdot
      bool canControl = true;          // Voiko automaattista säätöä tehdä
      float waterTempToUse = 0.0;      // Käytettävä vesilämpötila
      float targetTemp = 0.0;          // Tavoitelämpötila
      bool heatingDisabled = false;    // Onko lämmitys estetty
      
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
    
    vTaskDelay(pdMS_TO_TICKS(50));  // Anna muille säikeille ajoaikaa
  }
}

/**
 * Releen ohjauksen task.
 * 
 * Tämä task vastaanottaa relekomentoja jonosta ja ohjaa relettä turvallisesti.
 * Se tarkistaa turvaehdot ennen releen kytkemistä.
 * 
 * Prioriteetti: Korkea (TASK_PRIORITY_HIGH) - kriittinen turvallisuuden takia
 * Core: 1 (vapaa core)
 */
void relayTask(void *parameter) {
  Serial.println("Releetask aloitettu");
  
  // ALUSTA RELE TURVALLISEEN TILAAN
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
      
      bool shouldTurnOn = false;  // Lopullinen päätös releen tilasta
      
      // Käsittele komento tyypin mukaan
      switch (cmd.type) {
        case RelayCommand::SET_STATE:
          // Normaali tilan asetus - tarkista turvaehdot
          if (!anyFault && !heatingDisabled) {
            shouldTurnOn = cmd.state;
          } else {
            // Turvaehto estää käynnistyksen
            shouldTurnOn = false;
          }
          break;
          
        case RelayCommand::FORCE_OFF:
          // Pakotettu sammutus (esim. käyttäjä sammuttaa)
          shouldTurnOn = false;
          break;
          
        case RelayCommand::EMERGENCY_OFF:
          // Hätäsammutus - ohita kaikki logiikat ja sammuta välittömästi
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
      
      // OHJAA RELETTÄ
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
 * Watchdog-task.
 * 
 * Tämä task valvoo muita taskeja ja suorittaa hätätoimenpiteitä
 * jos jokin task lakkaa vastaamasta.
 * Käyttää ESP32:n hardware watchdogia.
 * 
 * Prioriteetti: Korkein (TASK_PRIORITY_HIGH + 1)
 * Core: 0 (samalla corella kuin Arduino loop)
 */
void watchdogTask(void *parameter) {
  Serial.println("Watchdog-task aloitettu");
  
  // Alusta ESP32:n hardware watchdog
  esp_task_wdt_init(30, true);  // 30 sekunnin timeout, panic reset
  esp_task_wdt_add(NULL);       // Lisää tämä task watchdogin valvontaan
  
  unsigned long lastTempTaskAlive = xTaskGetTickCount() * portTICK_PERIOD_MS;
  
  while (1) {
    // Tarkista onko lämpötilatask elossa (tarkistaa BIT_TEMP_TASK_ALIVE bitin)
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
    
    vTaskDelay(pdMS_TO_TICKS(1000));  // Tarkista 1 sekunnin välein
  }
}

/**
 * Flash-tallennuksen task (ei-blokkaava).
 * 
 * Tämä task tallentaa asetukset flash-muistiin viiveellä.
 * Estää liian useat flash-kirjoitukset, mikä pidentää flashin elinikää.
 * 
 * Prioriteetti: Matala (TASK_PRIORITY_LOW)
 * Core: 1 (vapaa core)
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
    
    vTaskDelay(pdMS_TO_TICKS(1000));  // Tarkista 1 sekunnin välein
  }
}

/**
 * Web-palvelin task.
 * 
 * Tämä task tarjoaa web-käyttöliittymän järjestelmän ohjaamiseen.
 * Se käsittelee HTTP-pyyntöjä ja palauttaa JSON-dataa sekä HTML-sivuja.
 * 
 * Prioriteetti: Matala (TASK_PRIORITY_LOW) - ei kriittinen
 * Core: 0 (samalla corella kuin WiFi)
 */
void webServerTask(void *parameter) {
  Serial.println("Web-palvelintask aloitettu");
  
  /**
   * Alusta WiFi Access Point.
   * Luo oman WiFi-verkon, johon yhdistetään selaimella.
   */
  WiFi.softAP(ssid, password);
  Serial.printf("WiFi Access Point luotu\n");
  Serial.printf("  SSID: %s\n", ssid);
  Serial.printf("  Salasana: %s\n", password);
  Serial.printf("  IP-osoite: %s\n", WiFi.softAPIP().toString().c_str());
  
  /**
   * HTML-käyttöliittymän pääsivu.
   * Sisältää JavaScriptin automaattiseen datan päivittämiseen.
   */

  
  /**
   / - HTTP GET -käsittelijä pääsivulle.
   * Palauttaa HTML-käyttöliittymän.
   */
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", htmlPage);
  });
  
  /**
   * /data - HTTP GET -käsittelijä JSON-datalle.
   * Kerää järjestelmän tilatiedot ja palauttaa ne JSON-muodossa.
   */
  server.on("/data", HTTP_GET, []() {
    // Kerää data suojatusti TIUKASSA MUTEX-JÄRJESTYKSESSÄ
    
    float outsideTemp = 0.0, waterTemp = 0.0, targetTemp = 0.0;
    bool heatingDisabled = false;
    String outsideStatus = "OK", waterStatus = "OK";
    
    // 1. Ota ensin sensorStatus, sitten temperature mutexit
    if (lockSensorAndTemperature()) {
      // Hae lämpötilat (käytä keskiarvoa jos vika)
      outsideTemp = systemState.outsideStatus.fault ? 
                   systemState.outsideData.getAverageValidValue() : 
                   systemState.outsideData.currentTemp;
      
      waterTemp = systemState.waterStatus.fault ? 
                  systemState.waterData.getAverageValidValue() : 
                  systemState.waterData.currentTemp;
      
      // Määritä anturien statukset
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
    
    // 2. Hae polttimen tila
    bool burnerState = false, manualMode = false;
    if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      burnerState = systemState.burnerState;
      manualMode = systemState.burnerManualOverride;
      xSemaphoreGive(xBurnerStateMutex);
    }
    
    /**
     * Muodosta JSON-vastaus.
     * Sisältää kaikki järjestelmän tilatiedot.
     */
    String json = "{";
    json += "\"outsideTemp\":" + String(outsideTemp, 1);
    json += ",\"waterTemp\":" + String(waterTemp, 1);
    json += ",\"targetTemp\":" + String(targetTemp, 1);
    json += ",\"burnerState\":" + String(burnerState ? "true" : "false");
    json += ",\"manualMode\":" + String(manualMode ? "true" : "false");
    json += ",\"outsideStatus\":\"" + outsideStatus + "\"";
    json += ",\"waterStatus\":\"" + waterStatus + "\"";
    json += ",\"heatingDisabled\":" + String(heatingDisabled ? "true" : "false");
    json += "}";
    
    server.send(200, "application/json", json);
  });
  
  /**
   * /control - HTTP GET -käsittelijä ohjauskomennoille.
   * Käsittelee polttimen ja toimintatilan muutokset.
   */
  server.on("/control", HTTP_GET, []() {
    // Käsittele manuaali/automaatti-tilan muutos
    if (server.hasArg("manual")) {
      bool manual = server.arg("manual").toInt() == 1;
      if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        systemState.burnerManualOverride = manual;
        xSemaphoreGive(xBurnerStateMutex);
      }
      Serial.printf("Manuaalitila asetettu: %s\n", manual ? "PÄÄLLÄ" : "POIS");
    }
    
    // Käsittele polttimen tilan muutos (vain manuaalitilassa)
    if (server.hasArg("burner")) {
      bool burner = server.arg("burner").toInt() == 1;
      bool manualMode = false;
      
      if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        manualMode = systemState.burnerManualOverride;
        if (manualMode) {
          systemState.burnerState = burner;
          
          // Lähetä komento releelle
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
  
  /**
   * /setcurve - HTTP GET -käsittelijä lämpötilakäyrän muutoksille.
   * Vastaanottaa uudet käyräpisteet ja tallentaa ne viiveellä.
   */
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
  
  /**
   * /emergency - HTTP GET -käsittelijä hätäsammutukselle.
   * Pakottaa polttimen välittömästi pois päältä.
   */
  server.on("/emergency", HTTP_GET, []() {
    RelayCommand cmd;
    cmd.type = RelayCommand::EMERGENCY_OFF;
    xQueueSend(xRelayControlQueue, &cmd, 0);
    
    Serial.println("Hätäsammutus suoritettu käyttäjän pyynnöstä");
    server.send(200, "text/plain", "Hätäsammutus suoritettu");
  });
  
  /**
   * Käynnistä web-palvelin.
   * Aloittaa HTTP-palvelimen kuuntelemaan porttia 80.
   */
  server.begin();
  Serial.println("HTTP-palvelin käynnistetty portissa 80");
  
  /**
   * Web-palvelimen pääsilmukka.
   * Käsittelee saapuvat HTTP-pyynnöt ja antaa ajoaikaa muille säikeille.
   */
  while (1) {
    server.handleClient();      // Käsittele HTTP-pyynnöt
    vTaskDelay(pdMS_TO_TICKS(10)); // Anna muille säikeille ajoaikaa
  }
}

// ==================== SETUP ====================

/**
 * Arduino setup()-funktio.
 * Suoritetaan kerran käynnistyessä.
 * Alustaa kaikki järjestelmän komponentit ja käynnistää säikeet.
 */
void setup() {
  // Alusta sarjamonitori debug-tulostusta varten
  Serial.begin(115200);
  delay(1000);  // Odota että sarjamonitori on valmis
  
  Serial.println("\n\n=== ÖLJYLÄMMITYSJÄRJESTELMÄ - FreeRTOS ===\n");
  Serial.println("Järjestelmä käynnistyy...");
  
  /**
   * 1. LUO MUTEXIT TIUKALLA JÄRJESTYKSLLÄ
   * Deadlockin välttämiseksi käytetään aina samaa järjestystä.
   */
  xSensorStatusMutex = xSemaphoreCreateMutex();
  xTemperatureMutex = xSemaphoreCreateMutex();
  xBurnerStateMutex = xSemaphoreCreateMutex();
  xSettingsMutex = xSemaphoreCreateMutex();
  
  if (xSensorStatusMutex == NULL || xTemperatureMutex == NULL || 
      xBurnerStateMutex == NULL || xSettingsMutex == NULL) {
    Serial.println("VIRHE: Mutexien luonti epäonnistui!");
    while(1);  // Pysähdy
  }
  
  /**
   * 2. LUO EVENT GROUP
   * Käytetään tehokkaaseen tilanvaihtojen ilmoittamiseen.
   */
  xSystemEvents = xEventGroupCreate();
  if (xSystemEvents == NULL) {
    Serial.println("VIRHE: Event Groupin luonti epäonnistui!");
    while(1);
  }
  
  /**
   * 3. LUO JONO
   * Releen ohjauskommunikaatioon säikeiden välillä.
   */
  xRelayControlQueue = xQueueCreate(10, sizeof(RelayCommand));
  if (xRelayControlQueue == NULL) {
    Serial.println("VIRHE: Jonon luonti epäonnistui!");
    while(1);
  }
  
  /**
   * 4. LATAA ASETUKSET FLASH-MUISTISTA
   * Käyttää Preferences-kirjastoa pysyvään tallennukseen.
   */
  preferences.begin("oilheater", false);
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
   * 5. LUO SÄIKEET (TASKS)
   * Jokaiselle toiminnolle oma säie oikealla prioriteetilla.
   */
  
  // Watchdog-task (korkein prioriteetti)
  xTaskCreatePinnedToCore(
    watchdogTask,              // Task-funktio
    "WatchdogTask",           // Taskin nimi (debuggausta varten)
    TASK_STACK_SMALL,         // Stack koko
    NULL,                     // Parametrit
    TASK_PRIORITY_HIGH + 1,   // Prioriteetti (korkein)
    &xWatchdogTaskHandle,     // Task handle
    0                         // Core 0
  );
  
  // Rele-task (korkea prioriteetti - turvallisuuskriittinen)
  xTaskCreatePinnedToCore(
    relayTask,
    "RelayTask",
    TASK_STACK_SMALL,
    NULL,
    TASK_PRIORITY_HIGH,
    &xRelayTaskHandle,
    1                         // Core 1
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
  
  // Web-palvelintask (ei-kriittinen)
  xTaskCreatePinnedToCore(
    webServerTask,
    "WebServerTask",
    TASK_STACK_LARGE,         // Suuri stack web-palvelimelle
    NULL,
    TASK_PRIORITY_LOW,
    &xWebServerTaskHandle,
    0                         // Core 0 (sama kuin WiFi)
  );
  
  Serial.println("Kaikki säikeet luotu onnistuneesti");
  Serial.println("\nJärjestelmä valmiina käyttöön!");
  Serial.println("Yhdistä WiFi-verkkoon: " + String(ssid));
  Serial.println("Avaa selaimessa: http://" + WiFi.softAPIP().toString());
}

// ==================== LOOP ====================

/**
 * Arduino loop()-funktio.
 * Perinteinen Arduino-silmukka, joka ajetaan core 0:lla.
 * Vapautetaan pääosin muille säikeille, koska kaikki toiminta on taskeissa.
 */
void loop() {
  /**
   * Perus-Arduino loop jää tyhjäksi, koska kaikki toiminta on siirretty taskeihin.
   * Tämä säikeen aikaa voidaan käyttää ei-kriittisille taustatehtäville.
   */
  
  // Vapauta ajoaikaa muille säikeille
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  // Voit lisätä tähän muita ei-kriittisiä tehtäviä core 0:lle
  // Esimerkiksi:
  // - LEDin vilkutus (järjestelmän elossa-indikaattori)
  // - Lisädebug-tulostus
  // - Ylimääräiset sensorit
}
