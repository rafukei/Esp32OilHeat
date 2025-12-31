/**
 * OIL HEATING SYSTEM - FreeRTOS + WiFi
 * 
 * This program controls an oil heating system using ESP32.
 * Features:
 * - Temperature measurement with DS18B20 sensors
 * - Automatic control with hysteresis
 * - Web interface for control
 * - WiFi management (own network or connect to existing network)
 * - Temperature curve adjustment
 * - Safety measures for sensor errors
 * - Watchdog monitoring
 * - Settings saved to flash memory
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

#define DEBUG_TEMP 1  // Debug output for temperatures

// ==================== CONFIGURATION ====================

// GPIO pin definitions
#define OUTSIDE_SENSOR_PIN 23    // Outside temperature sensor
#define WATER_SENSOR_PIN 22      // Water temperature sensor
#define RELAY_PIN 2              // Relay control

// Control parameters (configurable via web interface)
#define DEFAULT_HYSTERESIS 4.0   // Default hysteresis value (°C)

// Sensor error detection
#define SENSOR_FAULT_TEMP -50.0         // Temperature below this = error
#define SENSOR_FAULT_TIMEOUT 30000      // 30 seconds before serious error
#define MAX_CONSECUTIVE_FAILURES 3      // Allowed consecutive errors
#define TEMP_READ_INTERVAL_MS 5000      // Temperature reading interval (ms)

// WiFi connection parameters
#define WIFI_CONNECT_TIMEOUT 10000      // 10 seconds connection timeout
#define WIFI_RECONNECT_INTERVAL 30000   // 30 seconds reconnect interval

// Task priorities (higher = more important)
#define TASK_PRIORITY_HIGH 3            // Critical tasks
#define TASK_PRIORITY_MEDIUM 2          // Normal tasks
#define TASK_PRIORITY_LOW 1             // Background tasks

// Task stack sizes
#define TASK_STACK_SMALL 2048           // Small tasks
#define TASK_STACK_MEDIUM 3072          // Medium tasks
#define TASK_STACK_LARGE 4096           // Large tasks (web server)

// Event Group bits for system states
#define BIT_SENSOR_FAULT_OUTSIDE (1 << 0)  // Outside sensor serious error
#define BIT_SENSOR_FAULT_WATER   (1 << 1)  // Water sensor serious error
#define BIT_HEATING_DISABLED     (1 << 2)  // Heating disabled (too warm)
#define BIT_TEMP_TASK_ALIVE      (1 << 3)  // Temperature task alive (watchdog)

// ==================== GLOBAL RESOURCES ====================

// Mutexes (protected resources)
SemaphoreHandle_t xSensorStatusMutex;   // Sensor status data
SemaphoreHandle_t xTemperatureMutex;    // Temperature measurements
SemaphoreHandle_t xBurnerStateMutex;    // Burner state
SemaphoreHandle_t xSettingsMutex;       // Settings and curves
SemaphoreHandle_t xWifiConfigMutex;     // WiFi settings

// Queue for relay control commands
QueueHandle_t xRelayControlQueue;

// Event Group for system states
EventGroupHandle_t xSystemEvents;

// Task handles (references to threads)
TaskHandle_t xTemperatureTaskHandle = NULL;  // Temperature reader
TaskHandle_t xControlTaskHandle = NULL;      // Control logic
TaskHandle_t xWebServerTaskHandle = NULL;    // Web server
TaskHandle_t xRelayTaskHandle = NULL;        // Relay control
TaskHandle_t xWatchdogTaskHandle = NULL;     // Watchdog monitor
TaskHandle_t xFlashTaskHandle = NULL;        // Flash storage

// Sensor objects for DS18B20 sensors
OneWire oneWireOutside(OUTSIDE_SENSOR_PIN);
OneWire oneWireWater(WATER_SENSOR_PIN);
DallasTemperature sensorsOutside(&oneWireOutside);
DallasTemperature sensorsWater(&oneWireWater);

// Web server object on port 80
WebServer server(80);

// Persistent memory (flash) handling
Preferences preferences;

// HTML page (defined later)
extern const char* htmlPage;

// ==================== DATA STRUCTURES ====================

/**
 * WiFi configuration
 * Stores both own network and possible external network details
 */
struct WifiConfig {
  char ap_ssid[32] = "OilHeaterAP";      // Own network name
  char ap_password[32] = "enmuista";     // Own network password
  char sta_ssid[32] = "";                // External network name
  char sta_password[32] = "";            // External network password
  bool use_sta = false;                  // Use external network?
  bool connected = false;                // Is connection active?
  bool ap_mode = false;                  // Are we in AP mode?
};

/**
 * Temperature curve points
 * 4 points for linear interpolation
 */
struct CurvePoints {
  float points[4][2] = {
    {-20.0, 75.0},   // -20°C outside -> 75°C water
    {-10.0, 65.0},   // -10°C outside -> 65°C water
    {0.0, 55.0},     // 0°C outside -> 55°C water
    {10.0, 45.0}     // 10°C outside -> 45°C water
  };
};

/**
 * System settings that can be changed via web interface
 */
struct SystemSettings {
  float hysteresis = DEFAULT_HYSTERESIS;  // Hysteresis for temperature control (°C)
  bool settingsDirty = false;             // Have settings been changed?
  unsigned long lastSettingsChange = 0;   // Time of last change
};

/**
 * Sensor temperature data
 * Contains current and historical data
 */
struct SensorData {
  float currentTemp = 0.0;               // Latest measured temperature
  float lastValidTemp = 0.0;             // Latest valid temperature
  unsigned long lastValidReadTime = 0;   // Time of last valid reading
  
  // 10-value moving average for error situations
  float lastValidValues[10] = {0};
  int validValueIndex = 0;
  int validValueCount = 0;
  
  /**
   * Add new valid value to history
   * @param value Valid temperature
   */
  void addValidValue(float value) {
    lastValidValues[validValueIndex] = value;
    validValueIndex = (validValueIndex + 1) % 10;
    if (validValueCount < 10) validValueCount++;
  }
  
  /**
   * Calculate average from historical data
   * @return Temperature average
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
 * Sensor status information
 * Tracks error count and duration
 */
struct SensorStatus {
  bool fault = false;                    // Serious error
  bool temporaryFault = false;           // Temporary disturbance
  unsigned long faultStartTime = 0;      // Error start time
  int consecutiveFailures = 0;           // Consecutive errors
};

/**
 * Complete system state
 * All sub-data protected with their own mutexes
 */
struct SystemState {
  // Temperature data (protected by: xTemperatureMutex)
  float targetWaterTemp = 0.0;           // Calculated target temperature
  bool heatingDisabled = false;          // Heating disabled?
  
  // Burner state (protected by: xBurnerStateMutex)
  bool burnerState = false;              // Burner on/off
  bool burnerManualOverride = false;     // Manual mode active?
  
  // Sensor data (protected by: xSensorStatusMutex)
  SensorData outsideData;                // Outside sensor data
  SensorData waterData;                  // Water sensor data
  SensorStatus outsideStatus;            // Outside sensor status
  SensorStatus waterStatus;              // Water sensor status
  
  // Settings (protected by: xSettingsMutex)
  CurvePoints curve;                     // Temperature curve
  SystemSettings settings;               // System settings (hysteresis, etc.)
  
  // WiFi configuration (protected by: xWifiConfigMutex)
  WifiConfig wifiConfig;                 // WiFi configuration
};

// Global system state
SystemState systemState;

/**
 * Relay control message
 * Sent via queue to relay control task
 */
struct RelayCommand {
  enum CommandType {
    SET_STATE,      // Normal state setting
    FORCE_OFF,      // Forced shutdown
    EMERGENCY_OFF   // Emergency shutdown
  };
  
  CommandType type;  // Command type
  bool state;        // Desired state (for SET_STATE only)
};

// ==================== WIFI MANAGEMENT ====================

/**
 * Initialize WiFi connection
 * - Load settings from flash
 * - Try to connect to external network if STA enabled
 * - Create own network (AP) if STA not enabled or connection fails
 * @return true if WiFi initialized successfully
 */
bool initWiFi() {
  Serial.println("Initializing WiFi...");
  
  // Load WiFi settings from flash memory
  preferences.begin("oilheater", true);
  preferences.getString("ap_ssid", systemState.wifiConfig.ap_ssid, 32);
  preferences.getString("ap_password", systemState.wifiConfig.ap_password, 32);
  preferences.getString("sta_ssid", systemState.wifiConfig.sta_ssid, 32);
  preferences.getString("sta_password", systemState.wifiConfig.sta_password, 32);
  systemState.wifiConfig.use_sta = preferences.getBool("use_sta", false);
  preferences.end();
  
  Serial.printf("AP SSID: %s\n", systemState.wifiConfig.ap_ssid);
  Serial.printf("STA enabled: %s\n", systemState.wifiConfig.use_sta ? "YES" : "NO");
  
  bool connected = false;
  
  // Try to connect to external network if STA enabled
  if (systemState.wifiConfig.use_sta && strlen(systemState.wifiConfig.sta_ssid) > 0) {
    Serial.printf("Trying to connect to network: %s\n", systemState.wifiConfig.sta_ssid);
    
    // Disconnect any old connection
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    
    // Try to connect
    WiFi.begin(systemState.wifiConfig.sta_ssid, systemState.wifiConfig.sta_password);
    
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_CONNECT_TIMEOUT) {
      delay(500);
      Serial.print(".");
    }
    
    // Check if connection succeeded
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      systemState.wifiConfig.connected = true;
      systemState.wifiConfig.ap_mode = false;
      Serial.println("\nConnected to network!");
      Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("\nNetwork connection failed");
      WiFi.disconnect(true);
    }
  }
  
  // If STA not enabled or connection failed, create own network (AP)
  if (!connected) {
    Serial.println("Creating own network (Access Point)...");
    
    // Shut down WiFi completely
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    
    // Ensure AP settings are correct
    if (strlen(systemState.wifiConfig.ap_ssid) == 0) {
      strcpy(systemState.wifiConfig.ap_ssid, "OilHeaterAP");
    }
    if (strlen(systemState.wifiConfig.ap_password) == 0) {
      strcpy(systemState.wifiConfig.ap_password, "enmuista");
    }
    
    // Set static IP address for AP
    IPAddress local_ip(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    
    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.softAP(systemState.wifiConfig.ap_ssid, systemState.wifiConfig.ap_password);
    
    // Update status information
    systemState.wifiConfig.connected = true;
    systemState.wifiConfig.ap_mode = true;
    
    Serial.println("Own network created successfully!");
    Serial.printf("SSID: %s\n", systemState.wifiConfig.ap_ssid);
    Serial.printf("IP address: %s\n", WiFi.softAPIP().toString().c_str());
  }
  
  return true;
}

/**
 * Save WiFi settings to flash memory
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
  Serial.println("WiFi settings saved to flash memory");
}

/**
 * Reconnect to network
 * Used when WiFi settings are changed or connection is lost
 */
void reconnectWiFi() {
  Serial.println("Reconnecting to network...");
  
  // Shut down current connection
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  
  // Reinitialize WiFi
  initWiFi();
  
  // If web server is running, it will continue working with new IP address
  Serial.println("Connection reinitialized");
}

// ==================== HELPER FUNCTIONS ====================

/**
 * Lock sensorStatus and temperature mutexes in strict order
 * @return true if mutexes acquired, false if timeout
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
 * Release sensorStatus and temperature mutexes
 */
void unlockSensorAndTemperature() {
  xSemaphoreGive(xTemperatureMutex);
  xSemaphoreGive(xSensorStatusMutex);
}

/**
 * Check if temperature value is faulty
 * @param temperature Temperature to check
 * @param isWaterSensor True if water sensor
 * @return true if faulty, false if OK
 */
bool isTemperatureFaulty(float temperature, bool isWaterSensor) {
  // DS18B20 returns this when sensor not found
  if (temperature == DEVICE_DISCONNECTED_C) {
    return true;
  }
  
  // Too low temperature
  if (temperature < SENSOR_FAULT_TEMP) {
    return true;
  }
  
  // Too high temperatures (sensor type specific)
  if (isWaterSensor) {
    if (temperature > 120.0) return true; // Water shouldn't be over 120°C
  } else {
    if (temperature > 100.0) return true; // Outside shouldn't be over 100°C
  }
  
  return false;
}

/**
 * Update sensor status based on new measurement
 * @param status Sensor status structure
 * @param data Sensor temperature data
 * @param newValue New measured temperature
 * @param isWaterSensor True if water sensor
 */
void updateSensorStatus(SensorStatus& status, SensorData& data, 
                       float newValue, bool isWaterSensor) {
  unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
  bool isFaulty = isTemperatureFaulty(newValue, isWaterSensor);
  
  if (isFaulty) {
    // FAULTY READING
    status.consecutiveFailures++;
    
    // First error -> start timer
    if (status.consecutiveFailures == 1) {
      status.faultStartTime = currentTime;
    }
    
    // Reached maximum error count -> temporary disturbance
    if (status.consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
      status.temporaryFault = true;
    }
    
    // Error lasted over 30s -> serious error
    if (currentTime - status.faultStartTime > SENSOR_FAULT_TIMEOUT && 
        status.consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
      status.fault = true;
      status.temporaryFault = false;
      
      // Set corresponding bit in Event Group
      EventBits_t bits = BIT_SENSOR_FAULT_OUTSIDE;
      if (isWaterSensor) bits = BIT_SENSOR_FAULT_WATER;
      xEventGroupSetBits(xSystemEvents, bits);
    }
  } else {
    // ACCEPTED READING
    data.currentTemp = newValue;
    data.lastValidTemp = newValue;
    data.lastValidReadTime = currentTime;
    data.addValidValue(newValue);

    // Reset all errors
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
 * Calculate target temperature for water based on outside temperature
 * Uses 4-point linear curve
 * @param outsideTemp Current outside temperature
 * @param curve Temperature curve
 * @return Target temperature for water, 0.0 if heating disabled
 */
float calculateTargetTemperature(float outsideTemp, const CurvePoints& curve) {
  // If outside temperature above warmest point (10°C)
  if (outsideTemp >= curve.points[3][0]) {
    return 0.0;  // Heating off
  }
  
  // If outside temperature below coldest point (-20°C)
  if (outsideTemp <= curve.points[0][0]) {
    return curve.points[0][1];  // Maximum temperature
  }
  
  // Find correct segment on curve
  for (int i = 0; i < 3; i++) {
    if (outsideTemp >= curve.points[i][0] && outsideTemp <= curve.points[i+1][0]) {
      // Linear interpolation between two points
      float ratio = (outsideTemp - curve.points[i][0]) / 
                   (curve.points[i+1][0] - curve.points[i][0]);
      return curve.points[i][1] + ratio * (curve.points[i+1][1] - curve.points[i][1]);
    }
  }
  
  return 45.0; // Default value
}

/**
 * Save system settings to flash memory
 * FIXED: Uses proper keys for curve points
 */
void saveSystemSettings() {
  Serial.println("Saving system settings to flash...");
  
  preferences.begin("oilheater", false);
  
  if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Save temperature curve with unique keys
    for (int i = 0; i < 4; i++) {
      String keyOut = "curve_out" + String(i);
      String keyWater = "curve_water" + String(i);
      
      preferences.putFloat(keyOut.c_str(), systemState.curve.points[i][0]);
      preferences.putFloat(keyWater.c_str(), systemState.curve.points[i][1]);
      
      Serial.printf("Saved curve point %d: %.1f->%.1f\n", 
                   i, systemState.curve.points[i][0], systemState.curve.points[i][1]);
    }
    
    // Save system settings
    preferences.putFloat("hysteresis", systemState.settings.hysteresis);
    
    // Clear dirty flag after saving
    systemState.settings.settingsDirty = false;
    
    xSemaphoreGive(xSettingsMutex);
  }
  
  preferences.end();
  Serial.println("System settings saved to flash memory");
}

/**
 * Load system settings from flash memory
 * FIXED: Properly checks if values exist and loads them
 */
void loadSystemSettings() {
  Serial.println("Loading system settings from flash...");
  
  preferences.begin("oilheater", true);
  
  if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    bool curveLoaded = false;
    
    // Load temperature curve
    for (int i = 0; i < 4; i++) {
      String keyOut = "curve_out" + String(i);
      String keyWater = "curve_water" + String(i);
      
      // Try to load from flash (use -999 as sentinel for "not found")
      float outTemp = preferences.getFloat(keyOut.c_str(), -999.0);
      float waterTemp = preferences.getFloat(keyWater.c_str(), -999.0);
      
      if (outTemp != -999.0 && waterTemp != -999.0) {
        // Valid data found in flash
        systemState.curve.points[i][0] = outTemp;
        systemState.curve.points[i][1] = waterTemp;
        curveLoaded = true;
        Serial.printf("Loaded curve point %d: %.1f->%.1f\n", i, outTemp, waterTemp);
      } else {
        // Use default values
        Serial.printf("Using default for curve point %d: %.1f->%.1f\n", 
                     i, systemState.curve.points[i][0], systemState.curve.points[i][1]);
      }
    }
    
    if (curveLoaded) {
      Serial.println("Temperature curve loaded from flash");
    } else {
      Serial.println("Using default temperature curve");
    }
    
    // Load system settings
    float loadedHysteresis = preferences.getFloat("hysteresis", -999.0);
    if (loadedHysteresis != -999.0) {
      systemState.settings.hysteresis = loadedHysteresis;
      Serial.printf("Loaded hysteresis: %.1f\n", loadedHysteresis);
    } else {
      Serial.printf("Using default hysteresis: %.1f\n", DEFAULT_HYSTERESIS);
    }
    
    xSemaphoreGive(xSettingsMutex);
  }
  
  preferences.end();
  Serial.println("System settings loaded from flash memory");
}

// ==================== TASKS ====================

/**
 * Temperature reading task
 * Reads DS18B20 sensors non-blockingly using state machine
 * Updates sensor statuses and calculates target temperature
 */
void temperatureTask(void *parameter) {
  Serial.println("Temperature task started");
  
  // Notify watchdog task that we're alive
  xEventGroupSetBits(xSystemEvents, BIT_TEMP_TASK_ALIVE);
  
  // Initialize DS18B20 sensors
  sensorsOutside.begin();
  sensorsWater.begin();
  sensorsOutside.setResolution(12);     // 12-bit accuracy (0.0625°C)
  sensorsWater.setResolution(12);
  sensorsOutside.setWaitForConversion(false);  // Non-blocking reading
  sensorsWater.setWaitForConversion(false);
  
  // State machine for temperature reading
  enum ReadState { 
    IDLE,            // Wait for reading interval
    REQUEST_OUTSIDE, // Send request to outside sensor
    WAIT_OUTSIDE,    // Wait for outside measurement to complete
    READ_OUTSIDE,    // Read outside sensor result
    REQUEST_WATER,   // Send request to water sensor
    WAIT_WATER,      // Wait for water measurement to complete
    READ_WATER       // Read water sensor result
  };
  
  ReadState readState = IDLE;
  unsigned long requestTime = 0;
  
  while (1) {
    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    switch (readState) {
      case IDLE:
        // Wait specified time before next reading
        vTaskDelay(pdMS_TO_TICKS(TEMP_READ_INTERVAL_MS));
        
        // Notify watchdog that we're still alive
        xEventGroupSetBits(xSystemEvents, BIT_TEMP_TASK_ALIVE);
        
        // Check if sensors are found
        if (sensorsOutside.getDeviceCount() == 0) {
          Serial.println("WARNING: Outside sensor not found!");
          sensorsOutside.begin();
        }
        if (sensorsWater.getDeviceCount() == 0) {
          Serial.println("WARNING: Water sensor not found!");
          sensorsWater.begin();
        }
        
        // Start new measurement cycle
        sensorsOutside.requestTemperatures();
        readState = REQUEST_OUTSIDE;
        requestTime = currentTime;
        break;
        
      case REQUEST_OUTSIDE:
        // Wait for request to be processed
        if (currentTime - requestTime > 100) {
          readState = WAIT_OUTSIDE;
        }
        break;
        
      case WAIT_OUTSIDE:
        // Wait for DS18B20 measurement to complete (max 750ms for 12-bit)
        if (currentTime - requestTime > 750) {
          float outsideTemp = sensorsOutside.getTempCByIndex(0);
          
          #if DEBUG_TEMP
          Serial.printf("[TEMP] OUTSIDE: %.2f °C\n", outsideTemp);
          #endif
          
          // Update outside sensor status safely
          if (xSemaphoreTake(xSensorStatusMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            updateSensorStatus(systemState.outsideStatus, systemState.outsideData, 
                              outsideTemp, false);
            xSemaphoreGive(xSensorStatusMutex);
          }
          
          // Start water sensor measurement
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
          
          // Update water sensor status safely
          if (xSemaphoreTake(xSensorStatusMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            updateSensorStatus(systemState.waterStatus, systemState.waterData, 
                              waterTemp, true);
            xSemaphoreGive(xSensorStatusMutex);
          }
          
          // Calculate target temperature safely
          if (lockSensorAndTemperature()) {
            // Choose outside temperature to use (average if faulty)
            float outsideTempToUse = systemState.outsideStatus.fault ? 
                                    systemState.outsideData.getAverageValidValue() : 
                                    systemState.outsideData.currentTemp;
            
            // Get curve points and calculate target
            if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
              float targetTemp = calculateTargetTemperature(outsideTempToUse, systemState.curve);
              
              // Update system state
              systemState.targetWaterTemp = targetTemp;
              systemState.heatingDisabled = (outsideTempToUse >= systemState.curve.points[3][0]);
              
              // Set/clear Event Group bit for heating status
              if (systemState.heatingDisabled) {
                xEventGroupSetBits(xSystemEvents, BIT_HEATING_DISABLED);
              } else {
                xEventGroupClearBits(xSystemEvents, BIT_HEATING_DISABLED);
              }
              
              xSemaphoreGive(xSettingsMutex);
            }
            
            unlockSensorAndTemperature();
          }
          
          // Return to idle state
          readState = IDLE;
        }
        break;
    }
    
    // Give other threads time to run
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
 * Control logic task
 * FIXED: Simplified logic that properly turns off burner when temperature is too high
 */
void controlTask(void *parameter) {
  Serial.println("Control task started");
  
  unsigned long lastControlTime = 0;
  
  while (1) {
    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Execute control logic every second
    if (currentTime - lastControlTime >= 1000) {
      lastControlTime = currentTime;
      
      // Get all needed data safely
      float waterTempToUse = 0.0;
      float targetTemp = 0.0;
      float hysteresis = DEFAULT_HYSTERESIS;
      bool manualMode = false;
      bool currentBurnerState = false;
      bool heatingDisabled = false;
      
      // Check system states from Event Group
      EventBits_t events = xEventGroupGetBits(xSystemEvents);
      bool anyFault = (events & (BIT_SENSOR_FAULT_OUTSIDE | BIT_SENSOR_FAULT_WATER)) != 0;
      heatingDisabled = (events & BIT_HEATING_DISABLED) != 0;
      
      // Get temperatures and settings
      if (lockSensorAndTemperature()) {
        targetTemp = systemState.targetWaterTemp;
        
        // Choose water temperature to use based on error situations
        if (systemState.waterStatus.fault) {
          waterTempToUse = systemState.waterData.getAverageValidValue();
        } else if (systemState.waterStatus.temporaryFault) {
          waterTempToUse = systemState.waterData.lastValidTemp;
        } else {
          waterTempToUse = systemState.waterData.currentTemp;
        }
        
        unlockSensorAndTemperature();
      }
      
      // Get hysteresis
      if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        hysteresis = systemState.settings.hysteresis;
        xSemaphoreGive(xSettingsMutex);
      }
      
      // Get burner state and manual mode
      if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        currentBurnerState = systemState.burnerState;
        manualMode = systemState.burnerManualOverride;
        xSemaphoreGive(xBurnerStateMutex);
      }
      
      // SIMPLE CONTROL LOGIC
      bool canControl = !anyFault && !manualMode;
      bool newBurnerState = currentBurnerState;
      
      if (canControl) {
        // CONDITION 1: TURN OFF if water is too hot
        if (currentBurnerState && waterTempToUse > (targetTemp + hysteresis)) {
          newBurnerState = false;
          Serial.printf("[CONTROL] Turning OFF: Water %.1f > Target %.1f + Hyst %.1f\n", 
                       waterTempToUse, targetTemp, hysteresis);
        }
        // CONDITION 2: TURN OFF if heating disabled (targetTemp = 0)
        else if (currentBurnerState && targetTemp <= 0) {
          newBurnerState = false;
          Serial.println("[CONTROL] Turning OFF: Heating disabled (target <= 0)");
        }
        // CONDITION 3: TURN ON if water is too cold AND heating is enabled
        else if (!currentBurnerState && targetTemp > 0 && waterTempToUse < (targetTemp - hysteresis)) {
          newBurnerState = true;
          Serial.printf("[CONTROL] Turning ON: Water %.1f < Target %.1f - Hyst %.1f\n", 
                       waterTempToUse, targetTemp, hysteresis);
        }
        
        // If state changed, update it
        if (newBurnerState != currentBurnerState) {
          // Update burner state
          if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            systemState.burnerState = newBurnerState;
            xSemaphoreGive(xBurnerStateMutex);
          }
          
          // Send command to relay control task
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
 * Relay control task
 * Receives relay commands from queue and controls relay safely
 * Checks safety conditions before switching relay
 */
void relayTask(void *parameter) {
  Serial.println("Relay task started");
  
  // Initialize relay to safe state
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // LOW = relay open = burner off = SAFE STATE
  
  RelayCommand cmd;
  
  while (1) {
    // Wait for command from queue (blocking wait)
    if (xQueueReceive(xRelayControlQueue, &cmd, portMAX_DELAY)) {
      
      // Check system states from Event Group
      EventBits_t events = xEventGroupGetBits(xSystemEvents);
      bool anyFault = (events & (BIT_SENSOR_FAULT_OUTSIDE | BIT_SENSOR_FAULT_WATER)) != 0;
      bool heatingDisabled = (events & BIT_HEATING_DISABLED) != 0;
      
      bool shouldTurnOn = false;
      
      // Handle command based on type
      switch (cmd.type) {
        case RelayCommand::SET_STATE:
          // Normal state setting - check safety conditions
          if (!anyFault && !heatingDisabled) {
            shouldTurnOn = cmd.state;
          } else {
            shouldTurnOn = false;
          }
          break;
          
        case RelayCommand::FORCE_OFF:
          // Forced shutdown
          shouldTurnOn = false;
          break;
          
        case RelayCommand::EMERGENCY_OFF:
          // Emergency shutdown - bypass all logic
          shouldTurnOn = false;
          break;
      }
      
      // Special check: manual mode can override error situations
      if (!shouldTurnOn && cmd.type == RelayCommand::SET_STATE) {
        bool manualMode = false;
        if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          manualMode = systemState.burnerManualOverride;
          xSemaphoreGive(xBurnerStateMutex);
        }
        
        // In manual mode user can turn on even if there's an error
        if (manualMode && cmd.state) {
          shouldTurnOn = true;
        }
      }
      
      // Control relay
      digitalWrite(RELAY_PIN, shouldTurnOn ? HIGH : LOW);
      
      // Update burner state in memory
      if (cmd.type == RelayCommand::SET_STATE) {
        if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          systemState.burnerState = shouldTurnOn;
          xSemaphoreGive(xBurnerStateMutex);
        }
      }
      
      Serial.printf("[RELAY] State: %s\n", shouldTurnOn ? "ON" : "OFF");
    }
  }
}

/**
 * Watchdog task
 * Monitors other tasks and performs emergency actions
 * if any task stops responding
 */
void watchdogTask(void *parameter) {
  Serial.println("Watchdog task started");
  
  // Initialize ESP32 hardware watchdog
  esp_task_wdt_init(30, true);  // 30 second timeout, panic reset
  esp_task_wdt_add(NULL);       // Add this task to watchdog monitoring
  
  unsigned long lastTempTaskAlive = xTaskGetTickCount() * portTICK_PERIOD_MS;
  
  while (1) {
    // Check if temperature task is alive
    EventBits_t events = xEventGroupGetBits(xSystemEvents);
    if (events & BIT_TEMP_TASK_ALIVE) {
      // Task is alive, update timestamp
      lastTempTaskAlive = xTaskGetTickCount() * portTICK_PERIOD_MS;
      xEventGroupClearBits(xSystemEvents, BIT_TEMP_TASK_ALIVE);
    }
    
    // If temperature task hasn't been alive for 60 seconds
    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (currentTime - lastTempTaskAlive > 60000) {
      Serial.println("WATCHDOG: Temperature task not responding - emergency shutdown!");
      
      // Send emergency shutdown command to relay
      RelayCommand cmd;
      cmd.type = RelayCommand::EMERGENCY_OFF;
      xQueueSend(xRelayControlQueue, &cmd, 0);
      
      // Reset timestamp
      lastTempTaskAlive = currentTime;
    }
    
    // Reset hardware watchdog
    esp_task_wdt_reset();
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
 * Flash storage task (non-blocking)
 * Saves settings to flash memory with delay
 * Prevents too many flash writes, extending flash lifespan
 */
void flashTask(void *parameter) {
  Serial.println("Flash task started");
  
  while (1) {
    bool shouldSave = false;
    
    // Check if settings need to be saved
    if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (systemState.settings.settingsDirty) {
        unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Wait 5 seconds from last change
        if (currentTime - systemState.settings.lastSettingsChange > 5000) {
          shouldSave = true;
          systemState.settings.settingsDirty = false;
        }
      }
      xSemaphoreGive(xSettingsMutex);
    }
    
    // Save to flash memory
    if (shouldSave) {
      saveSystemSettings();
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ==================== HTML PAGE ====================

const char* htmlPage = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Oil Heating Control</title>
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
    .settings-box {
      background: #fce4ec;
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
    
    // Switch tab
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
    
    // Update system data
    function updateData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          // Temperatures
          document.getElementById('outsideTemp').innerText = data.outsideTemp.toFixed(1) + ' °C';
          document.getElementById('waterTemp').innerText = data.waterTemp.toFixed(1) + ' °C';
          document.getElementById('targetTemp').innerText = data.targetTemp.toFixed(1) + ' °C';
          
          // Sensor statuses
          document.getElementById('outsideStatus').innerText = data.outsideStatus;
          document.getElementById('outsideStatus').className = data.outsideStatus === 'ERROR' ? 'fault' : 
                                                              data.outsideStatus === 'WARNING' ? 'warning' : 'ok';
          document.getElementById('waterStatus').innerText = data.waterStatus;
          document.getElementById('waterStatus').className = data.waterStatus === 'ERROR' ? 'fault' : 
                                                            data.waterStatus === 'WARNING' ? 'warning' : 'ok';
          
          // Burner
          document.getElementById('burnerState').innerText = data.burnerState ? 'ON' : 'OFF';
          document.getElementById('burnerMode').innerText = data.manualMode ? 'Manual' : 'Automatic';
          document.getElementById('heatingDisabled').innerText = data.heatingDisabled ? 'YES' : 'NO';
          
          // WiFi
          document.getElementById('wifiMode').innerText = data.wifiMode;
          document.getElementById('wifiIP').innerText = data.wifiIP;
          
          // Hysteresis
          document.getElementById('currentHysteresis').innerText = data.hysteresis.toFixed(1) + ' °C';
        })
        .catch(error => {
          console.error('Error fetching data:', error);
        });
    }
    
    // Set manual/automatic mode
    function setManualState(state) {
      fetch('/control?manual=' + (state ? '1' : '0'))
        .then(() => updateData());
    }
    
    // Set burner state (manual mode only)
    function setBurnerState(state) {
      fetch('/control?burner=' + (state ? '1' : '0'))
        .then(() => updateData());
    }
    
    // Emergency stop
    function emergencyStop() {
      if (confirm('Are you sure you want to perform emergency shutdown?')) {
        fetch('/emergency')
          .then(() => {
            alert('Emergency shutdown performed!');
            updateData();
          });
      }
    }
    
    // Save temperature curve
    function saveCurve() {
      const curveData = [];
      for(let i = 0; i < 4; i++) {
        const outside = document.getElementById('editOutside' + i).value;
        const water = document.getElementById('editWater' + i).value;
        curveData.push(outside + ',' + water);
      }
      
      fetch('/setcurve?' + curveData.map((d, i) => `p${i}=${d}`).join('&'))
        .then(() => {
          alert('Curve saved!');
          updateData();
        });
    }
    
    // Load current temperature curve values
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
    
    // Save WiFi settings
    function saveWifiConfig() {
      const apSsid = document.getElementById('apSsid').value;
      const apPassword = document.getElementById('apPassword').value;
      const useSta = document.getElementById('useSta').checked;
      const staSsid = document.getElementById('staSsid').value;
      const staPassword = document.getElementById('staPassword').value;
      
      if (apSsid.length < 1) {
        alert('AP network name cannot be empty!');
        return;
      }
      
      if (useSta && staSsid.length < 1) {
        alert('Network name cannot be empty when STA mode is enabled!');
        return;
      }
      
      const params = new URLSearchParams({
        ap_ssid: apSsid,
        ap_password: apPassword,
        use_sta: useSta ? '1' : '0',
        sta_ssid: staSsid,
        sta_password: staPassword
      });
      
      if (confirm('Save WiFi settings?\nDevice will restart.')) {
        fetch('/setwifi?' + params.toString())
          .then(response => response.text())
          .then(result => {
            alert('WiFi settings saved!\n' + result);
          });
      }
    }
    
    // Load WiFi settings
    function loadWifiConfig() {
      fetch('/getwifi')
        .then(response => response.json())
        .then(data => {
          document.getElementById('apSsid').value = data.ap_ssid || '';
          document.getElementById('apPassword').value = data.ap_password || '';
          document.getElementById('useSta').checked = data.use_sta || false;
          document.getElementById('staSsid').value = data.sta_ssid || '';
          document.getElementById('staPassword').value = data.sta_password || '';
          
          // Update STA fields visibility
          toggleStaFields();
        })
        .catch(error => {
          console.error('Error loading WiFi settings:', error);
        });
    }
    
    // Show/hide STA fields
    function toggleStaFields() {
      const useSta = document.getElementById('useSta').checked;
      document.getElementById('staFields').style.display = useSta ? 'block' : 'none';
    }
    
    // Save system settings (hysteresis)
    function saveSystemSettings() {
      const hysteresis = document.getElementById('hysteresisValue').value;
      
      if (hysteresis < 0.5 || hysteresis > 10.0) {
        alert('Hysteresis must be between 0.5 and 10.0 °C!');
        return;
      }
      
      fetch('/setsettings?hysteresis=' + hysteresis)
        .then(response => response.text())
        .then(result => {
          alert('System settings saved!\n' + result);
          updateData();
        });
    }
    
    // Load system settings
    function loadSystemSettings() {
      fetch('/getsettings')
        .then(response => response.json())
        .then(data => {
          document.getElementById('hysteresisValue').value = data.hysteresis || 4.0;
        });
    }
    
    // Update data automatically every 5 seconds
    setInterval(updateData, 5000);
    
    // Initialization when page loads
    window.onload = function() {
      // Open first tab
      document.getElementsByClassName("tablink")[0].click();
      
      // Load settings
      loadWifiConfig();
      loadCurve();
      loadSystemSettings();
      
      // Update data
      updateData();
    };
  </script>
</head>
<body>
  <div class="container">
    <h1>Oil Heating Control</h1>
    
    <div class="tab">
      <button class="tablink" onclick="openTab(event, 'control')">Control</button>
      <button class="tablink" onclick="openTab(event, 'wifi')">WiFi Settings</button>
      <button class="tablink" onclick="openTab(event, 'curve')">Temperature Curve</button>
      <button class="tablink" onclick="openTab(event, 'settings')">System Settings</button>
    </div>
    
    <!-- CONTROL TAB -->
    <div id="control" class="tabcontent">
      <div class="sensor-box">
        <h2>Temperatures</h2>
        <p><strong>Outside temperature:</strong> <span id="outsideTemp">--</span></p>
        <p><strong>Water temperature:</strong> <span id="waterTemp">--</span></p>
        <p><strong>Target temperature:</strong> <span id="targetTemp">--</span></p>
      </div>
      
      <div class="status-box">
        <h2>System Status</h2>
        <p><strong>Outside sensor:</strong> <span id="outsideStatus" class="ok">OK</span></p>
        <p><strong>Water sensor:</strong> <span id="waterStatus" class="ok">OK</span></p>
        <p><strong>Heating disabled (too warm):</strong> <span id="heatingDisabled">--</span></p>
      </div>
      
      <div class="control-box">
        <h2>Oil Burner</h2>
        <p><strong>State:</strong> <span id="burnerState">--</span></p>
        <p><strong>Mode:</strong> <span id="burnerMode">--</span></p>
        
        <button onclick="setManualState(false)">Automatic Mode</button>
        <button onclick="setManualState(true)">Manual Mode</button>
        <br>
        <button onclick="setBurnerState(true)">Turn Burner ON</button>
        <button onclick="setBurnerState(false)">Turn Burner OFF</button>
        <br>
        <button onclick="emergencyStop()" class="danger">Emergency Stop</button>
      </div>
      
      <div class="sensor-box">
        <h2>WiFi Status</h2>
        <p><strong>Mode:</strong> <span id="wifiMode">--</span></p>
        <p><strong>IP Address:</strong> <span id="wifiIP">--</span></p>
      </div>
      
      <div class="settings-box">
        <h2>Current Settings</h2>
        <p><strong>Hysteresis:</strong> <span id="currentHysteresis">--</span></p>
      </div>
    </div>
    
    <!-- WIFI SETTINGS TAB -->
    <div id="wifi" class="tabcontent">
      <div class="config-box">
        <h2>WiFi Settings</h2>
        
        <h3>Own Network (Access Point)</h3>
        <p>Device creates its own WiFi network that you can connect to directly.</p>
        <p><strong>Network name (SSID):</strong><br>
        <input type="text" id="apSsid" placeholder="AP network name"></p>
        <p><strong>Password:</strong><br>
        <input type="password" id="apPassword" placeholder="AP password"></p>
        
        <h3>Connect to Existing Network (Optional)</h3>
        <p><input type="checkbox" id="useSta" onclick="toggleStaFields()">
        <label for="useSta">Connect to existing network</label></p>
        
        <div id="staFields" style="display: none;">
          <p><strong>Network name (SSID):</strong><br>
          <input type="text" id="staSsid" placeholder="Network name"></p>
          <p><strong>Password:</strong><br>
          <input type="password" id="staPassword" placeholder="Network password"></p>
          <p><em>Note: If connection fails, device will automatically create its own network.</em></p>
        </div>
        
        <button onclick="saveWifiConfig()">Save WiFi Settings</button>
        <button onclick="loadWifiConfig()">Load Current Settings</button>
        
        <p><em>Note: Device will restart automatically after saving.</em></p>
      </div>
    </div>
    
    <!-- TEMPERATURE CURVE TAB -->
    <div id="curve" class="tabcontent">
      <div class="curve-box">
        <h2>Temperature Curve Adjustment</h2>
        <p>Set 4 points for linear temperature curve.</p>
        
        <table>
          <tr>
            <th>Point</th>
            <th>Outside Temperature (°C)</th>
            <th>Water Temperature (°C)</th>
          </tr>
          <tr>
            <td>1 (coldest)</td>
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
            <td>4 (warmest)</td>
            <td><input type="number" id="editOutside3" step="0.1" value="10.0"></td>
            <td><input type="number" id="editWater3" step="0.1" value="45.0"></td>
          </tr>
        </table>
        
        <button onclick="saveCurve()">Save Curve</button>
        <button onclick="loadCurve()">Load Current Curve</button>
        
        <p><em>Curve is saved automatically with 5 second delay.</em></p>
      </div>
    </div>
    
    <!-- SYSTEM SETTINGS TAB -->
    <div id="settings" class="tabcontent">
      <div class="settings-box">
        <h2>System Settings</h2>
        
        <h3>Control Parameters</h3>
        <p><strong>Hysteresis:</strong><br>
        <input type="number" id="hysteresisValue" step="0.1" min="0.5" max="10.0" value="4.0"> °C</p>
        <p><em>Hysteresis prevents rapid on/off switching of the burner.<br>
        Smaller value = more precise control, larger value = more stable operation.</em></p>
        
        <button onclick="saveSystemSettings()">Save System Settings</button>
        <button onclick="loadSystemSettings()">Load Current Settings</button>
        
        <p><em>Settings are saved automatically with 5 second delay.</em></p>
      </div>
    </div>
  </div>
</body>
</html>
)rawliteral";

/**
 * Web server task
 * Provides web interface for system control
 */
void webServerTask(void *parameter) {
  Serial.println("Web server task started");
  
  // Wait for WiFi to be fully initialized
  delay(2000);
  
  /**
   * HTTP routes
   */
  
  // Main page
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", htmlPage);
  });
  
  // JSON data
  server.on("/data", HTTP_GET, []() {
    // Collect data safely
    float outsideTemp = 0.0, waterTemp = 0.0, targetTemp = 0.0;
    bool heatingDisabled = false;
    String outsideStatus = "OK", waterStatus = "OK";
    String wifiMode = "--", wifiIP = "--";
    float hysteresis = DEFAULT_HYSTERESIS;
    
    // Get temperatures
    if (lockSensorAndTemperature()) {
      outsideTemp = systemState.outsideStatus.fault ? 
                   systemState.outsideData.getAverageValidValue() : 
                   systemState.outsideData.currentTemp;
      
      waterTemp = systemState.waterStatus.fault ? 
                  systemState.waterData.getAverageValidValue() : 
                  systemState.waterData.currentTemp;
      
      if (systemState.outsideStatus.fault) {
        outsideStatus = "ERROR";
      } else if (systemState.outsideStatus.temporaryFault) {
        outsideStatus = "WARNING";
      }
      
      if (systemState.waterStatus.fault) {
        waterStatus = "ERROR";
      } else if (systemState.waterStatus.temporaryFault) {
        waterStatus = "WARNING";
      }
      
      targetTemp = systemState.targetWaterTemp;
      heatingDisabled = systemState.heatingDisabled;
      
      unlockSensorAndTemperature();
    }
    
    // Get burner state
    bool burnerState = false, manualMode = false;
    if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      burnerState = systemState.burnerState;
      manualMode = systemState.burnerManualOverride;
      xSemaphoreGive(xBurnerStateMutex);
    }
    
    // Get WiFi status
    if (systemState.wifiConfig.ap_mode) {
      wifiMode = "AP (Own Network)";
      wifiIP = WiFi.softAPIP().toString();
    } else if (systemState.wifiConfig.connected) {
      wifiMode = "STA (Network)";
      wifiIP = WiFi.localIP().toString();
    }
    
    // Get hysteresis value
    if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      hysteresis = systemState.settings.hysteresis;
      xSemaphoreGive(xSettingsMutex);
    }
    
    // Build JSON
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
    json += ",\"hysteresis\":" + String(hysteresis, 1);
    json += "}";
    
    server.send(200, "application/json", json);
  });
  
  // Control commands
  server.on("/control", HTTP_GET, []() {
    if (server.hasArg("manual")) {
      bool manual = server.arg("manual").toInt() == 1;
      if (xSemaphoreTake(xBurnerStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        systemState.burnerManualOverride = manual;
        xSemaphoreGive(xBurnerStateMutex);
      }
      Serial.printf("Manual mode set: %s\n", manual ? "ON" : "OFF");
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
        Serial.printf("Burner set manually: %s\n", burner ? "ON" : "OFF");
      }
    }
    
    server.send(200, "text/plain", "OK");
  });
  
  // Temperature curve setting
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
            systemState.settings.settingsDirty = true;
            systemState.settings.lastSettingsChange = xTaskGetTickCount() * portTICK_PERIOD_MS;
            xSemaphoreGive(xSettingsMutex);
            
            Serial.printf("Curve point %d updated: %.1f -> %.1f\n", i, outTemp, waterTemp);
          }
        }
      }
    }
    
    server.send(200, "text/plain", "Curve points received (will be saved to flash with delay)");
  });
  
  // Temperature curve retrieval
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
  
  // System settings retrieval
  server.on("/getsettings", HTTP_GET, []() {
    String json = "{";
    
    if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      json += "\"hysteresis\":" + String(systemState.settings.hysteresis, 1);
      xSemaphoreGive(xSettingsMutex);
    }
    
    json += "}";
    server.send(200, "application/json", json);
  });
  
  // System settings setting
  server.on("/setsettings", HTTP_GET, []() {
    if (server.hasArg("hysteresis")) {
      float hysteresis = server.arg("hysteresis").toFloat();
      
      // Validate hysteresis value
      if (hysteresis >= 0.5 && hysteresis <= 10.0) {
        if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          systemState.settings.hysteresis = hysteresis;
          systemState.settings.settingsDirty = true;
          systemState.settings.lastSettingsChange = xTaskGetTickCount() * portTICK_PERIOD_MS;
          xSemaphoreGive(xSettingsMutex);
          
          Serial.printf("Hysteresis updated: %.1f\n", hysteresis);
        }
        server.send(200, "text/plain", "System settings received (will be saved to flash with delay)");
      } else {
        server.send(400, "text/plain", "Error: Hysteresis must be between 0.5 and 10.0 °C");
      }
    } else {
      server.send(400, "text/plain", "Error: Missing parameters");
    }
  });
  
  // Emergency shutdown
  server.on("/emergency", HTTP_GET, []() {
    RelayCommand cmd;
    cmd.type = RelayCommand::EMERGENCY_OFF;
    xQueueSend(xRelayControlQueue, &cmd, 0);
    
    Serial.println("Emergency shutdown performed by user request");
    server.send(200, "text/plain", "Emergency shutdown performed");
  });
  
  // WiFi settings retrieval
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
  
  // WiFi settings saving
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
      server.send(200, "text/plain", "WiFi settings saved. Device will restart...");
      delay(1000);
      ESP.restart();
    } else {
      server.send(200, "text/plain", "No changes to WiFi settings.");
    }
  });
  
  // Page not found
  server.onNotFound([]() {
    server.send(404, "text/plain", "Page not found");
  });
  
  // Start web server
  server.begin();
  Serial.println("HTTP server started on port 80");
  
  // Print connection information
  if (systemState.wifiConfig.ap_mode) {
    Serial.println("\n=== WEB SERVER READY ===");
    Serial.println("Connect to own network:");
    Serial.println("  SSID: " + String(systemState.wifiConfig.ap_ssid));
    Serial.println("  Password: " + String(systemState.wifiConfig.ap_password));
    Serial.println("  Open in browser: http://" + WiFi.softAPIP().toString());
  } else {
    Serial.println("\n=== WEB SERVER READY ===");
    Serial.println("Connected to network:");
    Serial.println("  Open in browser: http://" + WiFi.localIP().toString());
  }
  
  // Web server main loop
  while (1) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==================== SETUP ====================

/**
 * Arduino setup() function
 * Executed once on startup
 * Initializes all system components and starts threads
 */
void setup() {
  // Initialize serial monitor for debug output
  Serial.begin(115200);
  delay(2000);  // Wait for serial monitor to be ready
  
  Serial.println("\n\n=== OIL HEATING SYSTEM - FreeRTOS ===\n");
  Serial.println("System starting...");
  
  /**
   * 1. CREATE MUTEXES
   * To avoid deadlock, always use same order
   */
  xSensorStatusMutex = xSemaphoreCreateMutex();
  xTemperatureMutex = xSemaphoreCreateMutex();
  xBurnerStateMutex = xSemaphoreCreateMutex();
  xSettingsMutex = xSemaphoreCreateMutex();
  xWifiConfigMutex = xSemaphoreCreateMutex();
  
  if (xSensorStatusMutex == NULL || xTemperatureMutex == NULL || 
      xBurnerStateMutex == NULL || xSettingsMutex == NULL || xWifiConfigMutex == NULL) {
    Serial.println("ERROR: Mutex creation failed!");
    while(1);
  }
  
  /**
   * 2. CREATE EVENT GROUP
   * Used for efficient state change notifications
   */
  xSystemEvents = xEventGroupCreate();
  if (xSystemEvents == NULL) {
    Serial.println("ERROR: Event Group creation failed!");
    while(1);
  }
  
  /**
   * 3. CREATE QUEUE
   * For relay control communication between threads
   */
  xRelayControlQueue = xQueueCreate(10, sizeof(RelayCommand));
  if (xRelayControlQueue == NULL) {
    Serial.println("ERROR: Queue creation failed!");
    while(1);
  }
  
  /**
   * 4. LOAD SETTINGS FROM FLASH MEMORY
   * FIXED: Properly loads temperature curve and system settings
   */
  loadSystemSettings();
  
  /**
   * 5. INITIALIZE WIFI
   * Load WiFi settings and connect to network or create own network
   */
  initWiFi();
  
  /**
   * 6. CREATE THREADS (TASKS)
   * Each function gets its own thread with appropriate priority
   */
  
  // Watchdog task (highest priority)
  xTaskCreatePinnedToCore(
    watchdogTask,
    "WatchdogTask",
    TASK_STACK_SMALL,
    NULL,
    TASK_PRIORITY_HIGH + 1,
    &xWatchdogTaskHandle,
    0  // Core 0
  );
  
  // Relay task (high priority - safety critical)
  xTaskCreatePinnedToCore(
    relayTask,
    "RelayTask",
    TASK_STACK_SMALL,
    NULL,
    TASK_PRIORITY_HIGH,
    &xRelayTaskHandle,
    1  // Core 1
  );
  
  // Temperature task
  xTaskCreatePinnedToCore(
    temperatureTask,
    "TempTask",
    TASK_STACK_MEDIUM,
    NULL,
    TASK_PRIORITY_HIGH,
    &xTemperatureTaskHandle,
    1
  );
  
  // Control task (FIXED: Now properly turns off burner when too hot)
  xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    TASK_STACK_MEDIUM,
    NULL,
    TASK_PRIORITY_MEDIUM,
    &xControlTaskHandle,
    1
  );
  
  // Flash task (background task)
  xTaskCreatePinnedToCore(
    flashTask,
    "FlashTask",
    TASK_STACK_SMALL,
    NULL,
    TASK_PRIORITY_LOW,
    &xFlashTaskHandle,
    1
  );
  
  // Wait for system to stabilize
  delay(3000);
  
  // Web server task (non-critical)
  xTaskCreatePinnedToCore(
    webServerTask,
    "WebServerTask",
    TASK_STACK_LARGE,
    NULL,
    TASK_PRIORITY_LOW,
    &xWebServerTaskHandle,
    0  // Core 0 (same as WiFi)
  );
  
  Serial.println("\nAll threads created successfully!");
  Serial.println("System ready for use!");
  
  // Debug output of current settings
  Serial.println("\n=== CURRENT SETTINGS ===");
  if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.printf("Hysteresis: %.1f°C\n", systemState.settings.hysteresis);
    Serial.println("Temperature curve:");
    for (int i = 0; i < 4; i++) {
      Serial.printf("  Point %d: %.1f°C -> %.1f°C\n", 
                   i, 
                   systemState.curve.points[i][0], 
                   systemState.curve.points[i][1]);
    }
    xSemaphoreGive(xSettingsMutex);
  }
}

// ==================== LOOP ====================

/**
 * Arduino loop() function
 * Traditional Arduino loop, runs on core 0
 * Checks WiFi connection and performs other non-critical tasks
 */
void loop() {
  static unsigned long lastWifiCheck = 0;
  unsigned long currentTime = millis();
  
  // Check WiFi connection every 30 seconds (STA mode only)
  if (currentTime - lastWifiCheck > WIFI_RECONNECT_INTERVAL) {
    lastWifiCheck = currentTime;
    
    if (systemState.wifiConfig.use_sta && !systemState.wifiConfig.ap_mode) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection lost, trying to reconnect...");
        reconnectWiFi();
      }
    }
  }
  
  // Give other threads time to run
  vTaskDelay(pdMS_TO_TICKS(1000));
}
