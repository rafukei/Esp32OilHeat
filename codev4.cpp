/**
 * OIL HEATING SYSTEM - FreeRTOS + WiFi
 * 
 * Clean thread-safe implementation for ESP32 with 10-minute delay feature
 * 
 * Features:
 * - Temperature measurement with DS18B20 sensors
 * - Automatic control with hysteresis
 * - Web interface with dark theme
 * - WiFi management (AP/STA modes)
 * - Temperature curve adjustment
 * - Safety measures for sensor errors
 * - Watchdog monitoring
 * - Settings saved to flash memory
 * - 10-minute delay after heating turns off due to warm outside temperature
 * 
 * Thread Safety:
 * - All global data encapsulated in SystemState class
 * - Mutex-protected data groups
 * - Atomic snapshot operations
 * - No deadlock risk with clear locking order
 * 
 * Created: 2024
 * Author: Heating System Engineer
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

// Debug configuration
#define DEBUG_TEMP 0        // Enable temperature debug output
#define DEBUG_CONTROL 1     // Enable control logic debug output

// ==================== HARDWARE CONFIGURATION ====================

// GPIO pin definitions
#define OUTSIDE_SENSOR_PIN 23    // Outside temperature sensor DS18B20
#define WATER_SENSOR_PIN 22      // Water temperature sensor DS18B20
#define RELAY_PIN 2              // Relay control (HIGH = ON, LOW = OFF)

// Sensor configuration
#define SENSOR_RESOLUTION 12     // DS18B20 resolution (9-12 bits, 12 = 0.0625°C)
#define TEMP_READ_INTERVAL_MS 5000  // Temperature reading interval (5 seconds)

// Safety limits
#define SENSOR_FAULT_TEMP -50.0         // Temperature below this = error
#define SENSOR_FAULT_TIMEOUT 30000      // 30 seconds before serious error
#define MAX_CONSECUTIVE_FAILURES 3      // Allowed consecutive errors
#define WATER_TEMP_MAX 120.0            // Maximum allowed water temperature
#define OUTSIDE_TEMP_MAX 100.0          // Maximum allowed outside temperature

// Control parameters
#define DEFAULT_HYSTERESIS 4.0          // Default hysteresis value (°C)
#define OUTSIDE_TEMP_UPDATE_INTERVAL 600000  // 10 minutes delay after heating off
#define CONTROL_LOOP_INTERVAL_MS 1000   // Control logic execution interval

// WiFi configuration
#define WIFI_CONNECT_TIMEOUT 10000      // 10 seconds connection timeout
#define WIFI_RECONNECT_INTERVAL 30000   // 30 seconds reconnect interval
#define DEFAULT_AP_SSID "OilHeaterAP"   // Default AP network name
#define DEFAULT_AP_PASSWORD "enmuista"  // Default AP password

// Task priorities (higher number = higher priority)
#define TASK_PRIORITY_CRITICAL 4        // Safety-critical tasks
#define TASK_PRIORITY_HIGH 3            // Critical control tasks
#define TASK_PRIORITY_MEDIUM 2          // Normal operation tasks
#define TASK_PRIORITY_LOW 1             // Background tasks

// Task stack sizes (bytes)
#define TASK_STACK_SMALL 2048           // Simple tasks
#define TASK_STACK_MEDIUM 3072          // Medium complexity tasks
#define TASK_STACK_LARGE 4096           // Complex tasks (web server)

// Event Group bits for system-wide event notifications
#define BIT_SENSOR_FAULT_OUTSIDE (1 << 0)  // Outside sensor serious error
#define BIT_SENSOR_FAULT_WATER   (1 << 1)  // Water sensor serious error
#define BIT_HEATING_DISABLED     (1 << 2)  // Heating disabled (too warm)
#define BIT_TEMP_TASK_ALIVE      (1 << 3)  // Temperature task alive (watchdog)
#define BIT_EMERGENCY_STOP       (1 << 4)  // Emergency stop activated

// ==================== FORWARD DECLARATIONS ====================

// Control snapshot structure (needed for forward declarations)
struct SystemControlSnapshot {
    float waterTemp;
    float outsideTemp;
    float targetTemp;
    float controlOutsideTemp;
    float hysteresis;
    bool waterFault;
    bool outsideFault;
    bool heatingDisabled;
    bool manualMode;
    bool burnerState;
};

// Global objects
extern EventGroupHandle_t xSystemEvents;
extern QueueHandle_t xRelayControlQueue;

// Utility function declarations
bool isTemperatureFaulty(float temperature, bool isWaterSensor);
void updateHeatingStatus(bool disabled);
void setBurnerState(bool state);
void applyHysteresis(const SystemControlSnapshot& snapshot, float targetTemp);
void controlBurnerSimple(const SystemControlSnapshot& snap, float targetTemp);

// ==================== DATA STRUCTURES ====================

/**
 * WiFi configuration structure
 */
struct WifiConfig {
    char ap_ssid[32] = DEFAULT_AP_SSID;
    char ap_password[32] = DEFAULT_AP_PASSWORD;
    char sta_ssid[32] = "";
    char sta_password[32] = "";
    bool use_sta = false;
    bool connected = false;
    bool ap_mode = false;
    
    bool isStaConfigured() const {
        return use_sta && strlen(sta_ssid) > 0;
    }
};

/**
 * Temperature curve points for 4-point linear interpolation
 */
struct CurvePoints {
    float points[4][2] = {
        {-20.0, 75.0},   // Very cold: -20°C outside -> 75°C water
        {-10.0, 65.0},   // Cold: -10°C outside -> 65°C water
        {0.0, 55.0},     // Freezing: 0°C outside -> 55°C water
        {10.0, 45.0}     // Mild: 10°C outside -> 45°C water (above = heating off)
    };
    
    bool isValid() const {
        // Check each point
        for (int i = 0; i < 4; i++) {
            if (points[i][0] < -50.0 || points[i][0] > 50.0) return false;
            if (points[i][1] < 20.0 || points[i][1] > 90.0) return false;
        }
        
        // Check monotonic increasing for outside temps
        for (int i = 0; i < 3; i++) {
            if (points[i][0] >= points[i+1][0]) return false;
        }
        
        // Check monotonic decreasing for water temps
        for (int i = 0; i < 3; i++) {
            if (points[i][1] <= points[i+1][1]) return false;
        }
        
        return true;
    }
};

/**
 * System settings
 */
struct SystemSettings {
    float hysteresis = DEFAULT_HYSTERESIS;
    bool settingsDirty = false;
    unsigned long lastSettingsChange = 0;
    
    bool isHysteresisValid() const {
        return hysteresis >= 0.5 && hysteresis <= 10.0;
    }
    
    void markDirty() {
        settingsDirty = true;
        lastSettingsChange = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
};

/**
 * Sensor temperature data with historical values
 */
struct SensorData {
    float currentTemp = 0.0;
    float lastValidTemp = 0.0;
    unsigned long lastValidReadTime = 0;
    
    float lastValidValues[10] = {0};
    int validValueIndex = 0;
    int validValueCount = 0;
    
    void addValidValue(float value) {
        lastValidValues[validValueIndex] = value;
        validValueIndex = (validValueIndex + 1) % 10;
        if (validValueCount < 10) validValueCount++;
    }
    
    float getAverageValidValue() const {
        if (validValueCount == 0) return 0.0;
        
        float sum = 0.0;
        for (int i = 0; i < validValueCount; i++) {
            sum += lastValidValues[i];
        }
        return sum / validValueCount;
    }
    
    void reset() {
        currentTemp = 0.0;
        lastValidTemp = 0.0;
        validValueIndex = 0;
        validValueCount = 0;
        memset(lastValidValues, 0, sizeof(lastValidValues));
    }
};

/**
 * Sensor status and error tracking
 */
struct SensorStatus {
    bool fault = false;
    bool temporaryFault = false;
    unsigned long faultStartTime = 0;
    int consecutiveFailures = 0;
    
    bool isInError() const {
        return fault || temporaryFault;
    }
    
    void reset() {
        fault = false;
        temporaryFault = false;
        faultStartTime = 0;
        consecutiveFailures = 0;
    }
};

/**
 * Control logic timing and data
 */
struct ControlData {
    unsigned long lastOutsideTempUpdate = 0;
    float controlOutsideTemp = 0.0;
    bool outsideTempNeedsUpdate = true;
    
    bool needsUpdate(unsigned long currentTime) const {
        return outsideTempNeedsUpdate || 
               (currentTime - lastOutsideTempUpdate >= OUTSIDE_TEMP_UPDATE_INTERVAL);
    }
    
    void update(float newTemp, unsigned long currentTime) {
        controlOutsideTemp = newTemp;
        lastOutsideTempUpdate = currentTime;
        outsideTempNeedsUpdate = false;
    }
};

/**
 * Burner control state
 */
struct BurnerData {
    bool burnerState = false;
    bool burnerManualOverride = false;
    
    bool isAutomaticMode() const {
        return !burnerManualOverride;
    }
    
    bool isManualMode() const {
        return burnerManualOverride;
    }
};

/**
 * Relay control command structure
 */
struct RelayCommand {
    enum CommandType {
        SET_STATE,
        FORCE_OFF,
        EMERGENCY_OFF
    };
    
    CommandType type;
    bool state;
    unsigned long timestamp;
    
    RelayCommand() : type(SET_STATE), state(false), timestamp(0) {}
    
    RelayCommand(CommandType cmdType, bool cmdState = false) 
        : type(cmdType), state(cmdState), timestamp(xTaskGetTickCount() * portTICK_PERIOD_MS) {}
};

// ==================== UTILITY FUNCTIONS ====================

/**
 * Check if temperature reading is faulty
 */
bool isTemperatureFaulty(float temperature, bool isWaterSensor) {
    if (temperature == DEVICE_DISCONNECTED_C) {
        return true;
    }
    
    if (temperature < SENSOR_FAULT_TEMP) {
        return true;
    }
    
    if (isWaterSensor) {
        if (temperature > WATER_TEMP_MAX) return true;
    } else {
        if (temperature > OUTSIDE_TEMP_MAX) return true;
    }
    
    return false;
}

// ==================== SYSTEM STATE CLASS ====================

/**
 * Thread-safe system state management
 */
class SystemState {
private:
    mutable SemaphoreHandle_t m_tempMutex;
    mutable SemaphoreHandle_t m_controlMutex;
    mutable SemaphoreHandle_t m_burnerMutex;
    mutable SemaphoreHandle_t m_settingsMutex;
    mutable SemaphoreHandle_t m_wifiMutex;

public:
    // Temperature and sensor data
    struct TemperatureData {
        SensorData outsideData;
        SensorData waterData;
        SensorStatus outsideStatus;
        SensorStatus waterStatus;
        float targetWaterTemp = 0.0;
        bool heatingDisabled = false;
        
        bool hasSeriousFault() const {
            return outsideStatus.fault || waterStatus.fault;
        }
        
        bool hasAnyFault() const {
            return outsideStatus.isInError() || waterStatus.isInError();
        }
    } tempData;
    
    // Control timing and logic data
    ControlData controlData;
    
    // Burner control state
    BurnerData burnerData;
    
    // System settings and configuration
    struct SettingsData {
        CurvePoints curve;
        SystemSettings settings;
        
        float calculateTargetTemp(float outsideTemp) const {
            if (outsideTemp >= curve.points[3][0]) {
                return 0.0;
            }
            
            if (outsideTemp <= curve.points[0][0]) {
                return curve.points[0][1];
            }
            
            for (int i = 0; i < 3; i++) {
                if (outsideTemp >= curve.points[i][0] && outsideTemp <= curve.points[i+1][0]) {
                    float ratio = (outsideTemp - curve.points[i][0]) / 
                                 (curve.points[i+1][0] - curve.points[i][0]);
                    return curve.points[i][1] + ratio * (curve.points[i+1][1] - curve.points[i][1]);
                }
            }
            
            return 45.0;
        }
    } settingsData;
    
    // WiFi configuration
    WifiConfig wifiConfig;

public:
    /**
     * Constructor - initialize all mutexes
     */
    SystemState() {
        m_tempMutex = xSemaphoreCreateMutex();
        m_controlMutex = xSemaphoreCreateMutex();
        m_burnerMutex = xSemaphoreCreateMutex();
        m_settingsMutex = xSemaphoreCreateMutex();
        m_wifiMutex = xSemaphoreCreateMutex();
        
        if (!m_tempMutex || !m_controlMutex || !m_burnerMutex || 
            !m_settingsMutex || !m_wifiMutex) {
            Serial.println("CRITICAL ERROR: Failed to create mutexes!");
            while(1);
        }
    }
    
    /**
     * Destructor - clean up mutexes
     */
    ~SystemState() {
        if (m_tempMutex) vSemaphoreDelete(m_tempMutex);
        if (m_controlMutex) vSemaphoreDelete(m_controlMutex);
        if (m_burnerMutex) vSemaphoreDelete(m_burnerMutex);
        if (m_settingsMutex) vSemaphoreDelete(m_settingsMutex);
        if (m_wifiMutex) vSemaphoreDelete(m_wifiMutex);
    }
    
    // ========== TEMPERATURE DATA METHODS ==========
    
    bool lockTempData(TickType_t timeout = portMAX_DELAY) const {
        return xSemaphoreTake(m_tempMutex, timeout) == pdTRUE;
    }
    
    void unlockTempData() const {
        xSemaphoreGive(m_tempMutex);
    }
    
    TemperatureData getTempDataSnapshot() const {
        TemperatureData snapshot;
        if (lockTempData(pdMS_TO_TICKS(100))) {
            snapshot = tempData;
            unlockTempData();
        }
        return snapshot;
    }
    
    bool updateSensorReading(float newTemp, bool isWaterSensor) {
        if (!lockTempData(pdMS_TO_TICKS(100))) return false;
        
        SensorStatus* status = isWaterSensor ? &tempData.waterStatus : &tempData.outsideStatus;
        SensorData* data = isWaterSensor ? &tempData.waterData : &tempData.outsideData;
        
        bool isFaulty = isTemperatureFaulty(newTemp, isWaterSensor);
        unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        if (isFaulty) {
            status->consecutiveFailures++;
            
            if (status->consecutiveFailures == 1) {
                status->faultStartTime = currentTime;
            }
            
            if (status->consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
                status->temporaryFault = true;
            }
            
            if (currentTime - status->faultStartTime > SENSOR_FAULT_TIMEOUT && 
                status->consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
                status->fault = true;
                status->temporaryFault = false;
                
                EventBits_t bits = isWaterSensor ? BIT_SENSOR_FAULT_WATER : BIT_SENSOR_FAULT_OUTSIDE;
                xEventGroupSetBits(xSystemEvents, bits);
            }
        } else {
            data->currentTemp = newTemp;
            data->lastValidTemp = newTemp;
            data->lastValidReadTime = currentTime;
            data->addValidValue(newTemp);
            
            status->consecutiveFailures = 0;
            status->temporaryFault = false;
            if (status->fault) {
                status->fault = false;
                EventBits_t bits = isWaterSensor ? BIT_SENSOR_FAULT_WATER : BIT_SENSOR_FAULT_OUTSIDE;
                xEventGroupClearBits(xSystemEvents, bits);
            }
        }
        
        unlockTempData();
        return true;
    }
    
    bool updateTargetTemperature(float outsideTemp) {
        if (!lockTempData(pdMS_TO_TICKS(100))) return false;
        
        CurvePoints curve;
        if (lockSettings(pdMS_TO_TICKS(50))) {
            curve = settingsData.curve;
            unlockSettings();
        } else {
            unlockTempData();
            return false;
        }
        
        tempData.targetWaterTemp = settingsData.calculateTargetTemp(outsideTemp);
        tempData.heatingDisabled = (outsideTemp >= curve.points[3][0]);
        
        unlockTempData();
        return true;
    }
    
    // ========== CONTROL DATA METHODS ==========
    
    bool lockControlData(TickType_t timeout = portMAX_DELAY) const {
        return xSemaphoreTake(m_controlMutex, timeout) == pdTRUE;
    }
    
    void unlockControlData() const {
        xSemaphoreGive(m_controlMutex);
    }
    
    ControlData getControlDataSnapshot() const {
        ControlData snapshot;
        if (lockControlData(pdMS_TO_TICKS(100))) {
            snapshot = controlData;
            unlockControlData();
        }
        return snapshot;
    }
    
    // ========== BURNER DATA METHODS ==========
    
    bool lockBurnerData(TickType_t timeout = portMAX_DELAY) const {
        return xSemaphoreTake(m_burnerMutex, timeout) == pdTRUE;
    }
    
    void unlockBurnerData() const {
        xSemaphoreGive(m_burnerMutex);
    }
    
    BurnerData getBurnerDataSnapshot() const {
        BurnerData snapshot;
        if (lockBurnerData(pdMS_TO_TICKS(100))) {
            snapshot = burnerData;
            unlockBurnerData();
        }
        return snapshot;
    }
    
    bool setBurnerState(bool newState) {
        if (!lockBurnerData(pdMS_TO_TICKS(100))) return false;
        
        bool changed = (burnerData.burnerState != newState);
        burnerData.burnerState = newState;
        
        unlockBurnerData();
        return changed;
    }
    
    bool setManualMode(bool manual) {
        if (!lockBurnerData(pdMS_TO_TICKS(100))) return false;
        
        bool changed = (burnerData.burnerManualOverride != manual);
        burnerData.burnerManualOverride = manual;
        
        unlockBurnerData();
        return changed;
    }
    
    // ========== SETTINGS METHODS ==========
    
    bool lockSettings(TickType_t timeout = portMAX_DELAY) const {
        return xSemaphoreTake(m_settingsMutex, timeout) == pdTRUE;
    }
    
    void unlockSettings() const {
        xSemaphoreGive(m_settingsMutex);
    }
    
    SettingsData getSettingsSnapshot() const {
        SettingsData snapshot;
        if (lockSettings(pdMS_TO_TICKS(100))) {
            snapshot = settingsData;
            unlockSettings();
        }
        return snapshot;
    }
    
    bool updateCurve(const CurvePoints& newCurve) {
        if (!lockSettings(pdMS_TO_TICKS(100))) return false;
        
        if (newCurve.isValid()) {
            settingsData.curve = newCurve;
            settingsData.settings.markDirty();
            unlockSettings();
            return true;
        }
        
        unlockSettings();
        return false;
    }
    
    bool updateHysteresis(float newHysteresis) {
        if (!lockSettings(pdMS_TO_TICKS(100))) return false;
        
        if (newHysteresis >= 0.5 && newHysteresis <= 10.0) {
            settingsData.settings.hysteresis = newHysteresis;
            settingsData.settings.markDirty();
            unlockSettings();
            return true;
        }
        
        unlockSettings();
        return false;
    }
    
    // ========== WIFI METHODS ==========
    
    bool lockWifiConfig(TickType_t timeout = portMAX_DELAY) const {
        return xSemaphoreTake(m_wifiMutex, timeout) == pdTRUE;
    }
    
    void unlockWifiConfig() const {
        xSemaphoreGive(m_wifiMutex);
    }
    
    WifiConfig getWifiConfigSnapshot() const {
        WifiConfig snapshot;
        if (lockWifiConfig(pdMS_TO_TICKS(100))) {
            snapshot = wifiConfig;
            unlockWifiConfig();
        }
        return snapshot;
    }
    
    // ========== COMPOUND OPERATIONS ==========
    
    SystemControlSnapshot getControlSnapshot() const {
        SystemControlSnapshot snapshot;
        
        // Get temperature data
        TemperatureData tempSnapshot = getTempDataSnapshot();
        snapshot.waterTemp = tempSnapshot.waterStatus.fault ? 
                           tempSnapshot.waterData.getAverageValidValue() : 
                           tempSnapshot.waterData.currentTemp;
        snapshot.outsideTemp = tempSnapshot.outsideStatus.fault ? 
                             tempSnapshot.outsideData.getAverageValidValue() : 
                             tempSnapshot.outsideData.currentTemp;
        snapshot.waterFault = tempSnapshot.waterStatus.fault;
        snapshot.outsideFault = tempSnapshot.outsideStatus.fault;
        snapshot.targetTemp = tempSnapshot.targetWaterTemp;
        snapshot.heatingDisabled = tempSnapshot.heatingDisabled;
        
        // Get control data
        ControlData controlSnapshot = getControlDataSnapshot();
        snapshot.controlOutsideTemp = controlSnapshot.controlOutsideTemp;
        
        // Get burner data
        BurnerData burnerSnapshot = getBurnerDataSnapshot();
        snapshot.burnerState = burnerSnapshot.burnerState;
        snapshot.manualMode = burnerSnapshot.burnerManualOverride;
        
        // Get settings
        SettingsData settingsSnapshot = getSettingsSnapshot();
        snapshot.hysteresis = settingsSnapshot.settings.hysteresis;
        
        return snapshot;
    }
    
    bool hasSeriousSensorFault() const {
        bool hasFault = false;
        if (lockTempData(pdMS_TO_TICKS(50))) {
            hasFault = tempData.outsideStatus.fault || tempData.waterStatus.fault;
            unlockTempData();
        }
        return hasFault;
    }
    
    void resetAllSensors() {
        if (lockTempData(pdMS_TO_TICKS(100))) {
            tempData.outsideData.reset();
            tempData.waterData.reset();
            tempData.outsideStatus.reset();
            tempData.waterStatus.reset();
            unlockTempData();
        }
    }
};

// ==================== GLOBAL OBJECTS DEFINITION ====================

// Global system state instance
SystemState systemState;

// FreeRTOS resources
EventGroupHandle_t xSystemEvents = NULL;
QueueHandle_t xRelayControlQueue = NULL;

// Hardware objects
OneWire oneWireOutside(OUTSIDE_SENSOR_PIN);
OneWire oneWireWater(WATER_SENSOR_PIN);
DallasTemperature sensorsOutside(&oneWireOutside);
DallasTemperature sensorsWater(&oneWireWater);

// Network objects
WebServer server(80);
Preferences preferences;

// Task handles
TaskHandle_t xTemperatureTaskHandle = NULL;
TaskHandle_t xControlTaskHandle = NULL;
TaskHandle_t xWebServerTaskHandle = NULL;
TaskHandle_t xRelayTaskHandle = NULL;
TaskHandle_t xWatchdogTaskHandle = NULL;
TaskHandle_t xFlashTaskHandle = NULL;

// ==================== HTML PAGE ====================

const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Oil Heating Control</title>
  <style>
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      margin: 20px;
      background-color: #121212;
      color: #e0e0e0;
    }
    .container {
      max-width: 900px;
      margin: auto;
      background: #1e1e1e;
      padding: 20px;
      border-radius: 15px;
      box-shadow: 0 0 20px rgba(0,0,0,0.5);
    }
    .sensor-box {
      background: #252525;
      padding: 20px;
      margin: 15px 0;
      border-radius: 10px;
      border-left: 4px solid #4CAF50;
    }
    .status-box {
      background: #252525;
      padding: 20px;
      margin: 15px 0;
      border-radius: 10px;
      border-left: 4px solid #2196F3;
    }
    .control-box {
      background: #252525;
      padding: 20px;
      margin: 15px 0;
      border-radius: 10px;
      border-left: 4px solid #FF9800;
    }
    .config-box {
      background: #252525;
      padding: 20px;
      margin: 15px 0;
      border-radius: 10px;
      border-left: 4px solid #9C27B0;
    }
    .curve-box {
      background: #252525;
      padding: 20px;
      margin: 15px 0;
      border-radius: 10px;
      border-left: 4px solid #009688;
    }
    .settings-box {
      background: #252525;
      padding: 20px;
      margin: 15px 0;
      border-radius: 10px;
      border-left: 4px solid #E91E63;
    }
    .temperature-display {
      font-size: 28px;
      font-weight: bold;
      color: #4CAF50;
      text-shadow: 0 0 10px rgba(76, 175, 80, 0.3);
    }
    .target-display {
      font-size: 24px;
      font-weight: bold;
      color: #2196F3;
    }
    .fault {
      color: #f44336;
      font-weight: bold;
      font-size: 18px;
    }
    .warning {
      color: #ff9800;
      font-weight: bold;
      font-size: 18px;
    }
    .ok {
      color: #4CAF50;
      font-weight: bold;
      font-size: 18px;
    }
    button {
      background-color: #2d2d2d;
      color: #e0e0e0;
      padding: 12px 20px;
      border: 1px solid #444;
      border-radius: 8px;
      cursor: pointer;
      margin: 8px;
      font-size: 16px;
      transition: all 0.3s;
    }
    button:hover {
      background-color: #3d3d3d;
      border-color: #666;
      transform: translateY(-2px);
    }
    button.danger {
      background-color: #5d1f1f;
      border-color: #f44336;
    }
    button.danger:hover {
      background-color: #7d2f2f;
    }
    button.success {
      background-color: #1f3d1f;
      border-color: #4CAF50;
    }
    button.success:hover {
      background-color: #2f4d2f;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      margin: 15px 0;
      background-color: #2d2d2d;
    }
    th, td {
      border: 1px solid #444;
      padding: 12px;
      text-align: center;
    }
    th {
      background-color: #333;
      color: #e0e0e0;
      font-weight: bold;
    }
    tr:nth-child(even) {
      background-color: #2a2a2a;
    }
    input[type=text], input[type=password], input[type=number] {
      width: 220px;
      padding: 10px;
      margin: 8px 0;
      border: 1px solid #444;
      border-radius: 6px;
      background-color: #2d2d2d;
      color: #e0e0e0;
      font-size: 16px;
    }
    input[type=number] {
      font-size: 18px;
      font-weight: bold;
    }
    .tab {
      overflow: hidden;
      border: 1px solid #444;
      background-color: #2d2d2d;
      border-radius: 10px 10px 0 0;
    }
    .tab button {
      background-color: inherit;
      float: left;
      border: none;
      border-bottom: 3px solid transparent;
      outline: none;
      cursor: pointer;
      padding: 14px 16px;
      transition: 0.3s;
      color: #aaa;
      font-weight: bold;
      border-radius: 0;
      margin: 0;
    }
    .tab button:hover {
      background-color: #3d3d3d;
      color: #fff;
    }
    .tab button.active {
      background-color: #1e1e1e;
      color: #4CAF50;
      border-bottom: 3px solid #4CAF50;
    }
    .tabcontent {
      display: none;
      padding: 25px;
      border: 1px solid #444;
      border-top: none;
      border-radius: 0 0 10px 10px;
    }
    h1 {
      color: #4CAF50;
      text-align: center;
      font-size: 32px;
      margin-bottom: 25px;
      text-shadow: 0 0 10px rgba(76, 175, 80, 0.2);
    }
    h2 {
      color: #e0e0e0;
      border-bottom: 2px solid #444;
      padding-bottom: 10px;
      margin-top: 0;
    }
    h3 {
      color: #bbb;
      margin-top: 20px;
    }
    p {
      line-height: 1.6;
    }
    .info-note {
      background-color: #2a3a2a;
      border-left: 4px solid #4CAF50;
      padding: 12px;
      margin: 15px 0;
      border-radius: 4px;
      font-size: 14px;
    }
    .warning-note {
      background-color: #3a2a2a;
      border-left: 4px solid #f44336;
      padding: 12px;
      margin: 15px 0;
      border-radius: 4px;
      font-size: 14px;
    }
    .control-label {
      font-size: 18px;
      font-weight: bold;
      color: #bbb;
      display: inline-block;
      width: 200px;
    }
    .control-value {
      font-size: 20px;
      font-weight: bold;
      color: #4CAF50;
    }
    .section-title {
      font-size: 22px;
      color: #4CAF50;
      margin-bottom: 15px;
      display: flex;
      align-items: center;
    }
    .section-title:before {
      content: "▶";
      margin-right: 10px;
      color: #4CAF50;
    }
    .temperature-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
      gap: 20px;
      margin: 20px 0;
    }
    .temp-card {
      background: linear-gradient(145deg, #2a2a2a, #252525);
      padding: 25px;
      border-radius: 12px;
      text-align: center;
      box-shadow: 0 5px 15px rgba(0,0,0,0.3);
    }
    .temp-card h3 {
      margin-top: 0;
      color: #bbb;
      font-size: 18px;
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
          if (document.getElementById('outsideTemp')) {
            document.getElementById('outsideTemp').innerHTML = '<span class="temperature-display">' + data.outsideTemp.toFixed(1) + ' °C</span>';
          }
          if (document.getElementById('waterTemp')) {
            document.getElementById('waterTemp').innerHTML = '<span class="temperature-display">' + data.waterTemp.toFixed(1) + ' °C</span>';
          }
          if (document.getElementById('targetTemp')) {
            document.getElementById('targetTemp').innerHTML = '<span class="target-display">' + data.targetTemp.toFixed(1) + ' °C</span>';
          }
          
          // Sensor statuses
          if (document.getElementById('outsideStatus')) {
            document.getElementById('outsideStatus').innerText = data.outsideStatus;
            document.getElementById('outsideStatus').className = data.outsideStatus === 'ERROR' ? 'fault' : 
                                                                data.outsideStatus === 'WARNING' ? 'warning' : 'ok';
          }
          if (document.getElementById('waterStatus')) {
            document.getElementById('waterStatus').innerText = data.waterStatus;
            document.getElementById('waterStatus').className = data.waterStatus === 'ERROR' ? 'fault' : 
                                                              data.waterStatus === 'WARNING' ? 'warning' : 'ok';
          }
          
          // Burner
          if (document.getElementById('burnerState')) {
            document.getElementById('burnerState').innerText = data.burnerState ? 'ON' : 'OFF';
            document.getElementById('burnerState').className = data.burnerState ? 'ok' : 'fault';
          }
          if (document.getElementById('burnerMode')) {
            document.getElementById('burnerMode').innerText = data.manualMode ? 'Manual' : 'Automatic';
          }
          if (document.getElementById('heatingDisabled')) {
            document.getElementById('heatingDisabled').innerText = data.heatingDisabled ? 'YES (10-min delay active)' : 'NO';
            document.getElementById('heatingDisabled').className = data.heatingDisabled ? 'warning' : 'ok';
          }
          
          // WiFi
          if (document.getElementById('wifiMode')) {
            document.getElementById('wifiMode').innerText = data.wifiMode;
          }
          if (document.getElementById('wifiIP')) {
            document.getElementById('wifiIP').innerText = data.wifiIP;
          }
          
          // Hysteresis
          if (document.getElementById('currentHysteresis')) {
            document.getElementById('currentHysteresis').innerHTML = '<span class="target-display">' + data.hysteresis.toFixed(1) + ' °C</span>';
          }
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
      if (confirm('⚠️ WARNING: Emergency shutdown?')) {
        fetch('/emergency')
          .then(() => {
            alert('✅ Emergency shutdown!');
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
        
        if (outside === '' || water === '') {
          alert('Error: All curve points must have values!');
          return;
        }
        if (parseFloat(outside) < -50 || parseFloat(outside) > 50) {
          alert('Error: Outside temperature must be between -50 and 50 °C!');
          return;
        }
        if (parseFloat(water) < 20 || parseFloat(water) > 90) {
          alert('Error: Water temperature must be between 20 and 90 °C!');
          return;
        }
        
        curveData.push(outside + ',' + water);
      }
      
      fetch('/setcurve?' + curveData.map((d, i) => `p${i}=${d}`).join('&'))
        .then(() => {
          alert('✅ Temperature curve saved!');
          updateData();
        });
    }
    
    // Load current temperature curve values
    function loadCurve() {
      fetch('/getcurve')
        .then(response => response.json())
        .then(data => {
          if (data && Array.isArray(data) && data.length >= 4) {
            for(let i = 0; i < 4; i++) {
              document.getElementById('editOutside' + i).value = data[i][0] || '';
              document.getElementById('editWater' + i).value = data[i][1] || '';
            }
          } else {
            loadDefaultCurveValues();
          }
        })
        .catch(error => {
          console.error('Error loading curve:', error);
          loadDefaultCurveValues();
        });
    }
    
    // Load default curve values
    function loadDefaultCurveValues() {
      const defaultCurve = [
        [-20.0, 75.0],
        [-10.0, 65.0],
        [0.0, 55.0],
        [10.0, 45.0]
      ];
      
      for(let i = 0; i < 4; i++) {
        document.getElementById('editOutside' + i).value = defaultCurve[i][0];
        document.getElementById('editWater' + i).value = defaultCurve[i][1];
      }
    }
    
    // Save WiFi settings
    function saveWifiConfig() {
      const apSsid = document.getElementById('apSsid').value;
      const apPassword = document.getElementById('apPassword').value;
      const useSta = document.getElementById('useSta').checked;
      const staSsid = document.getElementById('staSsid').value;
      const staPassword = document.getElementById('staPassword').value;
      
      if (apSsid.length < 1) {
        alert('Error: AP network name cannot be empty!');
        return;
      }
      
      if (useSta && staSsid.length < 1) {
        alert('Error: Network name cannot be empty when STA mode is enabled!');
        return;
      }
      
      const params = new URLSearchParams({
        ap_ssid: apSsid,
        ap_password: apPassword,
        use_sta: useSta ? '1' : '0',
        sta_ssid: staSsid,
        sta_password: staPassword
      });
      
      if (confirm('Save WiFi settings?\n\n⚠️ Device will restart.')) {
        fetch('/setwifi?' + params.toString())
          .then(response => response.text())
          .then(result => {
            alert('✅ WiFi settings saved!\n' + result);
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
        alert('Error: Hysteresis must be between 0.5 and 10.0 °C!');
        return;
      }
      
      fetch('/setsettings?hysteresis=' + hysteresis)
        .then(response => response.text())
        .then(result => {
          alert('✅ System settings saved!\n' + result);
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
      document.getElementsByClassName("tablink")[0].click();
      loadWifiConfig();
      loadCurve();
      loadSystemSettings();
      updateData();
    };
  </script>
</head>
<body>
  <div class="container">
    <h1>🔥 Oil Heating Control System</h1>
    
    <div class="tab">
      <button class="tablink" onclick="openTab(event, 'control')">Control Panel</button>
      <button class="tablink" onclick="openTab(event, 'wifi')">WiFi Settings</button>
      <button class="tablink" onclick="openTab(event, 'curve')">Temperature Curve</button>
      <button class="tablink" onclick="openTab(event, 'settings')">System Settings</button>
    </div>
    
    <!-- CONTROL TAB -->
    <div id="control" class="tabcontent">
      <div class="temperature-grid">
        <div class="temp-card">
          <h3>Outside Temperature</h3>
          <div id="outsideTemp" class="temperature-display">-- °C</div>
          <p>Status: <span id="outsideStatus" class="ok">OK</span></p>
        </div>
        
        <div class="temp-card">
          <h3>Water Temperature</h3>
          <div id="waterTemp" class="temperature-display">-- °C</div>
          <p>Status: <span id="waterStatus" class="ok">OK</span></p>
        </div>
        
        <div class="temp-card">
          <h3>Target Temperature</h3>
          <div id="targetTemp" class="target-display">-- °C</div>
          <p>Heating disabled: <span id="heatingDisabled" class="ok">--</span></p>
        </div>
      </div>
      
      <div class="status-box">
        <h2>System Status</h2>
        <p><span class="control-label">Outside sensor:</span> <span id="outsideStatusText" class="ok">OK</span></p>
        <p><span class="control-label">Water sensor:</span> <span id="waterStatusText" class="ok">OK</span></p>
        <p><span class="control-label">WiFi Mode:</span> <span id="wifiMode">--</span></p>
        <p><span class="control-label">IP Address:</span> <span id="wifiIP">--</span></p>
        <p><span class="control-label">Current Hysteresis:</span> <span id="currentHysteresis">-- °C</span></p>
      </div>
      
      <div class="control-box">
        <h2>Burner Control</h2>
        <p><span class="control-label">Burner State:</span> <span id="burnerState" class="ok">--</span></p>
        <p><span class="control-label">Control Mode:</span> <span id="burnerMode">--</span></p>
        
        <div class="info-note">
          <strong>Control Mode Info:</strong><br>
          • <strong>Automatic Mode:</strong> System controls burner with 10-minute delay feature<br>
          • <strong>Manual Mode:</strong> You control burner manually with ON/OFF buttons<br>
          • <strong>10-minute delay:</strong> When heating turns off due to warm outside temperature, it stays off for 10 minutes
        </div>
        
        <button onclick="setManualState(false)" class="success">Switch to Automatic Mode</button>
        <button onclick="setManualState(true)" class="success">Switch to Manual Mode</button>
        <br>
        <button onclick="setBurnerState(true)" class="success">Turn Burner ON</button>
        <button onclick="setBurnerState(false)" class="success">Turn Burner OFF</button>
        <br>
        <button onclick="emergencyStop()" class="danger">🚨 Emergency Stop</button>
        
        <div class="warning-note">
          <strong>⚠️ Emergency Stop Warning:</strong><br>
          This will immediately turn off the burner regardless of current mode or temperature conditions.
        </div>
      </div>
    </div>
    
    <!-- WIFI SETTINGS TAB -->
    <div id="wifi" class="tabcontent">
      <div class="config-box">
        <h2>WiFi Configuration</h2>
        
        <div class="info-note">
          <strong>Configuration Options:</strong><br>
          1. <strong>Access Point (AP) Mode:</strong> Device creates its own WiFi network<br>
          2. <strong>Station (STA) Mode:</strong> Device connects to your existing WiFi network<br>
          If STA connection fails, device will automatically fall back to AP mode.
        </div>
        
        <h3>Access Point (Own Network) Settings</h3>
        <p>Device creates its own WiFi network that you can connect to directly.</p>
        <p><strong>Network Name (SSID):</strong><br>
        <input type="text" id="apSsid" placeholder="Enter AP network name" value="OilHeaterAP"></p>
        <p><strong>Password:</strong><br>
        <input type="password" id="apPassword" placeholder="Enter AP password" value="enmuista"></p>
        
        <h3>Connect to Existing Network (Optional)</h3>
        <p><input type="checkbox" id="useSta" onclick="toggleStaFields()">
        <label for="useSta"><strong>Enable connection to existing network</strong></label></p>
        
        <div id="staFields" style="display: none;">
          <p><strong>Network Name (SSID):</strong><br>
          <input type="text" id="staSsid" placeholder="Enter your WiFi network name"></p>
          <p><strong>Password:</strong><br>
          <input type="password" id="staPassword" placeholder="Enter your WiFi password"></p>
        </div>
        
        <button onclick="saveWifiConfig()" class="success">💾 Save WiFi Settings</button>
        <button onclick="loadWifiConfig()">🔄 Load Current Settings</button>
        
        <div class="warning-note">
          <strong>⚠️ Important Notice:</strong><br>
          Device will restart automatically after saving WiFi settings.
        </div>
      </div>
    </div>
    
    <!-- TEMPERATURE CURVE TAB -->
    <div id="curve" class="tabcontent">
      <div class="curve-box">
        <h2>Temperature Curve Configuration</h2>
        
        <div class="info-note">
          <strong>How Temperature Curve Works:</strong><br>
          The system uses 4-point linear interpolation to calculate target water temperature based on outside temperature.<br>
          Example: If outside is -20°C → target water is 75°C, if outside is 10°C → target water is 45°C.<br>
          <strong>Important:</strong> Outside temperatures above 10°C will disable heating for 10 minutes.
        </div>
        
        <table>
          <tr>
            <th>Point</th>
            <th>Outside Temperature (°C)</th>
            <th>Target Water Temperature (°C)</th>
            <th>Description</th>
          </tr>
          <tr>
            <td>1</td>
            <td><input type="number" id="editOutside0" step="0.1" value="-20.0" style="font-size: 18px;"></td>
            <td><input type="number" id="editWater0" step="0.1" value="75.0" style="font-size: 18px;"></td>
            <td>Coldest point (max heating)</td>
          </tr>
          <tr>
            <td>2</td>
            <td><input type="number" id="editOutside1" step="0.1" value="-10.0" style="font-size: 18px;"></td>
            <td><input type="number" id="editWater1" step="0.1" value="65.0" style="font-size: 18px;"></td>
            <td>Cold weather</td>
          </tr>
          <tr>
            <td>3</td>
            <td><input type="number" id="editOutside2" step="0.1" value="0.0" style="font-size: 18px;"></td>
            <td><input type="number" id="editWater2" step="0.1" value="55.0" style="font-size: 18px;"></td>
            <td>Freezing point</td>
          </tr>
          <tr>
            <td>4</td>
            <td><input type="number" id="editOutside3" step="0.1" value="10.0" style="font-size: 18px;"></td>
            <td><input type="number" id="editWater3" step="0.1" value="45.0" style="font-size: 18px;"></td>
            <td>Warmest point (above = 10-min delay)</td>
          </tr>
        </table>
        
        <button onclick="saveCurve()" class="success">💾 Save Temperature Curve</button>
        <button onclick="loadCurve()">🔄 Load Current Curve</button>
        
        <div class="info-note">
          <strong>💡 Tip:</strong> Curve settings are saved to flash memory automatically.
        </div>
      </div>
    </div>
    
    <!-- SYSTEM SETTINGS TAB -->
    <div id="settings" class="tabcontent">
      <div class="settings-box">
        <h2>System Settings</h2>
        
        <div class="info-note">
          <strong>Control Stability Information:</strong><br>
          • <strong>Hysteresis:</strong> Prevents rapid on/off cycling of the burner<br>
          • <strong>10-minute Delay:</strong> When heating turns off due to warm outside temperature, it stays off for 10 minutes<br>
          • This prevents short cycling when outside temperature is near the cutoff point
        </div>
        
        <h3>Control Parameters</h3>
        <p><strong>Hysteresis Value:</strong><br>
        <input type="number" id="hysteresisValue" step="0.1" min="0.5" max="10.0" value="4.0" style="font-size: 20px; padding: 12px;"> °C</p>
        
        <p><em>Hysteresis prevents rapid on/off switching:<br>
        • <strong>Smaller value (0.5-2.0):</strong> More precise temperature control<br>
        • <strong>Larger value (2.0-10.0):</strong> More stable operation, less wear on burner</em></p>
        
        <button onclick="saveSystemSettings()" class="success">💾 Save System Settings</button>
        <button onclick="loadSystemSettings()">🔄 Load Current Settings</button>
        
        <div class="info-note">
          <strong>System Information:</strong><br>
          • 10-minute delay activates when outside temperature ≥ 10°C<br>
          • Water temperature is checked continuously for safety<br>
          • All settings are saved automatically<br>
          • Emergency stop overrides all normal operations
        </div>
      </div>
    </div>
  </div>
</body>
</html>
)rawliteral";

// ==================== HELPER FUNCTIONS ====================

/**
 * Update heating status in system state
 */
void updateHeatingStatus(bool disabled) {
    systemState.lockTempData();
    systemState.tempData.heatingDisabled = disabled;
    systemState.unlockTempData();
    
    if (disabled) {
        xEventGroupSetBits(xSystemEvents, BIT_HEATING_DISABLED);
    } else {
        xEventGroupClearBits(xSystemEvents, BIT_HEATING_DISABLED);
    }
}

/**
 * Set burner state (thread-safe wrapper)
 */
void setBurnerState(bool state) {
    if (systemState.setBurnerState(state)) {
        RelayCommand cmd(RelayCommand::SET_STATE, state);
        xQueueSend(xRelayControlQueue, &cmd, 0);
    }
}

/**
 * Apply hysteresis control
 */
void applyHysteresis(const SystemControlSnapshot& snapshot, float targetTemp) {
    bool shouldTurnOn = !snapshot.burnerState && 
                        snapshot.waterTemp < (targetTemp - snapshot.hysteresis);
    
    bool shouldTurnOff = snapshot.burnerState && 
                         snapshot.waterTemp > (targetTemp + snapshot.hysteresis);
    
    if (shouldTurnOn) {
        setBurnerState(true);
        #if DEBUG_CONTROL
        Serial.printf("[CONTROL] ON: Water %.1f < Target %.1f - %.1f\n",
                     snapshot.waterTemp, targetTemp, snapshot.hysteresis);
        #endif
    }
    else if (shouldTurnOff) {
        setBurnerState(false);
        #if DEBUG_CONTROL
        Serial.printf("[CONTROL] OFF: Water %.1f > Target %.1f + %.1f\n",
                     snapshot.waterTemp, targetTemp, snapshot.hysteresis);
        #endif
    }
}

/**
 * Simple burner control
 */
void controlBurnerSimple(const SystemControlSnapshot& snap, float targetTemp) {
    bool shouldBeOn = !snap.burnerState && 
                      snap.waterTemp < targetTemp - snap.hysteresis;
    
    bool shouldBeOff = snap.burnerState && 
                       snap.waterTemp > targetTemp + snap.hysteresis;
    
    if (shouldBeOn) setBurnerState(true);
    else if (shouldBeOff) setBurnerState(false);
}

// ==================== WIFI FUNCTIONS ====================

/**
 * Initialize WiFi connection
 */
bool initWiFi() {
    Serial.println("Initializing WiFi...");
    
    preferences.begin("oilheater", true);
    systemState.lockWifiConfig();
    preferences.getString("ap_ssid", systemState.wifiConfig.ap_ssid, 32);
    preferences.getString("ap_password", systemState.wifiConfig.ap_password, 32);
    preferences.getString("sta_ssid", systemState.wifiConfig.sta_ssid, 32);
    preferences.getString("sta_password", systemState.wifiConfig.sta_password, 32);
    systemState.wifiConfig.use_sta = preferences.getBool("use_sta", false);
    systemState.unlockWifiConfig();
    preferences.end();
    
    Serial.printf("AP SSID: %s\n", systemState.wifiConfig.ap_ssid);
    Serial.printf("STA enabled: %s\n", systemState.wifiConfig.use_sta ? "YES" : "NO");
    
    bool connected = false;
    
    // Try STA mode if configured
    if (systemState.wifiConfig.isStaConfigured()) {
        Serial.printf("Connecting to network: %s\n", systemState.wifiConfig.sta_ssid);
        
        WiFi.disconnect(true);
        WiFi.mode(WIFI_STA);
        WiFi.begin(systemState.wifiConfig.sta_ssid, systemState.wifiConfig.sta_password);
        
        unsigned long startTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_CONNECT_TIMEOUT) {
            delay(500);
            Serial.print(".");
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            connected = true;
            systemState.lockWifiConfig();
            systemState.wifiConfig.connected = true;
            systemState.wifiConfig.ap_mode = false;
            systemState.unlockWifiConfig();
            
            Serial.println("\nConnected to network!");
            Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
        } else {
            Serial.println("\nNetwork connection failed");
            WiFi.disconnect(true);
        }
    }
    
    // Fall back to AP mode
    if (!connected) {
        Serial.println("Starting Access Point mode...");
        
        WiFi.disconnect(true);
        WiFi.mode(WIFI_AP);
        
        systemState.lockWifiConfig();
        if (strlen(systemState.wifiConfig.ap_ssid) == 0) {
            strcpy(systemState.wifiConfig.ap_ssid, DEFAULT_AP_SSID);
        }
        if (strlen(systemState.wifiConfig.ap_password) == 0) {
            strcpy(systemState.wifiConfig.ap_password, DEFAULT_AP_PASSWORD);
        }
        systemState.unlockWifiConfig();
        
        IPAddress local_ip(192, 168, 4, 1);
        IPAddress gateway(192, 168, 4, 1);
        IPAddress subnet(255, 255, 255, 0);
        
        WiFi.softAPConfig(local_ip, gateway, subnet);
        WiFi.softAP(systemState.wifiConfig.ap_ssid, systemState.wifiConfig.ap_password);
        
        systemState.lockWifiConfig();
        systemState.wifiConfig.connected = true;
        systemState.wifiConfig.ap_mode = true;
        systemState.unlockWifiConfig();
        
        Serial.println("Access Point created!");
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
    
    auto wifiConfig = systemState.getWifiConfigSnapshot();
    preferences.putString("ap_ssid", wifiConfig.ap_ssid);
    preferences.putString("ap_password", wifiConfig.ap_password);
    preferences.putString("sta_ssid", wifiConfig.sta_ssid);
    preferences.putString("sta_password", wifiConfig.sta_password);
    preferences.putBool("use_sta", wifiConfig.use_sta);
    
    preferences.end();
    Serial.println("WiFi settings saved to flash");
}

/**
 * Load system settings from flash memory
 */
void loadSystemSettings() {
    Serial.println("Loading system settings from flash...");
    
    preferences.begin("oilheater", true);
    
    systemState.lockSettings();
    
    // Load temperature curve
    bool curveLoaded = false;
    for (int i = 0; i < 4; i++) {
        String keyOut = "curve_out" + String(i);
        String keyWater = "curve_water" + String(i);
        
        float outTemp = preferences.getFloat(keyOut.c_str(), -999.0);
        float waterTemp = preferences.getFloat(keyWater.c_str(), -999.0);
        
        if (outTemp != -999.0 && waterTemp != -999.0) {
            systemState.settingsData.curve.points[i][0] = outTemp;
            systemState.settingsData.curve.points[i][1] = waterTemp;
            curveLoaded = true;
        }
    }
    
    // Load hysteresis
    float hysteresis = preferences.getFloat("hysteresis", -999.0);
    if (hysteresis != -999.0) {
        systemState.settingsData.settings.hysteresis = hysteresis;
    }
    
    systemState.unlockSettings();
    
    preferences.end();
    
    if (curveLoaded) {
        Serial.println("Temperature curve loaded from flash");
    } else {
        Serial.println("Using default temperature curve");
    }
    
    Serial.printf("Hysteresis: %.1f°C\n", systemState.settingsData.settings.hysteresis);
}

/**
 * Save system settings to flash memory
 */
void saveSystemSettings() {
    auto settings = systemState.getSettingsSnapshot();
    
    if (!settings.settings.settingsDirty) {
        return;
    }
    
    Serial.println("Saving system settings to flash...");
    
    preferences.begin("oilheater", false);
    
    // Save temperature curve
    for (int i = 0; i < 4; i++) {
        String keyOut = "curve_out" + String(i);
        String keyWater = "curve_water" + String(i);
        
        preferences.putFloat(keyOut.c_str(), settings.curve.points[i][0]);
        preferences.putFloat(keyWater.c_str(), settings.curve.points[i][1]);
    }
    
    // Save hysteresis
    preferences.putFloat("hysteresis", settings.settings.hysteresis);
    
    preferences.end();
    
    // Clear dirty flag
    systemState.lockSettings();
    systemState.settingsData.settings.settingsDirty = false;
    systemState.unlockSettings();
    
    Serial.println("System settings saved to flash");
}

// ==================== TASK IMPLEMENTATIONS ====================

/**
 * Temperature reading task
 */
void temperatureTask(void *parameter) {
    Serial.println("Temperature task started");
    
    xEventGroupSetBits(xSystemEvents, BIT_TEMP_TASK_ALIVE);
    
    sensorsOutside.begin();
    sensorsWater.begin();
    sensorsOutside.setResolution(SENSOR_RESOLUTION);
    sensorsWater.setResolution(SENSOR_RESOLUTION);
    sensorsOutside.setWaitForConversion(false);
    sensorsWater.setWaitForConversion(false);
    
    enum ReadState { IDLE, REQUEST_OUTSIDE, WAIT_OUTSIDE, REQUEST_WATER, WAIT_WATER };
    ReadState readState = IDLE;
    unsigned long requestTime = 0;
    
    while (1) {
        unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        switch (readState) {
            case IDLE:
                vTaskDelay(pdMS_TO_TICKS(TEMP_READ_INTERVAL_MS));
                
                xEventGroupSetBits(xSystemEvents, BIT_TEMP_TASK_ALIVE);
                
                if (sensorsOutside.getDeviceCount() == 0) {
                    Serial.println("WARNING: Outside sensor not found!");
                    sensorsOutside.begin();
                }
                if (sensorsWater.getDeviceCount() == 0) {
                    Serial.println("WARNING: Water sensor not found!");
                    sensorsWater.begin();
                }
                
                sensorsOutside.requestTemperatures();
                readState = REQUEST_OUTSIDE;
                requestTime = currentTime;
                break;
                
            case REQUEST_OUTSIDE:
                if (currentTime - requestTime > 100) {
                    readState = WAIT_OUTSIDE;
                }
                break;
                
            case WAIT_OUTSIDE:
                if (currentTime - requestTime > 750) {
                    float outsideTemp = sensorsOutside.getTempCByIndex(0);
                    
                    #if DEBUG_TEMP
                    Serial.printf("[TEMP] OUTSIDE: %.2f °C\n", outsideTemp);
                    #endif
                    
                    systemState.updateSensorReading(outsideTemp, false);
                    
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
                    Serial.printf("[TEMP] WATER: %.2f °C\n", waterTemp);
                    #endif
                    
                    systemState.updateSensorReading(waterTemp, true);
                    
                    auto tempSnapshot = systemState.getTempDataSnapshot();
                    float outsideTempToUse = tempSnapshot.outsideStatus.fault ? 
                                           tempSnapshot.outsideData.getAverageValidValue() : 
                                           tempSnapshot.outsideData.currentTemp;
                    
                    systemState.updateTargetTemperature(outsideTempToUse);
                    
                    if (tempSnapshot.heatingDisabled) {
                        xEventGroupSetBits(xSystemEvents, BIT_HEATING_DISABLED);
                    } else {
                        xEventGroupClearBits(xSystemEvents, BIT_HEATING_DISABLED);
                    }
                    
                    readState = IDLE;
                }
                break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * Control logic - 10-min delay starts when relay is ON AND outside > cutoff
 * Hysteresis continues to work DURING delay and can cancel delay
 */
void controlTask(void *parameter) {
    Serial.println("Control task - Target temp freezing during delay");
    static float oldTagetTemp=0;
    unsigned long delayEndTime = 0;
    bool delayActive = false;
    vTaskDelay(pdMS_TO_TICKS(20000));
    while (1) {
        unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // 1. Get current readings
        SystemControlSnapshot snapshot = systemState.getControlSnapshot();
        auto settings = systemState.getSettingsSnapshot();
        float cutoffTemp = settings.curve.points[3][0];
        bool releOn = snapshot.burnerState;  // true = ON, false = OFF
        // 2. Calculate current target (for normal operation)
        float currentTarget = settings.calculateTargetTemp(snapshot.outsideTemp);
        
        if(releOn && oldTagetTemp>0 && currentTarget ==0 && !delayActive){ 
            currentTarget=oldTagetTemp;
            // Start delay and freeze values
            delayActive= true;
            delayEndTime = currentTime + 600000; 
            Serial.printf("[DELAY START]");
        }
        if(!delayActive){
          oldTagetTemp=currentTarget;
        }
        if(delayActive && releOn ){
          currentTarget=oldTagetTemp;
        }
        if(!releOn){
          delayActive= false;
        } 

        // Check if delay time has expired
         if (delayActive && currentTime >= delayEndTime) {
                delayActive = false;
                Serial.println("[DELAY] 10-min viive päättyi");       
                  // Hysteresis turns relay OFF
                setBurnerState(false);
                    // Cancel delay since relay turned off
                 delayActive = false;
                Serial.printf("[DELAY CANCEL] Hystereesi sammutti releen\n");
         }
         if(delayActive){
                // Show status
                unsigned long remaining = (delayEndTime - currentTime) / 1000;
                static unsigned long lastStatus = 0;
                if (currentTime - lastStatus > 30000) {
                    lastStatus = currentTime;
                    Serial.printf("[DELAY] Aktiivinen: %lu min jäljellä, Target=%.1f°C\n",
                                 remaining / 60, currentTarget);
                }
          }
        
         // Apply normal hysteresis with CURRENT target temp
         applyHysteresis(snapshot, currentTarget);
        
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * Relay control task
 */
void relayTask(void *parameter) {
    Serial.println("Relay task started");
    
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    
    RelayCommand cmd;
    
    while (1) {
        if (xQueueReceive(xRelayControlQueue, &cmd, portMAX_DELAY)) {
            
            EventBits_t events = xEventGroupGetBits(xSystemEvents);
            bool anySeriousFault = (events & (BIT_SENSOR_FAULT_OUTSIDE | BIT_SENSOR_FAULT_WATER)) != 0;
            bool emergencyStop = (events & BIT_EMERGENCY_STOP) != 0;
            bool heatingDisabled = (events & BIT_HEATING_DISABLED) != 0;
            
            bool shouldTurnOn = false;
            
            switch (cmd.type) {
                case RelayCommand::SET_STATE:
                    if (!anySeriousFault && !heatingDisabled && !emergencyStop) {
                        shouldTurnOn = cmd.state;
                    } else if (systemState.burnerData.isManualMode() && cmd.state) {
                        shouldTurnOn = true;
                    }
                    break;
                    
                case RelayCommand::FORCE_OFF:
                    shouldTurnOn = false;
                    break;
                    
                case RelayCommand::EMERGENCY_OFF:
                    shouldTurnOn = false;
                    xEventGroupSetBits(xSystemEvents, BIT_EMERGENCY_STOP);
                    break;
            }
            
            digitalWrite(RELAY_PIN, shouldTurnOn ? HIGH : LOW);
            
            if (cmd.type == RelayCommand::SET_STATE) {
                systemState.setBurnerState(shouldTurnOn);
            }
            
            Serial.printf("[RELAY] State: %s\n", shouldTurnOn ? "ON" : "OFF");
        }
    }
}

/**
 * Watchdog monitoring task
 */
void watchdogTask(void *parameter) {
    Serial.println("Watchdog task started");
    
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);
    
    unsigned long lastTempTaskAlive = millis();
    
    while (1) {
        unsigned long currentTime = millis();
        
        EventBits_t events = xEventGroupGetBits(xSystemEvents);
        if (events & BIT_TEMP_TASK_ALIVE) {
            lastTempTaskAlive = currentTime;
            xEventGroupClearBits(xSystemEvents, BIT_TEMP_TASK_ALIVE);
        }
        
        if (currentTime - lastTempTaskAlive > 60000) {
            Serial.println("WATCHDOG: Temperature task not responding!");
            
            RelayCommand cmd(RelayCommand::EMERGENCY_OFF);
            xQueueSend(xRelayControlQueue, &cmd, 0);
            
            lastTempTaskAlive = currentTime;
        }
        
        esp_task_wdt_reset();
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * Flash storage task
 */
void flashTask(void *parameter) {
    Serial.println("Flash task started");
    
    while (1) {
        auto settings = systemState.getSettingsSnapshot();
        
        if (settings.settings.settingsDirty) {
            unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            if (currentTime - settings.settings.lastSettingsChange > 5000) {
                saveSystemSettings();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * Web server task
 */
void webServerTask(void *parameter) {
    Serial.println("Web server task started");
    
    delay(2000);
    
    // ========== HTTP ROUTES ==========
    
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", htmlPage);
    });
    
    server.on("/data", HTTP_GET, []() {
        SystemControlSnapshot snapshot = systemState.getControlSnapshot();
        auto wifiSnapshot = systemState.getWifiConfigSnapshot();
        
        String outsideStatus = "OK";
        auto tempSnapshot = systemState.getTempDataSnapshot();
        if (tempSnapshot.outsideStatus.fault) outsideStatus = "ERROR";
        else if (tempSnapshot.outsideStatus.temporaryFault) outsideStatus = "WARNING";
        
        String waterStatus = "OK";
        if (tempSnapshot.waterStatus.fault) waterStatus = "ERROR";
        else if (tempSnapshot.waterStatus.temporaryFault) waterStatus = "WARNING";
        
        String json = "{";
        json += "\"outsideTemp\":" + String(snapshot.outsideTemp, 1);
        json += ",\"waterTemp\":" + String(snapshot.waterTemp, 1);
        json += ",\"targetTemp\":" + String(snapshot.targetTemp, 1);
        json += ",\"burnerState\":" + String(snapshot.burnerState ? "true" : "false");
        json += ",\"manualMode\":" + String(snapshot.manualMode ? "true" : "false");
        json += ",\"outsideStatus\":\"" + outsideStatus + "\"";
        json += ",\"waterStatus\":\"" + waterStatus + "\"";
        json += ",\"heatingDisabled\":" + String(snapshot.heatingDisabled ? "true" : "false");
        
        String wifiMode = wifiSnapshot.ap_mode ? "AP (Own Network)" : "STA (Network)";
        String wifiIP = wifiSnapshot.ap_mode ? 
                       WiFi.softAPIP().toString() : 
                       WiFi.localIP().toString();
        
        json += ",\"wifiMode\":\"" + wifiMode + "\"";
        json += ",\"wifiIP\":\"" + wifiIP + "\"";
        json += ",\"hysteresis\":" + String(snapshot.hysteresis, 1);
        json += "}";
        
        server.send(200, "application/json", json);
    });
    
    server.on("/control", HTTP_GET, []() {
        if (server.hasArg("manual")) {
            bool manual = server.arg("manual").toInt() == 1;
            systemState.setManualMode(manual);
            Serial.printf("Manual mode: %s\n", manual ? "ON" : "OFF");
        }
        
        if (server.hasArg("burner")) {
            bool burner = server.arg("burner").toInt() == 1;
            auto burnerSnapshot = systemState.getBurnerDataSnapshot();
            
            if (burnerSnapshot.burnerManualOverride) {
                systemState.setBurnerState(burner);
                
                RelayCommand cmd(RelayCommand::SET_STATE, burner);
                xQueueSend(xRelayControlQueue, &cmd, 0);
                
                Serial.printf("Manual burner control: %s\n", burner ? "ON" : "OFF");
            }
        }
        
        server.send(200, "text/plain", "OK");
    });
    
    server.on("/setcurve", HTTP_GET, []() {
        CurvePoints newCurve;
        bool valid = true;
        
        for (int i = 0; i < 4; i++) {
            if (server.hasArg("p" + String(i))) {
                String values = server.arg("p" + String(i));
                int commaPos = values.indexOf(',');
                
                if (commaPos > 0) {
                    newCurve.points[i][0] = values.substring(0, commaPos).toFloat();
                    newCurve.points[i][1] = values.substring(commaPos + 1).toFloat();
                } else {
                    valid = false;
                    break;
                }
            } else {
                valid = false;
                break;
            }
        }
        
        if (valid && newCurve.isValid()) {
            if (systemState.updateCurve(newCurve)) {
                server.send(200, "text/plain", "Curve updated successfully");
            } else {
                server.send(500, "text/plain", "Failed to update curve");
            }
        } else {
            server.send(400, "text/plain", "Invalid curve data");
        }
    });
    
    server.on("/getcurve", HTTP_GET, []() {
        auto settings = systemState.getSettingsSnapshot();
        
        String json = "[";
        for (int i = 0; i < 4; i++) {
            json += "[";
            json += String(settings.curve.points[i][0], 1);
            json += ",";
            json += String(settings.curve.points[i][1], 1);
            json += "]";
            if (i < 3) json += ",";
        }
        json += "]";
        
        server.send(200, "application/json", json);
    });
    
    server.on("/setsettings", HTTP_GET, []() {
        if (server.hasArg("hysteresis")) {
            float hysteresis = server.arg("hysteresis").toFloat();
            
            if (systemState.updateHysteresis(hysteresis)) {
                server.send(200, "text/plain", "Settings updated successfully");
            } else {
                server.send(400, "text/plain", "Invalid hysteresis value");
            }
        } else {
            server.send(400, "text/plain", "Missing parameters");
        }
    });
    
    server.on("/getsettings", HTTP_GET, []() {
        auto settings = systemState.getSettingsSnapshot();
        
        String json = "{";
        json += "\"hysteresis\":" + String(settings.settings.hysteresis, 1);
        json += "}";
        
        server.send(200, "application/json", json);
    });
    
    server.on("/emergency", HTTP_GET, []() {
        RelayCommand cmd(RelayCommand::EMERGENCY_OFF);
        xQueueSend(xRelayControlQueue, &cmd, 0);
        
        Serial.println("Emergency shutdown via web interface");
        server.send(200, "text/plain", "Emergency shutdown initiated");
    });
    
    server.on("/getwifi", HTTP_GET, []() {
        auto wifiConfig = systemState.getWifiConfigSnapshot();
        
        String json = "{";
        json += "\"ap_ssid\":\"" + String(wifiConfig.ap_ssid) + "\"";
        json += ",\"ap_password\":\"" + String(wifiConfig.ap_password) + "\"";
        json += ",\"use_sta\":" + String(wifiConfig.use_sta ? "true" : "false");
        json += ",\"sta_ssid\":\"" + String(wifiConfig.sta_ssid) + "\"";
        json += ",\"sta_password\":\"" + String(wifiConfig.sta_password) + "\"";
        json += "}";
        
        server.send(200, "application/json", json);
    });
    
    server.on("/setwifi", HTTP_GET, []() {
        auto wifiConfig = systemState.getWifiConfigSnapshot();
        bool changed = false;
        
        if (server.hasArg("ap_ssid")) {
            strncpy(wifiConfig.ap_ssid, server.arg("ap_ssid").c_str(), 31);
            changed = true;
        }
        if (server.hasArg("ap_password")) {
            strncpy(wifiConfig.ap_password, server.arg("ap_password").c_str(), 31);
            changed = true;
        }
        if (server.hasArg("use_sta")) {
            wifiConfig.use_sta = server.arg("use_sta").toInt() == 1;
            changed = true;
        }
        if (server.hasArg("sta_ssid")) {
            strncpy(wifiConfig.sta_ssid, server.arg("sta_ssid").c_str(), 31);
            changed = true;
        }
        if (server.hasArg("sta_password")) {
            strncpy(wifiConfig.sta_password, server.arg("sta_password").c_str(), 31);
            changed = true;
        }
        
        if (changed) {
            preferences.begin("oilheater", false);
            preferences.putString("ap_ssid", wifiConfig.ap_ssid);
            preferences.putString("ap_password", wifiConfig.ap_password);
            preferences.putString("sta_ssid", wifiConfig.sta_ssid);
            preferences.putString("sta_password", wifiConfig.sta_password);
            preferences.putBool("use_sta", wifiConfig.use_sta);
            preferences.end();
            
            server.send(200, "text/plain", "WiFi settings saved. Restarting...");
            delay(1000);
            ESP.restart();
        } else {
            server.send(200, "text/plain", "No changes made");
        }
    });
    
    server.onNotFound([]() {
        server.send(404, "text/plain", "Page not found");
    });
    
    server.begin();
    Serial.println("HTTP server started on port 80");
    
    auto wifiConfig = systemState.getWifiConfigSnapshot();
    if (wifiConfig.ap_mode) {
        Serial.println("\n=== WEB INTERFACE READY ===");
        Serial.println("Connect to: " + String(wifiConfig.ap_ssid));
        Serial.println("Password: " + String(wifiConfig.ap_password));
        Serial.println("URL: http://" + WiFi.softAPIP().toString());
    } else {
        Serial.println("\n=== WEB INTERFACE READY ===");
        Serial.println("URL: http://" + WiFi.localIP().toString());
    }
    
    while (1) {
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== ARDUINO SETUP ====================

/**
 * Setup function
 */
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n=== OIL HEATING CONTROL SYSTEM ===");
    Serial.println("Version: 3.0 (10-minute delay feature)");
    Serial.println("Initializing...\n");
    
    // 1. Create Event Group
    xSystemEvents = xEventGroupCreate();
    if (!xSystemEvents) {
        Serial.println("ERROR: Failed to create Event Group!");
        while(1);
    }
    
    // 2. Create Relay Command Queue
    xRelayControlQueue = xQueueCreate(10, sizeof(RelayCommand));
    if (!xRelayControlQueue) {
        Serial.println("ERROR: Failed to create relay queue!");
        while(1);
    }
    
    // 3. Load settings from flash
    loadSystemSettings();
    
    // 4. Initialize WiFi
    if (!initWiFi()) {
        Serial.println("ERROR: WiFi initialization failed!");
        while(1);
    }
    
    // 5. Create tasks
    
    // Watchdog (highest priority)
    xTaskCreatePinnedToCore(
        watchdogTask,
        "Watchdog",
        TASK_STACK_SMALL,
        NULL,
        TASK_PRIORITY_CRITICAL,
        &xWatchdogTaskHandle,
        0
    );
    
    // Relay control (high priority)
    xTaskCreatePinnedToCore(
        relayTask,
        "Relay",
        TASK_STACK_SMALL,
        NULL,
        TASK_PRIORITY_HIGH,
        &xRelayTaskHandle,
        1
    );
    
    // Temperature reading
    xTaskCreatePinnedToCore(
        temperatureTask,
        "Temp",
        TASK_STACK_MEDIUM,
        NULL,
        TASK_PRIORITY_HIGH,
        &xTemperatureTaskHandle,
        1
    );
    
    // Control logic
    xTaskCreatePinnedToCore(
        controlTask,
        "Control",
        TASK_STACK_MEDIUM,
        NULL,
        TASK_PRIORITY_MEDIUM,
        &xControlTaskHandle,
        1
    );
    
    // Flash storage
    xTaskCreatePinnedToCore(
        flashTask,
        "Flash",
        TASK_STACK_SMALL,
        NULL,
        TASK_PRIORITY_LOW,
        &xFlashTaskHandle,
        1
    );
    
    // Wait for system to stabilize
    delay(3000);
    
    // Web server
    xTaskCreatePinnedToCore(
        webServerTask,
        "WebServer",
        TASK_STACK_LARGE,
        NULL,
        TASK_PRIORITY_LOW,
        &xWebServerTaskHandle,
        0
    );
    
    Serial.println("\n=== SYSTEM INITIALIZATION COMPLETE ===");
    Serial.println("All tasks created successfully");
    
    // Print initial settings
    auto settings = systemState.getSettingsSnapshot();
    Serial.println("\nInitial Settings:");
    Serial.printf("  Hysteresis: %.1f°C\n", settings.settings.hysteresis);
    Serial.println("  Temperature Curve:");
    for (int i = 0; i < 4; i++) {
        Serial.printf("    Point %d: %.1f°C -> %.1f°C\n", 
                     i + 1,
                     settings.curve.points[i][0],
                     settings.curve.points[i][1]);
    }
    
    Serial.println("\nNEW FEATURE: 10-minute delay after heating turns off");
    Serial.println("  When outside temperature ≥ 10°C, heating stays off for 10 minutes");
    Serial.println("  Prevents short cycling when temperature is near cutoff point\n");
}

// ==================== ARDUINO LOOP ====================

/**
 * Main loop
 */
void loop() {
    static unsigned long lastWifiCheck = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastWifiCheck > WIFI_RECONNECT_INTERVAL) {
        lastWifiCheck = currentTime;
        
        auto wifiConfig = systemState.getWifiConfigSnapshot();
        if (wifiConfig.use_sta && !wifiConfig.ap_mode) {
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("WiFi disconnected, attempting reconnect...");
                WiFi.reconnect();
                
                systemState.lockWifiConfig();
                systemState.wifiConfig.connected = (WiFi.status() == WL_CONNECTED);
                systemState.unlockWifiConfig();
            }
        }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
}
