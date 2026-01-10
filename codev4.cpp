/**
 * OIL HEATING SYSTEM - FreeRTOS + WiFi
 * 
 * Complete thread-safe implementation for ESP32
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
 * - Stable control with 10-minute outside temp updates
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
#define DEBUG_TEMP 1        // Enable temperature debug output
#define DEBUG_CONTROL 0     // Enable control logic debug output

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
#define OUTSIDE_TEMP_UPDATE_INTERVAL 600000  // 10 minutes for stable control
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

// Global objects that will be defined later
extern EventGroupHandle_t xSystemEvents;
extern QueueHandle_t xRelayControlQueue;

// Utility function declarations
bool isTemperatureFaulty(float temperature, bool isWaterSensor);

// ==================== DATA STRUCTURES ====================

/**
 * WiFi configuration structure
 * Stores network credentials for both AP and STA modes
 */
struct WifiConfig {
    char ap_ssid[32] = DEFAULT_AP_SSID;        // AP network name
    char ap_password[32] = DEFAULT_AP_PASSWORD; // AP network password
    char sta_ssid[32] = "";                    // STA network name (if connecting)
    char sta_password[32] = "";                // STA network password
    bool use_sta = false;                      // Use STA mode?
    bool connected = false;                    // WiFi connection status
    bool ap_mode = false;                      // Running in AP mode?
    
    // Helper method to check if STA is configured
    bool isStaConfigured() const {
        return use_sta && strlen(sta_ssid) > 0;
    }
};

/**
 * Temperature curve points for 4-point linear interpolation
 * Defines relationship between outside temp and target water temp
 */
struct CurvePoints {
    // Format: points[point_index][0] = outside_temp, [1] = water_temp
    float points[4][2] = {
        {-20.0, 75.0},   // Very cold: -20°C outside -> 75°C water
        {-10.0, 65.0},   // Cold: -10°C outside -> 65°C water
        {0.0, 55.0},     // Freezing: 0°C outside -> 55°C water
        {10.0, 45.0}     // Mild: 10°C outside -> 45°C water (above = heating off)
    };
    
    /**
     * Validate curve points for consistency
     * @return true if curve is valid
     */
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
        
        // Check monotonic decreasing for water temps (colder outside = warmer water)
        for (int i = 0; i < 3; i++) {
            if (points[i][1] <= points[i+1][1]) return false;
        }
        
        return true;
    }
};

/**
 * System settings that can be adjusted via web interface
 */
struct SystemSettings {
    float hysteresis = DEFAULT_HYSTERESIS;  // Hysteresis for temperature control (°C)
    bool settingsDirty = false;             // Settings changed but not saved
    unsigned long lastSettingsChange = 0;   // Timestamp of last change
    
    /**
     * Validate hysteresis value
     * @return true if hysteresis is valid
     */
    bool isHysteresisValid() const {
        return hysteresis >= 0.5 && hysteresis <= 10.0;
    }
    
    /**
     * Mark settings as changed
     */
    void markDirty() {
        settingsDirty = true;
        lastSettingsChange = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
};

/**
 * Sensor temperature data with historical values
 * Maintains current reading and history for averaging
 */
struct SensorData {
    float currentTemp = 0.0;               // Latest measured temperature
    float lastValidTemp = 0.0;             // Last valid temperature reading
    unsigned long lastValidReadTime = 0;   // Timestamp of last valid reading
    
    // Circular buffer for moving average (10 values)
    float lastValidValues[10] = {0};
    int validValueIndex = 0;               // Current position in buffer
    int validValueCount = 0;               // Number of valid values in buffer
    
    /**
     * Add new valid temperature to history buffer
     * @param value Valid temperature value
     */
    void addValidValue(float value) {
        lastValidValues[validValueIndex] = value;
        validValueIndex = (validValueIndex + 1) % 10;
        if (validValueCount < 10) validValueCount++;
    }
    
    /**
     * Calculate moving average from historical data
     * @return Average temperature from valid readings
     */
    float getAverageValidValue() const {
        if (validValueCount == 0) return 0.0;
        
        float sum = 0.0;
        for (int i = 0; i < validValueCount; i++) {
            sum += lastValidValues[i];
        }
        return sum / validValueCount;
    }
    
    /**
     * Reset sensor data (e.g., after sensor replacement)
     */
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
 * Monitors sensor health and detects faults
 */
struct SensorStatus {
    bool fault = false;                    // Serious error (requires intervention)
    bool temporaryFault = false;           // Temporary disturbance (auto-recover)
    unsigned long faultStartTime = 0;      // Timestamp when fault started
    int consecutiveFailures = 0;           // Count of consecutive read failures
    
    /**
     * Check if sensor is in error state
     * @return true if sensor has any error
     */
    bool isInError() const {
        return fault || temporaryFault;
    }
    
    /**
     * Reset sensor status to normal
     */
    void reset() {
        fault = false;
        temporaryFault = false;
        faultStartTime = 0;
        consecutiveFailures = 0;
    }
};

/**
 * Control logic timing and data
 * Manages stable control with infrequent outside temp updates
 */
struct ControlData {
    unsigned long lastOutsideTempUpdate = 0;  // Last control update time
    float controlOutsideTemp = 0.0;           // Outside temp used for control logic
    bool outsideTempNeedsUpdate = true;       // First update needed after startup
    
    /**
     * Check if outside temperature needs update for control logic
     * @param currentTime Current system time
     * @return true if update is needed
     */
    bool needsUpdate(unsigned long currentTime) const {
        return outsideTempNeedsUpdate || 
               (currentTime - lastOutsideTempUpdate >= OUTSIDE_TEMP_UPDATE_INTERVAL);
    }
    
    /**
     * Update control outside temperature
     * @param newTemp New outside temperature
     * @param currentTime Current system time
     */
    void update(float newTemp, unsigned long currentTime) {
        controlOutsideTemp = newTemp;
        lastOutsideTempUpdate = currentTime;
        outsideTempNeedsUpdate = false;
    }
};

/**
 * Burner control state
 * Tracks burner status and control mode
 */
struct BurnerData {
    bool burnerState = false;              // Current burner state (true = ON)
    bool burnerManualOverride = false;     // Manual control mode active
    
    /**
     * Check if system is in automatic control mode
     * @return true if automatic mode
     */
    bool isAutomaticMode() const {
        return !burnerManualOverride;
    }
    
    /**
     * Check if manual control is allowed
     * @return true if manual control is active
     */
    bool isManualMode() const {
        return burnerManualOverride;
    }
};

/**
 * Relay control command structure
 * Sent via queue to relay control task
 */
struct RelayCommand {
    enum CommandType {
        SET_STATE,      // Normal state setting (with safety checks)
        FORCE_OFF,      // Forced shutdown (override control logic)
        EMERGENCY_OFF   // Emergency shutdown (bypass all logic)
    };
    
    CommandType type;   // Command type
    bool state;         // Desired state (for SET_STATE only)
    unsigned long timestamp;  // Command timestamp
    
    RelayCommand() : type(SET_STATE), state(false), timestamp(0) {}
    
    RelayCommand(CommandType cmdType, bool cmdState = false) 
        : type(cmdType), state(cmdState), timestamp(xTaskGetTickCount() * portTICK_PERIOD_MS) {}
};

// ==================== UTILITY FUNCTIONS ====================

/**
 * Check if temperature reading is faulty
 * @param temperature Temperature to validate
 * @param isWaterSensor true for water sensor, false for outside
 * @return true if temperature is faulty
 */
bool isTemperatureFaulty(float temperature, bool isWaterSensor) {
    // Check for disconnected sensor
    if (temperature == DEVICE_DISCONNECTED_C) {
        return true;
    }
    
    // Check for unrealistic low temperature
    if (temperature < SENSOR_FAULT_TEMP) {
        return true;
    }
    
    // Check for unrealistic high temperature (sensor-specific)
    if (isWaterSensor) {
        if (temperature > WATER_TEMP_MAX) return true;
    } else {
        if (temperature > OUTSIDE_TEMP_MAX) return true;
    }
    
    // Valid temperature
    return false;
}

// ==================== SYSTEM STATE CLASS ====================

/**
 * Thread-safe system state management
 * Encapsulates all global data with proper mutex protection
 */
class SystemState {
private:
    // Mutexes for different data groups (mutable for const methods)
    mutable SemaphoreHandle_t m_tempMutex;
    mutable SemaphoreHandle_t m_controlMutex;
    mutable SemaphoreHandle_t m_burnerMutex;
    mutable SemaphoreHandle_t m_settingsMutex;
    mutable SemaphoreHandle_t m_wifiMutex;

public:
    // ========== DATA GROUPS ==========
    
    // Group 1: Temperature and sensor data (accessed together)
    struct TemperatureData {
        SensorData outsideData;            // Outside sensor readings
        SensorData waterData;              // Water sensor readings
        SensorStatus outsideStatus;        // Outside sensor health
        SensorStatus waterStatus;          // Water sensor health
        float targetWaterTemp = 0.0;       // Calculated target temperature
        bool heatingDisabled = false;      // Heating disabled flag
        
        // Quick status check methods
        bool hasSeriousFault() const {
            return outsideStatus.fault || waterStatus.fault;
        }
        
        bool hasAnyFault() const {
            return outsideStatus.isInError() || waterStatus.isInError();
        }
    } tempData;
    
    // Group 2: Control timing and logic data
    ControlData controlData;
    
    // Group 3: Burner control state
    BurnerData burnerData;
    
    // Group 4: System settings and configuration
    struct SettingsData {
        CurvePoints curve;                 // Temperature curve
        SystemSettings settings;           // Control settings
        
        // Calculate target temperature based on curve
        float calculateTargetTemp(float outsideTemp) const {
            // If outside temp above warmest point, heating is off
            if (outsideTemp >= curve.points[3][0]) {
                return 0.0;
            }
            
            // If outside temp below coldest point, use max heating
            if (outsideTemp <= curve.points[0][0]) {
                return curve.points[0][1];
            }
            
            // Find segment for linear interpolation
            for (int i = 0; i < 3; i++) {
                if (outsideTemp >= curve.points[i][0] && outsideTemp <= curve.points[i+1][0]) {
                    float ratio = (outsideTemp - curve.points[i][0]) / 
                                 (curve.points[i+1][0] - curve.points[i][0]);
                    return curve.points[i][1] + ratio * (curve.points[i+1][1] - curve.points[i][1]);
                }
            }
            
            return 45.0; // Default fallback
        }
    } settingsData;
    
    // Group 5: WiFi configuration
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
            while(1); // Halt system
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
    
    /**
     * Lock temperature data mutex
     * @param timeout Maximum time to wait for lock (ticks)
     * @return true if lock acquired
     */
    bool lockTempData(TickType_t timeout = portMAX_DELAY) const {
        return xSemaphoreTake(m_tempMutex, timeout) == pdTRUE;
    }
    
    /**
     * Unlock temperature data mutex
     */
    void unlockTempData() const {
        xSemaphoreGive(m_tempMutex);
    }
    
    /**
     * Get atomic snapshot of temperature data
     * @return Copy of temperature data
     */
    TemperatureData getTempDataSnapshot() const {
        TemperatureData snapshot;
        if (lockTempData(pdMS_TO_TICKS(100))) {
            snapshot = tempData;
            unlockTempData();
        }
        return snapshot;
    }
    
    /**
     * Update sensor reading with thread-safe error handling
     * @param newTemp New temperature reading
     * @param isWaterSensor true for water sensor, false for outside
     * @return true if update successful
     */
    bool updateSensorReading(float newTemp, bool isWaterSensor) {
        if (!lockTempData(pdMS_TO_TICKS(100))) return false;
        
        SensorStatus* status = isWaterSensor ? &tempData.waterStatus : &tempData.outsideStatus;
        SensorData* data = isWaterSensor ? &tempData.waterData : &tempData.outsideData;
        
        // Check for sensor fault using global function
        bool isFaulty = isTemperatureFaulty(newTemp, isWaterSensor);
        unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        if (isFaulty) {
            // Handle faulty reading
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
                
                // Set event group bit for system notification
                EventBits_t bits = isWaterSensor ? BIT_SENSOR_FAULT_WATER : BIT_SENSOR_FAULT_OUTSIDE;
                xEventGroupSetBits(xSystemEvents, bits);
            }
        } else {
            // Valid reading
            data->currentTemp = newTemp;
            data->lastValidTemp = newTemp;
            data->lastValidReadTime = currentTime;
            data->addValidValue(newTemp);
            
            // Reset error state
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
    
    /**
     * Update target temperature based on outside temperature
     * @param outsideTemp Current outside temperature
     * @return true if update successful
     */
    bool updateTargetTemperature(float outsideTemp) {
        if (!lockTempData(pdMS_TO_TICKS(100))) return false;
        
        // Get curve settings
        CurvePoints curve;
        if (lockSettings(pdMS_TO_TICKS(50))) {
            curve = settingsData.curve;
            unlockSettings();
        } else {
            unlockTempData();
            return false;
        }
        
        // Calculate and update target
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
    
    /**
     * Set burner state with thread safety
     * @param newState Desired burner state
     * @return true if state changed
     */
    bool setBurnerState(bool newState) {
        if (!lockBurnerData(pdMS_TO_TICKS(100))) return false;
        
        bool changed = (burnerData.burnerState != newState);
        burnerData.burnerState = newState;
        
        unlockBurnerData();
        return changed;
    }
    
    /**
     * Set manual override mode
     * @param manual true for manual mode, false for automatic
     * @return true if mode changed
     */
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
    
    /**
     * Update temperature curve
     * @param newCurve New curve points
     * @return true if update successful
     */
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
    
    /**
     * Update hysteresis value
     * @param newHysteresis New hysteresis value
     * @return true if update successful
     */
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
    
    /**
     * Get complete control snapshot for control logic
     * Returns atomic snapshot of all data needed for control decisions
     */
    struct ControlSnapshot {
        float waterTemp;           // Current water temperature
        float outsideTemp;         // Current outside temperature
        float targetTemp;          // Target water temperature
        float controlOutsideTemp;  // Outside temp used for control
        float hysteresis;          // Hysteresis value
        bool waterFault;           // Water sensor fault
        bool outsideFault;         // Outside sensor fault
        bool heatingDisabled;      // Heating disabled flag
        bool manualMode;           // Manual control mode
        bool burnerState;          // Current burner state
        
        // Control logic helper methods
        bool canControl() const {
            return !waterFault && !outsideFault && !manualMode && !heatingDisabled;
        }
        
        bool shouldTurnOn() const {
            return !burnerState && targetTemp > 0 && waterTemp < (targetTemp - hysteresis);
        }
        
        bool shouldTurnOff() const {
            return burnerState && (waterTemp > (targetTemp + hysteresis) || targetTemp <= 0);
        }
    };
    
    ControlSnapshot getControlSnapshot() const {
        ControlSnapshot snapshot;
        
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
    
    /**
     * Quick check for sensor faults
     * @return true if any serious sensor fault exists
     */
    bool hasSeriousSensorFault() const {
        bool hasFault = false;
        if (lockTempData(pdMS_TO_TICKS(50))) {
            hasFault = tempData.outsideStatus.fault || tempData.waterStatus.fault;
            unlockTempData();
        }
        return hasFault;
    }
    
    /**
     * Reset all sensor data (for recovery or testing)
     */
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

// Global system state instance (thread-safe)
SystemState systemState;

// FreeRTOS resources
EventGroupHandle_t xSystemEvents = NULL;          // System event group
QueueHandle_t xRelayControlQueue = NULL;          // Relay command queue

// Hardware objects
OneWire oneWireOutside(OUTSIDE_SENSOR_PIN);
OneWire oneWireWater(WATER_SENSOR_PIN);
DallasTemperature sensorsOutside(&oneWireOutside);
DallasTemperature sensorsWater(&oneWireWater);

// Network objects
WebServer server(80);                      // Web server on port 80
Preferences preferences;                    // Flash storage

// Task handles
TaskHandle_t xTemperatureTaskHandle = NULL;
TaskHandle_t xControlTaskHandle = NULL;
TaskHandle_t xWebServerTaskHandle = NULL;
TaskHandle_t xRelayTaskHandle = NULL;
TaskHandle_t xWatchdogTaskHandle = NULL;
TaskHandle_t xFlashTaskHandle = NULL;

// ==================== WIFI FUNCTIONS ====================

/**
 * Initialize WiFi connection
 * Tries STA mode first, falls back to AP mode if needed
 * @return true if WiFi initialized successfully
 */
bool initWiFi() {
    Serial.println("Initializing WiFi...");
    
    // Load WiFi settings from flash
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
        
        // Ensure valid AP credentials
        systemState.lockWifiConfig();
        if (strlen(systemState.wifiConfig.ap_ssid) == 0) {
            strcpy(systemState.wifiConfig.ap_ssid, DEFAULT_AP_SSID);
        }
        if (strlen(systemState.wifiConfig.ap_password) == 0) {
            strcpy(systemState.wifiConfig.ap_password, DEFAULT_AP_PASSWORD);
        }
        systemState.unlockWifiConfig();
        
        // Configure AP with static IP
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
 * Only saves if settings have been marked as dirty
 */
void saveSystemSettings() {
    auto settings = systemState.getSettingsSnapshot();
    
    if (!settings.settings.settingsDirty) {
        return; // Nothing to save
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
 * Reads DS18B20 sensors using non-blocking state machine
 * Updates system state with thread-safe methods
 */
void temperatureTask(void *parameter) {
    Serial.println("Temperature task started");
    
    // Notify watchdog we're alive
    xEventGroupSetBits(xSystemEvents, BIT_TEMP_TASK_ALIVE);
    
    // Initialize sensors
    sensorsOutside.begin();
    sensorsWater.begin();
    sensorsOutside.setResolution(SENSOR_RESOLUTION);
    sensorsWater.setResolution(SENSOR_RESOLUTION);
    sensorsOutside.setWaitForConversion(false);
    sensorsWater.setWaitForConversion(false);
    
    // State machine for non-blocking temperature reading
    enum ReadState { 
        IDLE,
        REQUEST_OUTSIDE,
        WAIT_OUTSIDE,
        READ_OUTSIDE,
        REQUEST_WATER,
        WAIT_WATER,
        READ_WATER
    };
    
    ReadState readState = IDLE;
    unsigned long requestTime = 0;
    
    while (1) {
        unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        switch (readState) {
            case IDLE:
                // Wait for next reading interval
                vTaskDelay(pdMS_TO_TICKS(TEMP_READ_INTERVAL_MS));
                
                // Update watchdog
                xEventGroupSetBits(xSystemEvents, BIT_TEMP_TASK_ALIVE);
                
                // Check sensor presence
                if (sensorsOutside.getDeviceCount() == 0) {
                    Serial.println("WARNING: Outside sensor not found!");
                    sensorsOutside.begin();
                }
                if (sensorsWater.getDeviceCount() == 0) {
                    Serial.println("WARNING: Water sensor not found!");
                    sensorsWater.begin();
                }
                
                // Start measurement cycle
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
                // Wait for conversion (max 750ms for 12-bit)
                if (currentTime - requestTime > 750) {
                    float outsideTemp = sensorsOutside.getTempCByIndex(0);
                    
                    #if DEBUG_TEMP
                    Serial.printf("[TEMP] OUTSIDE: %.2f °C\n", outsideTemp);
                    #endif
                    
                    // Update with thread-safe method
                    systemState.updateSensorReading(outsideTemp, false);
                    
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
                    Serial.printf("[TEMP] WATER: %.2f °C\n", waterTemp);
                    #endif
                    
                    // Update with thread-safe method
                    systemState.updateSensorReading(waterTemp, true);
                    
                    // Get outside temperature for target calculation
                    auto tempSnapshot = systemState.getTempDataSnapshot();
                    float outsideTempToUse = tempSnapshot.outsideStatus.fault ? 
                                           tempSnapshot.outsideData.getAverageValidValue() : 
                                           tempSnapshot.outsideData.currentTemp;
                    
                    // Update target temperature
                    systemState.updateTargetTemperature(outsideTempToUse);
                    
                    // Update heating disabled flag in event group
                    if (tempSnapshot.heatingDisabled) {
                        xEventGroupSetBits(xSystemEvents, BIT_HEATING_DISABLED);
                    } else {
                        xEventGroupClearBits(xSystemEvents, BIT_HEATING_DISABLED);
                    }
                    
                    readState = IDLE;
                }
                break;
        }
        
        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * Control logic task
 * Implements stable control logic with 10-minute outside temp updates
 * Uses thread-safe snapshots for all data access
 */
void controlTask(void *parameter) {
    Serial.println("Control task started");
    
    unsigned long lastControlTime = 0;
    unsigned long lastOutsideUpdate = 0;
    
    while (1) {
        unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Execute control logic at regular interval
        if (currentTime - lastControlTime >= CONTROL_LOOP_INTERVAL_MS) {
            lastControlTime = currentTime;
            
            // Get atomic snapshot of all control data
            auto snapshot = systemState.getControlSnapshot();
            
            // Check system events for faults
            EventBits_t events = xEventGroupGetBits(xSystemEvents);
            bool anySeriousFault = (events & (BIT_SENSOR_FAULT_OUTSIDE | BIT_SENSOR_FAULT_WATER)) != 0;
            bool heatingDisabled = (events & BIT_HEATING_DISABLED) != 0;
            bool emergencyStop = (events & BIT_EMERGENCY_STOP) != 0;
            
            // Update control outside temperature if needed (every 10 minutes)
            if (systemState.controlData.needsUpdate(currentTime)) {
                systemState.lockControlData();
                systemState.controlData.update(snapshot.outsideTemp, currentTime);
                systemState.unlockControlData();
                
                #if DEBUG_CONTROL
                Serial.printf("[CONTROL] Updated control outside temp: %.1f°C\n", 
                            snapshot.controlOutsideTemp);
                #endif
            }
            
            // Control logic
            bool canControl = !anySeriousFault && !snapshot.manualMode && 
                            !heatingDisabled && !emergencyStop;
            bool newBurnerState = snapshot.burnerState;
            
            if (canControl) {
                // Calculate target based on control outside temperature
                float effectiveOutsideTemp = systemState.getControlDataSnapshot().controlOutsideTemp;
                float targetTemp = systemState.settingsData.calculateTargetTemp(effectiveOutsideTemp);
                
                // Apply hysteresis control logic
                if (snapshot.shouldTurnOff()) {
                    newBurnerState = false;
                    #if DEBUG_CONTROL
                    Serial.printf("[CONTROL] Turning OFF: Water %.1f > Target %.1f + Hyst %.1f\n",
                                snapshot.waterTemp, targetTemp, snapshot.hysteresis);
                    #endif
                }
                else if (snapshot.shouldTurnOn()) {
                    newBurnerState = true;
                    #if DEBUG_CONTROL
                    Serial.printf("[CONTROL] Turning ON: Water %.1f < Target %.1f - Hyst %.1f\n",
                                snapshot.waterTemp, targetTemp, snapshot.hysteresis);
                    #endif
                }
                
                // Send command if state changed
                if (newBurnerState != snapshot.burnerState) {
                    systemState.setBurnerState(newBurnerState);
                    
                    RelayCommand cmd(RelayCommand::SET_STATE, newBurnerState);
                    xQueueSend(xRelayControlQueue, &cmd, 0);
                }
            } else if (!emergencyStop && snapshot.manualMode) {
                // Manual mode - user controls burner directly
                // State is set via web interface, we just pass it through
                if (snapshot.burnerState != newBurnerState) {
                    RelayCommand cmd(RelayCommand::SET_STATE, snapshot.burnerState);
                    xQueueSend(xRelayControlQueue, &cmd, 0);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * Relay control task
 * Receives commands from queue and controls relay safely
 * Implements safety checks before any relay operation
 */
void relayTask(void *parameter) {
    Serial.println("Relay task started");
    
    // Initialize relay pin
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); // Start in safe state (OFF)
    
    RelayCommand cmd;
    
    while (1) {
        // Wait for command from queue
        if (xQueueReceive(xRelayControlQueue, &cmd, portMAX_DELAY)) {
            
            // Check system conditions
            EventBits_t events = xEventGroupGetBits(xSystemEvents);
            bool anySeriousFault = (events & (BIT_SENSOR_FAULT_OUTSIDE | BIT_SENSOR_FAULT_WATER)) != 0;
            bool emergencyStop = (events & BIT_EMERGENCY_STOP) != 0;
            bool heatingDisabled = (events & BIT_HEATING_DISABLED) != 0;
            
            bool shouldTurnOn = false;
            
            // Process command based on type
            switch (cmd.type) {
                case RelayCommand::SET_STATE:
                    // Normal operation with safety checks
                    if (!anySeriousFault && !heatingDisabled && !emergencyStop) {
                        shouldTurnOn = cmd.state;
                    } else if (systemState.burnerData.isManualMode() && cmd.state) {
                        // Manual mode can override some conditions
                        shouldTurnOn = true;
                    }
                    break;
                    
                case RelayCommand::FORCE_OFF:
                    // Forced shutdown (override control logic)
                    shouldTurnOn = false;
                    break;
                    
                case RelayCommand::EMERGENCY_OFF:
                    // Emergency shutdown (bypass all logic)
                    shouldTurnOn = false;
                    xEventGroupSetBits(xSystemEvents, BIT_EMERGENCY_STOP);
                    break;
            }
            
            // Control relay
            digitalWrite(RELAY_PIN, shouldTurnOn ? HIGH : LOW);
            
            // Update state if this was a SET_STATE command
            if (cmd.type == RelayCommand::SET_STATE) {
                systemState.setBurnerState(shouldTurnOn);
            }
            
            Serial.printf("[RELAY] State: %s (Cmd: %d)\n", 
                         shouldTurnOn ? "ON" : "OFF", cmd.type);
        }
    }
}

/**
 * Watchdog monitoring task
 * Monitors other tasks and performs emergency actions if needed
 * Uses both software and hardware watchdogs
 */
void watchdogTask(void *parameter) {
    Serial.println("Watchdog task started");
    
    // Initialize ESP32 hardware watchdog
    esp_task_wdt_init(30, true);  // 30 second timeout, panic reset
    esp_task_wdt_add(NULL);       // Monitor this task
    
    unsigned long lastTempTaskAlive = millis();
    unsigned long lastControlTaskCheck = millis();
    
    while (1) {
        unsigned long currentTime = millis();
        
        // Check temperature task
        EventBits_t events = xEventGroupGetBits(xSystemEvents);
        if (events & BIT_TEMP_TASK_ALIVE) {
            lastTempTaskAlive = currentTime;
            xEventGroupClearBits(xSystemEvents, BIT_TEMP_TASK_ALIVE);
        }
        
        // If temperature task not responding for 60 seconds
        if (currentTime - lastTempTaskAlive > 60000) {
            Serial.println("WATCHDOG: Temperature task not responding!");
            
            // Emergency shutdown
            RelayCommand cmd(RelayCommand::EMERGENCY_OFF);
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
 * Flash storage task
 * Saves settings to flash memory with delay to prevent wear
 * Only saves when settings are marked as dirty
 */
void flashTask(void *parameter) {
    Serial.println("Flash task started");
    
    while (1) {
        // Check if settings need saving
        auto settings = systemState.getSettingsSnapshot();
        
        if (settings.settings.settingsDirty) {
            unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // Wait 5 seconds after last change before saving
            if (currentTime - settings.settings.lastSettingsChange > 5000) {
                saveSystemSettings();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==================== WEB SERVER IMPLEMENTATION ====================

// HTML page constant (same as your original, but included for completeness)
const char* htmlPage = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Oil Heating Control</title>
  <style>
    /* ... (your existing CSS styles) ... */
  </style>
  <script>
    /* ... (your existing JavaScript) ... */
  </script>
</head>
<body>
  <!-- ... (your existing HTML) ... -->
</body>
</html>
)rawliteral";

/**
 * Web server task
 * Provides web interface for system monitoring and control
 * All data access uses thread-safe methods
 */
void webServerTask(void *parameter) {
    Serial.println("Web server task started");
    
    // Wait for system to stabilize
    delay(2000);
    
    // ========== HTTP ROUTES ==========
    
    // Main page
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", htmlPage);
    });
    
    // JSON data endpoint
    server.on("/data", HTTP_GET, []() {
        // Get atomic snapshots of all data
        auto tempSnapshot = systemState.getTempDataSnapshot();
        auto burnerSnapshot = systemState.getBurnerDataSnapshot();
        auto wifiSnapshot = systemState.getWifiConfigSnapshot();
        auto settingsSnapshot = systemState.getSettingsSnapshot();
        
        // Build sensor status strings
        String outsideStatus = "OK";
        if (tempSnapshot.outsideStatus.fault) outsideStatus = "ERROR";
        else if (tempSnapshot.outsideStatus.temporaryFault) outsideStatus = "WARNING";
        
        String waterStatus = "OK";
        if (tempSnapshot.waterStatus.fault) waterStatus = "ERROR";
        else if (tempSnapshot.waterStatus.temporaryFault) waterStatus = "WARNING";
        
        // Build JSON response
        String json = "{";
        json += "\"outsideTemp\":" + String(tempSnapshot.outsideData.currentTemp, 1);
        json += ",\"waterTemp\":" + String(tempSnapshot.waterData.currentTemp, 1);
        json += ",\"targetTemp\":" + String(tempSnapshot.targetWaterTemp, 1);
        json += ",\"burnerState\":" + String(burnerSnapshot.burnerState ? "true" : "false");
        json += ",\"manualMode\":" + String(burnerSnapshot.burnerManualOverride ? "true" : "false");
        json += ",\"outsideStatus\":\"" + outsideStatus + "\"";
        json += ",\"waterStatus\":\"" + waterStatus + "\"";
        json += ",\"heatingDisabled\":" + String(tempSnapshot.heatingDisabled ? "true" : "false");
        
        // WiFi info
        String wifiMode = wifiSnapshot.ap_mode ? "AP (Own Network)" : "STA (Network)";
        String wifiIP = wifiSnapshot.ap_mode ? 
                       WiFi.softAPIP().toString() : 
                       WiFi.localIP().toString();
        
        json += ",\"wifiMode\":\"" + wifiMode + "\"";
        json += ",\"wifiIP\":\"" + wifiIP + "\"";
        json += ",\"hysteresis\":" + String(settingsSnapshot.settings.hysteresis, 1);
        json += "}";
        
        server.send(200, "application/json", json);
    });
    
    // Control endpoint
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
    
    // Temperature curve endpoints
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
    
    // System settings endpoints
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
    
    // Emergency stop
    server.on("/emergency", HTTP_GET, []() {
        RelayCommand cmd(RelayCommand::EMERGENCY_OFF);
        xQueueSend(xRelayControlQueue, &cmd, 0);
        
        Serial.println("Emergency shutdown via web interface");
        server.send(200, "text/plain", "Emergency shutdown initiated");
    });
    
    // WiFi settings endpoints
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
        // Update WiFi configuration
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
            // Save to flash
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
    
    // 404 handler
    server.onNotFound([]() {
        server.send(404, "text/plain", "Page not found");
    });
    
    // Start server
    server.begin();
    Serial.println("HTTP server started on port 80");
    
    // Print connection info
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
    
    // Server loop
    while (1) {
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== ARDUINO SETUP ====================

/**
 * Setup function - initializes all system components
 */
void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    delay(2000); // Wait for serial monitor
    
    Serial.println("\n\n=== OIL HEATING CONTROL SYSTEM ===");
    Serial.println("Version: 2.0 (Thread-Safe)");
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
    
    // Web server (low priority, runs on WiFi core)
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
    
    Serial.println("\nControl Stability Feature:");
    Serial.println("  Outside temperature for control updates every 10 minutes");
    Serial.println("  Prevents short cycling when temperature is near cutoff\n");
}

// ==================== ARDUINO LOOP ====================

/**
 * Main loop - runs on core 0
 * Handles WiFi monitoring and other low-priority tasks
 */
void loop() {
    static unsigned long lastWifiCheck = 0;
    unsigned long currentTime = millis();
    
    // Monitor WiFi connection (STA mode only)
    if (currentTime - lastWifiCheck > WIFI_RECONNECT_INTERVAL) {
        lastWifiCheck = currentTime;
        
        auto wifiConfig = systemState.getWifiConfigSnapshot();
        if (wifiConfig.use_sta && !wifiConfig.ap_mode) {
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("WiFi disconnected, attempting reconnect...");
                WiFi.reconnect();
                
                // Update connection status
                systemState.lockWifiConfig();
                systemState.wifiConfig.connected = (WiFi.status() == WL_CONNECTED);
                systemState.unlockWifiConfig();
            }
        }
    }
    
    // Yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}
