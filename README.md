# Oil Heating System - ESP32 FreeRTOS Controller
![Picture](oil.png)

## 📋 Project Overview
An advanced oil heating control system using ESP32 with FreeRTOS for reliable, multi-threaded operation. Features temperature-based automatic control with web interface for remote monitoring and configuration.

## 🎯 Key Features
- **Multi-threaded FreeRTOS architecture** for reliable concurrent operation
- **Temperature measurement** with DS18B20 sensors (outside + water)
- **Automatic control** with configurable hysteresis
- **Web interface** with dark theme for remote control
- **Dual WiFi modes** (AP + Station)
- **Temperature curve adjustment** (4-point linear interpolation)
- **Safety measures** for sensor errors
- **Watchdog monitoring** for system reliability
- **Flash memory storage** for settings persistence
- **Stable control logic** with 10-minute outside temperature updates

## 🔌 Hardware Connections

### ESP32 Pin Configuration
| Component | GPIO Pin | Description |
|-----------|----------|-------------|
| Outside Sensor | GPIO 23 | DS18B20 temperature sensor |
| Water Sensor | GPIO 22 | DS18B20 temperature sensor |
| Relay Control | GPIO 2 | Controls oil burner relay |
| *Optional* | GPIO 16 | Serial2 RX (for future expansion) |
| *Optional* | GPIO 17 | Serial2 TX (for future expansion) |

### Power Requirements
- **ESP32**: 3.3V DC (via USB or external regulator)
- **DS18B20 Sensors**: 3.3V-5V DC (parasitic power mode supported)
- **Relay Module**: 5V DC (ensure proper isolation)

### Wiring Diagram
```
ESP32 GPIO2 ────┬─── Relay IN
                │
ESP32 3.3V ─────┼─── Relay VCC
                │
ESP32 GND ──────┼─── Relay GND
                │
ESP32 GPIO23 ───┼─── DS18B1 DATA (Outside)
                │
ESP32 3.3V ─────┼─── DS18B1 VDD
                │
ESP32 GND ──────┼─── DS18B1 GND
                │
ESP32 GPIO22 ───┼─── DS18B2 DATA (Water)
                │
ESP32 3.3V ─────┼─── DS18B2 VDD
                │
ESP32 GND ──────┴─── DS18B2 GND
```

## 🚀 Getting Started

### 1. Prerequisites
- **Hardware**:
  - ESP32 development board
  - 2× DS18B20 temperature sensors
  - Relay module (5V, 10A minimum)
  - USB cable for programming
  - Jumper wires and breadboard

- **Software**:
  - Arduino IDE 2.0+
  - ESP32 board support package
  - Required libraries:
    - `WiFi`
    - `WebServer`
    - `OneWire`
    - `DallasTemperature`
    - `Preferences`
    - `freertos/FreeRTOS.h`

### 2. Installation Steps

#### Step 1: Install ESP32 Board Support
1. Open Arduino IDE
2. Go to **File → Preferences**
3. Add to Additional Boards Manager URLs:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to **Tools → Board → Boards Manager**
5. Search for "esp32" and install

#### Step 2: Install Required Libraries
Open Arduino IDE and install via Library Manager:
- **OneWire** by Paul Stoffregen
- **DallasTemperature** by Miles Burton
- **Preferences** (included with ESP32)

#### Step 3: Upload Code
1. Connect ESP32 via USB
2. Select board: **ESP32 Dev Module**
3. Select correct COM port
4. Click **Upload**

### 3. Initial Setup

#### First Boot Configuration
1. After upload, open Serial Monitor (115200 baud)
2. System will create WiFi network: `OilHeaterAP`
3. Connect to network with password: `enmuista`
4. Open browser: `http://192.168.4.1`

#### Web Interface Setup
1. Navigate to **WiFi Settings** tab
2. Configure either:
   - **AP Mode**: Use device's own network
   - **STA Mode**: Connect to existing WiFi
3. Save settings (device will restart)

## ⚙️ Configuration

### Temperature Curve
System uses 4-point linear interpolation:

| Point | Outside Temp | Target Water Temp |
|-------|--------------|-------------------|
| 1 | -20°C | 75°C |
| 2 | -10°C | 65°C |
| 3 | 0°C | 55°C |
| 4 | 10°C | 45°C |

**Note**: Outside temperatures above 10°C disable heating completely.

### Control Parameters
- **Hysteresis**: 4.0°C (prevents rapid cycling)
- **Sensor Update**: Every 5 seconds
- **Outside Temp for Control**: Updates every 10 minutes (for stability)
- **Error Detection**: 3 consecutive failures → warning, 30s → error

## 🔧 FreeRTOS Task Architecture

| Task | Priority | Stack Size | Core | Description |
|------|----------|------------|------|-------------|
| Watchdog | 4 | 2048 | 0 | Monitors system health |
| Relay | 3 | 2048 | 1 | Controls burner relay |
| Temperature | 3 | 3072 | 1 | Reads DS18B20 sensors |
| Control | 2 | 3072 | 1 | Implements control logic |
| Flash | 1 | 2048 | 1 | Saves settings to flash |
| Web Server | 1 | 4096 | 0 | Handles web interface |

## 🌐 Web Interface Features

### Control Panel
- Real-time temperature display
- Burner status and control
- Sensor status monitoring
- Emergency stop button

### WiFi Settings
- AP/STA mode configuration
- Network credentials management
- IP address display

### Temperature Curve
- 4-point curve configuration
- Real-time preview
- Save to flash memory

### System Settings
- Hysteresis adjustment
- System information
- Settings persistence

## 🛡️ Safety Features

### 1. Sensor Error Handling
- **Temporary Faults**: Uses last valid reading
- **Serious Faults**: Disables automatic control
- **Moving Average**: 10-value buffer for error situations

### 2. Control Safety
- **Manual Override**: User can control burner manually
- **Emergency Stop**: Immediate shutdown
- **Temperature Limits**: Prevents unrealistic values
- **Watchdog**: Monitors task responsiveness

### 3. System Protection
- **Flash Write Protection**: Delayed writes to extend lifespan
- **WiFi Fallback**: AP mode if STA connection fails
- **Task Monitoring**: Watchdog ensures system responsiveness

## 📊 Serial Monitor Output
```
=== OIL HEATING SYSTEM - FreeRTOS ===

System starting...
Initializing WiFi...
AP SSID: OilHeaterAP
STA enabled: NO
Creating own network (Access Point)...
Own network created successfully!
SSID: OilHeaterAP
IP address: 192.168.4.1

All threads created successfully!
System ready for use!

=== WEB SERVER READY ===
Connect to own network:
  SSID: OilHeaterAP
  Password: enmuista
  Open in browser: http://192.168.4.1
```

## 🔍 Troubleshooting

### Common Issues

#### 1. Sensors Not Detected
```
WARNING: Outside sensor not found!
WARNING: Water sensor not found!
```
**Solution**:
- Check wiring connections
- Verify 4.7kΩ pull-up resistors
- Ensure proper power supply

#### 2. WiFi Connection Issues
**Solution**:
- Check WiFi credentials
- Ensure signal strength
- Restart device
- Use AP mode as fallback

#### 3. Web Interface Not Accessible
**Solution**:
- Verify IP address in Serial Monitor
- Check firewall settings
- Ensure correct URL format

### Debug Features
Enable debug output by setting:
```cpp
#define DEBUG_TEMP 1  // Temperature debug output
```

## 📈 Performance Optimization

### Memory Management
- **Stack Sizes**: Optimized for each task
- **Heap Usage**: Monitored via `ESP.getFreeHeap()`
- **Flash Usage**: ~1.2MB (out of 4MB)

### Power Consumption
- **Active Mode**: ~120mA
- **WiFi Active**: Additional ~80mA
- **Sleep Mode**: Not implemented (always active)

### Reliability Features
- **Mutex Protection**: All shared resources protected
- **Queue Communication**: Thread-safe message passing
- **Event Groups**: Efficient state change notifications

## 🔄 Update Procedure

### 1. Firmware Updates
1. Connect ESP32 via USB
2. Open Arduino IDE
3. Select correct board and port
4. Click **Upload**

### 2. Settings Backup
Settings are automatically saved to flash. To reset:
1. Use web interface to modify settings
2. Or manually edit flash via Serial Monitor

## 📚 API Reference

### HTTP Endpoints
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Main web interface |
| `/data` | GET | JSON system data |
| `/control` | GET | Control commands |
| `/setcurve` | GET | Set temperature curve |
| `/getcurve` | GET | Get temperature curve |
| `/setsettings` | GET | Set system settings |
| `/getsettings` | GET | Get system settings |
| `/emergency` | GET | Emergency shutdown |
| `/setwifi` | GET | Set WiFi settings |
| `/getwifi` | GET | Get WiFi settings |

### JSON Data Structure
```json
{
  "outsideTemp": 15.5,
  "waterTemp": 42.3,
  "targetTemp": 45.0,
  "burnerState": false,
  "manualMode": false,
  "outsideStatus": "OK",
  "waterStatus": "OK",
  "heatingDisabled": false,
  "wifiMode": "AP (Own Network)",
  "wifiIP": "192.168.4.1",
  "hysteresis": 4.0
}
```

## 🚨 Emergency Procedures

### Manual Override
1. Access web interface
2. Switch to **Manual Mode**
3. Use ON/OFF buttons

### Emergency Stop
1. Click **Emergency Stop** button
2. Confirm action
3. System immediately shuts down burner

### Physical Reset
- Press **EN/RST** button on ESP32
- Disconnect power for 10 seconds

## 📝 License & Credits

### License
This project is open-source. Modify and distribute as needed.

### Credits
- **ESP32 Arduino Core**: Espressif Systems
- **FreeRTOS**: Real Time Engineers Ltd.
- **DS18B20 Library**: Miles Burton
- **WebServer**: ESP32 Arduino

## 🔮 Future Enhancements

### Planned Features
1. **MQTT Support**: Remote monitoring integration
2. **Energy Monitoring**: Power consumption tracking
3. **Schedule Programming**: Time-based control
4. **Mobile App**: Dedicated application
5. **OTA Updates**: Wireless firmware updates

### Hardware Expansion
- Additional temperature sensors
- Flow rate monitoring
- Pressure sensors
- External display interface

## 🤝 Support & Contribution

### Getting Help
1. Check **Troubleshooting** section
2. Review Serial Monitor output
3. Verify hardware connections
4. Test with minimal configuration

### Contributing
1. Fork repository
2. Create feature branch
3. Submit pull request
4. Include detailed documentation

---

**⚠️ Disclaimer**: This system controls heating equipment. Ensure proper installation and safety measures. The authors are not responsible for any damage or injury resulting from improper use.
