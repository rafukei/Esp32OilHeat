ESP32 Oil Heating System - Hardware Connection Guide & Features Documentation
📋 Table of Contents
ESP32 Pin Connections

Required Components

Wiring Diagram

Assembly Instructions

System Features

Safety Features

Web Interface Guide

Troubleshooting

Technical Specifications

🎯 ESP32 Pin Connections
Required GPIO Pins:
ESP32 Pin	Function	Component	Notes
GPIO 2	Relay Control	Relay Module	CRITICAL: Controls burner on/off
GPIO 22	Water Temperature Sensor	DS18B20	Yellow wire (data)
GPIO 23	Outside Temperature Sensor	DS18B20	Yellow wire (data)
GPIO 3V3	Sensor Power	Both DS18B20	Red wire (VCC)
GPIO GND	Sensor Ground	Both DS18B20	Black wire (GND)
Power Connections:
Connection	Voltage	Purpose	Fuse Recommended
ESP32 VIN	5V DC	ESP32 Power	2A
Relay VCC	5V DC	Relay Power	1A
Relay JD-VCC	12V*	Relay Coil	*Check relay module
Sensor Wiring Details:
DS18B20 Sensors (2x):

text
ESP32 Pin 22/23 → 4.7kΩ resistor → DS18B20 DATA (Yellow)
ESP32 3V3 → DS18B20 VCC (Red)
ESP32 GND → DS18B20 GND (Black)
*Note: 4.7kΩ pull-up resistor required between DATA and 3V3*

🔧 Required Components
Essential Components:
ESP32 Development Board (ESP32-WROOM-32 recommended)

DS18B20 Temperature Sensors (Waterproof, 2 pieces)

5V Relay Module (1-channel, opto-isolated)

Power Supply (5V/2A for ESP32, 12V for relay if needed)

4.7kΩ Resistors (2 pieces, 1/4 watt)

Jumper Wires (Male-to-Female, Female-to-Female)

Breadboard (for prototyping)

Enclosure (IP65 rated for installation safety)

Optional Components:
Status LEDs (Red/Green for visual feedback)

Buzzer (for alarm signals)

LCD Display (20x4 I2C for local monitoring)

SD Card Module (for data logging)

Current Sensor (to monitor burner power consumption)

📐 Wiring Diagram
Simplified Connection Layout:
text
┌─────────────────────────────────────────────────────┐
│                    POWER SUPPLY                      │
│                      5V DC / 2A                      │
└──────────────────────────┬───────────────────────────┘
                           │
                    ┌──────▼──────┐
                    │   ESP32     │
                    │             │
                    │ GPIO 2 ─────┼───► RELAY IN1 (Burner Control)
                    │             │
                    │ GPIO 22 ────┼───► DS18B21 (Water Temp)
                    │             │         ├─ 4.7kΩ ─ 3V3
                    │ GPIO 23 ────┼───► DS18B22 (Outside Temp)
                    │             │         ├─ 4.7kΩ ─ 3V3
                    │   3V3 ──────┼───► Both DS18B20 VCC
                    │   GND ──────┼───► All Components GND
                    └─────────────┘
                           │
                    ┌──────▼──────┐
                    │    RELAY    │
                    │             │
                    │   COM ──────┼───► BURNER POWER (LIVE)
                    │   NO ───────┼───► BURNER CONTROL WIRE
                    │   NC ───────┼───► (Not connected)
                    └─────────────┘
Detailed Relay Connection to Burner:
text
ESP32 GPIO2 ───► Relay IN1
Relay COM ─────► Burner Power Supply (Live)
Relay NO ──────► Burner Control Terminal
Relay NC ──────► Leave open (or connect to indicator)
Relay GND ─────► Common Ground
🔨 Assembly Instructions
Step 1: Prepare Components
ESP32: Install in a safe location (away from heat/water)

DS18B20 Sensors:

Water sensor: Install in heating system water pipe (use thermal paste)

Outside sensor: Install in weatherproof enclosure (north-facing wall)

Relay Module: Mount near burner control circuit

Step 2: Connect Temperature Sensors
text
Water Sensor (DS18B20 #1):
Red   → ESP32 3V3
Black → ESP32 GND
Yellow→ ESP32 GPIO22 + 4.7kΩ resistor to 3V3

Outside Sensor (DS18B20 #2):
Red   → ESP32 3V3
Black → ESP32 GND
Yellow→ ESP32 GPIO23 + 4.7kΩ resistor to 3V3
Step 3: Connect Relay Module
text
ESP32 GPIO2  → Relay IN1
ESP32 5V     → Relay VCC (check module voltage)
ESP32 GND    → Relay GND
Relay COM    → Burner power supply (LIVE)
Relay NO     → Burner control terminal
Step 4: Power Connections
text
External 5V/2A Power Supply:
+5V → ESP32 VIN pin
GND → ESP32 GND pin (and all other GNDs)
Step 5: Initial Testing
Power on ESP32

Check Serial Monitor (115200 baud)

Verify sensors are detected

Test relay with manual control via web interface

Verify proper isolation between low-voltage and high-voltage circuits

⚙️ System Features
Core Features:
🔄 Dual Temperature Monitoring

Continuous water temperature measurement (every 5 seconds)

Outside temperature measurement (every 5 seconds)

10-minute stable update for control logic

Fault detection and historical data averaging

🔥 Intelligent Burner Control

4-point temperature curve control

Adjustable hysteresis (0.5-10.0°C)

Automatic/manual mode switching

Anti-short-cycle protection (10-min updates)

🌐 Web Interface

Dark theme with large temperature displays

Real-time monitoring

Configuration pages for all settings

Mobile-responsive design

No app installation required

🔒 Safety Systems

Dual-sensor fault detection

Emergency stop function

Burner runtime limits

Temperature range validation

Watchdog timer

💾 Data Management

Settings stored in flash memory

Automatic backup

5-second delay to prevent flash wear

Factory reset capability

Advanced Features:
📶 WiFi Connectivity

Dual-mode operation (AP/STA)

Automatic fallback to AP mode

Configurable network settings

Static IP support

🛡️ System Protection

FreeRTOS task monitoring

Hardware watchdog

Mutex-protected data access

Error recovery routines

🔧 Configuration Options

Temperature curve adjustment (4 points)

Hysteresis tuning

WiFi settings

Sensor calibration

🛡️ Safety Features
Critical Safety Systems:
Burner Control Safety:

Relay defaults to OFF on power loss

Emergency stop immediately cuts power

Manual override requires explicit user action

Dual confirmation for critical operations

Temperature Safety Limits:

text
Water Temperature:  20°C - 120°C (safe range)
Outside Temperature: -50°C - 100°C (safe range)
Fault Detection:    Immediate shutdown on out-of-range
Sensor Failure Response:

Temporary fault: Use last valid reading

Serious fault: Disable automatic control

Multiple fallback strategies

Clear error indicators

Electrical Safety:

Opto-isolated relay module

Proper grounding

Fuse protection recommended

Separation of high/low voltage

Operational Safety:
Control Stability:

10-minute outside temperature update interval

Prevents rapid cycling at cutoff temperatures

Hysteresis prevents oscillation

Minimum on/off times enforced

System Monitoring:

Task watchdog (60-second timeout)

Hardware watchdog (30-second timeout)

Memory leak protection

Network connection monitoring

🌐 Web Interface Guide
Main Control Panel:
text
┌─────────────────────────────────────────┐
│ 🔥 Oil Heating Control System           │
├─────────────────────────────────────────┤
│ Control Panel | WiFi | Curve | Settings │
├─────────────────────────────────────────┤
│ Outside:  12.5°C      [LARGE DISPLAY]   │
│ Water:    45.2°C      [LARGE DISPLAY]   │
│ Target:   52.0°C      [LARGE DISPLAY]   │
├─────────────────────────────────────────┤
│ Burner:   ON          Mode: Automatic   │
│ Heating:  Enabled     Hysteresis: 4.0°C │
├─────────────────────────────────────────┤
│ [Automatic Mode] [Manual Mode]          │
│ [Burner ON]      [Burner OFF]           │
│ [🚨 Emergency Stop]                     │
└─────────────────────────────────────────┘
Configuration Tabs:
Control Panel (Default):

Real-time temperature displays

Burner status and control

System status overview

Emergency stop button

WiFi Settings:

Access Point configuration

Network connection settings

IP address display

Connection status

Temperature Curve:

4-point curve adjustment

Visual representation

Save/load functionality

Validation checks

System Settings:

Hysteresis adjustment (0.5-10.0°C)

Factory reset

System information

Diagnostic tools

🔧 Troubleshooting
Common Issues and Solutions:
Problem	Possible Cause	Solution
ESP32 won't power on	Incorrect voltage	Use 5V power supply, check polarity
Sensors not detected	Wiring error	Check 4.7kΩ pull-up resistors
Relay not switching	Incorrect voltage	Check relay module voltage (5V/12V)
Web interface not loading	WiFi issue	Check AP/STA mode, reset WiFi
Burner cycles rapidly	Hysteresis too small	Increase hysteresis to 2.0-4.0°C
Temperature readings erratic	Sensor placement	Ensure good thermal contact
Settings not saving	Flash memory issue	Reset to factory, check power
Diagnostic Commands (Serial Monitor):
text
System Status: Check all task states
Sensor Test:   Verify DS18B20 communication
Relay Test:    Test relay operation
WiFi Scan:     Check available networks
Memory Check:  Monitor free heap memory
LED Status Indicators (If added):
Green LED: System operational

Red LED: Error condition

Blue LED: WiFi connected

Yellow LED: Burner active

📊 Technical Specifications
ESP32 Specifications:
Microcontroller: ESP32-WROOM-32

CPU: Dual-core 32-bit LX6, up to 240MHz

Memory: 520KB SRAM, 4MB Flash

WiFi: 802.11 b/g/n, 2.4GHz

GPIO: 34 programmable pins

Power: 5V DC, 500mA typical

Temperature Sensors:
Type: DS18B20 digital

Accuracy: ±0.5°C (10°C to 85°C)

Resolution: 9-12 bits (0.5°C to 0.0625°C)

Range: -55°C to +125°C

Interface: 1-Wire bus

Relay Module:
Type: 5V single-channel opto-isolated

Contact Rating: 10A/250V AC, 10A/30V DC

Isolation: 2500V opto-isolation

Response Time: <10ms

System Performance:
Temperature Update: 5 seconds

Control Update: 1 second

Outside Temp (Control): 10 minutes

Web Response: <100ms

Data Retention: >10 years (flash)

Environmental Ratings:
Operating Temperature: 0°C to 70°C

Storage Temperature: -40°C to 85°C

Humidity: 20% to 80% non-condensing

IP Rating: Depends on enclosure (recommend IP65)

Power Requirements:
text
ESP32:          5V DC @ 500mA (2.5W)
Relay Module:   5V DC @ 70mA (0.35W)
Sensors:        3.3V DC @ 5mA (0.017W)
Total:          Approx 3W continuous
🚀 Installation Checklist
Pre-Installation:
Verify all components are present

Test ESP32 with basic blink sketch

Test DS18B20 sensors with example code

Test relay module with manual control

Prepare installation tools

Wiring:
Connect DS18B20 sensors with pull-up resistors

Connect relay module to ESP32

Connect power supply (5V/2A)

Separate high-voltage and low-voltage wiring

Secure all connections

Software:
Upload compiled sketch to ESP32

Configure WiFi settings

Test web interface

Calibrate temperature sensors if needed

Set temperature curve points

Safety Verification:
Verify relay defaults to OFF on power loss

Test emergency stop function

Confirm proper grounding

Check isolation between circuits

Verify fuse protection

Final Testing:
Test automatic control mode

Test manual override

Verify temperature readings

Test web interface on mobile device

Monitor system for 24 hours

📞 Support & Maintenance
Regular Maintenance:
Monthly: Check sensor calibration

Quarterly: Verify web interface functionality

Annually: Inspect wiring and connections

As needed: Update firmware

Emergency Procedures:
Immediate Burner Shutdown: Use emergency stop on web interface

Power Off: Disconnect power to ESP32 system

Manual Control: Use burner's original controls if available

Contact Support: Provide error codes from Serial Monitor

Firmware Updates:
Backup settings before updating

Use stable releases only

Verify compatibility with hardware

Test all functions after update

✅ Conclusion
This ESP32 Oil Heating System provides intelligent, safe, and efficient control of your oil burner with modern web-based interface and robust safety features. The system is designed for reliability with multiple layers of protection and fault tolerance.

Key Benefits:

✅ Energy savings through optimized temperature control

✅ Enhanced safety with multiple protection systems

✅ Remote monitoring and control via web interface

✅ Easy configuration and adjustment

✅ Reliable operation with watchdog monitoring

✅ Data persistence through power cycles

Remember: Always consult with a qualified heating technician when connecting control systems to oil burners, as improper installation can be dangerous and may void equipment warranties.

For additional support or customization requests, please refer to the code comments or seek assistance from the developer community.
