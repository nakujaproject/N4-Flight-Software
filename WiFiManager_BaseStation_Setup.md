# WiFiManager Base Station IP Configuration

## Overview

The flight computer now supports dynamic configuration of the base station IP address and MQTT port through a user-friendly web interface using WiFiManager.

## How It Works

### 1. Automatic WiFi Setup
When the flight computer starts with `MQTT=1`, it will:
- Check for saved WiFi credentials
- If no credentials exist, or connection fails, it creates a WiFi access point named **"N4-Flight-Computer-Setup"**

### 2. Configuration Web Interface
1. Connect your phone/laptop to the **"N4-Flight-Computer-Setup"** WiFi network
2. Open a web browser and navigate to `192.168.4.1`
3. You'll see a configuration page with:
   - **WiFi Network Selection**: Choose your WiFi network and enter password
   - **Base Station IP Address**: Enter your laptop/base station IP (default: 192.168.100.248)
   - **MQTT Port**: Enter MQTT port number (default: 1883)

### 3. Configuration Steps
```
1. Select your WiFi network from the list
2. Enter WiFi password
3. Enter your laptop's IP address in "Base Station IP Address" field
4. Enter MQTT port (usually 1883)
5. Click "Save" to connect and store configuration
```

### 4. Persistent Storage
- Configuration is saved to ESP32's flash memory
- Settings persist across power cycles
- Can be updated anytime by forcing configuration mode

## Usage Examples

### Example 1: First Time Setup
```
1. Power on flight computer with MQTT=1
2. Flight computer creates "N4-Flight-Computer-Setup" network
3. Connect phone to this network
4. Open browser to 192.168.4.1
5. Configure:
   - WiFi: "MyHomeWiFi" / "password123"
   - Base Station IP: "192.168.1.100" (your laptop IP)
   - MQTT Port: "1883"
6. Click Save
7. Flight computer connects to WiFi and uses your laptop IP
```

### Example 2: Changing Base Station
```
1. Hold RESET button while powering on (forces config mode)
2. Connect to "N4-Flight-Computer-Setup"
3. Update Base Station IP to new laptop IP
4. Save configuration
```

## Technical Details

### Configuration Storage
- Uses ESP32 Preferences library
- Stored in namespace: "wifi-config"
- Keys: "basestation_ip", "mqtt_port"

### Default Values
- Base Station IP: `192.168.100.248`
- MQTT Port: `1883`

### WiFi Manager Features
- **Auto-reconnect**: Remembers WiFi credentials
- **Timeout**: Configuration portal closes after 3 minutes if no action
- **IP validation**: Checks IP address format
- **Port validation**: Ensures port is between 1-65535
- **Custom styling**: Dark theme for better visibility

## Code Integration

### Getting Configuration Values
```cpp
WIFIConfig wifi_config;

// Get current base station IP
const char* ip = wifi_config.getBaseStationIP();

// Get current MQTT port  
int port = wifi_config.getMQTTPort();

// Initialize MQTT with dynamic values
MQTTInit(wifi_config.getBaseStationIP(), wifi_config.getMQTTPort());
```

### Force Configuration Mode
To force the configuration portal (useful for changing settings):
```cpp
WiFiManager wm;
wm.resetSettings();  // Clear saved credentials
ESP.restart();       // Restart to enter config mode
```

## Troubleshooting

### Cannot Connect to Configuration Portal
- Ensure you're connected to "N4-Flight-Computer-Setup" network
- Try `http://192.168.4.1` in browser
- Clear browser cache if page doesn't load

### Invalid IP Address
- Use format: `192.168.1.100` (not `192.168.1.100:1883`)
- Don't include port in IP field
- Use separate MQTT Port field

### WiFi Connection Fails
- Double-check WiFi password
- Ensure WiFi network is 2.4GHz (ESP32 doesn't support 5GHz)
- Check if network has MAC address filtering

### Configuration Not Saving
- Wait for "Configuration saved" message in serial monitor
- Power cycle the device to test persistence
- Check serial output for error messages

## Serial Monitor Output

Successful configuration will show:
```
[WiFiConfig] Loaded config - IP: 192.168.1.100, Port: 1883
[WiFiConfig] Connected successfully. IP: 192.168.1.50
[WiFiConfig] Base station configured - IP: 192.168.1.100, Port: 1883
[WiFiConfig] Configuration saved to preferences
```
