# N4 Rocket Beacon Receiver

This ESP32 beacon receiver listens for ESP-NOW telemetry transmissions from the N4 flight computer and forwards the data to the base station with RSSI information.

## Features

- **ESP-NOW Reception**: Receives telemetry beacons from flight computer
- **RSSI Capture**: Measures signal strength of received beacons
- **Data Validation**: Validates CSV format and field count
- **HTTP Forwarding**: Sends parsed telemetry + RSSI to base station via HTTP POST
- **WiFi Management**: Automatic reconnection on WiFi loss
- **JSON Output**: Converts CSV telemetry to structured JSON with RSSI

## Data Format

### Received CSV Format (22 fields):
```
record_number,operation_mode,state,ax,ay,az,pitch,roll,gx,gy,gz,latitude,longitude,gps_altitude,gps_time,pressure,temperature,rel_altitude,velocity,drogue_pin_state,main_chute_pin_state,battery_voltage
```

### Output JSON Format:
```json
{
  "timestamp": 1234567890,
  "rssi": -45,
  "mac_address": "AA:BB:CC:DD:EE:FF",
  "record_number": 123,
  "operation_mode": 1,
  "state": 2,
  "acceleration": {
    "ax": 0.11, "ay": -0.05, "az": 1.02,
    "pitch": 3.47, "roll": 4.52
  },
  "gyroscope": {
    "gx": 330.91, "gy": -344.66, "gz": -120.85
  },
  "gps": {
    "latitude": 0.0000, "longitude": 0.0000,
    "altitude": 0.00, "time": 1234567890
  },
  "altimeter": {
    "pressure": 856.66, "temperature": 23.51,
    "altitude": 0.31, "velocity": 2.15
  },
  "pyro": {
    "drogue_state": 0, "main_state": 0
  },
  "battery_voltage": 21.09
}
```

## Configuration

Update these constants in `main.cpp`:

```cpp
const char* WIFI_SSID = "your-wifi-ssid";
const char* WIFI_PASSWORD = "your-wifi-password";  
const char* BASE_STATION_URL = "http://192.168.100.248:3001/api/telemetry";
```

## Hardware Setup

1. **ESP32 DevKit v1** (or compatible)
2. **WiFi Connection** for base station communication
3. **ESP-NOW** for flight computer beacon reception

## Building and Uploading

```bash
cd beacon-receiver
pio run --target upload --target monitor
```

## Monitoring

The receiver outputs detailed logs including:
- Beacon reception status with RSSI
- Data validation results  
- HTTP POST status
- WiFi connection status
- Parsed telemetry data

## Base Station Integration

The receiver sends HTTP POST requests to the base station endpoint. Ensure your base station accepts JSON payloads at the configured URL.

## Troubleshooting

- **No beacons received**: Check ESP-NOW compatibility and proximity to flight computer
- **HTTP errors**: Verify base station URL and network connectivity
- **Data validation failures**: Check CSV format and field count (should be 22 fields)
- **WiFi issues**: Monitor serial output for connection status and errors
