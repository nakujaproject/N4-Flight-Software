#include "sd_logger.h"

// External function to avoid SPI conflicts
extern void disableAllDevices();

SDLogger::SDLogger(uint8_t csPin) : cs(csPin) {}

void SDLogger::setFilename(const char* fname) {
    filename = fname;
}

bool SDLogger::initialized() const {
    return isInitialized;
}

bool SDLogger::begin() {
    pinMode(cs, OUTPUT);
    disableAllDevices();
    if (!SD.begin(cs)) {
        Serial.println("❌ SDLogger: SD init failed");
        return false;
    }

    isInitialized = true;
    writeHeaderIfNew();
    return true;
}

void SDLogger::writeHeaderIfNew() {
    if (!SD.exists(filename)) {
        File file = SD.open(filename, FILE_WRITE);
        if (file) {
            file.println("Millis,Record,OpMode,ax,ay,az,pitch,roll,gx,gy,gz,lat,lon,gps_alt,pressure,temp,rel_alt,kalman_alt,kalman_vel");
            file.close();
        }
    }
}

// bool SDLogger::log(const telemetry_type_t& packet, const gps_type_t& gps) {
//     if (!isInitialized) return false;

//     disableAllDevices();
//     digitalWrite(cs, LOW);
//     File file = SD.open(filename, FILE_APPEND);
//     if (!file) {
//         Serial.println("❌ SDLogger: Failed to open file");
//         digitalWrite(cs, HIGH);
//         return false;
//     }

//     file.printf("%lu,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
//         millis(),
//         packet.record_number,
//         packet.operation_mode,
//         packet.acc_data.ax,
//         packet.acc_data.ay,
//         packet.acc_data.az,
//         packet.acc_data.pitch,
//         packet.acc_data.roll,
//         packet.gyro_data.gx,
//         packet.gyro_data.gy,
//         packet.gyro_data.gz,
//         gps.latitude,
//         gps.longitude,
//         gps.gps_altitude,
//         packet.alt_data.pressure,
//         packet.alt_data.temperature,
//         packet.alt_data.rel_altitude
//         // packet.alt_data.kalman_altitude,
//         // packet.alt_data.kalman_vertical_velocity
//     );

//     file.close();
//     digitalWrite(cs, HIGH);
//     return true;
// }

bool SDLogger::log(const telemetry_type_t& packet, const gps_type_t& gps) {
    if (!isInitialized) return false;

    // Skip if data is stale
    // if (millis() - g_last_telemetry_update > 500) {
    //     Serial.println("⏩ Skipped stale telemetry");
    //     return false;
    // }

    disableAllDevices();
    digitalWrite(cs, LOW);
    File file = SD.open(filename, FILE_APPEND);
    if (!file) {
        Serial.println("❌ SDLogger: Failed to open file");
        digitalWrite(cs, HIGH);
        return false;
    }

    // Log full packet with Kalman values
    file.printf("%lu,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.8f,%.8f,%.2f,%u,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%d,%.2f,%.2f\n",
        millis(),
        packet.record_number,
        packet.operation_mode,
        packet.acc_data.ax,
        packet.acc_data.ay,
        packet.acc_data.az,
        packet.acc_data.pitch,
        packet.acc_data.roll,
        packet.gyro_data.gx,
        packet.gyro_data.gy,
        packet.gyro_data.gz,
        gps.latitude,
        gps.longitude,
        gps.gps_altitude,
        gps.time,
        packet.alt_data.pressure,
        packet.alt_data.temperature,
        packet.alt_data.rel_altitude,
        packet.alt_data.velocity,
        packet.drogue_pin_state,
        packet.main_chute_pin_state,
        packet.battery_voltage,
        packet.wifi_rssi,
        packet.alt_data.kalman_altitude,
        packet.alt_data.kalman_vertical_velocity
    );

    file.close();
    digitalWrite(cs, HIGH);
    return true;
}

