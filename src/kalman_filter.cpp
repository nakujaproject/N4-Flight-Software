#include <Wire.h>
#include <SFE_BMP180.h>
#include "mpu.h"  
#include <BasicLinearAlgebra.h>
#include "defs.h"
#include "data_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
using namespace BLA;

float kalmanFilter(float z);  



SFE_BMP180 bmp180;
#define MPU_ADDRESS     0x68
#define MPU_ACCEL_RANGE 16
#define GYRO_RANGE      1000

MPU6050 mpu(MPU_ADDRESS, MPU_ACCEL_RANGE, GYRO_RANGE);  
extern QueueHandle_t kalman2D_input_queue_handle;
extern QueueHandle_t debug_to_term_queue_handle; 


extern altimeter_type_t altimeter_packet; 
extern String filename;
 // Global variable for filename
extern SemaphoreHandle_t bmp180_mutex;
float pitch = 0;
float roll = 0;
float yaw = 0;

//unsigned long timer = 0;
extern float timeStep;  // Use timeStep defined in main.cpp

const int mainPin = 26; // Pin for LED
const int droguePin =27;

// Gyroscope offsets
float gyroX_offset = 0.0;
float gyroY_offset = 0.0;
float gyroZ_offset = 0.0;
int flightState =0;

// const char ssid[] = "*******";
// const char pass[] = "********";
//const int ledPin= 25;

//const char* topic_publish = "esp32/status";
// Simple Kalman filter variables for BMP180 altitude estimation
// float estimatedAltitude = 0.0;
// float errorCovariance_bmp = 1.0;
// float processVariance_bmp = 0.001;
// float measurementVariance_bmp = 0.1;
// float kalmanGain_bmp;
// float AltitudeKalman, VelocityVerticalKalman;
extern float AltitudeKalman, VelocityVerticalKalman;

// // Calculate the base pressure by averaging readings
// float calculateBasePressure(int numReadings) {
//   double T, P, totalPressure = 0;
//   char status;

//   for (int i = 0; i < numReadings; i++) {
//     status = bmp180.startTemperature();
//     if (status != 0) {
//       delay(200);
//       status = bmp180.getTemperature(T);
//       if (status != 0) {
//         status = bmp180.startPressure(3);
//         if (status != 0) {
//           delay(status);
//           status = bmp180.getPressure(P, T);
//           if (status != 0) {
//             totalPressure += P;
//           }
//         }
//       }
//     }
//     //delay(100);
//   }

//   return totalPressure / numReadings;
// }




// Calibrate the sensor by taking multiple readings
void calibrateSensor() {
  const int numReadings = 100;
   float x_sum = 0, y_sum = 0, z_sum = 0;
   AltitudeKalman =0; 
   VelocityVerticalKalman =0;

for (int i = 0; i < numReadings; i++) {
    x_sum += mpu.readXAngularVelocity();
    y_sum += mpu.readYAngularVelocity();
    z_sum += mpu.readZAngularVelocity();
    delay(10);
}


  // Compute average offsets
  gyroX_offset = x_sum / numReadings;
  gyroY_offset = y_sum / numReadings;
  gyroZ_offset = z_sum / numReadings;


}

// Kalman filter matrices for 2D filter (altitude & vertical velocity)
//float AltitudeKalman, VelocityVerticalKalman;
// BLA::Matrix<2,2> F, P, Q, I;
// BLA::Matrix<2,1> G, S, K;
// BLA::Matrix<1,2> H;
// BLA::Matrix<1,1> R, L, inv_L, Acc, M;

// MPU and orientation variables
float Po;
float AccYInertial;
float AccZInertial;
float AccYInertial_g;
float AccX_offset = 0.0;
float AngleRoll, AnglePitch;

// Shared variables for inter-task communication


  // Initialize Kalman filter matrices
 
// // BMP180 altitude reading task
// void taskBMP180(void *pvParameters) {
//   while (true) {
//    if (!bmp180.begin()) {
//     Serial.println("BMP180 init failed!");
//     while (1);
//   }
//   Serial.println("BMP180 init success");
// }
//   Serial.println("Warming up BMP180...");
//       delay(1000);

//     double T, P;
//     char status = bmp180.startTemperature();
//     if (status != 0) {
//       delay(10);
//       status = bmp180.getTemperature(T);
//       if (status != 0) {
//         status = bmp180.startPressure(3);
//         if (status != 0) {
//           delay(status);
//           status = bmp180.getPressure(P, T);
//           if (status != 0) {
//             bmpAltitude = bmp180.altitude(P, Po);

//             estimatedAltitude = kalmanFilter(bmpAltitude);
//           }
//         }
//       }
//     }
//     vTaskDelay(50/ portTICK_PERIOD_MS);
//   }




// Apply Kalman filter to new altitude measurements
// float kalmanFilter(float z) {
//   float estimatedAltitude_pred = estimatedAltitude;
//   float errorCovariance_pred = errorCovariance_bmp + processVariance_bmp;
//   kalmanGain_bmp = errorCovariance_pred / (errorCovariance_pred + measurementVariance_bmp);
//   estimatedAltitude = estimatedAltitude_pred + kalmanGain_bmp * (z - estimatedAltitude_pred);
//   errorCovariance_bmp = (1 - kalmanGain_bmp) * errorCovariance_pred;

//   return estimatedAltitude;
// }
// 2D Kalman filter task
// void taskKalman2D(void *pvParameters) {
//     telemetry_type_t input_data;
//     telemetry_type_t telemetry_data;
//     while (true) {
     

//     timer = millis();

//     // Read raw acceleration and gyroscope data
//     float ax = mpu.readXAcceleration();
//     float ay = mpu.readYAcceleration();
//     float az = mpu.readZAcceleration();

//     float gx = mpu.readXAngularVelocity();
//     float gy = mpu.readYAngularVelocity();
//     float gz = mpu.readZAngularVelocity();


//     // Calculate tilt-adjusted AccZInertial
//     // AngleRoll = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + (a.acceleration.z+1.0) * a.acceleration.z)) * 180 / PI;
//     // AnglePitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + (a.acceleration.z+1.0)* a.acceleration.z)) * 180 / PI;
//     // //Gravitational acceleration is in the downward direction and is taken as positive ,since we are moving upwards, we need to invert the sign so that g is negative
//     // AccZInertial = -((-sin(AnglePitch * (PI / 180)) * a.acceleration.x +
//     //         cos(AnglePitch * (PI / 180)) * sin(AngleRoll * (PI / 180)) * a.acceleration.y +
//     //         cos(AnglePitch * (PI / 180)) * cos(AngleRoll * (PI / 180)) * a.acceleration.z) - 8.93);//should be -9.81 , taken as 9 to account for error
//     // AccZInertial_g = AccZInertial / 9.81;
//     float AccYInertial = az-1.03;
//     float AccYInertial_g = AccYInertial / 9.81;
//     // Apply 2D Kalman filter
//     Acc = {AccYInertial};
//     S = F * S + G * Acc;
//     P = F * P * ~F + Q;
//     L = H * P * ~H + R;
//     inv_L = Inverse(L);
//     K = P * ~H * inv_L;
//     M = {(float)altimeter_packet.filtered_altitude_1d};
//     S = S + K * (M - H * S);
//     AltitudeKalman = S(0, 0);
//     VelocityVerticalKalman = S(1, 0);

//     altimeter_packet.kalman_altitude = AltitudeKalman;
//     altimeter_packet.kalman_vertical_velocity = VelocityVerticalKalman;
//     P = (I - K * H) * P;
   
//     telemetry_data.alt_data.kalman_altitude = altimeter_packet.kalman_altitude;
//     telemetry_data.alt_data.kalman_vertical_velocity = altimeter_packet.kalman_vertical_velocity;

//      // After updating altimeter_packet.kalman_altitude and altimeter_packet.kalman_vertical_velocity

//    telemetry_type_t telemetry_data;
//    telemetry_data.alt_data = altimeter_packet; // Copy all altimeter data, including filtered values
// // Fill other fields as needed (accel, gyro, gps, etc.)

//    xQueueSend(debug_to_term_queue_handle, &telemetry_data, 0); // Or your actual output queue


    
//     // Serial output
//     digitalWrite(ledPin,HIGH);
//     // Serial.print(" Raw acceleration:");Serial.print(az);Serial.print("\n");
//     // Serial.print("accleration in z direction:");Serial.print(AccZInertial);Serial.print("\n");
//     // Serial.print("Raw Altitude:");Serial.print(estimatedAltitude);Serial.print("\n");
//     // Serial.print("Filtered Altitude:"); Serial.print(AltitudeKalman); Serial.print("\n");
//     // Serial.print("VerticalVelocity:"); Serial.print(VelocityVerticalKalman); Serial.print("\n");
//     //Serial.print("AccZInertial (m/s²):"); Serial.print(AccYInertial); Serial.print("\n");
//     //Serial.print("AccZInertial (g):"); Serial.print(AccYInertial_g); Serial.print("\n");
    
//     }
//     vTaskDelay((timeStep * 1000) / portTICK_PERIOD_MS);
//     }


