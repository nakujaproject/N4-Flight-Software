/**
 * @file data_types.h
 * @brief defines the data types, structs and typedefs used to store flight data
 */

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <Arduino.h>

/**
 * A structure to represent acceleration data
 */
typedef struct Acceleration_Data{
    float ax;                   /*!< x axis acceleration */
    float ay;                   /*!< y axis acceleration */
    float az;                   /*!< z axis acceleration */
    float gx;                   /*!< x angular velocity */
    float gy;                   /*!< y angular velocity */
    float gz;                   /*!< z angular velocity */
    float pitch;                /*!< pitch angle */
    float roll;                 /*!< roll angle */
} accel_type_t;

/**
 * A structure to represent angular velocity data
 */
typedef struct Gyroscope_Data {
    double gx;                  /*!< x axis angular velocity */
    double gy;                  /*!< y axis angular velocity */
    double gz;                  /*!< z axis angular velocity */
} gyro_type_t;

/**
 * A structure to represent GPS data
 */
typedef struct GPS_Data{
    double latitude;            /*!< latitude coordinate */
    double longitude;           /*!< longitude coordinate */
    uint16_t gps_altitude;      /*!< altitude read by the GPS */
    uint time;                  /*!< time read by the GPS */
} gps_type_t;

/**
 * A structure to represent the altimeter data
 */
typedef struct Altimeter_Data{
    double pressure;             /*!< atmospheric pressure */
    double rel_altitude;             /*!< current relative altitude read by the altimeter */
    double velocity;             /*!< velocity from the altimeter */
    double temperature;          /*!< altimeter temperature */
    double AGL;                  /*!< altitude above ground level */
} altimeter_type_t;

/**
 * A structure to represent telemetry data. This is the data transmitted to ground
 */
typedef struct Telemetry_Data {
    uint32_t record_number;     /*!< current row number for flight data logging  */
    uint8_t operation_mode;     /*!< operation mode to tell whether we are in SAFE or FLIGHT mode */
    uint8_t state;              /*!< current flight state. See states.h */
    altimeter_type_t alt_data;  /*!< altimeter data */
    accel_type_t acc_data;      /*!< accelerometer data */
    gyro_type_t gyro_data;      /*!< gyroscope data */
    gps_type_t gps_data;        /*!< gps data */
} telemetry_type_t;

#endif