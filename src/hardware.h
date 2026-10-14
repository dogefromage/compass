#pragma once

#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <ArduinoEigenDense.h>
#include <Wire.h>

#define PIN_TRIGGER (D10)

#define PIN_COIL_A1 (D0)
#define PIN_COIL_A2 (D1)
#define PIN_COIL_B1 (D2)
#define PIN_COIL_B2 (D3)

#define PIN_GPS_TX (D6)
#define PIN_GPS_RX (D7)

// magnet calibration
extern Eigen::Matrix3f mag_correction_map;
extern Eigen::Vector3f mag_correction_offset;

// for compensating the needle
extern float needle_pertubation_phase_offset;
extern Eigen::Matrix<float, 3, 2> needle_pertubation_map;

extern HardwareSerial gpsSerial;

extern Eigen::Matrix<float, 2, 3> coil_body_matrix;

void initialize_hardware();

void read_accelerometer(Eigen::Vector3f& lin_acc, Eigen::Vector3f& lin_acc_raw);
void read_magnetometer(Eigen::Vector3f& body_field, Eigen::Vector3f& body_field_raw);
void read_gyro(Eigen::Vector3f& angular_vel, Eigen::Vector3f& angular_vel_raw);

Eigen::Vector3f calculate_earth_field_gauss(Eigen::Vector3f& earth_position);

void set_coils(float x, float y);

Eigen::Vector3f compensate_body_field_by_needle(Eigen::Vector3f& body_field, float needle_angle);
