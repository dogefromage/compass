#include "hardware.h"

#include <HardwareSerial.h>

#include <XYZgeomag.hpp>

#include "config.h"
#include "logging.h"

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

HardwareSerial gpsSerial(1);

Eigen::Matrix<float, 2, 3> coil_body_matrix{};

bool last_sample_rejected = false;
Eigen::Vector3f last_body_field_raw;
float mag_sensor_rejection_threshold = 30;

Eigen::Vector3f gyro_bias{0.002731, -0.002209, 0.013324};

// tools/accelerometer_cal.py
Eigen::Vector3f accelerometer_bias{-0.16923833, 0.236615, -1.940725};
Eigen::Vector3f accelerometer_scale{1.01008089, 1.02857425, 0.99842119};

void initialize_hardware() {
    pinMode(PIN_TRIGGER, INPUT_PULLUP);

    ledcAttach(PIN_COIL_A1, COIL_FREQUENCY, 8);
    ledcAttach(PIN_COIL_A2, COIL_FREQUENCY, 8);
    ledcAttach(PIN_COIL_B1, COIL_FREQUENCY, 8);
    ledcAttach(PIN_COIL_B2, COIL_FREQUENCY, 8);

    /* Initialise the sensor */
    if (!accel.begin()) {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while (1);
    }

    /* Initialise the sensor */
    if (!mag.begin()) {
        /* There was a problem detecting the LSM303 ... check your connections */
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while (1);
    }

    accel.setMode(LSM303_MODE_HIGH_RESOLUTION);
    accel.setRange(LSM303_RANGE_4G);

    mag.setMagRate(LSM303_MAGRATE_75);
    mag.setMagGain(LSM303_MAGGAIN_4_0);

    /* Enable auto-ranging */
    gyro.enableAutoRange(true);

    /* Initialise the sensor */
    if (!gyro.begin()) {
        /* There was a problem detecting the L3GD20 ... check your connections */
        Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
        while (1);
    }

    // init gps
    gpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);  // For GPS Module

    // coil body matrix
    // coil_body_matrix << 0.707, -0.707, 0, -0.707, -0.707, 0; // OLD
    float r2 = sqrtf(2) / 2.0f;
    coil_body_matrix << 0, r2, r2, 0, r2, -r2;
}

void read_accelerometer(Eigen::Vector3f& lin_acc, Eigen::Vector3f& lin_acc_raw) {
    sensors_event_t event;

    accel.getEvent(&event);

    lin_acc_raw[0] = event.acceleration.x;
    lin_acc_raw[1] = event.acceleration.y;
    lin_acc_raw[2] = event.acceleration.z;

    lin_acc = accelerometer_scale.cwiseProduct(lin_acc_raw - accelerometer_bias);
}

void read_magnetometer(Eigen::Vector3f& body_field, Eigen::Vector3f& body_field_raw) {
    sensors_event_t event;

    mag.getEvent(&event);

    body_field_raw = {event.magnetic.x, event.magnetic.y, event.magnetic.z};
    // log_vector("MAG_RAW", mag_raw);

    if ((body_field_raw - last_body_field_raw).norm() > mag_sensor_rejection_threshold &&
        !last_sample_rejected) {
        body_field_raw = last_body_field_raw;
        last_sample_rejected = true;
    } else {
        last_body_field_raw = body_field_raw;
        last_sample_rejected = false;
    }

    body_field = mag_correction_map * (body_field_raw - mag_correction_offset);
}

void read_gyro(Eigen::Vector3f& angular_vel, Eigen::Vector3f& angular_vel_raw) {
    sensors_event_t event;

    gyro.getEvent(&event);

    angular_vel_raw[0] = event.gyro.x;
    angular_vel_raw[1] = event.gyro.y;
    angular_vel_raw[2] = event.gyro.z;

    angular_vel = angular_vel_raw - gyro_bias;
}

Eigen::Vector3f compensate_body_field_by_needle(Eigen::Vector3f& body_field, float needle_angle) {
    // needle angle given in coil coordinates (XY unit circle of coils)
    float w = needle_angle + needle_pertubation_phase_offset;
    Eigen::Vector2f x = Eigen::Vector2f(cosf(w), sinf(w));
    Eigen::Vector3f p = needle_pertubation_map * x;
    return body_field - p;
}

Eigen::Vector3f calculate_earth_field_gauss(Eigen::Vector3f& earth_position) {
    geomag::Vector ecef{earth_position.x(), earth_position.y(), earth_position.z()};
    geomag::Vector earth_field = geomag::GeoMag(2027.5, ecef, geomag::WMM2025);

    Eigen::Vector3f res;
    res[0] = earth_field.x;
    res[1] = earth_field.y;
    res[2] = earth_field.z;

    res *= 10'000;  // [Gauss/Tesla]

    return res;
}

void set_coils(float coil_a, float coil_b) {
    if (coil_a > 0) {
        int val = (int)(coil_a * 255.0f);
        if (val > 255) {
            val = 255;
        }
        ledcWrite(PIN_COIL_A1, val);
        ledcWrite(PIN_COIL_A2, 0);
    } else if (coil_a < 0) {
        int val = (int)(-coil_a * 255.0f);
        if (val > 255) {
            val = 255;
        }
        ledcWrite(PIN_COIL_A2, val);
        ledcWrite(PIN_COIL_A1, 0);
    } else {
        ledcWrite(PIN_COIL_A1, 0);
        ledcWrite(PIN_COIL_A2, 0);
    }

    if (coil_b > 0) {
        int val = (int)(coil_b * 255.0f);
        if (val > 255) {
            val = 255;
        }
        ledcWrite(PIN_COIL_B1, val);
        ledcWrite(PIN_COIL_B2, 0);
    } else if (coil_b < 0) {
        int val = (int)(-coil_b * 255.0f);
        if (val > 255) {
            val = 255;
        }
        ledcWrite(PIN_COIL_B2, val);
        ledcWrite(PIN_COIL_B1, 0);
    } else {
        ledcWrite(PIN_COIL_B1, 0);
        ledcWrite(PIN_COIL_B2, 0);
    }
}
