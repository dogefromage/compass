#include "testing.h"

#include <Arduino.h>
#include <ArduinoEigenDense.h>

#include "hardware.h"
#include "logging.h"

void test_magnetic_regeneration_time() {
    /**
     * TEST TIME MAGNETIC SENSOR NEEDS TO FIND EARTH FIELD AFTER COILS WHERE ENABLED
     */
    for (int i = 0; i < 5; i++) {
        Serial.println("waiting");
        delay(1000);
    }

    int numTests = 10;
    int numDelays = 12;
    int64_t delays[] = {500000, 200000, 100000, 50000, 20000, 10000,
                        5000,   2000,   1000,   500,   200,   100};

    for (int i = 0; i < numDelays; i++) {
        Eigen::Vector3f avg = Eigen::Vector3f::Zero();
        for (int j = 0; j < numTests; j++) {
            set_coils(-1, 0);
            delay(10);
            set_coils(0, 0);

            delayMicroseconds(delays[i]);

            Eigen::Vector3f body_acc, angular_vel, body_field, body_field_raw;
            read_inertial_sensors(body_acc, angular_vel, body_field, body_field_raw);
            avg += body_field;
        }
        avg /= (float)numTests;

        Serial.print(delays[i]);
        Serial.print(" ");
        Serial.print(avg.x());
        Serial.print(" ");
        Serial.print(avg.y());
        Serial.print(" ");
        Serial.print(avg.z());
        Serial.println("");
    }

    Serial.println("Done!");
    while (1);
}

bool trigger_released() {
    static bool last_trigger = false;
    delay(10);
    bool curr_trigger = !digitalRead(PIN_TRIGGER);
    bool is_released = last_trigger && !curr_trigger;
    last_trigger = curr_trigger;
    return is_released;
}

void test_coil_field_strength() {
    for (int i = 0; i < 5; i++) {
        Serial.println("waiting");
        delay(1000);
    }

    set_coils(0, 0);
    Serial.println("align device with north!");

    int num_strengths = 11;
    float strengths[] = {-1.0, -0.5, -0.25, -0.125, -0.1, 0, 0.1, 0.125, 0.25, 0.5, 1};

    for (int i = 0; i < num_strengths; i++) {
        Serial.println("waiting for trigger release...");
        while (!trigger_released());

        float strength = strengths[i];
        Serial.print("Setting coil to ");
        Serial.println(strength);
        // set_coils(0, strength);
        set_coils(strength, 0);
    }

    Serial.println("Done!");
    while (1);
}

void test_needle_pertubation() {
    /**
     * dont move device but then start rotating the needle around when prompted
     */

    for (int i = 0; i < 2; i++) {
        Serial.println("waiting");
        delay(1000);
    }

    Eigen::Vector3f min, max;
    min << 1000, 1000, 1000;
    max << -1000, -1000, -1000;

    Eigen::Vector3f lin_acc, angular_vel, body_field, body_field_raw;

    Serial.println("spinning the magnet...");
    Serial.println("");

    int iterations = 50;
    float angle = 0;

    Serial.println("angle, x, y, z");

    for (int i = 0; i < iterations; i++) {
        // set
        float cos = cosf(angle);
        float sin = sinf(angle);
        angle += 6.28f / iterations;
        set_coils(cos, sin);
        delay(2000);
        set_coils(0, 0);
        delay(50);

        // measure
        read_inertial_sensors(lin_acc, angular_vel, body_field, body_field_raw);
        min = min.cwiseMin(body_field);
        max = max.cwiseMax(body_field);

        Serial.print(angle);
        Serial.print(", ");
        Serial.print(body_field.x());
        Serial.print(", ");
        Serial.print(body_field.y());
        Serial.print(", ");
        Serial.print(body_field.z());
        Serial.println();
    }
    Serial.println();

    Serial.print("pertubation diameter X in Gauss: ");
    Serial.println(max.x() - min.x());
    Serial.print("pertubation diameter Y in Gauss: ");
    Serial.println(max.y() - min.y());
    Serial.print("pertubation diameter Z in Gauss: ");
    Serial.println(max.z() - min.z());

    Serial.println("Done!");
    while (1);
}

void test_needle_pertubation_with_correction() {
    /**
     * dont move device but then start rotating the needle around when prompted
     */

    for (int i = 0; i < 2; i++) {
        Serial.println("waiting");
        delay(1000);
    }

    Eigen::Vector3f min, max;
    min << 1000, 1000, 1000;
    max << -1000, -1000, -1000;

    Eigen::Vector3f lin_acc, angular_vel, body_field, body_field_raw;

    Serial.println("spinning the magnet...");
    Serial.println("");

    int iterations = 50;
    float angle = 0;

    Serial.println("angle, x, y, z");
    for (int i = 0; i < iterations; i++) {
        // set
        float cos = cosf(angle);
        float sin = sinf(angle);
        angle += 6.28f / iterations;
        set_coils(cos, sin);
        delay(2000);
        set_coils(0, 0);
        delay(50);

        // measure
        read_inertial_sensors(lin_acc, angular_vel, body_field, body_field_raw);

        Eigen::Vector3f adjusted_body_field = compensate_body_field_by_needle(body_field, angle);

        min = min.cwiseMin(adjusted_body_field);
        max = max.cwiseMax(adjusted_body_field);

        Serial.print(angle);
        Serial.print(", ");
        Serial.print(adjusted_body_field.x());
        Serial.print(", ");
        Serial.print(adjusted_body_field.y());
        Serial.print(", ");
        Serial.print(adjusted_body_field.z());
        Serial.println();
    }
    Serial.println();

    Serial.print("pertubation diameter X in Gauss: ");
    Serial.println(max.x() - min.x());
    Serial.print("pertubation diameter Y in Gauss: ");
    Serial.println(max.y() - min.y());
    Serial.print("pertubation diameter Z in Gauss: ");
    Serial.println(max.z() - min.z());

    Serial.println("Done!");
    while (1);
}

void test_gyro_bias() {
    /**
     * before measuring this, remove old bias!
     */
    Eigen::Vector3f gyro_bias(0, 0, 0);

    Eigen::Vector3f lin_acc, angular_vel, body_field, body_field_raw;

    for (int i = 0; i < 1000; i++) {
        read_inertial_sensors(lin_acc, angular_vel, body_field, body_field_raw);
        gyro_bias += angular_vel;
        delay(2);
    }

    gyro_bias /= 1000.0f;

    Serial.print("GYRO BIAS ");
    Serial.print(gyro_bias.x(), 6);
    Serial.print(" ");
    Serial.print(gyro_bias.y(), 6);
    Serial.print(" ");
    Serial.print(gyro_bias.z(), 6);
    Serial.println();

    Serial.println("Done!");
    while (1);
}
