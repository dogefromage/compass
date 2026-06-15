#include <ArduinoEigenDense.h>

#include "Arduino.h"
#include "compass.h"
#include "hardware.h"
#include "testing.h"

Compass compass;

void setup(void) {
    Serial.begin(115200);
    delay(3000);
    Serial.println("Starting!!!");
    delay(500);

    initialize_hardware();
    compass.init();

    // uncomment to run test
    // test_magnetic_regeneration_time();
    // test_coil_field_strength();
    // test_needle_pertubation();
    // test_needle_pertubation_with_correction();
    // test_gyro_bias();
}

void loop(void) {
    delay(10);
    compass.set_trigger(!digitalRead(PIN_TRIGGER));
    compass.update();
}
