#pragma once

#include <ArduinoEigenDense.h>

#include "ahrs.h"
#include "config.h"
#include "gnss.h"

enum class CompassState {
    COMPASS_INERTIAL,
    COMPASS_COILS,
    COMPASS_REFRACTORY_PERIOD,
};

class Compass {
   public:
    uint64_t last_inertial_micros;
    uint64_t last_state_change_micros;
    Eigen::Vector3f last_local_estimated_field;
    float last_coil_needle_angle = 0;
    AHRS ahrs;
    GNSS gnss;
    bool _trigger = false;

    CompassState state = CompassState::COMPASS_REFRACTORY_PERIOD;

    void init();

    void spin_needle();

    void set_coils_by_local_vector(Eigen::Vector3f& local_desired_direction);

    void set_trigger(bool trigger);

    void update_inertial();
    void start_needle();
    void set_state(CompassState new_state);
    void update();
};
