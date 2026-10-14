#include "compass.h"

#include "hardware.h"
#include "logging.h"

void Compass::init() {
    last_inertial_micros = micros();
}

void Compass::spin_needle() {
    float t = micros() / 1'000'000.0f;
    float x = 6.28f * 0.5f * t;
    float a = sinf(x);
    float b = cosf(x);
    set_coils(a, b);
}

void Compass::set_coils_by_local_vector(Eigen::Vector3f& local_desired_direction) {
    /**
     * using test_coil_field_strength i measured the coils to be 6x stronger than the
     * horizontal earth component at inclination 63°.
     * therefore, the coils have aprox. the strength of 2.72 Gauss
     * theoretically the coils are suppost to be 2.28 Gauss by hardware choice
     */
    float raw_coil_strength_gauss = 2.72f;

    float remaining_distance = local_desired_direction.norm();
    log_format("REMAINING_DISTANCE %.2f meters\n", remaining_distance);

    if (remaining_distance < SPINNING_RADIUS_METERS) {
        spin_needle();
        return;
    }

    Eigen::Vector3f local_desired_direction_norm = local_desired_direction.normalized();

    // account for the duty cycle of switching between driving and measuring
    float drive_duty_cycle = TIME_DRIVE_PERIOD_MICROS /
                             (float)(TIME_DRIVE_PERIOD_MICROS + TIME_REFRACTORY_PERIOD_MICROS);
    float coil_strength_gauss = raw_coil_strength_gauss * drive_duty_cycle;

    // we need to compensate for the local earth field vector which has 1 gauss.
    // like this we do not exceed the maximum coil strength by the ***triangle inequality***
    Eigen::Vector3f local_drive_gauss =
        (coil_strength_gauss - 1.0f) * local_desired_direction_norm - gnss.local_field_gauss;

    log_vector("LOCAL_DESIRED_DIRECTION", local_desired_direction_norm);
    log_vector("LOCAL_FIELD_GAUSS", gnss.local_field_gauss);
    log_vector("LOCAL_DRIVE_GAUSS", local_drive_gauss);

    Eigen::Vector3f local_drive_normed = local_drive_gauss / coil_strength_gauss;

    // convert to body coordinates
    Eigen::Vector3f body_drive_normed = ahrs.vector_local_to_body(local_drive_normed);

    // map body coordinates of magnetic drive to coils which are wired diagonally
    Eigen::Vector2f coils_inputs = coil_body_matrix * body_drive_normed;
    set_coils(coils_inputs.x(), coils_inputs.y());

    // // record this, it represents the direction the needle goes actually in the coil coordinates
    // Eigen::Vector2f coil_last_driven_needle_dir =
    //     coil_body_matrix * ahrs.vector_local_to_body(local_desired_direction);
    // last_coil_needle_angle =
    //     atan2f(coil_last_driven_needle_dir.y(), coil_last_driven_needle_dir.x());
}

void Compass::set_trigger(bool trigger) {
    _trigger = trigger;
}

void Compass::update_inertial() {
    uint64_t current_micros = micros();
    if (current_micros == last_inertial_micros) {
        Serial.println("WARNING: 0 micros passed since last sense");
        return;
    }
    float inertial_dt = (current_micros - last_inertial_micros) / 1'000'000.0f;
    last_inertial_micros = current_micros;

    Eigen::Vector3f local_acc_goal = {0, 0, 9.81};

    Eigen::Vector3f body_acc_measured, body_acc_raw, angular_vel, angular_vel_raw,
        body_field_measured, body_field_raw;
    read_accelerometer(body_acc_measured, body_acc_raw);
    read_gyro(angular_vel, angular_vel_raw);
    read_magnetometer(body_field_measured, body_field_raw);

    // // scenario A: needle is undriven and points to north, in which case we do not
    // // need to correct the magnetometer reading which is influenced by the needles field
    // // scenario B: needle is driven and points to direction, then it must be corrected
    // if (_trigger) {
    //     body_field_measured =
    //         compensate_body_field_by_needle(body_field_measured, last_coil_needle_angle);
    // } else {
    //     Eigen::Vector2f coil_field =
    //         coil_body_matrix * ahrs.vector_local_to_body(gnss.local_field_gauss);
    //     float estimated_angle = atan2f(coil_field.y(), coil_field.x());
    //     body_field_measured = compensate_body_field_by_needle(body_field_measured,
    //     estimated_angle);
    // }

    // body_field_measured =
    //     project_by_vector(body_field_measured, ahrs.vector_local_to_body(local_acc_goal));
    // Eigen::Vector3f local_field = project_by_vector(gnss.local_field_gauss, local_acc_goal);
    Eigen::Vector3f local_field = gnss.local_field_gauss;

    // very important
    body_field_measured.normalize();
    local_field.normalize();

    ahrs.update(inertial_dt, angular_vel, body_acc_measured, local_acc_goal, body_field_measured,
                local_field);

    log_vector("ACC_LOCAL", local_acc_goal);
    log_vector("ACC_BODY_MEASURED", body_acc_measured);
    log_vector("FIELD_LOCAL", local_field);
    log_vector("FIELD_BODY_MEASURED", body_field_measured);
    log_vector("FIELD_BODY_RAW", body_field_raw);

    // log_quaternion("ATTITUDE", ahrs.attitude);
    log_tf("LOCAL_BODY_TRANSFORM", ahrs.attitude, Eigen::Vector3f::Zero());
}

void Compass::start_needle() {
    // // polyterasse center
    // float lat = 47.37616894307332;
    // float lon = 8.546724748932393;

    // // Pont Ambroix
    // float target_lat = 43.717199582831526;
    // float target_lon = 4.1519610736046895;

    // // garte
    // float target_lat = 47.450957853380146;
    // float target_lon = 7.671828585797041;
    // float target_alt = 468.5;

    // // baum irchel
    // float target_lat = 47.398775181909734;
    // float target_lon = 8.545220010759298;
    // // float target_alt = 400;  // would be actually 480

    // // baum irchel 2
    // float target_lat = 47.39901858383265;
    // float target_lon = 8.544908614874632;

    // GOAL SPÄTSTORFER
    float target_lat = 47.357152;
    float target_lon = 8.489720;
    // float target_alt = 759.3;

    Eigen::Matrix3f target_earth_local_matrix = get_earth_local_matrix(target_lat, target_lon);
    // take gnss altitude to make sure both are at same elevation
    Eigen::Vector3f earth_target_position =
        get_earth_position(target_lat, target_lon, gnss.gnss_alt);

    log_vector("EARTH_TARGET_POSITION", earth_target_position);

    Eigen::Vector3f earth_dir = earth_target_position - gnss.earth_position;
    Eigen::Vector3f local_dir = gnss.earth_local_matrix.transpose() * earth_dir;

    set_coils_by_local_vector(local_dir);
}

void Compass::set_state(CompassState new_state) {
    state = new_state;
    last_state_change_micros = micros();
    set_coils(0, 0);
}

void Compass::update() {
    gnss.update();

    if (!gnss.valid) {
        Serial.println("Waiting for GPS...");
        delay(1000);
        return;
    }

    Eigen::Quaternionf earth_local_rot = Eigen::Quaternionf(gnss.earth_local_matrix);
    log_tf("EARTH_LOCAL_TRANSFORM", earth_local_rot, gnss.earth_position);

    switch (state) {
        case CompassState::COMPASS_INERTIAL:
            // Serial.println("COMPASS_INERTIAL");
            update_inertial();
            // try only doing one update for now
            if (_trigger) {
                set_state(CompassState::COMPASS_COILS);
            }
            break;
        case CompassState::COMPASS_COILS:
            // Serial.println("COMPASS_COILS");
            start_needle();
            if (last_state_change_micros + TIME_DRIVE_PERIOD_MICROS < micros()) {
                set_state(CompassState::COMPASS_REFRACTORY_PERIOD);
            }
            break;
        case CompassState::COMPASS_REFRACTORY_PERIOD:
            // Serial.println("COMPASS_REFRACTORY_PERIOD");
            // do nothing here and wait for magnetic field to go back to normal
            if (last_state_change_micros + TIME_REFRACTORY_PERIOD_MICROS < micros()) {
                set_state(CompassState::COMPASS_INERTIAL);
            }
            break;
    }
}
