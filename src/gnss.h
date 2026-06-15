#pragma once

#include <ArduinoEigenDense.h>
#include <TinyGPS++.h>

Eigen::Matrix3f get_earth_local_matrix(float lat, float lon);

Eigen::Vector3f get_earth_position(Eigen::Matrix3f& earth_local_matrix);

class GNSS {
   public:
    TinyGPSPlus gps;
    bool valid = false;
    float lat, lon;
    Eigen::Matrix3f earth_local_matrix;
    Eigen::Vector3f earth_position, earth_field, local_field;

    void update_coordinate_systems(float lat, float lon);
    void update();
};
