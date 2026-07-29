#pragma once

#include <ArduinoEigenDense.h>
#include <TinyGPS++.h>

Eigen::Matrix3f get_earth_local_matrix(float lat, float lon);

Eigen::Vector3f get_earth_position(float lat, float lon, float alt);

class GNSS {
   public:
    TinyGPSPlus gps;
    bool valid = false;
    float gnss_lat, gnss_lon, gnss_alt;
    Eigen::Matrix3f earth_local_matrix;
    Eigen::Vector3f earth_position, earth_field_gauss, local_field_gauss;

    void update_coordinate_systems();
    void update();
};
