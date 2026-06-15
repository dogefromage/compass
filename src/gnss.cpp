#include "gnss.h"

#include "config.h"
#include "hardware.h"

Eigen::Matrix3f get_earth_local_matrix(float lat, float lon) {
    /**
     * expects lat lon in WGS 84
     *
     * Earth coordinate system:
     *  like the international bla bal one
     *  Z represents true north (mean rotation axis of earth)
     *  X points to greenwich meridian
     *  system is right handed
     *
     * Local coordinate system:
     *  acts as coords where device is located but oriented respective to ground
     *  this one has Z pointing upwards (respective to gravity)
     *  X points towards the rotational axis of earth tangentially
     *  Y is roughly pointing westwards
     */

    Eigen::AngleAxisf rot_lon(lon * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf rot_lat(-lat * M_PI / 180.0f, Eigen::Vector3f::UnitY());
    Eigen::Matrix3f permutation;
    permutation << 0, 0, 1, 0, -1, 0, 1, 0, 0;

    Eigen::Matrix3f earth_local = rot_lon * rot_lat * permutation;

    return earth_local;
}

Eigen::Vector3f get_earth_position(Eigen::Matrix3f& earth_local_matrix) {
    float avg_earth_radius = 6'371'000;
    return earth_local_matrix * Eigen::Vector3f::UnitZ() * avg_earth_radius;
}

void print_gps_data(TinyGPSPlus& gps) {
    // Display GPS data if valid
    if (gps.location.isValid()) {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
    } else {
        Serial.println("Location not available.");
    }

    if (gps.date.isValid()) {
        Serial.print("Date: ");
        Serial.print(gps.date.month());
        Serial.print("/");
        Serial.print(gps.date.day());
        Serial.print("/");
        Serial.println(gps.date.year());

    } else {
        Serial.println("Date not available.");
    }

    if (gps.time.isValid()) {
        Serial.print("Time: ");
        Serial.print(gps.time.hour());
        Serial.print(":");
        Serial.print(gps.time.minute());
        Serial.print(":");
        Serial.println(gps.time.second());
    } else {
        Serial.println("Time not available.");
    }
}

void GNSS::update_coordinate_systems(float lat, float lon) {
    earth_local_matrix = get_earth_local_matrix(lat, lon);
    earth_position = get_earth_position(earth_local_matrix);
    earth_field = calculate_earth_field(earth_position);
    local_field = earth_local_matrix.transpose() * earth_field;
}

void GNSS::update() {
#if SPOOF_GPS
    // irchel park
    lat = 47.39772392969787;
    lon = 8.543186485361696;
    valid = true;
#else
    // Process GPS data from the module
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();

        if (!valid) {
            Serial.write(c);  // print gps output
        }
        gps.encode(c);
    }

    // TODO use date
    valid = gps.location.isValid();
    lat = gps.location.lat();
    lon = gps.location.lng();
#endif

    if (valid) {
        update_coordinate_systems(lat, lon);
    }
}
