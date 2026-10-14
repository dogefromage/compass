#include "gnss.h"

#include "config.h"
#include "hardware.h"
#include "logging.h"

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

void geodeticToECEF(double latDeg, double lonDeg, double h, double &x, double &y, double &z) {
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double e2 = f * (2.0 - f);

    double lat = latDeg * PI / 180.0;
    double lon = lonDeg * PI / 180.0;

    double sinLat = sin(lat);
    double cosLat = cos(lat);

    double N = a / sqrt(1.0 - e2 * sinLat * sinLat);

    x = (N + h) * cosLat * cos(lon);
    y = (N + h) * cosLat * sin(lon);
    z = (N * (1.0 - e2) + h) * sinLat;
}

Eigen::Vector3f get_earth_position(float lat, float lon, float alt) {
    double x, y, z;
    geodeticToECEF(lat, lon, alt, x, y, z);
    return Eigen::Vector3f(x, y, z);
}

void print_gps_data(TinyGPSPlus &gps) {
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

void GNSS::update_coordinate_systems() {
    earth_local_matrix = get_earth_local_matrix(gnss_lat, gnss_lon);
    earth_position = get_earth_position(gnss_lat, gnss_lon, gnss_alt);
    earth_field_gauss = calculate_earth_field_gauss(earth_position);
    local_field_gauss = earth_local_matrix.transpose() * earth_field_gauss;
}

void GNSS::update() {
#if SPOOF_GPS
    // irchel park
    gnss_lat = 47.39772392969787;
    gnss_lon = 8.543186485361696;
    gnss_alt = 400;

    // // büre garte
    // gnss_lat = 47.45100098191752;
    // gnss_lon = 7.671724327829716;
    // gnss_alt = 468.5;

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
    gnss_lat = gps.location.lat();
    gnss_lon = gps.location.lng();
    gnss_alt = gps.altitude.meters();
#endif

    if (valid) {
        update_coordinate_systems();
        log_format("GNSS valid: %.8f, %.8f / %.2f\n", gnss_lat, gnss_lon, gnss_alt);
    } else {
        log_format("GNSS invalid\n");
    }
}
