#include "logging.h"

void log_vector(const char* name, const Eigen::Vector3f& v) {
    Serial.print(name);
    Serial.print(" xyz ");
    Serial.print(v.x());
    Serial.print(" ");
    Serial.print(v.y());
    Serial.print(" ");
    Serial.print(v.z());
    Serial.println();
}

void log_quaternion(const char* name, const Eigen::Quaternionf& q) {
    Serial.print(name);
    Serial.print(" wxyz ");
    Serial.print(q.w());
    Serial.print(" ");
    Serial.print(q.x());
    Serial.print(" ");
    Serial.print(q.y());
    Serial.print(" ");
    Serial.print(q.z());
    Serial.println();
}
