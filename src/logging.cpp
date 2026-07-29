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

void log_mat3(const char* name, const Eigen::Matrix3f& m) {
    Serial.print(name);
    Serial.print(" rowmajor_3x3 ");
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Serial.print(m.coeff(i, j));
            Serial.print(" ");
        }
    }
    Serial.println();
}

void log_tf(const char* name, const Eigen::Quaternionf& q, const Eigen::Vector3f& t) {
    Serial.print(name);
    Serial.print(" tf_wxyz_xyz ");
    Serial.print(q.w());
    Serial.print(" ");
    Serial.print(q.x());
    Serial.print(" ");
    Serial.print(q.y());
    Serial.print(" ");
    Serial.print(q.z());
    Serial.print(" ");
    Serial.print(t.x());
    Serial.print(" ");
    Serial.print(t.y());
    Serial.print(" ");
    Serial.print(t.z());
    Serial.println();
}
