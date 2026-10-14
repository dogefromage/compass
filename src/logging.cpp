#include "logging.h"

uint64_t log_cycle = 0;
uint64_t logging_divisor = 1;

void set_logging_divisor(uint64_t divisor) {
    logging_divisor = divisor;
}

void increment_log_cycle() {
    log_cycle++;
}

void log_vector(const char* name, const Eigen::Vector3f& v, int decimalplaces) {
    if (log_cycle % logging_divisor != 0) {
        return;
    }
    Serial.print(name);
    Serial.print(" xyz ");
    Serial.print(v.x(), decimalplaces);
    Serial.print(" ");
    Serial.print(v.y(), decimalplaces);
    Serial.print(" ");
    Serial.print(v.z(), decimalplaces);
    Serial.println();
}

void log_quaternion(const char* name, const Eigen::Quaternionf& q) {
    if (log_cycle % logging_divisor != 0) {
        return;
    }
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
    if (log_cycle % logging_divisor != 0) {
        return;
    }
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
    if (log_cycle % logging_divisor != 0) {
        return;
    }
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

void log_format(const char* fmt, ...) {
    if (log_cycle % logging_divisor != 0) {
        return;
    }
    char buf[256];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial.print(buf);
}
