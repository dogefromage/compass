#pragma once

#include <ArduinoEigenDense.h>
#include <stdarg.h>

void set_logging_divisor(uint64_t divisor);

void increment_log_cycle();

void log_vector(const char* name, const Eigen::Vector3f& v, int decimalplaces = 2);

void log_quaternion(const char* name, const Eigen::Quaternionf& v);

void log_mat3(const char* name, const Eigen::Matrix3f& m);

void log_tf(const char* name, const Eigen::Quaternionf& q, const Eigen::Vector3f& t);

void log_format(const char* fmt, ...);
