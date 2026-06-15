#pragma once

#include <ArduinoEigenDense.h>

void log_vector(const char* name, const Eigen::Vector3f& v);

void log_quaternion(const char* name, const Eigen::Quaternionf& v);
