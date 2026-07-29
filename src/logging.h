#pragma once

#include <ArduinoEigenDense.h>

void log_vector(const char* name, const Eigen::Vector3f& v);

void log_quaternion(const char* name, const Eigen::Quaternionf& v);

void log_mat3(const char* name, const Eigen::Matrix3f& m);

void log_tf(const char* name, const Eigen::Quaternionf& q, const Eigen::Vector3f& t);
