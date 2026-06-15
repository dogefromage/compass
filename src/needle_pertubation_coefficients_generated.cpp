// AUTOMATICALLY GENERATED FILE ---- tools/plot_pertubations.py
#include "hardware.h"

/**
This works as follows: The needle causes a disturbance in the magnetic field.

get_pertubation(angle):
    w = angle + needle_pertubation_phase_offset
    x = [ cos(w) sin(w) ]^T
    p = needle_pertubation_map @ x
    return p

The angle is given as the standard trigonometric angle determined by the coil axes X and Y like atan2.
The pertubation p lives in the frame of the calibrated magnetometer sensor, i.e. body frame.
*/

float needle_pertubation_phase_offset = -1.2179085729189447;
Eigen::Matrix<float, 3, 2> needle_pertubation_map = (Eigen::Matrix<float, 3, 2>() << 0.1330314133421814, 0.18883473944427412, 0.20885928541845972, -0.002646602839985738, 0.17985361999592808, -0.2493168852605815).finished();
