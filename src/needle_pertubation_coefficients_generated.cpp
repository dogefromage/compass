// AUTOMATICALLY GENERATED FILE ---- tools/calibrate_pertubations.py
#include "hardware.h"

/**
This works as follows: The needle causes a disturbance in the magnetic field.

get_pertubation(angle):
    w = angle + needle_pertubation_phase_offset
    x = [ cos(w) sin(w) ]^T
    p = needle_pertubation_map @ x
    return p

The angle is given as the standard trigonometric angle determined by the coil axes X and Y like
atan2. The pertubation p lives in the frame of the calibrated magnetometer sensor, i.e. body frame.
*/

float needle_pertubation_phase_offset = -2.085509224726343;
Eigen::Matrix<float, 3, 2> needle_pertubation_map =
    (Eigen::Matrix<float, 3, 2>() << 0.23159064590364123, 0.019144458846911603, 0.13273206565284745,
     -0.16136655029980995, -0.07394479940208426, -0.29825237896260354)
        .finished();
