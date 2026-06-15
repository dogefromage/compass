from itertools import count

import numpy as np
import serial
import time
from tqdm import tqdm
import viser
from scanf import scanf
import matplotlib.pyplot as plt
from scipy import linalg

DEVICE = "/dev/ttyACM0"
BAUD = 115200


def fit_ellipsoid(points):
    # points: (M, 3)

    x, y, z = points.T
    data = np.stack([x * x, y * y, z * z, x * y, x * z, y * z, x, y, z, np.ones_like(x)], axis=1)

    U, S, Vh = np.linalg.svd(data)
    coeffs = Vh[-1]
    # coeffs = np.linalg.lstsq(data, rhs)[0]
    A, B, C, D, E, F, G, H, I, J = coeffs

    M = np.array(
        [
            [A, D / 2, E / 2],
            [D / 2, B, F / 2],
            [E / 2, F / 2, C],
        ]
    )

    if np.linalg.det(M) == 0:
        raise Exception("Degenerate solution, det M == 0, apparently cylindrical shape or something like this")

    b = np.array([G, H, I])
    M_inv = np.linalg.inv(M)

    p = -0.5 * M_inv @ b

    c = J - 0.25 * b.T @ M_inv @ b

    if c == 0:
        raise Exception("c == 0, idk when and if this could happen")

    # it should hold that
    # (X - p)T @ M @ (X - p) + c = 0

    Q = M / -c

    # (X - p)T @ Q @ (X - p) = 1

    # diagonalize Q
    eigenvalues, eigenvectors = np.linalg.eig(Q)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[order]

    if not np.all(eigenvalues > 0):
        raise Exception("not all eigenvalues > 0, not PSD")

    return eigenvalues, eigenvectors, p


# class MagnetometerCalibrator:
#     def __init__(self, magnetic_field_strength=1000):
#         self.F = magnetic_field_strength
#         self.b = np.zeros([3, 1])  # Hard iron bias
#         self.A_1 = np.eye(3)  # Soft iron correction matrix

#     def calibrate(self, data):
#         # data shape: (N, 3)
#         s = data.T
#         M, n, d = self._ellipsoid_fit(s)

#         M_1 = linalg.inv(M)
#         self.b = -np.dot(M_1, n)
#         self.A_1 = np.real(self.F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))

#     def apply_calibration(self, data):
#         return (data - self.b.T) @ self.A_1.T


def plot_points(X, title):

    # Enable 3D plotting
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_aspect("equal")

    x, y, z = X.T

    # Scatter plot
    ax.scatter(
        x,
        y,
        z,
        c=z,
        cmap="viridis",
        s=1,
        alpha=0.8,
    )
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.title(title)


mag_cal_template = """
// AUTOMATICALLY GENERATED FILE ---- tools/calibration.py
#include "hardware.h"

Eigen::Matrix3f mag_correction_map = (Eigen::Matrix3f() << CORRECTION_MAP_ROW_MAJOR).finished();
Eigen::Vector3f mag_correction_offset = (Eigen::Vector3f() << CORRECTION_OFFSET).finished();
"""


def calibrate():

    path_npz = "data/mag.npz"
    points = np.load(path_npz)["points"]
    # points: (M, 3)

    magnitudes = np.linalg.norm(points, axis=1)
    inlier_threshold = 0.2 * magnitudes.max()
    # plt.hist(magnitudes, 20)
    # plt.show()

    # filter by masking
    points = points[magnitudes > inlier_threshold]

    eigenvalues, eigenvectors, p = fit_ellipsoid(points)

    print("eigenvalues")
    print(eigenvalues)
    print("eigenvectors")
    print(eigenvectors)
    print("p")
    print(p)

    semi_axes = eigenvectors / np.sqrt(eigenvalues)[None, :]
    plot_points(points, "raw measurements")
    for i in range(3):
        plt.plot(
            [p[0], semi_axes[0][i]],
            [p[1], semi_axes[1][i]],
            [p[2], semi_axes[2][i]],
            color="red",
        )
    plt.show()

    sqrt_lambda = np.diag(np.sqrt(eigenvalues))

    calibration_map = eigenvectors @ sqrt_lambda @ eigenvectors.T
    calibration_offset = p
    print("calibration_map")
    print(calibration_map)
    print("calibration_offset")
    print(calibration_offset)

    correction_map_str = ", ".join(map(str, calibration_map.ravel()))
    correction_off_str = ", ".join(map(str, calibration_offset))
    calibration_file = str(mag_cal_template)
    calibration_file = calibration_file.replace("CORRECTION_MAP_ROW_MAJOR", correction_map_str)
    calibration_file = calibration_file.replace("CORRECTION_OFFSET", correction_off_str)

    with open("src/hardware_mag_cal_generated.cpp", "w") as f:
        f.write(calibration_file)

    points_corrected = (calibration_map @ (points - calibration_offset[None, :]).T).T
    plot_points(points_corrected, "corrected points")
    plt.show()


def measure_and_save():
    ser = serial.Serial(DEVICE, BAUD, timeout=1)

    body_field_rows = []

    sample_count = 1000
    for i in tqdm(count(), "rotate the sensor!"):

        line = ser.readline().decode("utf-8").strip()
        if line:
            if line.startswith("FIELD RAW xyz"):
                data = scanf("FIELD RAW xyz %f %f %f", line)
                if data is None:
                    print(f"WARNING weird format: {line}")
                else:
                    body_field_rows.append(data)

        if len(body_field_rows) >= sample_count:
            break

    points = np.array(body_field_rows)

    np.savez("data/mag.npz", points=points)

    plot_points(points, "mag")
    plt.show()


if __name__ == "__main__":
    measure_and_save()
    calibrate()
