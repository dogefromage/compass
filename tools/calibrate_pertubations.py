from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset

files = [
    "data/pertubation1.csv",
    "data/pertubation2.csv",
    "data/pertubation3.csv",
]

# def plot_raw_data():

#     fig = plt.figure(figsize=(8, 6))
#     ax = fig.add_subplot(111, projection="3d")

#     for file, color in files:
#         df = pd.read_csv(file, header=0, skipinitialspace=True)

#         ax.scatter(df.x, df.y, df.z, c=df.angle, label=file, s=50)

#     ax.set_xlabel("X")
#     ax.set_ylabel("Y")
#     ax.set_zlabel("Z")
#     ax.set_title("3D Scatter Plot")
#     ax.legend()
#     plt.tight_layout()
#     plt.show()

# merge by mean
data = []
for file in files:
    df = pd.read_csv(file, header=0, skipinitialspace=True)

    df_np = df.to_numpy()
    df_np[:, 1:4] -= df_np[:, 1:4].mean(0)
    data.append(df_np)

data = np.concatenate(data, 0).T

# filter that one outlier
norms = np.linalg.norm(data[1:4], axis=0)
mask = norms < np.percentile(norms, 95)
data = data[:, mask]


class ShittyDataset(Dataset):
    def __init__(self, data):
        self.angles = torch.tensor(data[0].T)
        self.points = torch.tensor(data[1:4].T)

    def __len__(self):
        return len(self.points)

    def __getitem__(self, index):
        return self.angles[index], self.points[index]


loader = DataLoader(ShittyDataset(data), batch_size=16, shuffle=True)


class ShittyEllipse(nn.Module):
    def __init__(self):
        super().__init__()
        self.phase = nn.Parameter(torch.randn(1).double())
        self.map = nn.Parameter(torch.randn(3, 2).double())

    def forward(self, angles: torch.Tensor):
        w = angles + self.phase
        x = torch.stack([w.cos(), w.sin()], 0)
        return x.T @ self.map.T


model = ShittyEllipse()
optim = torch.optim.AdamW(model.parameters(), lr=0.001)

for epoch in range(1000):

    for angles, points in loader:
        est_points = model(angles)
        loss = torch.nn.functional.l1_loss(est_points, points)

        optim.zero_grad()
        loss.backward()
        optim.step()

        print("loss: ", loss.item())


t = torch.linspace(0, 6.28, 100)
with torch.no_grad():
    ellipse = model(t)
ellipse = ellipse.numpy()
t = t.numpy()


fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
ax.scatter(*data[1:4], c=data[0], label=file, s=50)

ax.scatter(ellipse[:, 0], ellipse[:, 1], ellipse[:, 2], "k", c=t)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("3D Scatter Plot")
ax.legend()
plt.tight_layout()
plt.show()


file_template = """\
// AUTOMATICALLY GENERATED FILE ---- tools/calibrate_pertubations.py
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

float needle_pertubation_phase_offset = PERTUBATION_PHASE;
Eigen::Matrix<float, 3, 2> needle_pertubation_map = (Eigen::Matrix<float, 3, 2>() << PERTUBATION_MAP).finished();
"""

pertubation_phase_str = str(model.get_parameter("phase").item())
pertubation_map_str = ", ".join(map(str, model.get_parameter("map").detach().numpy().ravel()))
calibration_file = str(file_template)
calibration_file = calibration_file.replace("PERTUBATION_PHASE", pertubation_phase_str)
calibration_file = calibration_file.replace("PERTUBATION_MAP", pertubation_map_str)

filename = "src/needle_pertubation_coefficients_generated.cpp"
with open(filename, "w") as f:
    f.write(calibration_file)
print(f"Written file: {filename}")
