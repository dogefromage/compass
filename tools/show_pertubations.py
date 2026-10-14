from pathlib import Path

from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset

files = list(Path("data/29.07").glob("*.csv"))

# files = [
#     ("data/pertubation1.csv", "blue"),
#     ("data/pertubation_corrected.csv", "red"),
# ]


fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")

for filename in files:
    df = pd.read_csv(filename, header=0, skipinitialspace=True)

    ax.scatter(df.x, df.y, df.z, label=filename.stem, s=50)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("3D Scatter Plot")
ax.legend()
plt.tight_layout()
plt.show()
