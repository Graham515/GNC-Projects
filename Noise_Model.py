import numpy as np
import matplotlib.pyplot as plt


Seed = False
if Seed:
    np.random.seed(42)

FS = 60
T = 3
N = T * FS

Noise1 = np.random.uniform(-0.001, 0.001, N)
Noise2 = np.random.uniform(-0.001, 0.001, N)


Baseline = 20.0        # Meters
True_Range = 3000.0    # Meters


A_Radian = np.arctan(Baseline / (2 * True_Range))
A_Turn = A_Radian / (2 * np.pi)


Angle_1_Turn =  A_Turn + Noise1
Angle_2_Turn = -A_Turn + Noise2

Angle_1_Radian = Angle_1_Turn * 2 * np.pi
Angle_2_Radian = Angle_2_Turn * 2 * np.pi


Range_Estimate = Baseline / (np.tan(Angle_1_Radian) - np.tan(Angle_2_Radian))
