#Mass spring damper system 
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import pandas as pd
import os

np.random.seed(5151)
Wind_Noise = 5
Sensor_Inaccuracy = 0.4
M = 5  # kg
K = 1  # N/m
C = 0.1 # Ns/m 
DT = 0.005

T = np.arange(0, 30, DT)
X_Real = np.zeros(len(T))
V_Real = np.zeros(len(T))

X_Real[0] = 0
V_Real[0] = 1

R = 0.1612
Q = np.array([[4.8958333e-10, 9.79166667e-8],
                    [1.46875e-7, 0.00005875]])
P = np.array([[1, 0],
              [0.07,    1.21]])


for i in range(len(T)-1):
    Random_Force = np.random.randn() * Wind_Noise
    accel = (-K/M * X_Real[i] - C/M * V_Real[i] + Random_Force / M)
    V_Real[i+1] = V_Real[i] + accel * DT
    X_Real[i+1] = X_Real[i] + V_Real[i] * DT

Measured_DT = 0.1
Measured_Times = np.arange(0, 30, Measured_DT)
Measured_Index = np.round(Measured_Times / DT).astype(int)

Z = X_Real[Measured_Index] + np.random.randn(len(Measured_Times)) * Sensor_Inaccuracy

Accel_Real = -K/M * X_Real - C/M * V_Real

A = np.array([[1, DT],
              [-K/M*DT, 1 - C/M * DT]])
H = np.array([[1, 0]])

X_H = np.array([0.0, 0.0])
X_EST = np.zeros((len(T), 2))
Measured_Index_1 = 0

P_trace = []  

for i in range(len(T)):
    X_H = A @ X_H
    P = A @ P @ A.T + Q

    X_EST[i] = X_H
    P_trace.append(P.copy())

    if Measured_Index_1 < len(Measured_Times) and abs(T[i] - Measured_Times[Measured_Index_1]) < 1e-8:
        Z_NOW = Z[Measured_Index_1]
        INNOVATION = Z_NOW - H @ X_H
        S = H @ P @ H.T + R
        K = P @ H.T / S

        X_H = X_H + K.flatten() * INNOVATION
        P = (np.eye(2) - K @ H) @ P

        X_EST[i] = X_H
        Measured_Index_1 += 1


innovations = []
for i in range(len(Measured_Times)):
    idx = Measured_Index[i]
    pred = X_EST[idx, 0]
    meas = Z[i]
    innovations.append(meas - pred)


innovation_variance = np.var(innovations)
print(f"Innovation variance: {innovation_variance}")
print(f"Expected (R): {R}")
print(f"Ratio: {innovation_variance/R}")

chi_squared = np.sum(np.array(innovations)**2) / R
dof = len(innovations)
print(f"Chi-squared statistic: {chi_squared}")
print(f"Expected range (95% conf): [{dof - 2*np.sqrt(2*dof)}, {dof + 2*np.sqrt(2*dof)}]")

pos_rmse = np.sqrt(np.mean((X_Real - X_EST[:,0])**2))
vel_rmse = np.sqrt(np.mean((V_Real - X_EST[:,1])**2))
print(f"Position RMSE: {pos_rmse:.4f} m")
print(f"Velocity RMSE: {vel_rmse:.4f} m/s")


P_trace = np.array(P_trace)
pos_3sigma = 3 * np.sqrt(P_trace[:, 0, 0])
vel_3sigma = 3 * np.sqrt(P_trace[:, 1, 1])


plotting = 1


if plotting == True:
    fig, axs = plt.subplots(3, 2, figsize=(14, 14))

    axs[0, 0].plot(T, X_Real, label="True Position", linewidth=2)
    axs[0, 0].plot(T, X_EST[:,0], label="Estimated Position", linestyle="--")
    axs[0, 0].scatter(Measured_Times, Z, color="red", s=10, alpha=0.6, label="Measurements")
    axs[0, 0].set_title("Mass-Spring-Damper Position Tracking")
    axs[0, 0].set_xlabel("Time (s)")
    axs[0, 0].set_ylabel("Position (m)")
    axs[0, 0].grid()
    axs[0, 0].legend()

    axs[0, 1].plot(T, V_Real, label="True Velocity", linewidth=2)
    axs[0, 1].plot(T, X_EST[:,1], label="Estimated Velocity", linestyle="--")
    axs[0, 1].set_title("Velocity Tracking")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel("Velocity (m/s)")
    axs[0, 1].grid()
    axs[0, 1].legend()

    axs[1, 0].plot(Measured_Times, innovations, label="Innovation (Residual)")
    axs[1, 0].axhline(0, color="black", linewidth=1)
    axs[1, 0].set_title("Innovation Sequence")
    axs[1, 0].set_xlabel("Measurement Time (s)")
    axs[1, 0].set_ylabel("Innovation")
    axs[1, 0].grid()
    axs[1, 0].legend()

    sigma = np.sqrt(R)
    axs[1, 1].plot(Measured_Times, innovations, label="Innovation", linewidth=1.5)
    axs[1, 1].axhline(0, color="black", linewidth=1)
    axs[1, 1].axhline( 2*sigma, color="green", linestyle="--", label="+2σ")
    axs[1, 1].axhline(-2*sigma, color="green", linestyle="--", label="-2σ")
    axs[1, 1].set_title("Innovation with ±2σ Bounds")
    axs[1, 1].set_xlabel("Measurement Time (s)")
    axs[1, 1].set_ylabel("Innovation")
    axs[1, 1].grid()
    axs[1, 1].legend()

    pos_error = X_Real - X_EST[:,0]
    axs[2, 0].plot(T, pos_error, label="Position Error")
    axs[2, 0].plot(T,  pos_3sigma, linestyle="--", color="green", label="+3σ")
    axs[2, 0].plot(T, -pos_3sigma, linestyle="--", color="green", label="-3σ")
    axs[2, 0].set_title("Position Error with ±3σ Bounds")
    axs[2, 0].set_xlabel("Time (s)")
    axs[2, 0].set_ylabel("Error (m)")
    axs[2, 0].grid()
    axs[2, 0].legend()

    vel_error = V_Real - X_EST[:,1]
    axs[2, 1].plot(T, vel_error, label="Velocity Error")
    axs[2, 1].plot(T,  vel_3sigma, linestyle="--", color="green", label="+3σ")
    axs[2, 1].plot(T, -vel_3sigma, linestyle="--", color="green", label="-3σ")
    axs[2, 1].set_title("Velocity Error with ±3σ Bounds")
    axs[2, 1].set_xlabel("Time (s)")
    axs[2, 1].set_ylabel("Error (m/s)")
    axs[2, 1].grid()
    axs[2, 1].legend()

    plt.tight_layout()
    plt.show() 