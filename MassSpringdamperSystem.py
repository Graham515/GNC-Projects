import numpy as np
import matplotlib.pyplot as plt

def pid_controller(Kp, Ki, Kd, setpoint, measured_value, dt, integral, previous_error):
    error = setpoint - measured_value
    integral += error * dt
    derivative = (error - previous_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    return output, integral, error

M = 5.0       #kg
K = 1.0       #N/m
C = 0.1       #Ns/m
DT = 0.005   

T = np.arange(0, 30, DT)

x = 0.0       
v = 0.0       

xs = []
vs = []
ts = []

Kp = 10
Ki = 0.5
Kd = 2

setpoint = 1.0     
integral = 0.0
prev_error = 0.0

for t in T:
   
    F_pid, integral, prev_error = pid_controller(
        Kp, Ki, Kd, setpoint, x, DT, integral, prev_error
    )

    a = (F_pid - C * v - K * x) / M
   
    v += a * DT
    x += v * DT
   
    xs.append(x)
    vs.append(v)
    ts.append(t)

plt.figure()
plt.plot(ts, xs)
plt.title("Mass-Spring-Damper PID Control")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.grid()
plt.show()
