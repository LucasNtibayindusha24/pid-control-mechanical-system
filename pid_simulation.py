import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# System Parameters
# -----------------------------
m = 1.0      # mass (kg)
k = 20.0     # spring constant (N/m)
c = 2.0      # damping coefficient (N·s/m)

# -----------------------------
# PID Gains
# -----------------------------
Kp = 80.0
Ki = 10.0
Kd = 15.0

# -----------------------------
# Simulation Parameters
# -----------------------------
dt = 0.001
t_end = 5.0
time = np.arange(0, t_end, dt)

# -----------------------------
# Desired Position
# -----------------------------
x_target = 1.0  # meters

# -----------------------------
# State Variables
# -----------------------------
x = 0.0        # position
v = 0.0        # velocity

# PID internal variables
integral_error = 0.0
previous_error = 0.0

# Data storage
x_history = []
v_history = []
force_history = []

# -----------------------------
# Simulation Loop
# -----------------------------
for t in time:
    error = x_target - x
    integral_error += error * dt
    derivative_error = (error - previous_error) / dt
    
    # PID Control Force
    F = (Kp * error +
         Ki * integral_error +
         Kd * derivative_error)
    
    # System Dynamics
    a = (F - c * v - k * x) / m
    
    # Integrate motion
    v += a * dt
    x += v * dt
    
    previous_error = error
    
    # Store data
    x_history.append(x)
    v_history.append(v)
    force_history.append(F)

# -----------------------------
# Plot Results
# -----------------------------
plt.figure()
plt.plot(time, x_history, label="Position (m)")
plt.plot(time, np.ones_like(time) * x_target, '--', label="Target Position")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("PID Control of Mass-Spring-Damper System")
plt.legend()
plt.grid()
plt.show()

plt.figure()
plt.plot(time, force_history)
plt.xlabel("Time (s)")
plt.ylabel("Control Force (N)")
plt.title("Control Force vs Time")
plt.grid()
plt.show()
Add PID-controlled mass-spring-damper simulation
