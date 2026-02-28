#!/usr/bin/env python3
"""
poc_rpi.py - Raspberry Pi PoC Verification Script
Uses ctypes to call C core, with matplotlib real-time visualization
"""
import ctypes
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Compile C library:
# gcc -shared -fPIC -o libcontrol.so ../pid.c ../kalman.c ../ekf_attitude.c -lm

lib = ctypes.CDLL('./libcontrol.so')

# --- PID struct mapping ---
class PID_t(ctypes.Structure):
    _fields_ = [
        ('kp', ctypes.c_float),
        ('ki', ctypes.c_float),
        ('kd', ctypes.c_float),
        ('dt', ctypes.c_float),
        ('out_min', ctypes.c_float),
        ('out_max', ctypes.c_float),
        ('integral_min', ctypes.c_float),
        ('integral_max', ctypes.c_float),
        ('integral', ctypes.c_float),
        ('prev_measurement', ctypes.c_float),
        ('prev_error', ctypes.c_float),
        ('initialized', ctypes.c_int),
    ]

class KF1D_t(ctypes.Structure):
    _fields_ = [
        ('x_est', ctypes.c_float),
        ('p_est', ctypes.c_float),
        ('q', ctypes.c_float),
        ('r', ctypes.c_float),
        ('k', ctypes.c_float),
    ]

# --- Function signatures ---
lib.pid_init.argtypes = [ctypes.POINTER(PID_t), ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float]
lib.pid_init.restype = None

lib.pid_compute.argtypes = [ctypes.POINTER(PID_t), ctypes.c_float, ctypes.c_float]
lib.pid_compute.restype = ctypes.c_float

lib.kf1d_init.argtypes = [ctypes.POINTER(KF1D_t), ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float]
lib.kf1d_init.restype = None

lib.kf1d_update.argtypes = [ctypes.POINTER(KF1D_t), ctypes.c_float]
lib.kf1d_update.restype = ctypes.c_float


def run_simulation():
    """Simulate Kalman + PID control loop"""
    dt = 0.02  # 50 Hz
    steps = 500

    pid = PID_t()
    lib.pid_init(ctypes.byref(pid), 2.0, 0.5, 0.1, dt, -100.0, 100.0)

    kf = KF1D_t()
    lib.kf1d_init(ctypes.byref(kf), 0.01, 1.0, 0.0, 1.0)

    setpoint = 50.0
    plant_state = 0.0   # Simple first-order system
    tau = 1.0            # Time constant

    log = {'time': [], 'setpoint': [], 'true': [],
           'noisy': [], 'filtered': [], 'output': []}

    for i in range(steps):
        t = i * dt

        # Sensor noise
        noise = np.random.normal(0, 2.0)
        measurement = plant_state + noise

        # Kalman Filter
        filtered = lib.kf1d_update(ctypes.byref(kf), measurement)

        # PID
        output = lib.pid_compute(ctypes.byref(pid), setpoint, filtered)

        # Simulate first-order system: dx/dt = (-x + K*u) / tau
        plant_state += (-plant_state + output) / tau * dt

        log['time'].append(t)
        log['setpoint'].append(setpoint)
        log['true'].append(plant_state)
        log['noisy'].append(measurement)
        log['filtered'].append(filtered)
        log['output'].append(output)

    return log


def plot_results(log):
    """Visualize results"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(log['time'], log['setpoint'], 'r--', label='Setpoint')
    axes[0].plot(log['time'], log['noisy'], 'gray', alpha=0.4, label='Noisy Sensor')
    axes[0].plot(log['time'], log['filtered'], 'b-', label='Kalman Estimate')
    axes[0].plot(log['time'], log['true'], 'g-', linewidth=2, label='True State')
    axes[0].set_ylabel('Value')
    axes[0].legend()
    axes[0].set_title('Kalman Filter + PID Control Simulation')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(log['time'], log['output'], 'orange', label='PID Output')
    axes[1].set_ylabel('Control Output')
    axes[1].set_xlabel('Time (s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('kalman_pid_result.png', dpi=150)
    plt.show()


if __name__ == '__main__':
    log = run_simulation()
    plot_results(log)
    print("Simulation complete. Results saved to kalman_pid_result.png")
