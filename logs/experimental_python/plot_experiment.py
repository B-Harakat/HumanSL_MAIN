import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Try to import scipy filters, fall back to simple filter if not available
try:
    from scipy.signal import butter, filtfilt
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("SciPy not available, using simple exponential smoothing")

# Read the CSV data
data = pd.read_csv('../experimental_results/TolgaHomingDidnotTriggerTubeWasAtTarget.csv')

# Time vector (10ms per iteration)
dt = 0.01  # 10ms
time_full = np.arange(len(data)) * dt

# Extract data between 5 and 30 seconds
start_idx = int(5.0 / dt)  # 5 seconds
end_idx = int(30.0 / dt)   # 30 seconds

# Filter data to the specified time range
data_filtered = data.iloc[start_idx:end_idx]
time = time_full[start_idx:end_idx]

# Extract right arm data (7 joints: 0-6) from filtered data
right_arm_positions = data_filtered[[f'q_cur_right_{i}' for i in range(7)]]
right_arm_torques = data_filtered[[f'u_cur_right_{i}' for i in range(7)]]

# Import additional filters
from scipy.signal import savgol_filter

# Filter functions
def butterworth_lowpass_filter(data, cutoff_freq, sampling_freq, order=6):
    """Apply Butterworth low-pass filter"""
    nyquist = sampling_freq / 2
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def savitzky_golay_filter(data, window_length=31, polyorder=3):
    """Apply Savitzky-Golay filter"""
    return savgol_filter(data, window_length, polyorder)

def combined_filter(data):
    """Apply combined filtering approach"""
    # First apply Butterworth to remove high frequency noise
    butter_filtered = butterworth_lowpass_filter(data, cutoff_freq=5.0, sampling_freq=100.0, order=4)
    # Then apply Savitzky-Golay to smooth while preserving trends
    final_filtered = savitzky_golay_filter(butter_filtered, window_length=21, polyorder=3)
    return final_filtered

def exponential_smoothing_filter(data, alpha=0.1):
    """Apply exponential smoothing filter (fallback if scipy not available)"""
    filtered = np.zeros_like(data)
    filtered[0] = data[0]
    for i in range(1, len(data)):
        filtered[i] = alpha * data[i] + (1 - alpha) * filtered[i-1]
    return filtered

def apply_filter(data):
    """Apply appropriate filter based on availability"""
    if SCIPY_AVAILABLE:
        # Use original Butterworth configuration
        return butterworth_lowpass_filter(data, cutoff_freq=5.0, sampling_freq=100.0, order=4)
    else:
        # Use exponential smoothing as fallback
        return exponential_smoothing_filter(data, alpha=0.15)

# Calculate velocity by numerical differentiation
right_arm_velocities = pd.DataFrame()
for i in range(7):
    col_name = f'q_cur_right_{i}'
    velocity = np.gradient(right_arm_positions[col_name], dt)
    # Apply low-pass filter to smooth velocity
    velocity_filtered = apply_filter(velocity)
    right_arm_velocities[f'qd_right_{i}'] = velocity_filtered

# Calculate acceleration by numerical differentiation of velocity
right_arm_accelerations = pd.DataFrame()
for i in range(7):
    col_name = f'qd_right_{i}'
    acceleration = np.gradient(right_arm_velocities[col_name], dt)
    # Apply low-pass filter to smooth acceleration
    acceleration_filtered = apply_filter(acceleration)
    right_arm_accelerations[f'qdd_right_{i}'] = acceleration_filtered

# Create subplots for all joint data
fig, axes = plt.subplots(4, 7, figsize=(20, 16))
fig.suptitle('Right Arm Joint Data', fontsize=16, fontweight='bold')

joint_names = [f'Joint {i}' for i in range(7)]

# Plot position
for i in range(7):
    axes[0, i].plot(time, right_arm_positions[f'q_cur_right_{i}'], 'b-', linewidth=1.5)
    axes[0, i].set_title(f'{joint_names[i]} Position')
    axes[0, i].set_ylabel('Position (deg)')
    axes[0, i].grid(True, alpha=0.3)

# Plot velocity
for i in range(7):
    axes[1, i].plot(time, right_arm_velocities[f'qd_right_{i}'], 'g-', linewidth=1.5)
    axes[1, i].set_title(f'{joint_names[i]} Velocity')
    axes[1, i].set_ylabel('Velocity (deg/s)')
    axes[1, i].grid(True, alpha=0.3)

# Plot acceleration
for i in range(7):
    axes[2, i].plot(time, right_arm_accelerations[f'qdd_right_{i}'], 'r-', linewidth=1.5)
    axes[2, i].set_title(f'{joint_names[i]} Acceleration')
    axes[2, i].set_ylabel('Acceleration (deg/s�)')
    axes[2, i].grid(True, alpha=0.3)

# Plot torque
for i in range(7):
    axes[3, i].plot(time, right_arm_torques[f'u_cur_right_{i}'], 'm-', linewidth=1.5)
    axes[3, i].set_title(f'{joint_names[i]} Torque')
    axes[3, i].set_ylabel('Torque (Nm)')
    axes[3, i].set_xlabel('Time (s)')
    axes[3, i].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Also create individual plots for better visualization
fig2, axes2 = plt.subplots(2, 2, figsize=(15, 10))
fig2.suptitle('Right Arm Joint Data Summary', fontsize=16, fontweight='bold')

# Plot all joints position
axes2[0, 0].set_title('All Joints Position')
for i in range(7):
    axes2[0, 0].plot(time, right_arm_positions[f'q_cur_right_{i}'], label=f'Joint {i}', linewidth=1.5)
axes2[0, 0].set_ylabel('Position (deg)')
axes2[0, 0].legend()
axes2[0, 0].grid(True, alpha=0.3)

# Plot all joints velocity
axes2[0, 1].set_title('All Joints Velocity')
for i in range(7):
    axes2[0, 1].plot(time, right_arm_velocities[f'qd_right_{i}'], label=f'Joint {i}', linewidth=1.5)
axes2[0, 1].set_ylabel('Velocity (deg/s)')
axes2[0, 1].legend()
axes2[0, 1].grid(True, alpha=0.3)

# Plot all joints acceleration
axes2[1, 0].set_title('All Joints Acceleration')
for i in range(7):
    axes2[1, 0].plot(time, right_arm_accelerations[f'qdd_right_{i}'], label=f'Joint {i}', linewidth=1.5)
axes2[1, 0].set_ylabel('Acceleration (deg/s�)')
axes2[1, 0].set_xlabel('Time (s)')
axes2[1, 0].legend()
axes2[1, 0].grid(True, alpha=0.3)

# Plot all joints torque
axes2[1, 1].set_title('All Joints Torque')
for i in range(7):
    axes2[1, 1].plot(time, right_arm_torques[f'u_cur_right_{i}'], label=f'Joint {i}', linewidth=1.5)
axes2[1, 1].set_ylabel('Torque (Nm)')
axes2[1, 1].set_xlabel('Time (s)')
axes2[1, 1].legend()
axes2[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Print some statistics
print("Right Arm Joint Statistics:")
print("=" * 50)
print("\nPosition Statistics (degrees):")
print(right_arm_positions.describe())
print("\nVelocity Statistics (deg/s):")
print(right_arm_velocities.describe())
print("\nAcceleration Statistics (deg/s�):")
print(right_arm_accelerations.describe())
print("\nTorque Statistics (Nm):")
print(right_arm_torques.describe())