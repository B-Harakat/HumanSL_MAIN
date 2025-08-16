#!/usr/bin/env python3

import yaml
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def load_trajectory_record(filename):
    """Load trajectory record from YAML file."""
    if not os.path.exists(filename):
        print(f"Error: File {filename} does not exist")
        return None
        
    with open(filename, 'r') as file:
        data = yaml.safe_load(file)
    
    target_trajectory = np.array(data['target_trajectory'])
    actual_trajectory = np.array(data['actual_trajectory'])
    
    return target_trajectory, actual_trajectory

def plot_joint_comparison(target_traj, actual_traj, filename):
    """Plot comparison of target vs actual angles for all 7 joints."""
    
    # Create time axis
    time_target = np.linspace(0, len(target_traj) - 1, len(target_traj))
    time_actual = np.linspace(0, len(actual_traj) - 1, len(actual_traj))
    
    # Create subplots for 7 joints
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    fig.suptitle(f'Joint Angle Comparison: {os.path.basename(filename)}', fontsize=16)
    
    # Flatten axes array for easier indexing
    axes_flat = axes.flatten()
    
    joint_names = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7']
    
    for i in range(7):
        ax = axes_flat[i]
        
        # Plot target and actual trajectories
        ax.plot(time_target, target_traj[:, i], 'b-', label='Target', linewidth=2)
        ax.plot(time_actual, actual_traj[:, i], 'r--', label='Actual', linewidth=2)
        
        ax.set_title(joint_names[i])
        ax.set_xlabel('Time Step')
        ax.set_ylabel('Angle (rad)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Calculate and display RMS error
        if len(target_traj) == len(actual_traj):
            rms_error = np.sqrt(np.mean((target_traj[:, i] - actual_traj[:, i])**2))
            ax.text(0.02, 0.98, f'RMS Error: {rms_error:.4f}', 
                   transform=ax.transAxes, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Hide the extra subplots (we only need 7)
    for i in range(7, 9):
        axes_flat[i].set_visible(False)
    
    plt.tight_layout()
    plt.show()

def plot_joint_errors(target_traj, actual_traj, filename):
    """Plot error for each joint over time."""
    
    if len(target_traj) != len(actual_traj):
        print("Warning: Target and actual trajectories have different lengths")
        min_len = min(len(target_traj), len(actual_traj))
        target_traj = target_traj[:min_len]
        actual_traj = actual_traj[:min_len]
    
    errors = target_traj - actual_traj
    time_axis = np.linspace(0, len(errors) - 1, len(errors))
    
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    fig.suptitle(f'Joint Angle Errors: {os.path.basename(filename)}', fontsize=16)
    
    axes_flat = axes.flatten()
    joint_names = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7']
    
    for i in range(7):
        ax = axes_flat[i]
        
        ax.plot(time_axis, errors[:, i], 'g-', linewidth=2)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        
        ax.set_title(f'{joint_names[i]} Error')
        ax.set_xlabel('Time Step')
        ax.set_ylabel('Error (rad)')
        ax.grid(True, alpha=0.3)
        
        # Display statistics
        mean_error = np.mean(errors[:, i])
        std_error = np.std(errors[:, i])
        max_error = np.max(np.abs(errors[:, i]))
        
        stats_text = f'Mean: {mean_error:.4f}\nStd: {std_error:.4f}\nMax: {max_error:.4f}'
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
               verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
    
    # Hide extra subplots
    for i in range(7, 9):
        axes_flat[i].set_visible(False)
    
    plt.tight_layout()
    plt.show()

def main():
    if len(sys.argv) != 2:
        print("Usage: python display_record.py <yaml_file>")
        print("Example: python display_record.py trajectory_record.yaml")
        return
    
    filename = sys.argv[1]
    
    # Load data
    result = load_trajectory_record(filename)
    if result is None:
        return
    
    target_traj, actual_traj = result
    
    print(f"Loaded trajectory data:")
    print(f"  Target trajectory shape: {target_traj.shape}")
    print(f"  Actual trajectory shape: {actual_traj.shape}")
    print(f"  Number of joints: {target_traj.shape[1] if len(target_traj.shape) > 1 else 1}")
    
    # Plot comparisons
    plot_joint_comparison(target_traj, actual_traj, filename)
    plot_joint_errors(target_traj, actual_traj, filename)

if __name__ == "__main__":
    main()