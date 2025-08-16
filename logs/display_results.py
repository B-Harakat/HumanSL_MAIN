import yaml
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys

def read_trajectory_data(yaml_file):
    """Read trajectory data from YAML file."""
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    
    # Extract trajectory and metadata
    trajectory = np.array(data['trajectory_result']['trajectory'])
    metadata = data['trajectory_result']['metadata']
    dt = metadata['effective_dt']
    
    return trajectory, dt, metadata

def compute_derivatives(positions, dt):
    """Compute velocity, acceleration, and jerk using numerical differentiation."""
    # Velocity (first derivative)
    velocity = np.gradient(positions, dt, axis=0)
    
    # Acceleration (second derivative)
    acceleration = np.gradient(velocity, dt, axis=0)
    
    # Jerk (third derivative)
    jerk = np.gradient(acceleration, dt, axis=0)
    
    return velocity, acceleration, jerk

def compute_cumulative_costs(acceleration, jerk, dt):
    """Compute cumulative sum of squared acceleration and jerk costs."""
    # Cost is typically sum of squares
    acc_cost_per_step = np.sum(acceleration**2, axis=1) * dt
    jerk_cost_per_step = np.sum(jerk**2, axis=1) * dt
    
    # Cumulative sum
    cumulative_acc_cost = np.cumsum(acc_cost_per_step)
    cumulative_jerk_cost = np.cumsum(jerk_cost_per_step)
    
    return cumulative_acc_cost, cumulative_jerk_cost

def plot_trajectory_analysis(yaml_file):
    """Main function to plot trajectory analysis."""
    # Read data
    trajectory, dt, metadata = read_trajectory_data(yaml_file)
    num_joints = trajectory.shape[1]
    
    # Create time array
    time = np.arange(len(trajectory)) * dt
    
    # Compute derivatives
    velocity, acceleration, jerk = compute_derivatives(trajectory, dt)
    
    # Compute cumulative costs
    cumulative_acc_cost, cumulative_jerk_cost = compute_cumulative_costs(acceleration, jerk, dt)
    
    # Create figure with subplots
    fig = plt.figure(figsize=(12, 8))
    
    # Joint labels
    joint_labels = [f'Joint {i}' for i in range(num_joints)]
    colors = plt.cm.tab10(np.linspace(0, 1, num_joints))
    
    # Plot 1: Position
    plt.subplot(2, 3, 1)
    for i in range(num_joints):
        plt.plot(time, trajectory[:, i], label=joint_labels[i], color=colors[i])
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.title('Joint Positions')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot 2: Velocity
    plt.subplot(2, 3, 2)
    for i in range(num_joints):
        plt.plot(time, velocity[:, i], label=joint_labels[i], color=colors[i])
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (rad/s)')
    plt.title('Joint Velocities')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot 3: Acceleration
    plt.subplot(2, 3, 3)
    for i in range(num_joints):
        plt.plot(time, acceleration[:, i], label=joint_labels[i], color=colors[i])
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (rad/s²)')
    plt.title('Joint Accelerations')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot 4: Jerk
    plt.subplot(2, 3, 4)
    for i in range(num_joints):
        plt.plot(time, jerk[:, i], label=joint_labels[i], color=colors[i])
    plt.xlabel('Time (s)')
    plt.ylabel('Jerk (rad/s³)')
    plt.title('Joint Jerks')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot 5: Cumulative Acceleration Cost
    plt.subplot(2, 3, 5)
    plt.plot(time, cumulative_acc_cost, 'b-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Cumulative Acceleration Cost')
    plt.title('Cumulative Acceleration Cost')
    plt.grid(True, alpha=0.3)
    
    # Plot 6: Cumulative Jerk Cost
    plt.subplot(2, 3, 6)
    plt.plot(time, cumulative_jerk_cost, 'r-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Cumulative Jerk Cost')
    plt.title('Cumulative Jerk Cost')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Print some summary statistics
    print("Trajectory Analysis Summary:")
    print(f"Duration: {time[-1]:.3f} seconds")
    print(f"Number of time steps: {len(trajectory)}")
    print(f"Time step (dt): {dt:.3f} seconds")
    print(f"Number of joints: {num_joints}")
    print(f"Final acceleration cost: {cumulative_acc_cost[-1]:.6f}")
    print(f"Final jerk cost: {cumulative_jerk_cost[-1]:.6f}")
    print(f"Final error from metadata: {metadata['final_error']:.6f}")
    
    # Show max values for each joint
    print("\nMaximum absolute values per joint:")
    for i in range(num_joints):
        print(f"Joint {i}: pos={np.max(np.abs(trajectory[:, i])):.4f}, "
              f"vel={np.max(np.abs(velocity[:, i])):.4f}, "
              f"acc={np.max(np.abs(acceleration[:, i])):.4f}, "
              f"jerk={np.max(np.abs(jerk[:, i])):.4f}")
    
    plt.show()
    
    return trajectory, velocity, acceleration, jerk, time, cumulative_acc_cost, cumulative_jerk_cost

def main():
    """Main function to handle command line arguments and run trajectory analysis."""
    parser = argparse.ArgumentParser(description='Analyze and plot trajectory data from YAML file')
    parser.add_argument('yaml_file', help='Path to the YAML file containing trajectory data')
    
    args = parser.parse_args()
    
    try:
        # Run the analysis
        results = plot_trajectory_analysis(args.yaml_file)
        
        # Unpack results if you need them for further analysis
        trajectory, velocity, acceleration, jerk, time, cum_acc_cost, cum_jerk_cost = results
        
    except FileNotFoundError:
        print(f"Error: File '{args.yaml_file}' not found.")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()