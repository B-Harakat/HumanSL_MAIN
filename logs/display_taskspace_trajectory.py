import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import ezc3d

def extract_marker_trajectory(c3d_file_path, subject_name, marker_name, start_frame, end_frame):
    """
    Extract marker trajectory from C3D file between specified frames.
    
    Parameters:
    -----------
    c3d_file_path : str
        Path to the C3D file
    subject_name : str
        Subject name (e.g., 'rossana')
    marker_name : str
        Marker name (e.g., 'RFIN')
    start_frame : int
        Starting frame number
    end_frame : int
        Ending frame number
        
    Returns:
    --------
    trajectory : numpy.ndarray, shape (n_valid_frames, 3)
        Valid marker positions in meters (converted from mm)
    frame_indices : numpy.ndarray
        Corresponding frame indices for valid positions
    """
    try:
        # Load the C3D file
        c3d_data = ezc3d.c3d(c3d_file_path)
        
        # Get marker labels and find the target marker
        marker_labels = c3d_data['parameters']['POINT']['LABELS']['value']
        
        # Find marker index - handle different possible naming conventions
        marker_idx = None
        target_marker = f"{subject_name}:{marker_name}".upper()
        
        for i, label in enumerate(marker_labels):
            if isinstance(label, list):
                label_str = ''.join(label).strip()
            else:
                label_str = str(label).strip()
                
            if target_marker in label_str.upper() or marker_name.upper() in label_str.upper():
                marker_idx = i
                print(f"Found marker: {label_str} at index {i}")
                break
        
        if marker_idx is None:
            print(f"Available markers: {marker_labels}")
            raise ValueError(f"Marker {target_marker} not found in C3D file")
        
        # Extract marker data
        point_data = c3d_data['data']['points']  # Shape: (4, n_markers, n_frames)
        
        # Get the specific marker data for the frame range
        marker_data = point_data[:3, marker_idx, start_frame:end_frame+1]  # x, y, z coordinates
        
        # Convert from mm to meters and transpose to get (n_frames, 3)
        marker_data = marker_data.T / 1000.0
        
        # Filter out invalid/missing data (typically marked with very large negative values or zeros)
        valid_mask = np.all(np.abs(marker_data) < 10.0, axis=1)  # Reasonable range in meters
        valid_mask &= np.all(marker_data != 0.0, axis=1)  # Remove zero entries
        
        valid_trajectory = marker_data[valid_mask]
        valid_frame_indices = np.arange(start_frame, end_frame + 1)[valid_mask]
        
        print(f"Extracted {len(valid_trajectory)} valid frames out of {end_frame - start_frame + 1} total frames")
        
        return valid_trajectory, valid_frame_indices
        
    except Exception as e:
        print(f"Error loading C3D file: {e}")
        raise

def fit_cubic_spline_3points(start_pos, end_pos, percentage, height):
    """
    Fit a natural cubic spline through 3 points and return 100 evenly spaced points.
    
    Parameters:
    -----------
    start_pos : array-like, shape (3,)
        Starting position (x, y, z)
    end_pos : array-like, shape (3,)
        Ending position (x, y, z)
    percentage : float
        Percentage along the straight line between start and end (0.0 to 1.0)
    height : float
        Height offset in z-direction from the line point
        
    Returns:
    --------
    points : numpy.ndarray, shape (100, 3)
        100 evenly spaced points along the cubic spline
    """
    # Convert to numpy arrays
    start_pos = np.array(start_pos)
    end_pos = np.array(end_pos)
    
    # Calculate the middle point
    line_point = start_pos + percentage * (end_pos - start_pos)
    middle_point = line_point.copy()
    middle_point[2] += height  # Add height offset in z-direction
    
    # Create the 3 control points
    control_points = np.array([start_pos, middle_point, end_pos])
    
    # Parameter values for the 3 points (evenly spaced for natural cubic spline)
    t_control = np.array([0.0, 0.5, 1.0])
    
    # Fit cubic spline for each dimension separately
    spline_x = CubicSpline(t_control, control_points[:, 0], bc_type='natural')
    spline_y = CubicSpline(t_control, control_points[:, 1], bc_type='natural')
    spline_z = CubicSpline(t_control, control_points[:, 2], bc_type='natural')
    
    # Generate 100 evenly spaced parameter values
    t_eval = np.linspace(0.0, 1.0, 100)
    
    # Evaluate the spline at these parameter values
    x_points = spline_x(t_eval)
    y_points = spline_y(t_eval)
    z_points = spline_z(t_eval)
    
    # Combine into final result
    points = np.column_stack([x_points, y_points, z_points])
    
    return points

def visualize_marker_and_spline(actual_trajectory, fitted_spline, start_pos, end_pos, 
                               percentage, height, show_control_points=True):
    """
    Visualize both the actual marker trajectory and fitted spline in 3D space.
    
    Parameters:
    -----------
    actual_trajectory : numpy.ndarray, shape (n_frames, 3)
        Actual marker positions from C3D file
    fitted_spline : numpy.ndarray, shape (100, 3)
        Fitted cubic spline points
    start_pos : array-like, shape (3,)
        Starting position for spline
    end_pos : array-like, shape (3,)
        Ending position for spline
    percentage : float
        Percentage along the straight line between start and end
    height : float
        Height offset in z-direction from the line point
    show_control_points : bool, default=True
        Whether to show the control points and connecting lines
        
    Returns:
    --------
    fig : matplotlib.figure.Figure
        The figure object for further customization if needed
    """
    # Calculate control points for visualization
    start_pos = np.array(start_pos)
    end_pos = np.array(end_pos)
    line_point = start_pos + percentage * (end_pos - start_pos)
    middle_point = line_point.copy()
    middle_point[2] += height
    
    # Create the 3D plot
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the actual marker trajectory as scatter points
    ax.scatter(actual_trajectory[:, 0], actual_trajectory[:, 1], actual_trajectory[:, 2], 
               c='orange', s=5, alpha=0.6, label='Actual Marker Positions', marker='o')
    
    # Plot the fitted spline curve
    ax.plot(fitted_spline[:, 0], fitted_spline[:, 1], fitted_spline[:, 2], 
            'b-', linewidth=3, label='Fitted Cubic Spline')
    
    if show_control_points:
        # Plot control points
        ax.scatter(*start_pos, color='red', s=150, label='Start Point', marker='^')
        ax.scatter(*middle_point, color='green', s=150, label='Middle Control Point', marker='s')
        ax.scatter(*end_pos, color='red', s=150, label='End Point', marker='^')
        
        # Plot the straight line between start and end
        line_points = np.array([start_pos, end_pos])
        ax.plot(line_points[:, 0], line_points[:, 1], line_points[:, 2], 
                'r--', alpha=0.5, label='Direct Line')
        
        # Plot line from line_point to middle_point (showing height offset)
        height_line = np.array([line_point, middle_point])
        ax.plot(height_line[:, 0], height_line[:, 1], height_line[:, 2], 
                'g--', alpha=0.7, label='Height Offset')
    
    # Customize the plot
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Z (meters)')
    ax.set_title('RFIN Marker Trajectory: Actual Data vs Fitted Cubic Spline\n(Frames 190-400)')
    ax.legend()
    
    # Make the plot look better
    ax.grid(True, alpha=0.3)
    
    # Set equal aspect ratio for better visualization
    max_range = np.array([actual_trajectory.max() - actual_trajectory.min(),
                         actual_trajectory.max() - actual_trajectory.min(),
                         actual_trajectory.max() - actual_trajectory.min()]).max() / 2.0
    
    mid_x = (actual_trajectory[:, 0].max() + actual_trajectory[:, 0].min()) * 0.5
    mid_y = (actual_trajectory[:, 1].max() + actual_trajectory[:, 1].min()) * 0.5
    mid_z = (actual_trajectory[:, 2].max() + actual_trajectory[:, 2].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    return fig

def analyze_marker_trajectory(c3d_file_path="../config/left_01_03.c3d", 
                            subject_name="rossana", 
                            marker_name="RFIN",
                            start_frame=190, 
                            end_frame=400,
                            percentage=0.5,
                            height=0.25):  # 5cm height offset in meters
    """
    Complete analysis of marker trajectory from C3D file.
    
    Parameters:
    -----------
    c3d_file_path : str
        Path to the C3D file
    subject_name : str
        Subject name
    marker_name : str
        Marker name
    start_frame : int
        Starting frame number
    end_frame : int
        Ending frame number
    percentage : float
        Percentage along the straight line for middle control point
    height : float
        Height offset in meters for middle control point
        
    Returns:
    --------
    actual_trajectory : numpy.ndarray
        Actual marker positions
    fitted_spline : numpy.ndarray
        Fitted spline points
    fig : matplotlib.figure.Figure
        Visualization figure
    """
    print(f"Analyzing {marker_name} marker trajectory for subject {subject_name}")
    print(f"File: {c3d_file_path}")
    print(f"Frame range: {start_frame} to {end_frame}")
    print("-" * 50)
    
    # Extract marker trajectory from C3D file
    actual_trajectory, valid_frames = extract_marker_trajectory(
        c3d_file_path, subject_name, marker_name, start_frame, end_frame)
    
    if len(actual_trajectory) < 2:
        raise ValueError("Not enough valid marker data points found")
    
    # Use first and last valid positions for spline fitting
    start_pos = actual_trajectory[0]
    end_pos = actual_trajectory[-1]
    
    print(f"Start position (frame {valid_frames[0]}): {start_pos}")
    print(f"End position (frame {valid_frames[-1]}): {end_pos}")
    print(f"Distance between start and end: {np.linalg.norm(end_pos - start_pos):.4f} meters")
    
    # Fit cubic spline using the 3-point method
    fitted_spline = fit_cubic_spline_3points(start_pos, end_pos, percentage, height)
    
    # Visualize both trajectories
    fig = visualize_marker_and_spline(actual_trajectory, fitted_spline, 
                                    start_pos, end_pos, percentage, height)
    
    return actual_trajectory, fitted_spline, fig

# Main execution
if __name__ == "__main__":
    try:
        # Analyze the RFIN marker trajectory
        actual_traj, fitted_spline, fig = analyze_marker_trajectory(percentage=0.35, height = 0.23)
        
        # Show the plot
        plt.show()
        
        # Print some statistics
        print("\n" + "="*50)
        print("ANALYSIS SUMMARY")
        print("="*50)
        print(f"Actual trajectory points: {len(actual_traj)}")
        print(f"Fitted spline points: {len(fitted_spline)}")
        print(f"Actual trajectory range:")
        print(f"  X: {actual_traj[:, 0].min():.4f} to {actual_traj[:, 0].max():.4f} meters")
        print(f"  Y: {actual_traj[:, 1].min():.4f} to {actual_traj[:, 1].max():.4f} meters")
        print(f"  Z: {actual_traj[:, 2].min():.4f} to {actual_traj[:, 2].max():.4f} meters")
        
    except Exception as e:
        print(f"Error during analysis: {e}")
        print("Please check that:")
        print("1. The C3D file exists at '../config/left_01_03.c3d'")
        print("2. The subject name 'rossana' is correct")
        print("3. The marker name 'RFIN' exists in the file")
        print("4. Frames 190-400 contain valid data")