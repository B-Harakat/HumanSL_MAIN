#!/usr/bin/env python3
"""
Robot Trajectory Visualizer
Reads YAML data exported from C++ and visualizes robot arm trajectory with obstacles
"""

import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
import roboticstoolbox as rtb
from spatialmath import SE3
import argparse
import glob
import os


class TrajectoryVisualizer:
    def __init__(self, yaml_file, dh_file):
        """Initialize visualizer with trajectory and DH parameter data"""
        self.load_trajectory_data(yaml_file)
        self.load_dh_parameters(dh_file)
        self.setup_robot_model()
        self.generate_sphere_definitions()
        self.current_frame = 0
        
        # For end-effector trace
        self.ee_trace_points = []
        
    def load_trajectory_data(self, yaml_file):
        """Load trajectory data from YAML file"""
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
            
        self.metadata = data['metadata']
        self.robot_base = data['robot_base']
        self.occupancy_grid = data['occupancy_grid']
        self.trajectory = data['trajectory']
        
        print(f"Loaded trajectory with {len(self.trajectory)} frames")
        print(f"Total time: {self.metadata['total_time_sec']:.2f} seconds")
        print(f"Occupied cells: {self.occupancy_grid['occupied_cells']}")
        
    def load_dh_parameters(self, dh_file):
        """Load DH parameters from YAML file"""
        with open(dh_file, 'r') as f:
            data = yaml.safe_load(f)
            
        self.dh_params = data['dh_parameters']
        print(f"Loaded DH parameters for {len(self.dh_params)} joints")
        
    def setup_robot_model(self):
        """Create robot model using DH parameters"""
        # Extract DH parameters
        a = [joint['a'] for joint in self.dh_params]
        alpha = [joint['alpha'] for joint in self.dh_params]
        d = [joint['d'] for joint in self.dh_params]
        theta_offset = [joint['theta_offset'] for joint in self.dh_params]
        
        # Create robot model
        self.robot = rtb.DHRobot([
            rtb.RevoluteMDH(a=a[i], alpha=alpha[i], d=d[i], offset=theta_offset[i])
            for i in range(7)
        ], name="Kinova_Gen3")
        
        # Create base transform from robot_base data
        base_pos = np.array(self.robot_base['position'])
        base_rot = np.array(self.robot_base['rotation_matrix'])
        
        # Debug: print shapes and values
        print(f"Base position shape: {base_pos.shape}, values: {base_pos}")
        print(f"Base rotation shape: {base_rot.shape}")
        print(f"Base rotation matrix:\n{base_rot}")
        
        # Ensure rotation matrix is properly shaped and valid
        if base_rot.shape != (3, 3):
            raise ValueError(f"Expected 3x3 rotation matrix, got shape {base_rot.shape}")
            
        # Check if it's a valid rotation matrix (determinant should be 1)
        det = np.linalg.det(base_rot)
        print(f"Rotation matrix determinant: {det}")
        
        # Create SE3 transform
        try:
            self.base_transform = SE3.Rt(base_rot, base_pos)
        except Exception as e:
            print(f"Failed to create SE3 transform: {e}")
            print("Trying alternative approach...")
            # Try creating identity transform if base transform fails
            self.base_transform = SE3.Tx(base_pos[0]) * SE3.Ty(base_pos[1]) * SE3.Tz(base_pos[2])
            print("Using position-only transform")
        
    def generate_sphere_definitions(self):
        """Generate sphere definitions matching the C++ generateArmSpheres function"""
        base_radius = 0.05
        arm_id_offset = 0
        
        self.sphere_defs = []
        
        # Joint 0: Base to Joint 1 (d[0] = 0.2848, ~28.5cm link)
        self.sphere_defs.append((arm_id_offset + 0, base_radius, np.array([0.0, 0.0, 0.0])))
        self.sphere_defs.append((arm_id_offset + 0, base_radius * 0.7, np.array([0.0, 0.07, 0.0])))
        self.sphere_defs.append((arm_id_offset + 0, base_radius * 0.6, np.array([0.0, 0.12, 0.0])))
        self.sphere_defs.append((arm_id_offset + 0, base_radius * 0.5, np.array([0.0, 0.17, 0.0])))
        
        # Joint 1: Short connector (d[1] = -0.0118, ~1.2cm) - SKIP
        
        # Joint 2: Upper arm long link (d[2] = -0.4208, ~42.1cm link)
        self.sphere_defs.append((arm_id_offset + 2, base_radius * 1.3, np.array([0.0, 0.0, 0.0])))
        self.sphere_defs.append((arm_id_offset + 2, base_radius, np.array([0.0, 0.05, 0.0])))
        self.sphere_defs.append((arm_id_offset + 2, base_radius, np.array([0.0, 0.10, 0.0])))
        self.sphere_defs.append((arm_id_offset + 2, base_radius, np.array([0.0, 0.18, 0.0])))
        self.sphere_defs.append((arm_id_offset + 2, base_radius, np.array([0.0, 0.26, 0.0])))
        self.sphere_defs.append((arm_id_offset + 2, base_radius, np.array([0.0, 0.34, 0.0])))
        self.sphere_defs.append((arm_id_offset + 2, base_radius, np.array([0.0, 0.42, 0.0])))
        
        # Joint 3: Short connector (d[3] = -0.0128, ~1.3cm) - SKIP
        
        # Joint 4: Forearm long link (d[4] = -0.3143, ~31.4cm link)
        self.sphere_defs.append((arm_id_offset + 4, base_radius * 1.3, np.array([0.0, 0.0, 0.0])))
        self.sphere_defs.append((arm_id_offset + 4, base_radius, np.array([0.0, 0.08, 0.0])))
        self.sphere_defs.append((arm_id_offset + 4, base_radius, np.array([0.0, 0.16, 0.0])))
        self.sphere_defs.append((arm_id_offset + 4, base_radius, np.array([0.0, 0.24, 0.0])))
        self.sphere_defs.append((arm_id_offset + 4, base_radius, np.array([0.0, 0.31, 0.0])))
        
        # Joint 5: No link (d[5] = 0.0) - SKIP
        
        # Joint 6: Wrist to interface (d[6] = -0.1674, ~16.7cm link)
        self.sphere_defs.append((arm_id_offset + 6, base_radius, np.array([0.0, 0.0, -0.14])))
        self.sphere_defs.append((arm_id_offset + 6, base_radius, np.array([0.0, 0.0, -0.20])))
        self.sphere_defs.append((arm_id_offset + 6, base_radius, np.array([0.0, 0.0, -0.25])))
        self.sphere_defs.append((arm_id_offset + 6, base_radius, np.array([0.0, 0.0, -0.30])))
        
        # Gripper/End-effector spheres
        self.sphere_defs.extend([
            (arm_id_offset + 6, base_radius*0.4, np.array([0.07, 0.0, 0.04])),
            (arm_id_offset + 6, base_radius*0.4, np.array([-0.07, 0.0, 0.04])),
            (arm_id_offset + 6, base_radius*0.4, np.array([0.07, 0.0, 0.0])),
            (arm_id_offset + 6, base_radius*0.4, np.array([-0.07, 0.0, 0.0])),
            (arm_id_offset + 6, base_radius*0.4, np.array([0.07, 0.0, -0.04])),
            (arm_id_offset + 6, base_radius*0.4, np.array([-0.07, 0.0, -0.04])),
            (arm_id_offset + 6, base_radius*0.4, np.array([0.07, 0.0, -0.08])),
            (arm_id_offset + 6, base_radius*0.4, np.array([-0.07, 0.0, -0.08])),
            (arm_id_offset + 6, base_radius*0.4, np.array([0.07, 0.0, -0.12])),
            (arm_id_offset + 6, base_radius*0.4, np.array([-0.07, 0.0, -0.12])),
            (arm_id_offset + 6, base_radius*0.4, np.array([0.07, 0.0, -0.14])),
            (arm_id_offset + 6, base_radius*0.4, np.array([-0.07, 0.0, -0.14]))
        ])
        
        print(f"Generated {len(self.sphere_defs)} sphere definitions")
        
    def compute_sphere_positions(self, joint_config):
        """Compute world positions of all spheres for given joint configuration"""
        # Get forward kinematics for all joints
        fk_transforms = self.robot.fkine_all(joint_config)
        
        # Apply base transform
        world_transforms = [self.base_transform * T for T in fk_transforms]
        
        sphere_positions = []
        sphere_radii = []
        
        for joint_id, radius, local_pos in self.sphere_defs:
            # Transform sphere local position to world coordinates
            world_pos = world_transforms[joint_id] * np.append(local_pos, 1)[:3]
            sphere_positions.append(world_pos)
            sphere_radii.append(radius)
            
        return np.array(sphere_positions), np.array(sphere_radii)
        
    def create_obstacle_mesh(self):
        """Create obstacle cubes from occupancy grid"""
        obstacles = []
        grid = self.occupancy_grid
        
        for i in range(grid['dimensions'][0]):  # rows
            for j in range(grid['dimensions'][1]):  # cols
                for k in range(grid['dimensions'][2]):  # z
                    if grid['data'][i][j][k] == 1:  # occupied
                        # Convert grid indices to world coordinates
                        world_x = grid['origin'][0] + (i + 0.5) * grid['cell_size']
                        world_y = grid['origin'][1] + (j + 0.5) * grid['cell_size']
                        world_z = grid['origin'][2] + (k + 0.5) * grid['cell_size']
                        obstacles.append([world_x, world_y, world_z])
                        
        return np.array(obstacles) if obstacles else np.empty((0, 3))
        
    def draw_cube(self, ax, center, size):
        """Draw a cube at given center with given size"""
        # Define cube vertices relative to center
        r = size / 2
        vertices = np.array([
            [-r, -r, -r], [r, -r, -r], [r, r, -r], [-r, r, -r],  # bottom face
            [-r, -r, r], [r, -r, r], [r, r, r], [-r, r, r]        # top face
        ]) + center
        
        # Define cube faces (indices into vertices)
        faces = [
            [0, 1, 2, 3],  # bottom
            [4, 5, 6, 7],  # top
            [0, 1, 5, 4],  # front
            [2, 3, 7, 6],  # back
            [1, 2, 6, 5],  # right
            [0, 3, 7, 4]   # left
        ]
        
        # Draw each face
        for face in faces:
            face_vertices = vertices[face]
            # Close the face by adding first vertex at end
            face_vertices = np.vstack([face_vertices, face_vertices[0]])
            ax.plot(face_vertices[:, 0], face_vertices[:, 1], face_vertices[:, 2], 
                   'r-', alpha=0.6, linewidth=0.5)
                   
    def draw_sphere(self, ax, center, radius, color='b', alpha=0.7):
        """Draw a sphere at given center with given radius"""
        u = np.linspace(0, 2 * np.pi, 10)
        v = np.linspace(0, np.pi, 10)
        x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
        y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
        z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
        ax.plot_surface(x, y, z, color=color, alpha=alpha)
        
    def update_frame(self, frame_idx):
        """Update visualization for given frame index"""
        self.current_frame = int(frame_idx)
        
        # Clear previous arm visualization
        self.ax.clear()
        
        # Set up the plot
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title(f'Robot Trajectory - Frame {self.current_frame + 1}/{len(self.trajectory)}')
        
        # Draw obstacles
        for obstacle in self.obstacles:
            self.draw_cube(self.ax, obstacle, self.occupancy_grid['cell_size'])
            
        # Get joint configuration for current frame
        joint_config = np.array(self.trajectory[self.current_frame]['joint_config'])
        
        # Compute sphere positions
        sphere_positions, sphere_radii = self.compute_sphere_positions(joint_config)
        
        # Draw arm spheres
        for pos, radius in zip(sphere_positions, sphere_radii):
            self.draw_sphere(self.ax, pos, radius, color='blue', alpha=0.7)
            
        # Update end-effector trace (last sphere is end-effector)
        if len(sphere_positions) > 0:
            ee_pos = sphere_positions[-1]  # Last sphere is end-effector
            
            # Add current end-effector position to trace
            if self.current_frame < len(self.ee_trace_points):
                # Truncate trace if scrubbing backwards
                self.ee_trace_points = self.ee_trace_points[:self.current_frame + 1]
            else:
                # Extend trace up to current frame
                for i in range(len(self.ee_trace_points), self.current_frame + 1):
                    if i < len(self.trajectory):
                        config = np.array(self.trajectory[i]['joint_config'])
                        positions, _ = self.compute_sphere_positions(config)
                        if len(positions) > 0:
                            self.ee_trace_points.append(positions[-1])
                            
            # Draw end-effector trace
            if len(self.ee_trace_points) > 1:
                trace_points = np.array(self.ee_trace_points)
                self.ax.plot(trace_points[:, 0], trace_points[:, 1], trace_points[:, 2], 
                           'g-', linewidth=2, alpha=0.8, label='EE Trace')
                           
        # Set equal aspect ratio and reasonable limits
        self.set_plot_limits()
        
        # Redraw
        self.fig.canvas.draw()
        
    def set_plot_limits(self):
        """Set reasonable plot limits based on robot workspace and obstacles"""
        # Get bounds from obstacles and robot base
        if len(self.obstacles) > 0:
            obs_min = np.min(self.obstacles, axis=0) - self.occupancy_grid['cell_size']
            obs_max = np.max(self.obstacles, axis=0) + self.occupancy_grid['cell_size']
        else:
            obs_min = np.array([-1, -1, -1])
            obs_max = np.array([1, 1, 1])
            
        base_pos = np.array(self.robot_base['position'])
        
        # Expand bounds to include robot workspace (rough estimate)
        workspace_radius = 1.0  # ~1m reach for typical 7-DOF arm
        bounds_min = np.minimum(obs_min, base_pos - workspace_radius)
        bounds_max = np.maximum(obs_max, base_pos + workspace_radius)
        
        # Set limits with some padding
        padding = 0.2
        self.ax.set_xlim(bounds_min[0] - padding, bounds_max[0] + padding)
        self.ax.set_ylim(bounds_min[1] - padding, bounds_max[1] + padding)
        self.ax.set_zlim(bounds_min[2] - padding, bounds_max[2] + padding)
        
    def slider_callback(self, val):
        """Callback for frame slider"""
        self.update_frame(val)
        
    def visualize(self):
        """Create and show the interactive visualization"""
        # Create obstacles
        self.obstacles = self.create_obstacle_mesh()
        
        # Initialize end-effector trace
        self.ee_trace_points = []
        
        # Create figure and 3D axis
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Create slider
        slider_ax = plt.axes([0.2, 0.02, 0.6, 0.03])
        self.frame_slider = Slider(
            slider_ax, 'Frame', 0, len(self.trajectory) - 1, 
            valinit=0, valfmt='%d', valstep=1
        )
        self.frame_slider.on_changed(self.slider_callback)
        
        # Initial visualization
        self.update_frame(0)
        
        # Add instructions
        self.fig.suptitle('Robot Trajectory Visualizer\nUse mouse to rotate/zoom, slider to change frame', 
                         fontsize=12)
        
        plt.tight_layout()
        plt.show()


def main():
    """Main function with command line argument parsing"""
    parser = argparse.ArgumentParser(description='Visualize robot trajectory from YAML data')
    parser.add_argument('--trajectory', '-t', type=str, 
                       help='Path to trajectory YAML file')
    parser.add_argument('--dh', '-d', type=str, default='../config/dh_params.yaml',
                       help='Path to DH parameters YAML file')
    
    args = parser.parse_args()
    
    # If no trajectory file specified, try to find the latest one
    if args.trajectory is None:
        yaml_files = glob.glob('../logs/trajectory_animation_*.yaml')
        if not yaml_files:
            print("No trajectory YAML files found in ../logs/")
            print("Please specify a trajectory file with --trajectory")
            return
        
        # Get the most recent file
        args.trajectory = max(yaml_files, key=os.path.getctime)
        print(f"Using latest trajectory file: {args.trajectory}")
    
    # Check if files exist
    if not os.path.exists(args.trajectory):
        print(f"Trajectory file not found: {args.trajectory}")
        return
        
    if not os.path.exists(args.dh):
        print(f"DH parameters file not found: {args.dh}")
        return
    
    # Create and run visualizer
    try:
        visualizer = TrajectoryVisualizer(args.trajectory, args.dh)
        visualizer.visualize()
    except Exception as e:
        print(f"Error creating visualization: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()