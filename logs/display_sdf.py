#!/usr/bin/env python3
"""
SDF Visualization Tool
Reads sdf_visualization.yaml and displays interactive 2D slices with Z-level slider
"""

import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import matplotlib.patches as patches

class SDFVisualizer:
    def __init__(self, yaml_file='sdf_visualization.yaml'):
        """Load SDF data from YAML file"""
        print(f"Loading SDF data from {yaml_file}...")
        
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        
        self.sdf_data = data['sdf']
        self.origin = np.array(self.sdf_data['origin'])
        self.cell_size = self.sdf_data['cell_size']
        self.dimensions = self.sdf_data['dimensions']  # [cols, rows, z_levels]
        
        self.cols = self.dimensions[0]  # X dimension
        self.rows = self.dimensions[1]  # Y dimension
        self.z_levels = self.dimensions[2]  # Z dimension
        
        # Convert YAML data to numpy arrays
        self.sdf_volume = np.zeros((self.z_levels, self.rows, self.cols))
        
        for z in range(self.z_levels):
            z_key = f'z_level_{z}'
            if z_key in self.sdf_data['data']:
                self.sdf_volume[z] = np.array(self.sdf_data['data'][z_key])
        
        # Calculate world coordinate extents
        self.x_min = self.origin[0]
        self.x_max = self.origin[0] + (self.cols - 1) * self.cell_size
        self.y_min = self.origin[1]
        self.y_max = self.origin[1] + (self.rows - 1) * self.cell_size
        self.z_min = self.origin[2]
        self.z_max = self.origin[2] + (self.z_levels - 1) * self.cell_size
        
        # Calculate global min/max for fixed colormap
        self.vmin = np.min(self.sdf_volume)
        self.vmax = np.max(self.sdf_volume)
        
        print(f"SDF loaded successfully:")
        print(f"  Dimensions: {self.cols} x {self.rows} x {self.z_levels}")
        print(f"  Cell size: {self.cell_size:.4f}")
        print(f"  World bounds: X[{self.x_min:.2f}, {self.x_max:.2f}], "
              f"Y[{self.y_min:.2f}, {self.y_max:.2f}], Z[{self.z_min:.2f}, {self.z_max:.2f}]")
        print(f"  Value range: [{self.vmin:.4f}, {self.vmax:.4f}]")
    
    def visualize(self):
        """Create interactive visualization with Z-level slider"""
        
        # Create figure and subplots
        fig, ax = plt.subplots(figsize=(12, 10))
        plt.subplots_adjust(bottom=0.15)
        
        # Initial Z level
        initial_z = self.z_levels // 2
        
        # Create initial plot
        self.im = ax.imshow(
            self.sdf_volume[initial_z], 
            extent=[self.x_min, self.x_max, self.y_max, self.y_min],  # Note: y is flipped for image display
            cmap='RdBu', 
            vmin=self.vmin, 
            vmax=self.vmax,
            aspect='equal',
            interpolation='nearest'
        )
        
        # Add colorbar
        cbar = plt.colorbar(self.im, ax=ax, shrink=0.8)
        cbar.set_label('Signed Distance', fontsize=12)
        
        # Set labels and title
        ax.set_xlabel('X (meters)', fontsize=12)
        ax.set_ylabel('Y (meters)', fontsize=12)
        ax.grid(True, alpha=0.3)
        
        # Update title
        z_world = self.z_min + initial_z * self.cell_size
        ax.set_title(f'SDF Slice - Z Level {initial_z}/{self.z_levels-1} (Z = {z_world:.3f}m)', fontsize=14)
        
        # Create slider
        ax_slider = plt.axes([0.15, 0.05, 0.65, 0.03])
        self.z_slider = Slider(
            ax_slider, 'Z Level', 0, self.z_levels-1, 
            valinit=initial_z, valfmt='%d'
        )
        
        # Store references for callbacks
        self.ax = ax
        self.fig = fig
        
        # Connect slider callback
        self.z_slider.on_changed(self.update_slice)
        
        # Connect mouse motion for hover
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_hover)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        # Add text box for displaying values
        self.hover_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                                 verticalalignment='top', fontsize=10,
                                 bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7))
        
        plt.show()
    
    def update_slice(self, val):
        """Update the displayed slice when slider changes"""
        z_idx = int(self.z_slider.val)
        
        # Update image data
        self.im.set_array(self.sdf_volume[z_idx])
        
        # Update title
        z_world = self.z_min + z_idx * self.cell_size
        self.ax.set_title(f'SDF Slice - Z Level {z_idx}/{self.z_levels-1} (Z = {z_world:.3f}m)', fontsize=14)
        
        # Redraw
        self.fig.canvas.draw()
    
    def on_hover(self, event):
        """Handle mouse hover to display coordinates and values"""
        if event.inaxes == self.ax:
            x_world = event.xdata
            y_world = event.ydata
            
            if x_world is not None and y_world is not None:
                # Convert world coordinates to grid indices
                col = int((x_world - self.x_min) / self.cell_size)
                row = int((y_world - self.y_min) / self.cell_size)
                z_idx = int(self.z_slider.val)
                z_world = self.z_min + z_idx * self.cell_size
                
                # Check bounds
                if 0 <= col < self.cols and 0 <= row < self.rows:
                    sdf_value = self.sdf_volume[z_idx, row, col]
                    
                    self.hover_text.set_text(
                        f'World: ({x_world:.3f}, {y_world:.3f}, {z_world:.3f})\n'
                        f'Grid: ({col}, {row}, {z_idx})\n'
                        f'SDF: {sdf_value:.4f}'
                    )
                else:
                    self.hover_text.set_text('')
                
                self.fig.canvas.draw_idle()
    
    def on_click(self, event):
        """Handle mouse click to print detailed information"""
        if event.inaxes == self.ax and event.button == 1:  # Left click
            x_world = event.xdata
            y_world = event.ydata
            
            if x_world is not None and y_world is not None:
                # Convert world coordinates to grid indices
                col = int((x_world - self.x_min) / self.cell_size)
                row = int((y_world - self.y_min) / self.cell_size)
                z_idx = int(self.z_slider.val)
                z_world = self.z_min + z_idx * self.cell_size
                
                # Check bounds and print info
                if 0 <= col < self.cols and 0 <= row < self.rows:
                    sdf_value = self.sdf_volume[z_idx, row, col]
                    
                    print(f"\n=== Clicked Point Information ===")
                    print(f"World coordinates: ({x_world:.4f}, {y_world:.4f}, {z_world:.4f})")
                    print(f"Grid indices: ({col}, {row}, {z_idx})")
                    print(f"SDF value: {sdf_value:.6f}")
                    
                    if sdf_value < 0:
                        print(f"Status: Inside obstacle (distance to surface: {abs(sdf_value):.4f}m)")
                    elif sdf_value > 0:
                        print(f"Status: Free space (distance to nearest obstacle: {sdf_value:.4f}m)")
                    else:
                        print(f"Status: On obstacle boundary")

def main():
    """Main function to run the SDF visualizer"""
    try:
        visualizer = SDFVisualizer()
        visualizer.visualize()
    except FileNotFoundError:
        print("Error: sdf_visualization.yaml not found!")
        print("Make sure to run the C++ export function first.")
    except Exception as e:
        print(f"Error loading or visualizing SDF: {e}")

if __name__ == "__main__":
    main()