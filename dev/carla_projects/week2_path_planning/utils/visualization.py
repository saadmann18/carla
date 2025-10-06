"""
Visualization utilities for path planning and debug drawing
"""

import carla
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
import os
import time
from matplotlib.patches import Circle, Rectangle
from matplotlib.collections import LineCollection
import seaborn as sns

class DebugDrawer:
    """Handles debug drawing in Carla simulator"""
    
    def __init__(self, world: carla.World, default_life_time: float = 60.0):
        self.world = world
        self.debug = world.debug
        self.default_life_time = default_life_time
        
    def draw_point(self, location: carla.Location, size: float = 0.1, 
                   color: carla.Color = carla.Color(255, 0, 0), 
                   life_time: Optional[float] = None):
        """Draw a point in the world"""
        if life_time is None:
            life_time = self.default_life_time
            
        self.debug.draw_point(location, size=size, color=color, life_time=life_time)
    
    def draw_line(self, start: carla.Location, end: carla.Location,
                  thickness: float = 0.1, color: carla.Color = carla.Color(255, 0, 0),
                  life_time: Optional[float] = None):
        """Draw a line between two points"""
        if life_time is None:
            life_time = self.default_life_time
            
        self.debug.draw_line(start, end, thickness=thickness, color=color, life_time=life_time)
    
    def draw_arrow(self, start: carla.Location, end: carla.Location,
                   thickness: float = 0.1, arrow_size: float = 0.3,
                   color: carla.Color = carla.Color(0, 255, 0),
                   life_time: Optional[float] = None):
        """Draw an arrow from start to end"""
        if life_time is None:
            life_time = self.default_life_time
            
        self.debug.draw_arrow(start, end, thickness=thickness, arrow_size=arrow_size,
                            color=color, life_time=life_time)
    
    def draw_box(self, box: carla.BoundingBox, rotation: carla.Rotation,
                 thickness: float = 0.1, color: carla.Color = carla.Color(255, 0, 0),
                 life_time: Optional[float] = None):
        """Draw a bounding box"""
        if life_time is None:
            life_time = self.default_life_time
            
        self.debug.draw_box(box, rotation, thickness=thickness, color=color, life_time=life_time)
    
    def draw_string(self, location: carla.Location, text: str,
                    draw_shadow: bool = False, color: carla.Color = carla.Color(255, 255, 255),
                    life_time: Optional[float] = None):
        """Draw text at a location"""
        if life_time is None:
            life_time = self.default_life_time
            
        self.debug.draw_string(location, text, draw_shadow=draw_shadow,
                             color=color, life_time=life_time)
    
    def draw_path(self, waypoints: List[Tuple[float, float, float]],
                  color: carla.Color = carla.Color(0, 255, 0),
                  thickness: float = 0.1, life_time: Optional[float] = None):
        """Draw a path as connected line segments"""
        if life_time is None:
            life_time = self.default_life_time
            
        for i in range(len(waypoints) - 1):
            start = carla.Location(waypoints[i][0], waypoints[i][1], waypoints[i][2])
            end = carla.Location(waypoints[i+1][0], waypoints[i+1][1], waypoints[i+1][2])
            self.draw_line(start, end, thickness=thickness, color=color, life_time=life_time)
    
    def draw_waypoints(self, waypoints: List[Tuple[float, float, float]],
                      size: float = 0.2, color: carla.Color = carla.Color(255, 255, 0),
                      life_time: Optional[float] = None):
        """Draw waypoints as points"""
        if life_time is None:
            life_time = self.default_life_time
            
        for waypoint in waypoints:
            location = carla.Location(waypoint[0], waypoint[1], waypoint[2])
            self.draw_point(location, size=size, color=color, life_time=life_time)
    
    def draw_vehicle_trajectory(self, vehicle: carla.Vehicle, history_length: int = 50,
                               color: carla.Color = carla.Color(255, 0, 255),
                               life_time: Optional[float] = None):
        """Draw vehicle trajectory (requires position history)"""
        # This would need to be implemented with position tracking
        pass
    
    def clear_all(self):
        """Clear all debug drawings (not directly supported by Carla)"""
        # Carla doesn't have a direct clear method, drawings expire based on life_time
        pass

class PathVisualizer:
    """Creates matplotlib visualizations for path planning results"""
    
    def __init__(self, figsize: Tuple[int, int] = (12, 8), style: str = 'seaborn-v0_8'):
        self.figsize = figsize
        try:
            plt.style.use(style)
        except:
            plt.style.use('default')
        
        # Color palette
        self.colors = {
            'path': '#2E8B57',      # Sea Green
            'start': '#32CD32',     # Lime Green
            'goal': '#DC143C',      # Crimson
            'obstacle': '#FF6347',  # Tomato
            'waypoint': '#4169E1',  # Royal Blue
            'tree': '#708090',      # Slate Gray
            'vehicle': '#FFD700'    # Gold
        }
    
    def plot_path_planning_result(self, path: List[Tuple[float, float]],
                                 start: Tuple[float, float], goal: Tuple[float, float],
                                 obstacles: List[Tuple[float, float, float]] = None,
                                 bounds: Tuple[float, float, float, float] = None,
                                 title: str = "Path Planning Result",
                                 save_path: Optional[str] = None) -> plt.Figure:
        """Plot basic path planning result"""
        
        fig, ax = plt.subplots(figsize=self.figsize)
        
        # Draw obstacles
        if obstacles:
            for obs_x, obs_y, radius in obstacles:
                circle = Circle((obs_x, obs_y), radius, color=self.colors['obstacle'], 
                              alpha=0.7, label='Obstacles' if obs_x == obstacles[0][0] else "")
                ax.add_patch(circle)
        
        # Draw path
        if path and len(path) > 1:
            path_x, path_y = zip(*path)
            ax.plot(path_x, path_y, color=self.colors['path'], linewidth=3, 
                   marker='o', markersize=2, label='Path')
        
        # Draw start and goal
        ax.scatter(start[0], start[1], color=self.colors['start'], s=200, 
                  marker='o', edgecolors='black', linewidth=2, label='Start', zorder=5)
        ax.scatter(goal[0], goal[1], color=self.colors['goal'], s=200, 
                  marker='s', edgecolors='black', linewidth=2, label='Goal', zorder=5)
        
        # Set bounds
        if bounds:
            ax.set_xlim(bounds[0], bounds[1])
            ax.set_ylim(bounds[2], bounds[3])
        
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_rrt_tree(self, nodes: List[Any], path: List[Tuple[float, float]],
                     start: Tuple[float, float], goal: Tuple[float, float],
                     obstacles: List[Tuple[float, float, float]] = None,
                     bounds: Tuple[float, float, float, float] = None,
                     title: str = "RRT Tree Exploration",
                     save_path: Optional[str] = None) -> plt.Figure:
        """Plot RRT tree with exploration"""
        
        fig, ax = plt.subplots(figsize=self.figsize)
        
        # Draw tree edges
        for node in nodes:
            if hasattr(node, 'parent') and node.parent is not None:
                ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 
                       color=self.colors['tree'], alpha=0.3, linewidth=0.5)
        
        # Draw tree nodes
        if nodes:
            node_x = [node.x for node in nodes]
            node_y = [node.y for node in nodes]
            ax.scatter(node_x, node_y, color=self.colors['tree'], s=1, alpha=0.6)
        
        # Draw obstacles
        if obstacles:
            for obs_x, obs_y, radius in obstacles:
                circle = Circle((obs_x, obs_y), radius, color=self.colors['obstacle'], 
                              alpha=0.7, label='Obstacles' if obs_x == obstacles[0][0] else "")
                ax.add_patch(circle)
        
        # Draw path
        if path and len(path) > 1:
            path_x, path_y = zip(*path)
            ax.plot(path_x, path_y, color=self.colors['path'], linewidth=3, 
                   label='Final Path', zorder=4)
        
        # Draw start and goal
        ax.scatter(start[0], start[1], color=self.colors['start'], s=200, 
                  marker='o', edgecolors='black', linewidth=2, label='Start', zorder=5)
        ax.scatter(goal[0], goal[1], color=self.colors['goal'], s=200, 
                  marker='s', edgecolors='black', linewidth=2, label='Goal', zorder=5)
        
        # Set bounds
        if bounds:
            ax.set_xlim(bounds[0], bounds[1])
            ax.set_ylim(bounds[2], bounds[3])
        
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_comparison_metrics(self, results: Dict[str, List[float]], 
                               metric_names: List[str],
                               title: str = "Algorithm Comparison",
                               save_path: Optional[str] = None) -> plt.Figure:
        """Plot comparison metrics between algorithms"""
        
        num_metrics = len(metric_names)
        fig, axes = plt.subplots(1, num_metrics, figsize=(4*num_metrics, 6))
        
        if num_metrics == 1:
            axes = [axes]
        
        algorithms = list(results.keys())
        
        for i, metric in enumerate(metric_names):
            ax = axes[i]
            
            # Prepare data
            data = []
            labels = []
            for algorithm in algorithms:
                if metric in results[algorithm]:
                    data.append(results[algorithm][metric])
                    labels.append(algorithm)
            
            # Create box plot
            if data:
                box_plot = ax.boxplot(data, labels=labels, patch_artist=True)
                
                # Color boxes
                colors = plt.cm.Set3(np.linspace(0, 1, len(data)))
                for patch, color in zip(box_plot['boxes'], colors):
                    patch.set_facecolor(color)
            
            ax.set_title(metric.replace('_', ' ').title())
            ax.grid(True, alpha=0.3)
        
        plt.suptitle(title)
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_vehicle_trajectory(self, trajectory: List[Dict], 
                               path: List[Tuple[float, float]] = None,
                               title: str = "Vehicle Trajectory",
                               save_path: Optional[str] = None) -> plt.Figure:
        """Plot vehicle trajectory with control data"""
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        # Extract data
        timestamps = [entry['timestamp'] for entry in trajectory]
        x_positions = [entry['vehicle_x'] for entry in trajectory]
        y_positions = [entry['vehicle_y'] for entry in trajectory]
        speeds = [entry['current_speed'] for entry in trajectory]
        steering = [entry['steer_command'] for entry in trajectory]
        throttle = [entry['throttle_command'] for entry in trajectory]
        brake = [entry['brake_command'] for entry in trajectory]
        
        # Plot 1: Trajectory
        ax1.plot(x_positions, y_positions, color=self.colors['vehicle'], linewidth=2, label='Vehicle Path')
        if path:
            path_x, path_y = zip(*path)
            ax1.plot(path_x, path_y, color=self.colors['path'], linewidth=2, 
                    linestyle='--', alpha=0.7, label='Reference Path')
        ax1.set_xlabel('X (meters)')
        ax1.set_ylabel('Y (meters)')
        ax1.set_title('Vehicle Trajectory')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')
        
        # Plot 2: Speed
        ax2.plot(timestamps, speeds, color='blue', linewidth=2)
        if trajectory and 'target_speed' in trajectory[0]:
            target_speeds = [entry['target_speed'] for entry in trajectory]
            ax2.plot(timestamps, target_speeds, color='red', linewidth=2, 
                    linestyle='--', label='Target Speed')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (m/s)')
        ax2.set_title('Speed Profile')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        # Plot 3: Steering
        ax3.plot(timestamps, steering, color='green', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Steering Command')
        ax3.set_title('Steering Control')
        ax3.grid(True, alpha=0.3)
        ax3.set_ylim(-1.1, 1.1)
        
        # Plot 4: Throttle/Brake
        ax4.plot(timestamps, throttle, color='orange', linewidth=2, label='Throttle')
        ax4.plot(timestamps, brake, color='red', linewidth=2, label='Brake')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Command')
        ax4.set_title('Throttle/Brake Control')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.set_ylim(-0.1, 1.1)
        
        plt.suptitle(title)
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_performance_heatmap(self, data: Dict[str, Dict[str, float]],
                                title: str = "Performance Heatmap",
                                save_path: Optional[str] = None) -> plt.Figure:
        """Plot performance heatmap for algorithm comparison"""
        
        # Convert to DataFrame for easier plotting
        import pandas as pd
        df = pd.DataFrame(data).T
        
        fig, ax = plt.subplots(figsize=self.figsize)
        
        # Create heatmap
        sns.heatmap(df, annot=True, fmt='.3f', cmap='RdYlGn_r', 
                   center=df.values.mean(), ax=ax)
        
        ax.set_title(title)
        ax.set_xlabel('Metrics')
        ax.set_ylabel('Algorithms')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def create_animation_frames(self, planning_steps: List[Dict],
                               save_dir: str = "animation_frames") -> List[str]:
        """Create animation frames for planning process"""
        os.makedirs(save_dir, exist_ok=True)
        frame_files = []
        
        for i, step in enumerate(planning_steps):
            fig = self.plot_path_planning_result(
                path=step.get('current_path', []),
                start=step['start'],
                goal=step['goal'],
                obstacles=step.get('obstacles', []),
                bounds=step.get('bounds'),
                title=f"Planning Step {i+1}"
            )
            
            frame_file = os.path.join(save_dir, f"frame_{i:04d}.png")
            plt.savefig(frame_file, dpi=150, bbox_inches='tight')
            plt.close(fig)
            frame_files.append(frame_file)
        
        return frame_files
    
    def save_all_plots(self, save_dir: str):
        """Save all currently open plots"""
        os.makedirs(save_dir, exist_ok=True)
        
        for i, fig_num in enumerate(plt.get_fignums()):
            fig = plt.figure(fig_num)
            fig.savefig(os.path.join(save_dir, f"plot_{i+1}.png"), 
                       dpi=300, bbox_inches='tight')

def create_sample_visualization():
    """Create sample visualizations for testing"""
    visualizer = PathVisualizer()
    
    # Sample data
    path = [(10, 10), (20, 15), (30, 25), (40, 30), (50, 40)]
    start = (10, 10)
    goal = (50, 40)
    obstacles = [(25, 20, 5), (35, 35, 4)]
    bounds = (0, 60, 0, 50)
    
    # Create visualization
    fig = visualizer.plot_path_planning_result(
        path=path,
        start=start,
        goal=goal,
        obstacles=obstacles,
        bounds=bounds,
        title="Sample Path Planning Visualization"
    )
    
    plt.show()
    return fig

def main():
    """Test visualization functions"""
    print("Testing visualization utilities...")
    create_sample_visualization()
    print("Visualization test completed!")

if __name__ == "__main__":
    main()
