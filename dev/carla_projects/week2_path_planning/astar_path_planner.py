"""
Day 9 - A* Path Planner
Implementation of A* algorithm for path planning with visualization
"""

import carla
import numpy as np
import networkx as nx
import heapq
from typing import List, Tuple, Optional, Dict
import matplotlib.pyplot as plt
import pickle
import os
from dataclasses import dataclass

@dataclass
class PathNode:
    """Node for A* pathfinding"""
    index: int
    g_cost: float  # Distance from start
    h_cost: float  # Heuristic distance to goal
    f_cost: float  # Total cost
    parent: Optional['PathNode'] = None
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost

class AStarPathPlanner:
    """A* path planner for Carla road network"""
    
    def __init__(self, road_graph: nx.DiGraph, waypoints_data: List[Dict]):
        self.road_graph = road_graph
        self.waypoints_data = waypoints_data
        self.last_path = []
        
    def heuristic(self, node1_idx: int, node2_idx: int) -> float:
        """Calculate Euclidean distance heuristic between two nodes"""
        pos1 = self.waypoints_data[node1_idx]['location']
        pos2 = self.waypoints_data[node2_idx]['location']
        
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def find_path(self, start_idx: int, goal_idx: int) -> List[int]:
        """Find path using A* algorithm"""
        print(f"Planning path from node {start_idx} to node {goal_idx}")
        
        # Initialize open and closed sets
        open_set = []
        closed_set = set()
        came_from = {}
        
        # Initialize costs
        g_score = {node: float('inf') for node in self.road_graph.nodes()}
        g_score[start_idx] = 0
        
        f_score = {node: float('inf') for node in self.road_graph.nodes()}
        f_score[start_idx] = self.heuristic(start_idx, goal_idx)
        
        # Add start node to open set
        heapq.heappush(open_set, (f_score[start_idx], start_idx))
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
                
            closed_set.add(current)
            
            # Check if we reached the goal
            if current == goal_idx:
                path = self._reconstruct_path(came_from, current)
                self.last_path = path
                print(f"Path found with {len(path)} waypoints")
                return path
            
            # Explore neighbors
            for neighbor in self.road_graph.neighbors(current):
                if neighbor in closed_set:
                    continue
                
                # Calculate tentative g_score
                edge_weight = self.road_graph[current][neighbor].get('weight', 1.0)
                tentative_g = g_score[current] + edge_weight
                
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_idx)
                    
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        print("No path found!")
        return []
    
    def _reconstruct_path(self, came_from: Dict[int, int], current: int) -> List[int]:
        """Reconstruct path from came_from dictionary"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def find_nearest_node(self, location: Tuple[float, float, float]) -> int:
        """Find the nearest waypoint node to a given location"""
        min_distance = float('inf')
        nearest_idx = 0
        
        for i, wp_data in enumerate(self.waypoints_data):
            wp_loc = wp_data['location']
            distance = np.sqrt((location[0] - wp_loc[0])**2 + 
                             (location[1] - wp_loc[1])**2)
            
            if distance < min_distance:
                min_distance = distance
                nearest_idx = i
                
        return nearest_idx
    
    def calculate_path_metrics(self, path: List[int]) -> Dict[str, float]:
        """Calculate path metrics like total distance and smoothness"""
        if len(path) < 2:
            return {'total_distance': 0.0, 'smoothness': 0.0, 'num_waypoints': len(path)}
        
        total_distance = 0.0
        direction_changes = 0
        
        prev_direction = None
        
        for i in range(len(path) - 1):
            current_pos = self.waypoints_data[path[i]]['location']
            next_pos = self.waypoints_data[path[i + 1]]['location']
            
            # Calculate distance
            distance = np.sqrt((next_pos[0] - current_pos[0])**2 + 
                             (next_pos[1] - current_pos[1])**2)
            total_distance += distance
            
            # Calculate direction change
            direction = np.arctan2(next_pos[1] - current_pos[1], 
                                 next_pos[0] - current_pos[0])
            
            if prev_direction is not None:
                angle_diff = abs(direction - prev_direction)
                if angle_diff > np.pi:
                    angle_diff = 2 * np.pi - angle_diff
                if angle_diff > np.pi / 4:  # 45 degrees threshold
                    direction_changes += 1
            
            prev_direction = direction
        
        smoothness = 1.0 / (1.0 + direction_changes)  # Higher is smoother
        
        return {
            'total_distance': total_distance,
            'smoothness': smoothness,
            'num_waypoints': len(path),
            'direction_changes': direction_changes
        }
    
    def visualize_path(self, path: List[int], save_path: str = "output_data/day9_astar_paths"):
        """Visualize the planned path"""
        os.makedirs(save_path, exist_ok=True)
        
        # Extract coordinates
        all_x = [wp['location'][0] for wp in self.waypoints_data]
        all_y = [wp['location'][1] for wp in self.waypoints_data]
        
        path_x = [self.waypoints_data[i]['location'][0] for i in path]
        path_y = [self.waypoints_data[i]['location'][1] for i in path]
        
        # Create plot
        plt.figure(figsize=(12, 8))
        plt.scatter(all_x, all_y, c='lightgray', s=1, alpha=0.5, label='All waypoints')
        plt.plot(path_x, path_y, 'r-', linewidth=2, label='A* Path')
        plt.scatter(path_x[0], path_y[0], c='green', s=100, marker='o', label='Start')
        plt.scatter(path_x[-1], path_y[-1], c='red', s=100, marker='s', label='Goal')
        
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.title('A* Path Planning Result')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # Save plot
        plt.savefig(f"{save_path}/astar_path_visualization.png", dpi=300, bbox_inches='tight')
        plt.show()
        
        # Save path data
        path_data = {
            'path_indices': path,
            'path_coordinates': [(self.waypoints_data[i]['location'][0], 
                                self.waypoints_data[i]['location'][1]) for i in path],
            'metrics': self.calculate_path_metrics(path)
        }
        
        with open(f"{save_path}/astar_path_data.pkl", 'wb') as f:
            pickle.dump(path_data, f)
        
        print(f"Path visualization saved to {save_path}/")
    
    def draw_path_in_carla(self, world: carla.World, path: List[int], 
                          life_time: float = 60.0):
        """Draw the path in Carla simulator"""
        debug = world.debug
        
        # Draw path waypoints
        for i, node_idx in enumerate(path):
            wp_data = self.waypoints_data[node_idx]
            location = carla.Location(wp_data['location'][0], 
                                    wp_data['location'][1], 
                                    wp_data['location'][2] + 0.5)
            
            color = carla.Color(0, 255, 0) if i == 0 else \
                   carla.Color(255, 0, 0) if i == len(path) - 1 else \
                   carla.Color(255, 255, 0)
            
            debug.draw_point(location, size=0.2, color=color, life_time=life_time)
        
        # Draw path connections
        for i in range(len(path) - 1):
            start_data = self.waypoints_data[path[i]]
            end_data = self.waypoints_data[path[i + 1]]
            
            start_loc = carla.Location(start_data['location'][0], 
                                     start_data['location'][1], 
                                     start_data['location'][2] + 0.5)
            end_loc = carla.Location(end_data['location'][0], 
                                   end_data['location'][1], 
                                   end_data['location'][2] + 0.5)
            
            debug.draw_line(start_loc, end_loc, 
                          thickness=0.1, 
                          color=carla.Color(0, 0, 255), 
                          life_time=life_time)

def load_road_network(data_dir: str = "output_data/day8_road_graphs"):
    """Load road network data from files"""
    with open(f"{data_dir}/waypoints.pkl", 'rb') as f:
        waypoints_data = pickle.load(f)
    
    road_graph = nx.read_gpickle(f"{data_dir}/road_graph.pkl")
    
    return road_graph, waypoints_data

def main():
    """Main function to test A* path planning"""
    try:
        # Load road network
        print("Loading road network...")
        road_graph, waypoints_data = load_road_network()
        
        # Create A* planner
        planner = AStarPathPlanner(road_graph, waypoints_data)
        
        # Plan a path (using first and last waypoints as example)
        start_idx = 0
        goal_idx = min(100, len(waypoints_data) - 1)  # Safe goal selection
        
        path = planner.find_path(start_idx, goal_idx)
        
        if path:
            # Calculate and display metrics
            metrics = planner.calculate_path_metrics(path)
            print(f"Path metrics: {metrics}")
            
            # Visualize path
            planner.visualize_path(path)
            
            # Connect to Carla for visualization (optional)
            try:
                client = carla.Client('localhost', 2000)
                client.set_timeout(5.0)
                world = client.get_world()
                planner.draw_path_in_carla(world, path)
                print("Path drawn in Carla simulator")
            except:
                print("Could not connect to Carla for visualization")
        
        print("A* path planning completed successfully!")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
