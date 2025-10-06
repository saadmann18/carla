"""
Day 12 - RRT (Rapidly-exploring Random Tree) Path Planner
Implementation of RRT and RRT* algorithms for path planning
"""

import carla
import numpy as np
import random
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional, Dict
import pickle
import os
import time
from dataclasses import dataclass

@dataclass
class RRTNode:
    """Node for RRT tree"""
    x: float
    y: float
    parent: Optional['RRTNode'] = None
    cost: float = 0.0  # For RRT*
    
    def distance_to(self, other: 'RRTNode') -> float:
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

class RRTPlanner:
    """RRT and RRT* path planner"""
    
    def __init__(self, start: Tuple[float, float], goal: Tuple[float, float],
                 bounds: Tuple[float, float, float, float],  # (min_x, max_x, min_y, max_y)
                 obstacles: List[Tuple[float, float, float]] = None,  # (x, y, radius)
                 step_size: float = 5.0,
                 goal_tolerance: float = 3.0,
                 max_iterations: int = 5000):
        
        self.start = RRTNode(start[0], start[1])
        self.goal = RRTNode(goal[0], goal[1])
        self.bounds = bounds
        self.obstacles = obstacles or []
        self.step_size = step_size
        self.goal_tolerance = goal_tolerance
        self.max_iterations = max_iterations
        
        # RRT tree
        self.nodes = [self.start]
        self.path = []
        
        # RRT* parameters
        self.rewire_radius = 15.0
        self.use_rrt_star = False
        
        # Statistics
        self.planning_time = 0.0
        self.iterations_used = 0
        
    def set_obstacles(self, obstacles: List[Tuple[float, float, float]]):
        """Set obstacle list (x, y, radius)"""
        self.obstacles = obstacles
    
    def is_collision_free(self, node1: RRTNode, node2: RRTNode) -> bool:
        """Check if path between two nodes is collision-free"""
        # Check bounds
        if not (self.bounds[0] <= node2.x <= self.bounds[1] and 
                self.bounds[2] <= node2.y <= self.bounds[3]):
            return False
        
        # Check obstacles
        for obs_x, obs_y, obs_radius in self.obstacles:
            # Check if line segment intersects with circular obstacle
            if self._line_circle_intersection(node1, node2, obs_x, obs_y, obs_radius):
                return False
        
        return True
    
    def _line_circle_intersection(self, node1: RRTNode, node2: RRTNode,
                                 circle_x: float, circle_y: float, radius: float) -> bool:
        """Check if line segment intersects with circle"""
        # Vector from node1 to node2
        dx = node2.x - node1.x
        dy = node2.y - node1.y
        
        # Vector from node1 to circle center
        fx = node1.x - circle_x
        fy = node1.y - circle_y
        
        # Quadratic equation coefficients
        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = (fx * fx + fy * fy) - radius * radius
        
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            return False  # No intersection
        
        # Check if intersection points are on the line segment
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        
        return (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1)
    
    def sample_random_point(self) -> RRTNode:
        """Sample a random point in the configuration space"""
        # Bias towards goal with 10% probability
        if random.random() < 0.1:
            return RRTNode(self.goal.x, self.goal.y)
        
        x = random.uniform(self.bounds[0], self.bounds[1])
        y = random.uniform(self.bounds[2], self.bounds[3])
        return RRTNode(x, y)
    
    def find_nearest_node(self, target: RRTNode) -> RRTNode:
        """Find the nearest node in the tree to the target"""
        min_distance = float('inf')
        nearest_node = self.nodes[0]
        
        for node in self.nodes:
            distance = node.distance_to(target)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        
        return nearest_node
    
    def steer(self, from_node: RRTNode, to_node: RRTNode) -> RRTNode:
        """Steer from one node towards another with step size limit"""
        distance = from_node.distance_to(to_node)
        
        if distance <= self.step_size:
            return RRTNode(to_node.x, to_node.y)
        
        # Limit step size
        angle = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + self.step_size * np.cos(angle)
        new_y = from_node.y + self.step_size * np.sin(angle)
        
        return RRTNode(new_x, new_y)
    
    def plan_rrt(self) -> List[Tuple[float, float]]:
        """Plan path using basic RRT algorithm"""
        print("Planning path with RRT...")
        start_time = time.time()
        
        for i in range(self.max_iterations):
            # Sample random point
            random_node = self.sample_random_point()
            
            # Find nearest node
            nearest_node = self.find_nearest_node(random_node)
            
            # Steer towards random point
            new_node = self.steer(nearest_node, random_node)
            
            # Check if path is collision-free
            if self.is_collision_free(nearest_node, new_node):
                new_node.parent = nearest_node
                new_node.cost = nearest_node.cost + nearest_node.distance_to(new_node)
                self.nodes.append(new_node)
                
                # Check if goal is reached
                if new_node.distance_to(self.goal) <= self.goal_tolerance:
                    # Connect to goal
                    goal_node = RRTNode(self.goal.x, self.goal.y)
                    if self.is_collision_free(new_node, goal_node):
                        goal_node.parent = new_node
                        goal_node.cost = new_node.cost + new_node.distance_to(goal_node)
                        self.nodes.append(goal_node)
                        
                        # Reconstruct path
                        self.path = self._reconstruct_path(goal_node)
                        self.planning_time = time.time() - start_time
                        self.iterations_used = i + 1
                        
                        print(f"Path found in {self.iterations_used} iterations, {self.planning_time:.2f}s")
                        return self.path
        
        self.planning_time = time.time() - start_time
        self.iterations_used = self.max_iterations
        print("No path found within iteration limit")
        return []
    
    def plan_rrt_star(self) -> List[Tuple[float, float]]:
        """Plan path using RRT* algorithm with rewiring"""
        print("Planning path with RRT*...")
        self.use_rrt_star = True
        start_time = time.time()
        
        for i in range(self.max_iterations):
            # Sample random point
            random_node = self.sample_random_point()
            
            # Find nearest node
            nearest_node = self.find_nearest_node(random_node)
            
            # Steer towards random point
            new_node = self.steer(nearest_node, random_node)
            
            # Check if path is collision-free
            if self.is_collision_free(nearest_node, new_node):
                # Find nodes within rewire radius
                near_nodes = self._find_near_nodes(new_node)
                
                # Choose parent with minimum cost
                min_cost = nearest_node.cost + nearest_node.distance_to(new_node)
                best_parent = nearest_node
                
                for near_node in near_nodes:
                    if self.is_collision_free(near_node, new_node):
                        cost = near_node.cost + near_node.distance_to(new_node)
                        if cost < min_cost:
                            min_cost = cost
                            best_parent = near_node
                
                new_node.parent = best_parent
                new_node.cost = min_cost
                self.nodes.append(new_node)
                
                # Rewire tree
                self._rewire_tree(new_node, near_nodes)
                
                # Check if goal is reached
                if new_node.distance_to(self.goal) <= self.goal_tolerance:
                    # Connect to goal
                    goal_node = RRTNode(self.goal.x, self.goal.y)
                    if self.is_collision_free(new_node, goal_node):
                        goal_node.parent = new_node
                        goal_node.cost = new_node.cost + new_node.distance_to(goal_node)
                        self.nodes.append(goal_node)
                        
                        # Reconstruct path
                        self.path = self._reconstruct_path(goal_node)
                        self.planning_time = time.time() - start_time
                        self.iterations_used = i + 1
                        
                        print(f"Path found in {self.iterations_used} iterations, {self.planning_time:.2f}s")
                        return self.path
        
        self.planning_time = time.time() - start_time
        self.iterations_used = self.max_iterations
        print("No path found within iteration limit")
        return []
    
    def _find_near_nodes(self, node: RRTNode) -> List[RRTNode]:
        """Find nodes within rewire radius"""
        near_nodes = []
        for tree_node in self.nodes:
            if tree_node.distance_to(node) <= self.rewire_radius:
                near_nodes.append(tree_node)
        return near_nodes
    
    def _rewire_tree(self, new_node: RRTNode, near_nodes: List[RRTNode]):
        """Rewire tree to improve paths"""
        for near_node in near_nodes:
            if near_node == new_node.parent:
                continue
                
            new_cost = new_node.cost + new_node.distance_to(near_node)
            if new_cost < near_node.cost and self.is_collision_free(new_node, near_node):
                near_node.parent = new_node
                near_node.cost = new_cost
                # Update costs of descendants
                self._update_descendants_cost(near_node)
    
    def _update_descendants_cost(self, node: RRTNode):
        """Update costs of all descendants of a node"""
        for child_node in self.nodes:
            if child_node.parent == node:
                child_node.cost = node.cost + node.distance_to(child_node)
                self._update_descendants_cost(child_node)
    
    def _reconstruct_path(self, goal_node: RRTNode) -> List[Tuple[float, float]]:
        """Reconstruct path from goal to start"""
        path = []
        current = goal_node
        
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        
        path.reverse()
        return path
    
    def visualize_tree_and_path(self, save_path: str = "output_data/day12_rrt_runs"):
        """Visualize RRT tree and found path"""
        os.makedirs(save_path, exist_ok=True)
        
        plt.figure(figsize=(12, 8))
        
        # Draw tree edges
        for node in self.nodes:
            if node.parent is not None:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 
                        'b-', alpha=0.3, linewidth=0.5)
        
        # Draw obstacles
        for obs_x, obs_y, radius in self.obstacles:
            circle = plt.Circle((obs_x, obs_y), radius, color='red', alpha=0.5)
            plt.gca().add_patch(circle)
        
        # Draw nodes
        node_x = [node.x for node in self.nodes]
        node_y = [node.y for node in self.nodes]
        plt.scatter(node_x, node_y, c='blue', s=2, alpha=0.6)
        
        # Draw start and goal
        plt.scatter(self.start.x, self.start.y, c='green', s=100, marker='o', label='Start')
        plt.scatter(self.goal.x, self.goal.y, c='red', s=100, marker='s', label='Goal')
        
        # Draw path
        if self.path:
            path_x = [p[0] for p in self.path]
            path_y = [p[1] for p in self.path]
            plt.plot(path_x, path_y, 'r-', linewidth=3, label='Path')
        
        plt.xlim(self.bounds[0], self.bounds[1])
        plt.ylim(self.bounds[2], self.bounds[3])
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.title(f'{"RRT*" if self.use_rrt_star else "RRT"} Path Planning')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # Save plot
        algorithm = "rrt_star" if self.use_rrt_star else "rrt"
        plt.savefig(f"{save_path}/{algorithm}_visualization.png", dpi=300, bbox_inches='tight')
        plt.show()
        
        # Save planning data
        planning_data = {
            'algorithm': algorithm,
            'path': self.path,
            'planning_time': self.planning_time,
            'iterations_used': self.iterations_used,
            'tree_size': len(self.nodes),
            'path_length': self.calculate_path_length(),
            'obstacles': self.obstacles
        }
        
        with open(f"{save_path}/{algorithm}_planning_data.pkl", 'wb') as f:
            pickle.dump(planning_data, f)
        
        print(f"Visualization saved to {save_path}/")
    
    def calculate_path_length(self) -> float:
        """Calculate total path length"""
        if len(self.path) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(self.path) - 1):
            dx = self.path[i+1][0] - self.path[i][0]
            dy = self.path[i+1][1] - self.path[i][1]
            total_length += np.sqrt(dx*dx + dy*dy)
        
        return total_length
    
    def get_planning_metrics(self) -> Dict[str, float]:
        """Get planning performance metrics"""
        return {
            'planning_time': self.planning_time,
            'iterations_used': self.iterations_used,
            'tree_size': len(self.nodes),
            'path_length': self.calculate_path_length(),
            'path_found': len(self.path) > 0
        }

def create_test_environment() -> Tuple[Tuple[float, float], Tuple[float, float], 
                                     Tuple[float, float, float, float], 
                                     List[Tuple[float, float, float]]]:
    """Create a test environment with obstacles"""
    start = (10.0, 10.0)
    goal = (90.0, 90.0)
    bounds = (0.0, 100.0, 0.0, 100.0)
    
    # Create some obstacles
    obstacles = [
        (30.0, 30.0, 8.0),
        (50.0, 20.0, 6.0),
        (70.0, 60.0, 10.0),
        (40.0, 70.0, 7.0),
        (80.0, 30.0, 5.0)
    ]
    
    return start, goal, bounds, obstacles

def main():
    """Main function to test RRT planners"""
    try:
        # Create test environment
        start, goal, bounds, obstacles = create_test_environment()
        
        # Test RRT
        print("Testing RRT...")
        rrt_planner = RRTPlanner(start, goal, bounds, obstacles)
        rrt_path = rrt_planner.plan_rrt()
        
        if rrt_path:
            rrt_planner.visualize_tree_and_path()
            rrt_metrics = rrt_planner.get_planning_metrics()
            print(f"RRT metrics: {rrt_metrics}")
        
        # Test RRT*
        print("\nTesting RRT*...")
        rrt_star_planner = RRTPlanner(start, goal, bounds, obstacles)
        rrt_star_path = rrt_star_planner.plan_rrt_star()
        
        if rrt_star_path:
            rrt_star_planner.visualize_tree_and_path()
            rrt_star_metrics = rrt_star_planner.get_planning_metrics()
            print(f"RRT* metrics: {rrt_star_metrics}")
        
        print("RRT planning tests completed successfully!")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
