"""
Day 13 - Test RRT with Dynamic Traffic
Tests RRT planner in dynamic environments with moving obstacles
"""

import carla
import numpy as np
import time
import random
from typing import List, Tuple, Dict
from rrt_planner import RRTPlanner, RRTNode
import pickle
import os

class DynamicObstacle:
    """Represents a moving obstacle"""
    
    def __init__(self, x: float, y: float, radius: float, 
                 velocity: Tuple[float, float] = (0.0, 0.0)):
        self.x = x
        self.y = y
        self.radius = radius
        self.velocity = velocity  # (vx, vy) in m/s
        self.start_time = time.time()
        
    def update_position(self, current_time: float):
        """Update obstacle position based on velocity"""
        dt = current_time - self.start_time
        self.x += self.velocity[0] * dt
        self.y += self.velocity[1] * dt
        self.start_time = current_time
    
    def get_position(self) -> Tuple[float, float, float]:
        """Get current position as (x, y, radius)"""
        return (self.x, self.y, self.radius)

class DynamicRRTPlanner(RRTPlanner):
    """RRT planner that handles dynamic obstacles"""
    
    def __init__(self, start: Tuple[float, float], goal: Tuple[float, float],
                 bounds: Tuple[float, float, float, float],
                 static_obstacles: List[Tuple[float, float, float]] = None,
                 dynamic_obstacles: List[DynamicObstacle] = None,
                 **kwargs):
        
        super().__init__(start, goal, bounds, static_obstacles or [], **kwargs)
        self.dynamic_obstacles = dynamic_obstacles or []
        self.planning_start_time = None
        
    def update_dynamic_obstacles(self):
        """Update positions of dynamic obstacles"""
        current_time = time.time()
        if self.planning_start_time is None:
            self.planning_start_time = current_time
            
        for obstacle in self.dynamic_obstacles:
            obstacle.update_position(current_time)
        
        # Update obstacle list for collision checking
        dynamic_positions = [obs.get_position() for obs in self.dynamic_obstacles]
        self.obstacles = self.obstacles[:len(self.obstacles) - len(self.dynamic_obstacles)] + dynamic_positions
    
    def is_collision_free(self, node1: RRTNode, node2: RRTNode) -> bool:
        """Check collision with both static and dynamic obstacles"""
        # Update dynamic obstacle positions
        self.update_dynamic_obstacles()
        
        # Use parent class collision checking
        return super().is_collision_free(node1, node2)
    
    def plan_with_replanning(self, replan_interval: float = 2.0) -> List[Tuple[float, float]]:
        """Plan with periodic replanning to handle dynamic obstacles"""
        print("Planning with dynamic obstacle avoidance...")
        
        best_path = []
        last_replan_time = time.time()
        
        for i in range(self.max_iterations):
            current_time = time.time()
            
            # Replan if interval has passed
            if current_time - last_replan_time >= replan_interval:
                print(f"Replanning at iteration {i}...")
                # Reset tree but keep some nodes near current best path
                self._partial_tree_reset()
                last_replan_time = current_time
            
            # Standard RRT iteration
            random_node = self.sample_random_point()
            nearest_node = self.find_nearest_node(random_node)
            new_node = self.steer(nearest_node, random_node)
            
            if self.is_collision_free(nearest_node, new_node):
                new_node.parent = nearest_node
                new_node.cost = nearest_node.cost + nearest_node.distance_to(new_node)
                self.nodes.append(new_node)
                
                # Check if goal is reached
                if new_node.distance_to(self.goal) <= self.goal_tolerance:
                    goal_node = RRTNode(self.goal.x, self.goal.y)
                    if self.is_collision_free(new_node, goal_node):
                        goal_node.parent = new_node
                        goal_node.cost = new_node.cost + new_node.distance_to(goal_node)
                        self.nodes.append(goal_node)
                        
                        best_path = self._reconstruct_path(goal_node)
                        self.path = best_path
                        self.iterations_used = i + 1
                        
                        print(f"Dynamic path found in {self.iterations_used} iterations")
                        return best_path
        
        self.iterations_used = self.max_iterations
        return best_path
    
    def _partial_tree_reset(self):
        """Reset part of the tree while keeping useful nodes"""
        if not self.path:
            return
            
        # Keep nodes that are close to the current best path
        keep_distance = 10.0
        nodes_to_keep = [self.start]
        
        for node in self.nodes[1:]:  # Skip start node
            min_dist_to_path = float('inf')
            for path_point in self.path:
                dist = np.sqrt((node.x - path_point[0])**2 + (node.y - path_point[1])**2)
                min_dist_to_path = min(min_dist_to_path, dist)
            
            if min_dist_to_path <= keep_distance:
                nodes_to_keep.append(node)
        
        self.nodes = nodes_to_keep
        print(f"Kept {len(nodes_to_keep)} nodes after partial reset")

class TrafficSimulator:
    """Simulates traffic scenario for testing RRT planner"""
    
    def __init__(self, world: carla.World):
        self.world = world
        self.vehicles = []
        self.dynamic_obstacles = []
        
    def spawn_traffic_vehicles(self, num_vehicles: int = 5) -> List[DynamicObstacle]:
        """Spawn traffic vehicles and create dynamic obstacles"""
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bps = blueprint_library.filter('vehicle.*')
        spawn_points = self.world.get_map().get_spawn_points()
        
        for i in range(min(num_vehicles, len(spawn_points))):
            # Choose random vehicle blueprint and spawn point
            vehicle_bp = random.choice(vehicle_bps)
            spawn_point = spawn_points[i]
            
            try:
                vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
                self.vehicles.append(vehicle)
                
                # Create dynamic obstacle
                loc = spawn_point.location
                velocity = (random.uniform(-5, 5), random.uniform(-5, 5))  # Random velocity
                obstacle = DynamicObstacle(loc.x, loc.y, 3.0, velocity)  # 3m radius
                self.dynamic_obstacles.append(obstacle)
                
                # Set vehicle to autopilot for realistic movement
                vehicle.set_autopilot(True)
                
            except Exception as e:
                print(f"Failed to spawn vehicle {i}: {e}")
        
        print(f"Spawned {len(self.vehicles)} traffic vehicles")
        return self.dynamic_obstacles
    
    def update_dynamic_obstacles_from_vehicles(self):
        """Update dynamic obstacle positions based on actual vehicle positions"""
        for i, vehicle in enumerate(self.vehicles):
            if i < len(self.dynamic_obstacles):
                loc = vehicle.get_location()
                self.dynamic_obstacles[i].x = loc.x
                self.dynamic_obstacles[i].y = loc.y
    
    def cleanup(self):
        """Clean up spawned vehicles"""
        for vehicle in self.vehicles:
            try:
                vehicle.destroy()
            except:
                pass
        self.vehicles.clear()

def run_traffic_test(world: carla.World, start: Tuple[float, float], 
                    goal: Tuple[float, float], bounds: Tuple[float, float, float, float],
                    static_obstacles: List[Tuple[float, float, float]] = None) -> Dict:
    """Run RRT test with dynamic traffic"""
    
    # Create traffic simulator
    traffic_sim = TrafficSimulator(world)
    
    try:
        # Spawn traffic vehicles
        dynamic_obstacles = traffic_sim.spawn_traffic_vehicles(num_vehicles=3)
        
        # Create dynamic RRT planner
        planner = DynamicRRTPlanner(
            start=start,
            goal=goal,
            bounds=bounds,
            static_obstacles=static_obstacles or [],
            dynamic_obstacles=dynamic_obstacles,
            step_size=3.0,
            max_iterations=2000
        )
        
        # Plan path with replanning
        start_time = time.time()
        path = planner.plan_with_replanning(replan_interval=1.5)
        planning_time = time.time() - start_time
        
        # Update obstacles from actual vehicle positions periodically
        for _ in range(10):  # Update 10 times during planning
            time.sleep(0.2)
            traffic_sim.update_dynamic_obstacles_from_vehicles()
        
        # Collect results
        results = {
            'path_found': len(path) > 0,
            'path_length': planner.calculate_path_length() if path else 0,
            'planning_time': planning_time,
            'iterations_used': planner.iterations_used,
            'tree_size': len(planner.nodes),
            'num_dynamic_obstacles': len(dynamic_obstacles),
            'path': path
        }
        
        # Visualize results
        if path:
            planner.visualize_tree_and_path("output_data/day13_traffic_tests")
        
        return results
        
    finally:
        traffic_sim.cleanup()

def create_urban_scenario() -> Tuple[Tuple[float, float], Tuple[float, float], 
                                   Tuple[float, float, float, float], 
                                   List[Tuple[float, float, float]]]:
    """Create an urban-like scenario with buildings as static obstacles"""
    start = (20.0, 20.0)
    goal = (180.0, 180.0)
    bounds = (0.0, 200.0, 0.0, 200.0)
    
    # Static obstacles representing buildings
    static_obstacles = [
        (60.0, 40.0, 15.0),   # Building 1
        (100.0, 80.0, 20.0),  # Building 2
        (140.0, 60.0, 12.0),  # Building 3
        (80.0, 120.0, 18.0),  # Building 4
        (160.0, 140.0, 16.0), # Building 5
        (40.0, 100.0, 10.0),  # Building 6
        (120.0, 160.0, 14.0), # Building 7
    ]
    
    return start, goal, bounds, static_obstacles

def run_multiple_traffic_tests(world: carla.World, num_tests: int = 5) -> List[Dict]:
    """Run multiple traffic tests and collect statistics"""
    print(f"Running {num_tests} traffic tests...")
    
    results = []
    start, goal, bounds, static_obstacles = create_urban_scenario()
    
    for test_num in range(num_tests):
        print(f"\n--- Running test {test_num + 1}/{num_tests} ---")
        
        try:
            test_result = run_traffic_test(world, start, goal, bounds, static_obstacles)
            test_result['test_number'] = test_num + 1
            results.append(test_result)
            
            print(f"Test {test_num + 1} results: {test_result}")
            
            # Wait between tests
            time.sleep(2.0)
            
        except Exception as e:
            print(f"Test {test_num + 1} failed: {e}")
            results.append({
                'test_number': test_num + 1,
                'path_found': False,
                'error': str(e)
            })
    
    return results

def analyze_traffic_test_results(results: List[Dict]) -> Dict:
    """Analyze results from multiple traffic tests"""
    successful_tests = [r for r in results if r.get('path_found', False)]
    
    if not successful_tests:
        return {'success_rate': 0.0, 'message': 'No successful tests'}
    
    analysis = {
        'total_tests': len(results),
        'successful_tests': len(successful_tests),
        'success_rate': len(successful_tests) / len(results),
        'avg_planning_time': np.mean([r['planning_time'] for r in successful_tests]),
        'avg_path_length': np.mean([r['path_length'] for r in successful_tests]),
        'avg_iterations': np.mean([r['iterations_used'] for r in successful_tests]),
        'avg_tree_size': np.mean([r['tree_size'] for r in successful_tests])
    }
    
    return analysis

def main():
    """Main function to test RRT with dynamic traffic"""
    try:
        # Connect to Carla
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        # Run multiple traffic tests
        test_results = run_multiple_traffic_tests(world, num_tests=3)
        
        # Analyze results
        analysis = analyze_traffic_test_results(test_results)
        print(f"\n--- Traffic Test Analysis ---")
        print(f"Success rate: {analysis.get('success_rate', 0):.2%}")
        print(f"Average planning time: {analysis.get('avg_planning_time', 0):.2f}s")
        print(f"Average path length: {analysis.get('avg_path_length', 0):.2f}m")
        
        # Save results
        output_dir = "output_data/day13_traffic_tests"
        os.makedirs(output_dir, exist_ok=True)
        
        with open(f"{output_dir}/traffic_test_results.pkl", 'wb') as f:
            pickle.dump({'test_results': test_results, 'analysis': analysis}, f)
        
        print(f"Results saved to {output_dir}/")
        print("Traffic testing completed successfully!")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
