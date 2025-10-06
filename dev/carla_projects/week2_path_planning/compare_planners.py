"""
Day 14 - Compare A* vs RRT Performance
Comprehensive comparison of A* and RRT planners across different scenarios
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import pickle
import os
from typing import List, Dict, Tuple
from astar_path_planner import AStarPathPlanner, load_road_network
from rrt_planner import RRTPlanner
import pandas as pd

class PlannerComparison:
    """Compare different path planning algorithms"""
    
    def __init__(self):
        self.results = []
        
    def create_test_scenarios(self) -> List[Dict]:
        """Create different test scenarios for comparison"""
        scenarios = []
        
        # Scenario 1: Simple open space
        scenarios.append({
            'name': 'Open Space',
            'start': (10.0, 10.0),
            'goal': (90.0, 90.0),
            'bounds': (0.0, 100.0, 0.0, 100.0),
            'obstacles': []
        })
        
        # Scenario 2: Sparse obstacles
        scenarios.append({
            'name': 'Sparse Obstacles',
            'start': (10.0, 10.0),
            'goal': (90.0, 90.0),
            'bounds': (0.0, 100.0, 0.0, 100.0),
            'obstacles': [
                (30.0, 30.0, 8.0),
                (70.0, 60.0, 10.0),
                (50.0, 80.0, 6.0)
            ]
        })
        
        # Scenario 3: Dense obstacles
        scenarios.append({
            'name': 'Dense Obstacles',
            'start': (5.0, 5.0),
            'goal': (95.0, 95.0),
            'bounds': (0.0, 100.0, 0.0, 100.0),
            'obstacles': [
                (20.0, 20.0, 8.0),
                (35.0, 15.0, 6.0),
                (50.0, 25.0, 7.0),
                (25.0, 40.0, 9.0),
                (45.0, 45.0, 8.0),
                (65.0, 35.0, 6.0),
                (80.0, 50.0, 10.0),
                (60.0, 70.0, 7.0),
                (75.0, 80.0, 8.0),
                (40.0, 75.0, 6.0)
            ]
        })
        
        # Scenario 4: Narrow passage
        scenarios.append({
            'name': 'Narrow Passage',
            'start': (10.0, 50.0),
            'goal': (90.0, 50.0),
            'bounds': (0.0, 100.0, 0.0, 100.0),
            'obstacles': [
                (45.0, 30.0, 18.0),  # Large obstacle creating narrow passage
                (55.0, 70.0, 18.0),  # Large obstacle creating narrow passage
            ]
        })
        
        # Scenario 5: Maze-like environment
        scenarios.append({
            'name': 'Maze Environment',
            'start': (5.0, 5.0),
            'goal': (95.0, 95.0),
            'bounds': (0.0, 100.0, 0.0, 100.0),
            'obstacles': [
                # Create maze-like structure
                (25.0, 25.0, 12.0),
                (75.0, 25.0, 12.0),
                (25.0, 75.0, 12.0),
                (75.0, 75.0, 12.0),
                (50.0, 15.0, 8.0),
                (50.0, 85.0, 8.0),
                (15.0, 50.0, 8.0),
                (85.0, 50.0, 8.0),
                (50.0, 50.0, 6.0),
            ]
        })
        
        return scenarios
    
    def test_rrt_planner(self, scenario: Dict, num_runs: int = 5) -> Dict:
        """Test RRT planner on a scenario"""
        print(f"Testing RRT on {scenario['name']}...")
        
        results = {
            'algorithm': 'RRT',
            'scenario': scenario['name'],
            'runs': []
        }
        
        for run in range(num_runs):
            planner = RRTPlanner(
                start=scenario['start'],
                goal=scenario['goal'],
                bounds=scenario['bounds'],
                obstacles=scenario['obstacles'],
                step_size=3.0,
                max_iterations=3000
            )
            
            start_time = time.time()
            path = planner.plan_rrt()
            end_time = time.time()
            
            run_result = {
                'run_number': run + 1,
                'path_found': len(path) > 0,
                'planning_time': end_time - start_time,
                'path_length': planner.calculate_path_length() if path else float('inf'),
                'iterations_used': planner.iterations_used,
                'tree_size': len(planner.nodes)
            }
            
            results['runs'].append(run_result)
        
        return results
    
    def test_rrt_star_planner(self, scenario: Dict, num_runs: int = 5) -> Dict:
        """Test RRT* planner on a scenario"""
        print(f"Testing RRT* on {scenario['name']}...")
        
        results = {
            'algorithm': 'RRT*',
            'scenario': scenario['name'],
            'runs': []
        }
        
        for run in range(num_runs):
            planner = RRTPlanner(
                start=scenario['start'],
                goal=scenario['goal'],
                bounds=scenario['bounds'],
                obstacles=scenario['obstacles'],
                step_size=3.0,
                max_iterations=3000
            )
            
            start_time = time.time()
            path = planner.plan_rrt_star()
            end_time = time.time()
            
            run_result = {
                'run_number': run + 1,
                'path_found': len(path) > 0,
                'planning_time': end_time - start_time,
                'path_length': planner.calculate_path_length() if path else float('inf'),
                'iterations_used': planner.iterations_used,
                'tree_size': len(planner.nodes)
            }
            
            results['runs'].append(run_result)
        
        return results
    
    def test_astar_with_grid(self, scenario: Dict, grid_resolution: float = 2.0, num_runs: int = 5) -> Dict:
        """Test A* planner on a scenario using grid-based approach"""
        print(f"Testing A* (Grid) on {scenario['name']}...")
        
        results = {
            'algorithm': 'A* (Grid)',
            'scenario': scenario['name'],
            'runs': []
        }
        
        # Create grid-based representation
        bounds = scenario['bounds']
        grid_width = int((bounds[1] - bounds[0]) / grid_resolution)
        grid_height = int((bounds[3] - bounds[2]) / grid_resolution)
        
        # Create occupancy grid
        grid = np.zeros((grid_height, grid_width))
        
        # Mark obstacles in grid
        for obs_x, obs_y, obs_radius in scenario['obstacles']:
            grid_x = int((obs_x - bounds[0]) / grid_resolution)
            grid_y = int((obs_y - bounds[2]) / grid_resolution)
            
            # Mark cells within obstacle radius
            radius_cells = int(obs_radius / grid_resolution) + 1
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if 0 <= grid_y + dy < grid_height and 0 <= grid_x + dx < grid_width:
                        if dx*dx + dy*dy <= radius_cells*radius_cells:
                            grid[grid_y + dy, grid_x + dx] = 1
        
        for run in range(num_runs):
            start_time = time.time()
            path = self._astar_grid_search(grid, scenario['start'], scenario['goal'], 
                                         bounds, grid_resolution)
            end_time = time.time()
            
            path_length = 0.0
            if path:
                for i in range(len(path) - 1):
                    dx = path[i+1][0] - path[i][0]
                    dy = path[i+1][1] - path[i][1]
                    path_length += np.sqrt(dx*dx + dy*dy)
            
            run_result = {
                'run_number': run + 1,
                'path_found': len(path) > 0 if path else False,
                'planning_time': end_time - start_time,
                'path_length': path_length if path else float('inf'),
                'iterations_used': grid_width * grid_height,  # Worst case
                'tree_size': len(path) if path else 0
            }
            
            results['runs'].append(run_result)
        
        return results
    
    def _astar_grid_search(self, grid: np.ndarray, start: Tuple[float, float], 
                          goal: Tuple[float, float], bounds: Tuple[float, float, float, float],
                          resolution: float) -> List[Tuple[float, float]]:
        """Simple A* implementation for grid-based planning"""
        import heapq
        
        # Convert world coordinates to grid coordinates
        start_grid = (int((start[1] - bounds[2]) / resolution), 
                     int((start[0] - bounds[0]) / resolution))
        goal_grid = (int((goal[1] - bounds[2]) / resolution), 
                    int((goal[0] - bounds[0]) / resolution))
        
        if (start_grid[0] < 0 or start_grid[0] >= grid.shape[0] or 
            start_grid[1] < 0 or start_grid[1] >= grid.shape[1] or
            goal_grid[0] < 0 or goal_grid[0] >= grid.shape[0] or 
            goal_grid[1] < 0 or goal_grid[1] >= grid.shape[1]):
            return []
        
        # A* search
        open_set = [(0, start_grid)]
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self._heuristic(start_grid, goal_grid)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal_grid:
                # Reconstruct path
                path = []
                while current in came_from:
                    # Convert back to world coordinates
                    world_x = bounds[0] + current[1] * resolution
                    world_y = bounds[2] + current[0] * resolution
                    path.append((world_x, world_y))
                    current = came_from[current]
                path.reverse()
                return path
            
            # Check neighbors
            for dy, dx in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor = (current[0] + dy, current[1] + dx)
                
                if (0 <= neighbor[0] < grid.shape[0] and 
                    0 <= neighbor[1] < grid.shape[1] and 
                    grid[neighbor[0], neighbor[1]] == 0):
                    
                    tentative_g = g_score[current] + np.sqrt(dx*dx + dy*dy)
                    
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal_grid)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return []  # No path found
    
    def _heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def run_comprehensive_comparison(self, num_runs: int = 5) -> List[Dict]:
        """Run comprehensive comparison across all scenarios and algorithms"""
        scenarios = self.create_test_scenarios()
        all_results = []
        
        for scenario in scenarios:
            print(f"\n--- Testing scenario: {scenario['name']} ---")
            
            # Test RRT
            rrt_results = self.test_rrt_planner(scenario, num_runs)
            all_results.append(rrt_results)
            
            # Test RRT*
            rrt_star_results = self.test_rrt_star_planner(scenario, num_runs)
            all_results.append(rrt_star_results)
            
            # Test A* (Grid)
            astar_results = self.test_astar_with_grid(scenario, grid_resolution=1.0, num_runs=num_runs)
            all_results.append(astar_results)
        
        self.results = all_results
        return all_results
    
    def analyze_results(self) -> Dict:
        """Analyze comparison results"""
        if not self.results:
            return {}
        
        analysis = {}
        
        # Group results by algorithm and scenario
        for result in self.results:
            algorithm = result['algorithm']
            scenario = result['scenario']
            
            if algorithm not in analysis:
                analysis[algorithm] = {}
            
            if scenario not in analysis[algorithm]:
                analysis[algorithm][scenario] = {
                    'success_rate': 0.0,
                    'avg_planning_time': 0.0,
                    'avg_path_length': 0.0,
                    'std_planning_time': 0.0,
                    'std_path_length': 0.0
                }
            
            # Calculate statistics
            successful_runs = [r for r in result['runs'] if r['path_found']]
            total_runs = len(result['runs'])
            
            if successful_runs:
                planning_times = [r['planning_time'] for r in successful_runs]
                path_lengths = [r['path_length'] for r in successful_runs if r['path_length'] != float('inf')]
                
                analysis[algorithm][scenario] = {
                    'success_rate': len(successful_runs) / total_runs,
                    'avg_planning_time': np.mean(planning_times),
                    'avg_path_length': np.mean(path_lengths) if path_lengths else float('inf'),
                    'std_planning_time': np.std(planning_times),
                    'std_path_length': np.std(path_lengths) if path_lengths else 0.0
                }
        
        return analysis
    
    def create_comparison_plots(self, analysis: Dict, save_dir: str = "output_data/day14_comparisons"):
        """Create visualization plots for comparison results"""
        os.makedirs(save_dir, exist_ok=True)
        
        algorithms = list(analysis.keys())
        scenarios = list(analysis[algorithms[0]].keys()) if algorithms else []
        
        # Success rate comparison
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))
        
        # Plot 1: Success Rate
        success_data = []
        for algorithm in algorithms:
            success_rates = [analysis[algorithm][scenario]['success_rate'] for scenario in scenarios]
            success_data.append(success_rates)
        
        x = np.arange(len(scenarios))
        width = 0.25
        
        for i, (algorithm, rates) in enumerate(zip(algorithms, success_data)):
            ax1.bar(x + i * width, rates, width, label=algorithm)
        
        ax1.set_xlabel('Scenarios')
        ax1.set_ylabel('Success Rate')
        ax1.set_title('Success Rate Comparison')
        ax1.set_xticks(x + width)
        ax1.set_xticklabels(scenarios, rotation=45)
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Planning Time
        for i, algorithm in enumerate(algorithms):
            times = [analysis[algorithm][scenario]['avg_planning_time'] for scenario in scenarios]
            ax2.bar(x + i * width, times, width, label=algorithm)
        
        ax2.set_xlabel('Scenarios')
        ax2.set_ylabel('Average Planning Time (s)')
        ax2.set_title('Planning Time Comparison')
        ax2.set_xticks(x + width)
        ax2.set_xticklabels(scenarios, rotation=45)
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Path Length
        for i, algorithm in enumerate(algorithms):
            lengths = []
            for scenario in scenarios:
                length = analysis[algorithm][scenario]['avg_path_length']
                lengths.append(length if length != float('inf') else 0)
            ax3.bar(x + i * width, lengths, width, label=algorithm)
        
        ax3.set_xlabel('Scenarios')
        ax3.set_ylabel('Average Path Length (m)')
        ax3.set_title('Path Length Comparison')
        ax3.set_xticks(x + width)
        ax3.set_xticklabels(scenarios, rotation=45)
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f"{save_dir}/planner_comparison.png", dpi=300, bbox_inches='tight')
        plt.show()
        
        # Create summary table
        self.create_summary_table(analysis, save_dir)
    
    def create_summary_table(self, analysis: Dict, save_dir: str):
        """Create summary table of results"""
        rows = []
        
        for algorithm in analysis:
            for scenario in analysis[algorithm]:
                data = analysis[algorithm][scenario]
                rows.append({
                    'Algorithm': algorithm,
                    'Scenario': scenario,
                    'Success Rate': f"{data['success_rate']:.2%}",
                    'Avg Planning Time (s)': f"{data['avg_planning_time']:.3f}",
                    'Avg Path Length (m)': f"{data['avg_path_length']:.2f}" if data['avg_path_length'] != float('inf') else "N/A",
                    'Planning Time Std': f"{data['std_planning_time']:.3f}",
                    'Path Length Std': f"{data['std_path_length']:.2f}"
                })
        
        df = pd.DataFrame(rows)
        df.to_csv(f"{save_dir}/comparison_summary.csv", index=False)
        print(f"Summary table saved to {save_dir}/comparison_summary.csv")
    
    def save_results(self, save_dir: str = "output_data/day14_comparisons"):
        """Save all results to files"""
        os.makedirs(save_dir, exist_ok=True)
        
        with open(f"{save_dir}/comparison_results.pkl", 'wb') as f:
            pickle.dump(self.results, f)
        
        analysis = self.analyze_results()
        with open(f"{save_dir}/comparison_analysis.pkl", 'wb') as f:
            pickle.dump(analysis, f)
        
        print(f"Results saved to {save_dir}/")

def main():
    """Main function to run planner comparison"""
    try:
        print("Starting comprehensive planner comparison...")
        
        # Create comparison instance
        comparison = PlannerComparison()
        
        # Run comparison
        results = comparison.run_comprehensive_comparison(num_runs=3)
        
        # Analyze results
        analysis = comparison.analyze_results()
        
        # Create visualizations
        comparison.create_comparison_plots(analysis)
        
        # Save results
        comparison.save_results()
        
        # Print summary
        print("\n--- Comparison Summary ---")
        for algorithm in analysis:
            print(f"\n{algorithm}:")
            for scenario in analysis[algorithm]:
                data = analysis[algorithm][scenario]
                print(f"  {scenario}: Success={data['success_rate']:.2%}, "
                      f"Time={data['avg_planning_time']:.3f}s, "
                      f"Length={data['avg_path_length']:.2f}m")
        
        print("\nPlanner comparison completed successfully!")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
