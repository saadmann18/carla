"""
Unit tests for A* path planning algorithm
"""

import unittest
import numpy as np
import networkx as nx
from unittest.mock import Mock, patch
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from astar_path_planner import AStarPathPlanner, PathNode

class TestPathNode(unittest.TestCase):
    """Test cases for PathNode class"""
    
    def test_path_node_creation(self):
        """Test PathNode creation and properties"""
        node = PathNode(index=5, g_cost=10.0, h_cost=15.0, f_cost=25.0)
        
        self.assertEqual(node.index, 5)
        self.assertEqual(node.g_cost, 10.0)
        self.assertEqual(node.h_cost, 15.0)
        self.assertEqual(node.f_cost, 25.0)
        self.assertIsNone(node.parent)
    
    def test_path_node_comparison(self):
        """Test PathNode comparison for priority queue"""
        node1 = PathNode(index=1, g_cost=10.0, h_cost=5.0, f_cost=15.0)
        node2 = PathNode(index=2, g_cost=8.0, h_cost=10.0, f_cost=18.0)
        
        # node1 should be less than node2 (lower f_cost)
        self.assertTrue(node1 < node2)
        self.assertFalse(node2 < node1)

class TestAStarPathPlanner(unittest.TestCase):
    """Test cases for AStarPathPlanner class"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Create a simple test graph
        self.graph = nx.DiGraph()
        
        # Add nodes (5x5 grid)
        for i in range(25):
            self.graph.add_node(i)
        
        # Add edges (grid connectivity)
        for i in range(5):
            for j in range(5):
                node_id = i * 5 + j
                
                # Right connection
                if j < 4:
                    right_id = i * 5 + (j + 1)
                    self.graph.add_edge(node_id, right_id, weight=1.0)
                
                # Down connection
                if i < 4:
                    down_id = (i + 1) * 5 + j
                    self.graph.add_edge(node_id, down_id, weight=1.0)
        
        # Create waypoint data
        self.waypoints_data = []
        for i in range(5):
            for j in range(5):
                self.waypoints_data.append({
                    'location': (float(j), float(i), 0.0),
                    'rotation': (0.0, 0.0, 0.0),
                    'road_id': 1,
                    'lane_id': -1,
                    'lane_width': 3.5
                })
        
        # Create planner
        self.planner = AStarPathPlanner(self.graph, self.waypoints_data)
    
    def test_heuristic_calculation(self):
        """Test heuristic distance calculation"""
        # Distance from (0,0) to (3,4) should be 5.0
        distance = self.planner.heuristic(0, 23)  # node 0 = (0,0), node 23 = (3,4)
        expected_distance = np.sqrt(3**2 + 4**2)
        
        self.assertAlmostEqual(distance, expected_distance, places=5)
    
    def test_find_path_simple(self):
        """Test finding a simple path"""
        # Find path from top-left (0,0) to bottom-right (4,4)
        start_idx = 0   # (0,0)
        goal_idx = 24   # (4,4)
        
        path = self.planner.find_path(start_idx, goal_idx)
        
        # Verify path exists
        self.assertGreater(len(path), 0)
        
        # Verify start and end
        self.assertEqual(path[0], start_idx)
        self.assertEqual(path[-1], goal_idx)
        
        # Verify path connectivity
        for i in range(len(path) - 1):
            current_node = path[i]
            next_node = path[i + 1]
            self.assertTrue(self.graph.has_edge(current_node, next_node))
    
    def test_find_path_no_solution(self):
        """Test behavior when no path exists"""
        # Create disconnected graph
        disconnected_graph = nx.DiGraph()
        disconnected_graph.add_node(0)
        disconnected_graph.add_node(1)
        # No edges - nodes are disconnected
        
        waypoints_data = [
            {'location': (0.0, 0.0, 0.0)},
            {'location': (10.0, 10.0, 0.0)}
        ]
        
        planner = AStarPathPlanner(disconnected_graph, waypoints_data)
        path = planner.find_path(0, 1)
        
        # Should return empty path
        self.assertEqual(len(path), 0)
    
    def test_find_nearest_node(self):
        """Test finding nearest waypoint node"""
        # Location close to waypoint at (2, 3)
        test_location = (2.1, 2.9, 0.0)
        nearest_idx = self.planner.find_nearest_node(test_location)
        
        # Should find node at index 17 (row 3, col 2 = 3*5 + 2 = 17)
        expected_idx = 17
        self.assertEqual(nearest_idx, expected_idx)
    
    def test_calculate_path_metrics(self):
        """Test path metrics calculation"""
        # Create a simple path
        path = [0, 1, 6, 11, 16, 21]  # Diagonal-ish path
        
        metrics = self.planner.calculate_path_metrics(path)
        
        # Verify metrics structure
        self.assertIn('total_distance', metrics)
        self.assertIn('smoothness', metrics)
        self.assertIn('num_waypoints', metrics)
        self.assertIn('direction_changes', metrics)
        
        # Verify reasonable values
        self.assertGreater(metrics['total_distance'], 0)
        self.assertEqual(metrics['num_waypoints'], len(path))
        self.assertGreaterEqual(metrics['smoothness'], 0)
        self.assertLessEqual(metrics['smoothness'], 1)
    
    def test_calculate_path_metrics_empty_path(self):
        """Test path metrics for empty path"""
        metrics = self.planner.calculate_path_metrics([])
        
        self.assertEqual(metrics['total_distance'], 0.0)
        self.assertEqual(metrics['num_waypoints'], 0)
    
    def test_calculate_path_metrics_single_point(self):
        """Test path metrics for single point"""
        metrics = self.planner.calculate_path_metrics([5])
        
        self.assertEqual(metrics['total_distance'], 0.0)
        self.assertEqual(metrics['num_waypoints'], 1)

class TestAStarIntegration(unittest.TestCase):
    """Integration tests for A* algorithm"""
    
    def test_optimal_path_finding(self):
        """Test that A* finds optimal paths"""
        # Create a graph where there are multiple paths but one is clearly shorter
        graph = nx.DiGraph()
        
        # Nodes: 0=start, 1=intermediate, 2=goal
        # Path 1: 0->1->2 (cost: 1+1=2)
        # Path 2: 0->2 (cost: 10)
        graph.add_edge(0, 1, weight=1.0)
        graph.add_edge(1, 2, weight=1.0)
        graph.add_edge(0, 2, weight=10.0)
        
        waypoints_data = [
            {'location': (0.0, 0.0, 0.0)},
            {'location': (1.0, 0.0, 0.0)},
            {'location': (2.0, 0.0, 0.0)}
        ]
        
        planner = AStarPathPlanner(graph, waypoints_data)
        path = planner.find_path(0, 2)
        
        # Should find the shorter path: 0->1->2
        expected_path = [0, 1, 2]
        self.assertEqual(path, expected_path)
    
    def test_path_reconstruction(self):
        """Test path reconstruction from came_from dictionary"""
        graph = nx.DiGraph()
        graph.add_edges_from([(0, 1), (1, 2), (2, 3)])
        
        waypoints_data = [
            {'location': (float(i), 0.0, 0.0)} for i in range(4)
        ]
        
        planner = AStarPathPlanner(graph, waypoints_data)
        
        # Test internal path reconstruction
        came_from = {1: 0, 2: 1, 3: 2}
        path = planner._reconstruct_path(came_from, 3)
        
        expected_path = [0, 1, 2, 3]
        self.assertEqual(path, expected_path)

class TestAStarPerformance(unittest.TestCase):
    """Performance tests for A* algorithm"""
    
    def test_large_graph_performance(self):
        """Test A* performance on larger graphs"""
        import time
        
        # Create a larger grid graph (20x20)
        size = 20
        graph = nx.DiGraph()
        
        # Add nodes
        for i in range(size * size):
            graph.add_node(i)
        
        # Add edges (4-connected grid)
        for i in range(size):
            for j in range(size):
                node_id = i * size + j
                
                # Right
                if j < size - 1:
                    graph.add_edge(node_id, node_id + 1, weight=1.0)
                
                # Down
                if i < size - 1:
                    graph.add_edge(node_id, node_id + size, weight=1.0)
                
                # Left
                if j > 0:
                    graph.add_edge(node_id, node_id - 1, weight=1.0)
                
                # Up
                if i > 0:
                    graph.add_edge(node_id, node_id - size, weight=1.0)
        
        # Create waypoint data
        waypoints_data = []
        for i in range(size):
            for j in range(size):
                waypoints_data.append({
                    'location': (float(j), float(i), 0.0)
                })
        
        planner = AStarPathPlanner(graph, waypoints_data)
        
        # Time the pathfinding
        start_time = time.time()
        path = planner.find_path(0, size * size - 1)  # Corner to corner
        planning_time = time.time() - start_time
        
        # Verify path found and performance
        self.assertGreater(len(path), 0)
        self.assertLess(planning_time, 1.0)  # Should complete in less than 1 second
        
        # Verify path optimality (Manhattan distance for grid)
        expected_min_length = (size - 1) + (size - 1) + 1  # +1 for the path length
        self.assertEqual(len(path), expected_min_length)

@patch('matplotlib.pyplot.show')
@patch('matplotlib.pyplot.savefig')
class TestAStarVisualization(unittest.TestCase):
    """Test A* visualization functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Simple 3x3 grid
        self.graph = nx.DiGraph()
        for i in range(9):
            self.graph.add_node(i)
        
        # Add grid connections
        connections = [(0,1), (1,2), (3,4), (4,5), (6,7), (7,8), 
                      (0,3), (3,6), (1,4), (4,7), (2,5), (5,8)]
        for start, end in connections:
            self.graph.add_edge(start, end, weight=1.0)
        
        self.waypoints_data = [
            {'location': (float(i%3), float(i//3), 0.0)} for i in range(9)
        ]
        
        self.planner = AStarPathPlanner(self.graph, self.waypoints_data)
    
    def test_visualize_path(self, mock_savefig, mock_show):
        """Test path visualization"""
        path = [0, 1, 4, 7, 8]  # Simple path
        
        # Should not raise any exceptions
        self.planner.visualize_path(path, save_path="test_output")
        
        # Verify plot was saved
        mock_savefig.assert_called()
        mock_show.assert_called()

def run_tests():
    """Run all A* tests"""
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test cases
    test_suite.addTest(unittest.makeSuite(TestPathNode))
    test_suite.addTest(unittest.makeSuite(TestAStarPathPlanner))
    test_suite.addTest(unittest.makeSuite(TestAStarIntegration))
    test_suite.addTest(unittest.makeSuite(TestAStarPerformance))
    test_suite.addTest(unittest.makeSuite(TestAStarVisualization))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    return result.wasSuccessful()

if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
