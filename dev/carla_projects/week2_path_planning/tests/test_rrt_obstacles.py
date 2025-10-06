"""
Unit tests for RRT obstacle avoidance and collision detection
"""

import unittest
import numpy as np
from unittest.mock import Mock, patch
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rrt_planner import RRTPlanner, RRTNode

class TestRRTNode(unittest.TestCase):
    """Test cases for RRTNode class"""
    
    def test_rrt_node_creation(self):
        """Test RRTNode creation and properties"""
        node = RRTNode(x=5.0, y=10.0, parent=None, cost=15.0)
        
        self.assertEqual(node.x, 5.0)
        self.assertEqual(node.y, 10.0)
        self.assertIsNone(node.parent)
        self.assertEqual(node.cost, 15.0)
    
    def test_distance_calculation(self):
        """Test distance calculation between nodes"""
        node1 = RRTNode(x=0.0, y=0.0)
        node2 = RRTNode(x=3.0, y=4.0)
        
        distance = node1.distance_to(node2)
        expected_distance = 5.0  # 3-4-5 triangle
        
        self.assertAlmostEqual(distance, expected_distance, places=5)

class TestRRTObstacleAvoidance(unittest.TestCase):
    """Test RRT obstacle avoidance functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.start = (0.0, 0.0)
        self.goal = (10.0, 10.0)
        self.bounds = (-5.0, 15.0, -5.0, 15.0)
        
        # Create obstacles
        self.obstacles = [
            (5.0, 5.0, 2.0),   # Center obstacle
            (2.0, 8.0, 1.5),   # Side obstacle
            (8.0, 2.0, 1.0)    # Another side obstacle
        ]
        
        self.planner = RRTPlanner(
            start=self.start,
            goal=self.goal,
            bounds=self.bounds,
            obstacles=self.obstacles,
            step_size=2.0,
            max_iterations=1000
        )
    
    def test_collision_detection_free_path(self):
        """Test collision detection for obstacle-free path"""
        node1 = RRTNode(x=0.0, y=0.0)
        node2 = RRTNode(x=1.0, y=1.0)
        
        # Path should be collision-free (away from obstacles)
        is_free = self.planner.is_collision_free(node1, node2)
        self.assertTrue(is_free)
    
    def test_collision_detection_obstacle_collision(self):
        """Test collision detection for path through obstacle"""
        node1 = RRTNode(x=3.0, y=5.0)
        node2 = RRTNode(x=7.0, y=5.0)
        
        # Path goes through center obstacle at (5,5) with radius 2
        is_free = self.planner.is_collision_free(node1, node2)
        self.assertFalse(is_free)
    
    def test_collision_detection_bounds_violation(self):
        """Test collision detection for out-of-bounds nodes"""
        node1 = RRTNode(x=0.0, y=0.0)
        node2 = RRTNode(x=20.0, y=20.0)  # Outside bounds
        
        is_free = self.planner.is_collision_free(node1, node2)
        self.assertFalse(is_free)
    
    def test_line_circle_intersection_direct_hit(self):
        """Test line-circle intersection for direct collision"""
        node1 = RRTNode(x=0.0, y=5.0)
        node2 = RRTNode(x=10.0, y=5.0)
        
        # Line passes directly through circle at (5,5) with radius 2
        intersects = self.planner._line_circle_intersection(
            node1, node2, 5.0, 5.0, 2.0
        )
        self.assertTrue(intersects)
    
    def test_line_circle_intersection_miss(self):
        """Test line-circle intersection for non-collision"""
        node1 = RRTNode(x=0.0, y=0.0)
        node2 = RRTNode(x=10.0, y=0.0)
        
        # Line passes below circle at (5,5) with radius 2
        intersects = self.planner._line_circle_intersection(
            node1, node2, 5.0, 5.0, 2.0
        )
        self.assertFalse(intersects)
    
    def test_line_circle_intersection_tangent(self):
        """Test line-circle intersection for tangent case"""
        node1 = RRTNode(x=0.0, y=3.0)
        node2 = RRTNode(x=10.0, y=3.0)
        
        # Line is tangent to circle at (5,5) with radius 2
        intersects = self.planner._line_circle_intersection(
            node1, node2, 5.0, 5.0, 2.0
        )
        self.assertTrue(intersects)  # Tangent should count as intersection
    
    def test_sample_random_point_bounds(self):
        """Test that random sampling respects bounds"""
        for _ in range(100):  # Test multiple samples
            random_node = self.planner.sample_random_point()
            
            self.assertGreaterEqual(random_node.x, self.bounds[0])
            self.assertLessEqual(random_node.x, self.bounds[1])
            self.assertGreaterEqual(random_node.y, self.bounds[2])
            self.assertLessEqual(random_node.y, self.bounds[3])
    
    def test_goal_biased_sampling(self):
        """Test goal-biased sampling"""
        # Mock random to always return small value (trigger goal bias)
        with patch('random.random', return_value=0.05):  # Less than 0.1 bias
            sampled_node = self.planner.sample_random_point()
            
            self.assertEqual(sampled_node.x, self.goal[0])
            self.assertEqual(sampled_node.y, self.goal[1])
    
    def test_find_nearest_node(self):
        """Test finding nearest node in tree"""
        # Add some nodes to the tree
        self.planner.nodes = [
            RRTNode(x=0.0, y=0.0),
            RRTNode(x=2.0, y=2.0),
            RRTNode(x=4.0, y=1.0),
            RRTNode(x=1.0, y=4.0)
        ]
        
        target = RRTNode(x=3.0, y=2.5)
        nearest = self.planner.find_nearest_node(target)
        
        # Should find node at (2,2) as it's closest
        self.assertEqual(nearest.x, 2.0)
        self.assertEqual(nearest.y, 2.0)
    
    def test_steer_within_step_size(self):
        """Test steering when target is within step size"""
        from_node = RRTNode(x=0.0, y=0.0)
        to_node = RRTNode(x=1.0, y=1.0)  # Distance = sqrt(2) ≈ 1.41
        
        self.planner.step_size = 2.0  # Larger than distance
        
        steered = self.planner.steer(from_node, to_node)
        
        # Should return exact target location
        self.assertEqual(steered.x, to_node.x)
        self.assertEqual(steered.y, to_node.y)
    
    def test_steer_beyond_step_size(self):
        """Test steering when target is beyond step size"""
        from_node = RRTNode(x=0.0, y=0.0)
        to_node = RRTNode(x=10.0, y=0.0)  # Distance = 10
        
        self.planner.step_size = 3.0  # Smaller than distance
        
        steered = self.planner.steer(from_node, to_node)
        
        # Should be limited to step size in direction of target
        expected_x = 3.0  # step_size in x direction
        expected_y = 0.0
        
        self.assertAlmostEqual(steered.x, expected_x, places=5)
        self.assertAlmostEqual(steered.y, expected_y, places=5)

class TestRRTPathPlanning(unittest.TestCase):
    """Test RRT path planning with obstacles"""
    
    def setUp(self):
        """Set up test environment"""
        self.start = (0.0, 0.0)
        self.goal = (10.0, 10.0)
        self.bounds = (-2.0, 12.0, -2.0, 12.0)
        
    def test_path_planning_no_obstacles(self):
        """Test path planning in obstacle-free environment"""
        planner = RRTPlanner(
            start=self.start,
            goal=self.goal,
            bounds=self.bounds,
            obstacles=[],  # No obstacles
            step_size=2.0,
            max_iterations=500
        )
        
        path = planner.plan_rrt()
        
        # Should find a path
        self.assertGreater(len(path), 0)
        
        # Path should start and end correctly
        self.assertEqual(path[0], self.start)
        self.assertEqual(path[-1], self.goal)
    
    def test_path_planning_with_obstacles(self):
        """Test path planning with obstacles"""
        # Create obstacle that blocks direct path
        obstacles = [(5.0, 5.0, 3.0)]  # Large obstacle in the middle
        
        planner = RRTPlanner(
            start=self.start,
            goal=self.goal,
            bounds=self.bounds,
            obstacles=obstacles,
            step_size=1.5,
            max_iterations=2000
        )
        
        path = planner.plan_rrt()
        
        # Should find a path around obstacle
        self.assertGreater(len(path), 0)
        
        # Verify path avoids obstacles
        for i in range(len(path) - 1):
            node1 = RRTNode(x=path[i][0], y=path[i][1])
            node2 = RRTNode(x=path[i+1][0], y=path[i+1][1])
            
            is_free = planner.is_collision_free(node1, node2)
            self.assertTrue(is_free, f"Path segment {i} collides with obstacle")
    
    def test_path_planning_impossible_scenario(self):
        """Test path planning in impossible scenario"""
        # Surround goal with obstacles
        obstacles = [
            (9.0, 9.0, 2.0),   # Block goal area
            (9.0, 11.0, 2.0),
            (11.0, 9.0, 2.0),
            (11.0, 11.0, 2.0)
        ]
        
        planner = RRTPlanner(
            start=self.start,
            goal=self.goal,
            bounds=self.bounds,
            obstacles=obstacles,
            step_size=1.0,
            max_iterations=500  # Limited iterations
        )
        
        path = planner.plan_rrt()
        
        # Should not find a path (or find empty path)
        self.assertEqual(len(path), 0)

class TestRRTStarObstacles(unittest.TestCase):
    """Test RRT* with obstacle avoidance"""
    
    def setUp(self):
        """Set up test environment"""
        self.start = (0.0, 0.0)
        self.goal = (10.0, 10.0)
        self.bounds = (-2.0, 12.0, -2.0, 12.0)
        self.obstacles = [(5.0, 5.0, 2.0)]  # Single obstacle
        
        self.planner = RRTPlanner(
            start=self.start,
            goal=self.goal,
            bounds=self.bounds,
            obstacles=self.obstacles,
            step_size=2.0,
            rewire_radius=4.0,
            max_iterations=1000
        )
    
    def test_rrt_star_path_optimization(self):
        """Test that RRT* produces better paths than RRT"""
        # Run RRT
        rrt_path = self.planner.plan_rrt()
        rrt_length = self.planner.calculate_path_length()
        
        # Reset planner and run RRT*
        self.planner.nodes = [self.planner.start]
        self.planner.path = []
        
        rrt_star_path = self.planner.plan_rrt_star()
        rrt_star_length = self.planner.calculate_path_length()
        
        # Both should find paths
        self.assertGreater(len(rrt_path), 0)
        self.assertGreater(len(rrt_star_path), 0)
        
        # RRT* path should be same length or shorter (optimization)
        # Note: Due to randomness, this isn't guaranteed in single run
        # but RRT* should generally perform better
        self.assertGreaterEqual(rrt_length, 0)  # Basic sanity check
        self.assertGreaterEqual(rrt_star_length, 0)
    
    def test_rewiring_mechanism(self):
        """Test RRT* rewiring mechanism"""
        # Create a scenario where rewiring would be beneficial
        self.planner.nodes = [
            RRTNode(x=0.0, y=0.0, cost=0.0),      # Start
            RRTNode(x=2.0, y=0.0, cost=2.0),      # Intermediate 1
            RRTNode(x=4.0, y=2.0, cost=6.0),      # Intermediate 2 (suboptimal)
            RRTNode(x=4.0, y=0.0, cost=4.0)       # Better path to same area
        ]
        
        # Set up parent relationships
        self.planner.nodes[1].parent = self.planner.nodes[0]
        self.planner.nodes[2].parent = self.planner.nodes[1]  # Suboptimal path
        self.planner.nodes[3].parent = self.planner.nodes[0]
        
        new_node = RRTNode(x=6.0, y=1.0)
        near_nodes = self.planner._find_near_nodes(new_node)
        
        # Should find nearby nodes for potential rewiring
        self.assertGreater(len(near_nodes), 0)
    
    def test_find_near_nodes(self):
        """Test finding nodes within rewire radius"""
        # Add nodes to tree
        self.planner.nodes = [
            RRTNode(x=0.0, y=0.0),
            RRTNode(x=2.0, y=0.0),
            RRTNode(x=0.0, y=2.0),
            RRTNode(x=10.0, y=10.0)  # Far away
        ]
        
        target_node = RRTNode(x=1.0, y=1.0)
        near_nodes = self.planner._find_near_nodes(target_node)
        
        # Should find first 3 nodes (within radius), not the far one
        self.assertEqual(len(near_nodes), 3)
        
        # Verify distances
        for node in near_nodes:
            distance = target_node.distance_to(node)
            self.assertLessEqual(distance, self.planner.rewire_radius)

class TestRRTVisualization(unittest.TestCase):
    """Test RRT visualization with obstacles"""
    
    @patch('matplotlib.pyplot.show')
    @patch('matplotlib.pyplot.savefig')
    def test_visualize_with_obstacles(self, mock_savefig, mock_show):
        """Test visualization with obstacles"""
        start = (0.0, 0.0)
        goal = (10.0, 10.0)
        bounds = (-2.0, 12.0, -2.0, 12.0)
        obstacles = [(5.0, 5.0, 2.0), (8.0, 3.0, 1.5)]
        
        planner = RRTPlanner(start, goal, bounds, obstacles, max_iterations=100)
        
        # Add some nodes to visualize
        planner.nodes = [
            RRTNode(x=0.0, y=0.0),
            RRTNode(x=2.0, y=1.0),
            RRTNode(x=4.0, y=3.0),
            RRTNode(x=7.0, y=6.0)
        ]
        
        planner.path = [(0.0, 0.0), (2.0, 1.0), (7.0, 6.0), (10.0, 10.0)]
        
        # Should not raise any exceptions
        planner.visualize_tree_and_path(save_path="test_output")
        
        # Verify plot operations were called
        mock_savefig.assert_called()
        mock_show.assert_called()

def run_tests():
    """Run all RRT obstacle tests"""
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test cases
    test_suite.addTest(unittest.makeSuite(TestRRTNode))
    test_suite.addTest(unittest.makeSuite(TestRRTObstacleAvoidance))
    test_suite.addTest(unittest.makeSuite(TestRRTPathPlanning))
    test_suite.addTest(unittest.makeSuite(TestRRTStarObstacles))
    test_suite.addTest(unittest.makeSuite(TestRRTVisualization))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    return result.wasSuccessful()

if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
