"""
Unit tests for road network extraction functionality
"""

import unittest
import numpy as np
import networkx as nx
from unittest.mock import Mock, MagicMock, patch
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from road_network_extractor import RoadNetworkExtractor

class TestRoadNetworkExtractor(unittest.TestCase):
    """Test cases for RoadNetworkExtractor class"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Mock Carla world and map
        self.mock_world = Mock()
        self.mock_map = Mock()
        self.mock_world.get_map.return_value = self.mock_map
        
        # Create test waypoints
        self.test_waypoints = self._create_test_waypoints()
        self.mock_map.generate_waypoints.return_value = self.test_waypoints
        
        # Create extractor instance
        self.extractor = RoadNetworkExtractor(self.mock_world)
    
    def _create_test_waypoints(self):
        """Create mock waypoints for testing"""
        waypoints = []
        
        # Create a simple straight line of waypoints
        for i in range(5):
            wp = Mock()
            wp.transform.location.x = float(i * 2)
            wp.transform.location.y = 0.0
            wp.transform.location.z = 0.0
            wp.transform.rotation.pitch = 0.0
            wp.transform.rotation.yaw = 0.0
            wp.transform.rotation.roll = 0.0
            wp.road_id = 1
            wp.lane_id = -1
            wp.lane_width = 3.5
            
            # Mock next waypoints
            if i < 4:
                next_wp = Mock()
                next_wp.transform.location.x = float((i + 1) * 2)
                next_wp.transform.location.y = 0.0
                next_wp.transform.location.z = 0.0
                wp.next.return_value = [next_wp]
            else:
                wp.next.return_value = []
            
            waypoints.append(wp)
        
        return waypoints
    
    def test_extract_waypoints(self):
        """Test waypoint extraction"""
        waypoints = self.extractor.extract_waypoints(distance=2.0)
        
        # Verify waypoints were extracted
        self.assertEqual(len(waypoints), 5)
        self.mock_map.generate_waypoints.assert_called_once_with(2.0)
        
        # Verify waypoints are stored
        self.assertEqual(len(self.extractor.waypoints), 5)
    
    def test_build_road_graph(self):
        """Test road graph construction"""
        # First extract waypoints
        self.extractor.extract_waypoints()
        
        # Build graph
        graph = self.extractor.build_road_graph()
        
        # Verify graph structure
        self.assertIsInstance(graph, nx.DiGraph)
        self.assertEqual(graph.number_of_nodes(), 5)
        
        # Verify nodes have correct attributes
        for i in range(5):
            self.assertIn(i, graph.nodes())
            node_data = graph.nodes[i]
            self.assertIn('location', node_data)
            self.assertIn('road_id', node_data)
            self.assertIn('lane_id', node_data)
    
    def test_find_closest_waypoint_index(self):
        """Test finding closest waypoint"""
        self.extractor.extract_waypoints()
        
        # Create a target waypoint close to waypoint 2
        target_wp = Mock()
        target_wp.transform.location.x = 4.1
        target_wp.transform.location.y = 0.1
        target_wp.transform.location.z = 0.0
        
        closest_idx = self.extractor._find_closest_waypoint_index(target_wp, threshold=1.0)
        
        # Should find waypoint at index 2 (x=4.0)
        self.assertEqual(closest_idx, 2)
    
    def test_find_closest_waypoint_index_no_match(self):
        """Test finding closest waypoint when none within threshold"""
        self.extractor.extract_waypoints()
        
        # Create a target waypoint far from all waypoints
        target_wp = Mock()
        target_wp.transform.location.x = 100.0
        target_wp.transform.location.y = 100.0
        target_wp.transform.location.z = 0.0
        
        closest_idx = self.extractor._find_closest_waypoint_index(target_wp, threshold=1.0)
        
        # Should return -1 (no match)
        self.assertEqual(closest_idx, -1)
    
    @patch('os.makedirs')
    @patch('pickle.dump')
    @patch('networkx.write_gpickle')
    @patch('builtins.open', create=True)
    def test_save_road_network(self, mock_open, mock_nx_write, mock_pickle, mock_makedirs):
        """Test saving road network data"""
        self.extractor.extract_waypoints()
        self.extractor.build_road_graph()
        
        # Mock file operations
        mock_file = MagicMock()
        mock_open.return_value.__enter__.return_value = mock_file
        
        # Save network
        self.extractor.save_road_network("test_output")
        
        # Verify directory creation
        mock_makedirs.assert_called_once_with("test_output", exist_ok=True)
        
        # Verify file operations
        self.assertEqual(mock_open.call_count, 1)  # waypoints.pkl
        mock_pickle.assert_called_once()
        mock_nx_write.assert_called_once()
    
    def test_visualize_network(self):
        """Test network visualization"""
        self.extractor.extract_waypoints()
        self.extractor.build_road_graph()
        
        # Mock debug drawing
        mock_debug = Mock()
        self.mock_world.debug = mock_debug
        
        # Visualize network
        self.extractor.visualize_network(debug_draw_time=10.0)
        
        # Verify debug drawing calls
        self.assertEqual(mock_debug.draw_point.call_count, 5)  # 5 waypoints
        # Note: draw_line calls depend on graph connectivity

class TestRoadGraphIntegration(unittest.TestCase):
    """Integration tests for road graph functionality"""
    
    def test_graph_connectivity(self):
        """Test that created graph has proper connectivity"""
        # Create a more complex waypoint structure
        waypoints = []
        
        # Create waypoints in a T-junction pattern
        positions = [(0, 0), (2, 0), (4, 0), (2, 2), (2, 4)]
        
        for i, (x, y) in enumerate(positions):
            wp = Mock()
            wp.transform.location.x = float(x)
            wp.transform.location.y = float(y)
            wp.transform.location.z = 0.0
            wp.road_id = 1
            wp.lane_id = -1
            wp.lane_width = 3.5
            
            # Set up next waypoints based on T-junction connectivity
            next_waypoints = []
            if i == 0:  # First waypoint connects to second
                next_wp = Mock()
                next_wp.transform.location.x = 2.0
                next_wp.transform.location.y = 0.0
                next_wp.transform.location.z = 0.0
                next_waypoints.append(next_wp)
            elif i == 1:  # Second waypoint connects to third and fourth
                for next_x, next_y in [(4, 0), (2, 2)]:
                    next_wp = Mock()
                    next_wp.transform.location.x = float(next_x)
                    next_wp.transform.location.y = float(next_y)
                    next_wp.transform.location.z = 0.0
                    next_waypoints.append(next_wp)
            elif i == 3:  # Fourth waypoint connects to fifth
                next_wp = Mock()
                next_wp.transform.location.x = 2.0
                next_wp.transform.location.y = 4.0
                next_wp.transform.location.z = 0.0
                next_waypoints.append(next_wp)
            
            wp.next.return_value = next_waypoints
            waypoints.append(wp)
        
        # Create extractor with mock world
        mock_world = Mock()
        mock_map = Mock()
        mock_world.get_map.return_value = mock_map
        mock_map.generate_waypoints.return_value = waypoints
        
        extractor = RoadNetworkExtractor(mock_world)
        extractor.extract_waypoints()
        graph = extractor.build_road_graph()
        
        # Verify graph properties
        self.assertEqual(graph.number_of_nodes(), 5)
        
        # Verify specific connections exist
        # Node 1 (center of T) should have highest degree
        center_node_degree = max(graph.degree(node) for node in graph.nodes())
        self.assertGreaterEqual(center_node_degree, 2)

class TestRoadGraphPerformance(unittest.TestCase):
    """Performance tests for road graph operations"""
    
    def test_large_waypoint_set_performance(self):
        """Test performance with large number of waypoints"""
        import time
        
        # Create a large set of waypoints (grid pattern)
        waypoints = []
        grid_size = 20  # 20x20 = 400 waypoints
        
        for i in range(grid_size):
            for j in range(grid_size):
                wp = Mock()
                wp.transform.location.x = float(i * 2)
                wp.transform.location.y = float(j * 2)
                wp.transform.location.z = 0.0
                wp.road_id = 1
                wp.lane_id = -1
                wp.lane_width = 3.5
                wp.next.return_value = []  # Simplified - no connections
                waypoints.append(wp)
        
        # Create extractor
        mock_world = Mock()
        mock_map = Mock()
        mock_world.get_map.return_value = mock_map
        mock_map.generate_waypoints.return_value = waypoints
        
        extractor = RoadNetworkExtractor(mock_world)
        
        # Time the extraction
        start_time = time.time()
        extractor.extract_waypoints()
        extraction_time = time.time() - start_time
        
        # Time the graph building
        start_time = time.time()
        graph = extractor.build_road_graph()
        graph_time = time.time() - start_time
        
        # Verify reasonable performance (should complete in reasonable time)
        self.assertLess(extraction_time, 1.0)  # Less than 1 second
        self.assertLess(graph_time, 5.0)       # Less than 5 seconds
        
        # Verify correct number of nodes
        self.assertEqual(graph.number_of_nodes(), grid_size * grid_size)

def run_tests():
    """Run all road graph tests"""
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test cases
    test_suite.addTest(unittest.makeSuite(TestRoadNetworkExtractor))
    test_suite.addTest(unittest.makeSuite(TestRoadGraphIntegration))
    test_suite.addTest(unittest.makeSuite(TestRoadGraphPerformance))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    return result.wasSuccessful()

if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
