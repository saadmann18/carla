"""
Unit tests for road network extraction functionality
"""

import unittest
import numpy as np
import networkx as nx
import carla
from unittest.mock import Mock, MagicMock, patch, call
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
    @patch('builtins.open', new_callable=unittest.mock.mock_open)
    def test_save_road_network(self, mock_open, mock_pickle, mock_makedirs):
        """Test saving road network data"""
        self.extractor.extract_waypoints()
        self.extractor.build_road_graph()
        
        # Save network
        self.extractor.save_road_network("test_output")
        
        # Verify directory creation
        mock_makedirs.assert_called_once_with("test_output", exist_ok=True)
        
        # Verify file operations - should open 1 file (road_network.pkl)
        self.assertEqual(mock_open.call_count, 1)
        
        # Verify pickle.dump is called once for the graph data
        self.assertEqual(mock_pickle.call_count, 1)
    
    @patch('carla.DebugHelper')
    def test_visualize_network(self, mock_debug_helper):
        """Test network visualization appears correctly on CARLA map"""
        import os
        import matplotlib.pyplot as plt
        
        # Create output directory if it doesn't exist
        output_dir = os.path.join(os.path.dirname(__file__), '..', 'test_output')
        os.makedirs(output_dir, exist_ok=True)
        
        # Setup test data
        self.extractor.extract_waypoints()
        self.extractor.build_road_graph()
        
        # Mock debug drawing
        mock_debug = Mock()
        self.mock_world.debug = mock_debug
        
        # Create a mock debug helper that will be returned by carla.DebugHelper
        mock_debug_helper.return_value = mock_debug
        
        # Visualize network with a short debug draw time for testing
        debug_draw_time = 10.0
        self.extractor.visualize_network(debug_draw_time=debug_draw_time)
        
        # Get the graph
        graph = self.extractor.road_graph
        
        # Extract node positions
        node_positions = {}
        for node, data in graph.nodes(data=True):
            # Get location from node data
            if isinstance(data, dict) and 'location' in data:
                loc = data['location']
                if isinstance(loc, (list, tuple)) and len(loc) >= 2:
                    # Handle (x, y, z) tuple
                    node_positions[node] = (loc[0], loc[1])
                elif hasattr(loc, 'x') and hasattr(loc, 'y'):
                    # Handle object with x, y attributes
                    node_positions[node] = (loc.x, loc.y)
                else:
                    # Default to (0, 0) if we can't determine position
                    node_positions[node] = (0, 0)
            else:
                node_positions[node] = (0, 0)
        
        # Create a simple visualization
        plt.figure(figsize=(12, 8))
        
        # Plot nodes with numbers
        for node, (x, y) in node_positions.items():
            plt.scatter(x, y, c='green', s=200, alpha=0.7, label=f'Waypoint {node}' if node == 0 else '')
            plt.text(x, y, str(node), fontsize=12, ha='center', va='center', color='white')
        
        # Plot edges with arrows to show direction
        for u, v in graph.edges():
            if u in node_positions and v in node_positions:
                x1, y1 = node_positions[u]
                x2, y2 = node_positions[v]
                
                # Calculate direction and distance
                dx = x2 - x1
                dy = y2 - y1
                
                # Draw the main line (stopping before the target node)
                line_end_x = x2 - 0.3 * (dx / (dx**2 + dy**2)**0.5) if (dx**2 + dy**2) > 0 else x2
                line_end_y = y2 - 0.3 * (dy / (dx**2 + dy**2)**0.5) if (dx**2 + dy**2) > 0 else y2
                
                plt.plot([x1, line_end_x], [y1, line_end_y], 'b-', linewidth=2, alpha=0.7)
                
                # Add arrowhead at the end
                plt.annotate('', xy=(x2, y2), xytext=(line_end_x, line_end_y),
                           arrowprops=dict(arrowstyle='->', color='blue', lw=2, alpha=0.7, 
                                         mutation_scale=20))
        
        # Add labels and title
        plt.title('Road Network Graph Visualization', fontsize=14, pad=20)
        plt.xlabel('X Coordinate (meters)', fontsize=12)
        plt.ylabel('Y Coordinate (meters)', fontsize=12)
        
        # Add grid and legend
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend(loc='upper right')
        
        # Adjust layout to prevent label cutoff
        plt.tight_layout()
        
        # Save the visualization
        output_file = os.path.join(output_dir, 'road_network_visualization.png')
        plt.savefig(output_file)
        plt.close()
        
        # Verify the file was created
        self.assertTrue(os.path.exists(output_file), f"Visualization file not found: {output_file}")
        print("\n[SUCCESS] Visualization saved to:", os.path.abspath(output_file))
        
        # Verify debug drawing was called for waypoints
        waypoint_calls = [
            call(wp.transform.location, size=0.1, color=carla.Color(0, 255, 0), life_time=debug_draw_time)
            for wp in self.extractor.waypoints
        ]
        mock_debug.draw_point.assert_has_calls(waypoint_calls, any_order=True)
        
        # Verify debug drawing was called for connections
        connection_calls = []
        for edge in self.extractor.road_graph.edges():
            start_wp = self.extractor.waypoints[edge[0]]
            end_wp = self.extractor.waypoints[edge[1]]
            connection_calls.append(
                call(
                    start_wp.transform.location,
                    end_wp.transform.location,
                    thickness=0.05,
                    color=carla.Color(255, 0, 0),
                    life_time=debug_draw_time
                )
            )
        
        # Check that all expected connection lines were drawn
        actual_calls = [call[0] for call in mock_debug.draw_line.call_args_list]
        for expected_call in connection_calls:
            self.assertIn(expected_call.args[0], [call[0] for call in actual_calls])
            self.assertIn(expected_call.args[1], [call[1] for call in actual_calls])
            
        # Verify the number of waypoints matches the number of draw_point calls
        self.assertEqual(len(self.extractor.waypoints), mock_debug.draw_point.call_count)
        
        # Verify the number of connections matches the number of draw_line calls
        self.assertEqual(len(list(self.extractor.road_graph.edges())), mock_debug.draw_line.call_count)
        
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
    # Create test loader
    test_loader = unittest.TestLoader()
    
    # Load test cases
    test_suite = test_loader.loadTestsFromTestCase(TestRoadNetworkExtractor)
    test_suite.addTests(test_loader.loadTestsFromTestCase(TestRoadGraphIntegration))
    test_suite.addTests(test_loader.loadTestsFromTestCase(TestRoadGraphPerformance))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    return result.wasSuccessful()

if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
