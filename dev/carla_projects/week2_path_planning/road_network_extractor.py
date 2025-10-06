"""
Day 8 - Road Network Extractor
Extracts waypoints and creates road graph from Carla map
Supports both dense waypoint and topology-based graphs
"""

import carla
import numpy as np
import networkx as nx
from typing import List, Dict, Tuple
import pickle
import os
import argparse

class RoadNetworkExtractor:
    """Extract road network and waypoints from Carla map"""
    
    def __init__(self, world: carla.World):
        self.world = world
        self.map = world.get_map()
        self.waypoints = []
        self.road_graph = nx.DiGraph()
        
    def extract_waypoints(self, distance: float = 2.0) -> List[carla.Waypoint]:
        """Extract waypoints from the map at specified distance intervals"""
        print("Extracting waypoints from map...")
        
        # Get all waypoints at specified distance
        waypoint_list = self.map.generate_waypoints(distance)
        self.waypoints = waypoint_list
        
        print(f"Extracted {len(waypoint_list)} waypoints")
        return waypoint_list
    
    def build_road_graph(self) -> nx.DiGraph:
        """Build a directed graph from waypoints"""
        print("Building dense waypoint graph...")
        
        if not self.waypoints:
            self.extract_waypoints()
            
        # Add nodes to graph with serializable data only
        for i, wp in enumerate(self.waypoints):
            self.road_graph.add_node(i, 
                                   location=(wp.transform.location.x, 
                                           wp.transform.location.y, 
                                           wp.transform.location.z),
                                   rotation=(wp.transform.rotation.pitch,
                                            wp.transform.rotation.yaw,
                                            wp.transform.rotation.roll),
                                   road_id=wp.road_id,
                                   lane_id=wp.lane_id,
                                   lane_width=wp.lane_width)
        
        # Add edges based on next waypoints
        for i, wp in enumerate(self.waypoints):
            next_waypoints = wp.next(2.0)
            for next_wp in next_waypoints:
                # Find closest waypoint in our list
                closest_idx = self._find_closest_waypoint_index(next_wp)
                if closest_idx != -1 and closest_idx != i:
                    distance = np.linalg.norm([
                        wp.transform.location.x - next_wp.transform.location.x,
                        wp.transform.location.y - next_wp.transform.location.y
                    ])
                    self.road_graph.add_edge(i, closest_idx, weight=distance)
        
        print(f"Built dense graph with {self.road_graph.number_of_nodes()} nodes and {self.road_graph.number_of_edges()} edges")
        return self.road_graph
    
    def build_topology_graph(self) -> nx.DiGraph:
        """Build a graph from the map's topology (higher level than dense waypoints)"""
        print("Building topology graph...")
        topology = self.map.get_topology()
        G = nx.DiGraph()
        
        for start_wp, end_wp in topology:
            # Create unique node IDs
            start_id = f"{start_wp.road_id}_{start_wp.lane_id}_{start_wp.s:.2f}"
            end_id = f"{end_wp.road_id}_{end_wp.lane_id}_{end_wp.s:.2f}"
            
            # Add nodes with metadata
            if start_id not in G:
                G.add_node(start_id, 
                          location=(start_wp.transform.location.x,
                                  start_wp.transform.location.y,
                                  start_wp.transform.location.z),
                          rotation=(start_wp.transform.rotation.pitch,
                                  start_wp.transform.rotation.yaw,
                                  start_wp.transform.rotation.roll),
                          road_id=start_wp.road_id,
                          lane_id=start_wp.lane_id,
                          lane_width=start_wp.lane_width)
            
            # Add edge with distance
            distance = start_wp.transform.location.distance(end_wp.transform.location)
            G.add_edge(start_id, end_id, weight=distance)
        
        print(f"Built topology graph with {G.number_of_nodes()} nodes and {G.number_of_edges()} edges")
        return G
    
    def compare_graphs(self):
        """Compare the two graph building approaches"""
        print("\n=== Graph Comparison ===")
        
        # Build both graphs
        print("\nBuilding dense waypoint graph...")
        dense_graph = self.build_road_graph()
        print("\nBuilding topology graph...")
        topology_graph = self.build_topology_graph()
        
        # Compare basic stats
        print("\nComparison Results:")
        print(f"{'Metric':<25} {'Dense':<10} {'Topology':<10}")
        print("-" * 45)
        print(f"{'Number of Nodes':<25} {dense_graph.number_of_nodes():<10} {topology_graph.number_of_nodes():<10}")
        print(f"{'Number of Edges':<25} {dense_graph.number_of_edges():<10} {topology_graph.number_of_edges():<10}")
        
        # Calculate average degree
        dense_degree = sum(dict(dense_graph.degree()).values()) / dense_graph.number_of_nodes()
        topo_degree = sum(dict(topology_graph.degree()).values()) / topology_graph.number_of_nodes()
        print(f"{'Average Degree':<25} {dense_degree:<10.2f} {topo_degree:<10.2f}")
        
        # Memory usage comparison
        import sys
        dense_size = sys.getsizeof(pickle.dumps(dense_graph))
        topo_size = sys.getsizeof(pickle.dumps(topology_graph))
        print(f"{'Memory (MB)':<25} {dense_size/1e6:<10.2f} {topo_size/1e6:<10.2f}")

    def _find_closest_waypoint_index(self, target_wp: carla.Waypoint, threshold: float = 1.0) -> int:
        """Find the closest waypoint index in our waypoint list"""
        min_distance = float('inf')
        closest_idx = -1
        
        for i, wp in enumerate(self.waypoints):
            distance = np.linalg.norm([
                wp.transform.location.x - target_wp.transform.location.x,
                wp.transform.location.y - target_wp.transform.location.y
            ])
            
            if distance < min_distance and distance < threshold:
                min_distance = distance
                closest_idx = i
                
        return closest_idx
    
    def save_road_network(self, output_dir: str = "output_data/day8_road_graphs"):
        """Save waypoints and road graph to files"""
        os.makedirs(output_dir, exist_ok=True)
        
        # Convert graph to a serializable format
        graph_data = {
            'nodes': {},
            'edges': list(self.road_graph.edges(data=True))
        }
        
        for node_id, node_data in self.road_graph.nodes(data=True):
            graph_data['nodes'][node_id] = node_data
        
        # Save the graph data
        with open(f"{output_dir}/road_network.pkl", 'wb') as f:
            pickle.dump(graph_data, f, protocol=pickle.HIGHEST_PROTOCOL)
        
        print(f"Saved road network data to {output_dir}/")
    
    def visualize_network(self, debug_draw_time: float = 60.0):
        """Visualize the road network using Carla's debug drawing"""
        debug = self.world.debug
        
        # Draw waypoints
        for wp in self.waypoints:
            debug.draw_point(wp.transform.location, 
                           size=0.1, 
                           color=carla.Color(0, 255, 0), 
                           life_time=debug_draw_time)
        
        # Draw connections
        for edge in self.road_graph.edges():
            start_wp = self.waypoints[edge[0]]
            end_wp = self.waypoints[edge[1]]
            
            debug.draw_line(start_wp.transform.location,
                          end_wp.transform.location,
                          thickness=0.05,
                          color=carla.Color(255, 0, 0),
                          life_time=debug_draw_time)

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Extract and analyze Carla road networks')
    parser.add_argument('--graph-type', type=str, choices=['dense', 'topology', 'compare'], 
                       default='dense', help='Type of graph to build (default: dense)')
    parser.add_argument('--distance', type=float, default=2.0,
                       help='Distance between waypoints for dense graph (default: 2.0)')
    return parser.parse_args()

def main():
    """Main function to test road network extraction"""
    args = parse_arguments()
    
    try:
        # Connect to Carla
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        # Initialize extractor
        extractor = RoadNetworkExtractor(world)
        
        if args.graph_type == 'compare':
            extractor.compare_graphs()
        else:
            if args.graph_type == 'dense':
                print(f"Building dense waypoint graph (distance: {args.distance}m)")
                extractor.extract_waypoints(distance=args.distance)
                graph = extractor.build_road_graph()
            else:  # topology
                print("Building topology graph")
                graph = extractor.build_topology_graph()
            
            # Save and visualize
            output_dir = f"output_data/day8_{args.graph_type}_graph"
            os.makedirs(output_dir, exist_ok=True)
            
            # Save the graph
            graph_data = {
                'nodes': dict(graph.nodes(data=True)),
                'edges': list(graph.edges(data=True))
            }
            with open(f"{output_dir}/road_network.pkl", 'wb') as f:
                pickle.dump(graph_data, f, protocol=pickle.HIGHEST_PROTOCOL)
            
            print(f"Saved {args.graph_type} graph to {output_dir}/")
            
            # Visualize if it's the dense graph
            if args.graph_type == 'dense':
                extractor.visualize_network()
        
        print("\nRoad network extraction completed successfully!")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()