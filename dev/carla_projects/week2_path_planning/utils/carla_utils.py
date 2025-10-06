"""
Carla utility functions for connection, spawning, and transform helpers
"""

import carla
import random
import time
import numpy as np
from typing import List, Optional, Tuple, Dict
import logging

class CarlaConnection:
    """Manages connection to Carla simulator"""
    
    def __init__(self, host: str = 'localhost', port: int = 2000, timeout: float = 10.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.client = None
        self.world = None
        self.logger = logging.getLogger(__name__)
        
    def connect(self) -> bool:
        """Connect to Carla server"""
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(self.timeout)
            self.world = self.client.get_world()
            
            # Test connection
            _ = self.world.get_map()
            
            self.logger.info(f"Connected to Carla server at {self.host}:{self.port}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to connect to Carla: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Carla server"""
        self.client = None
        self.world = None
        self.logger.info("Disconnected from Carla server")
    
    def is_connected(self) -> bool:
        """Check if connected to Carla"""
        try:
            if self.world is not None:
                _ = self.world.get_map()
                return True
        except:
            pass
        return False
    
    def get_available_maps(self) -> List[str]:
        """Get list of available maps"""
        if not self.is_connected():
            return []
        
        try:
            return self.client.get_available_maps()
        except Exception as e:
            self.logger.error(f"Failed to get available maps: {e}")
            return []
    
    def load_map(self, map_name: str) -> bool:
        """Load a specific map"""
        if not self.is_connected():
            return False
        
        try:
            self.world = self.client.load_world(map_name)
            self.logger.info(f"Loaded map: {map_name}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to load map {map_name}: {e}")
            return False

class CarlaVehicleManager:
    """Manages vehicle spawning and control in Carla"""
    
    def __init__(self, world: carla.World):
        self.world = world
        self.blueprint_library = world.get_blueprint_library()
        self.spawned_vehicles = []
        self.logger = logging.getLogger(__name__)
    
    def get_vehicle_blueprints(self, filter_pattern: str = "vehicle.*") -> List[carla.ActorBlueprint]:
        """Get available vehicle blueprints"""
        return self.blueprint_library.filter(filter_pattern)
    
    def spawn_vehicle(self, blueprint_name: str = "vehicle.tesla.model3", 
                     spawn_point: Optional[carla.Transform] = None,
                     autopilot: bool = False) -> Optional[carla.Vehicle]:
        """Spawn a vehicle at specified location"""
        try:
            # Get blueprint
            vehicle_bp = self.blueprint_library.find(blueprint_name)
            if vehicle_bp is None:
                # Try to find similar blueprint
                vehicle_bps = self.get_vehicle_blueprints()
                if vehicle_bps:
                    vehicle_bp = random.choice(vehicle_bps)
                else:
                    self.logger.error("No vehicle blueprints available")
                    return None
            
            # Get spawn point
            if spawn_point is None:
                spawn_points = self.world.get_map().get_spawn_points()
                if not spawn_points:
                    self.logger.error("No spawn points available")
                    return None
                spawn_point = random.choice(spawn_points)
            
            # Spawn vehicle
            vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            self.spawned_vehicles.append(vehicle)
            
            # Set autopilot if requested
            if autopilot:
                vehicle.set_autopilot(True)
            
            self.logger.info(f"Spawned vehicle {vehicle.id} at {spawn_point.location}")
            return vehicle
            
        except Exception as e:
            self.logger.error(f"Failed to spawn vehicle: {e}")
            return None
    
    def spawn_multiple_vehicles(self, count: int, blueprint_pattern: str = "vehicle.*",
                              autopilot: bool = True) -> List[carla.Vehicle]:
        """Spawn multiple vehicles for traffic simulation"""
        vehicles = []
        spawn_points = self.world.get_map().get_spawn_points()
        vehicle_bps = self.get_vehicle_blueprints(blueprint_pattern)
        
        if not spawn_points or not vehicle_bps:
            self.logger.error("No spawn points or vehicle blueprints available")
            return vehicles
        
        # Shuffle spawn points to avoid clustering
        random.shuffle(spawn_points)
        
        for i in range(min(count, len(spawn_points))):
            try:
                vehicle_bp = random.choice(vehicle_bps)
                vehicle = self.world.spawn_actor(vehicle_bp, spawn_points[i])
                
                if autopilot:
                    vehicle.set_autopilot(True)
                
                vehicles.append(vehicle)
                self.spawned_vehicles.append(vehicle)
                
            except Exception as e:
                self.logger.warning(f"Failed to spawn vehicle {i}: {e}")
        
        self.logger.info(f"Spawned {len(vehicles)} vehicles")
        return vehicles
    
    def destroy_vehicle(self, vehicle: carla.Vehicle):
        """Destroy a specific vehicle"""
        try:
            vehicle.destroy()
            if vehicle in self.spawned_vehicles:
                self.spawned_vehicles.remove(vehicle)
            self.logger.info(f"Destroyed vehicle {vehicle.id}")
        except Exception as e:
            self.logger.error(f"Failed to destroy vehicle: {e}")
    
    def destroy_all_vehicles(self):
        """Destroy all spawned vehicles"""
        for vehicle in self.spawned_vehicles[:]:  # Copy list to avoid modification during iteration
            self.destroy_vehicle(vehicle)
        self.spawned_vehicles.clear()
        self.logger.info("Destroyed all spawned vehicles")
    
    def get_vehicle_state(self, vehicle: carla.Vehicle) -> Dict:
        """Get comprehensive vehicle state information"""
        try:
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            acceleration = vehicle.get_acceleration()
            control = vehicle.get_control()
            
            return {
                'id': vehicle.id,
                'location': {
                    'x': transform.location.x,
                    'y': transform.location.y,
                    'z': transform.location.z
                },
                'rotation': {
                    'pitch': transform.rotation.pitch,
                    'yaw': transform.rotation.yaw,
                    'roll': transform.rotation.roll
                },
                'velocity': {
                    'x': velocity.x,
                    'y': velocity.y,
                    'z': velocity.z,
                    'magnitude': np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                },
                'acceleration': {
                    'x': acceleration.x,
                    'y': acceleration.y,
                    'z': acceleration.z
                },
                'control': {
                    'throttle': control.throttle,
                    'steer': control.steer,
                    'brake': control.brake,
                    'hand_brake': control.hand_brake,
                    'reverse': control.reverse
                }
            }
        except Exception as e:
            self.logger.error(f"Failed to get vehicle state: {e}")
            return {}

class CarlaWorldManager:
    """Manages Carla world settings and environment"""
    
    def __init__(self, world: carla.World):
        self.world = world
        self.original_settings = None
        self.logger = logging.getLogger(__name__)
    
    def set_synchronous_mode(self, enabled: bool, fixed_delta_seconds: float = 0.05):
        """Set synchronous mode for deterministic simulation"""
        try:
            settings = self.world.get_settings()
            
            # Store original settings
            if self.original_settings is None:
                self.original_settings = settings
            
            settings.synchronous_mode = enabled
            if enabled:
                settings.fixed_delta_seconds = fixed_delta_seconds
            
            self.world.apply_settings(settings)
            self.logger.info(f"Synchronous mode {'enabled' if enabled else 'disabled'}")
            
        except Exception as e:
            self.logger.error(f"Failed to set synchronous mode: {e}")
    
    def tick_world(self, timeout: float = 2.0) -> bool:
        """Manually tick the world in synchronous mode"""
        try:
            self.world.tick(timeout)
            return True
        except Exception as e:
            self.logger.error(f"Failed to tick world: {e}")
            return False
    
    def set_weather(self, weather_preset: str = "ClearNoon"):
        """Set weather conditions"""
        try:
            weather_presets = {
                'ClearNoon': carla.WeatherParameters.ClearNoon,
                'CloudyNoon': carla.WeatherParameters.CloudyNoon,
                'WetNoon': carla.WeatherParameters.WetNoon,
                'WetCloudyNoon': carla.WeatherParameters.WetCloudyNoon,
                'MidRainyNoon': carla.WeatherParameters.MidRainyNoon,
                'HardRainNoon': carla.WeatherParameters.HardRainNoon,
                'SoftRainNoon': carla.WeatherParameters.SoftRainNoon,
                'ClearSunset': carla.WeatherParameters.ClearSunset,
                'CloudySunset': carla.WeatherParameters.CloudySunset,
                'WetSunset': carla.WeatherParameters.WetSunset,
                'WetCloudySunset': carla.WeatherParameters.WetCloudySunset,
                'MidRainSunset': carla.WeatherParameters.MidRainSunset,
                'HardRainSunset': carla.WeatherParameters.HardRainSunset,
                'SoftRainSunset': carla.WeatherParameters.SoftRainSunset
            }
            
            if weather_preset in weather_presets:
                self.world.set_weather(weather_presets[weather_preset])
                self.logger.info(f"Weather set to {weather_preset}")
            else:
                self.logger.warning(f"Unknown weather preset: {weather_preset}")
                
        except Exception as e:
            self.logger.error(f"Failed to set weather: {e}")
    
    def get_spawn_points(self) -> List[carla.Transform]:
        """Get all available spawn points"""
        try:
            return self.world.get_map().get_spawn_points()
        except Exception as e:
            self.logger.error(f"Failed to get spawn points: {e}")
            return []
    
    def find_nearest_spawn_point(self, location: carla.Location) -> Optional[carla.Transform]:
        """Find the nearest spawn point to a given location"""
        spawn_points = self.get_spawn_points()
        if not spawn_points:
            return None
        
        min_distance = float('inf')
        nearest_spawn = None
        
        for spawn_point in spawn_points:
            distance = location.distance(spawn_point.location)
            if distance < min_distance:
                min_distance = distance
                nearest_spawn = spawn_point
        
        return nearest_spawn
    
    def restore_original_settings(self):
        """Restore original world settings"""
        if self.original_settings is not None:
            try:
                self.world.apply_settings(self.original_settings)
                self.logger.info("Restored original world settings")
            except Exception as e:
                self.logger.error(f"Failed to restore original settings: {e}")

def transform_to_tuple(transform: carla.Transform) -> Tuple[float, float, float, float, float, float]:
    """Convert Carla Transform to tuple (x, y, z, pitch, yaw, roll)"""
    return (
        transform.location.x,
        transform.location.y,
        transform.location.z,
        transform.rotation.pitch,
        transform.rotation.yaw,
        transform.rotation.roll
    )

def tuple_to_transform(data: Tuple[float, float, float, float, float, float]) -> carla.Transform:
    """Convert tuple to Carla Transform"""
    return carla.Transform(
        carla.Location(data[0], data[1], data[2]),
        carla.Rotation(data[3], data[4], data[5])
    )

def calculate_distance_2d(pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
    """Calculate 2D Euclidean distance between two positions"""
    return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

def calculate_heading_error(vehicle_transform: carla.Transform, target_location: carla.Location) -> float:
    """Calculate heading error between vehicle and target"""
    # Vector from vehicle to target
    target_vector = np.array([
        target_location.x - vehicle_transform.location.x,
        target_location.y - vehicle_transform.location.y
    ])
    
    # Vehicle forward vector
    yaw_rad = np.radians(vehicle_transform.rotation.yaw)
    forward_vector = np.array([
        np.cos(yaw_rad),
        np.sin(yaw_rad)
    ])
    
    # Calculate angle between vectors
    if np.linalg.norm(target_vector) > 0:
        target_vector = target_vector / np.linalg.norm(target_vector)
        dot_product = np.clip(np.dot(forward_vector, target_vector), -1.0, 1.0)
        angle_error = np.arccos(dot_product)
        
        # Determine sign using cross product
        cross_product = np.cross(forward_vector, target_vector)
        if cross_product < 0:
            angle_error = -angle_error
            
        return angle_error
    
    return 0.0

def setup_logging(level: int = logging.INFO):
    """Setup logging for Carla utilities"""
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler('carla_utils.log')
        ]
    )

# Example usage and testing
def main():
    """Example usage of Carla utilities"""
    setup_logging()
    
    # Connect to Carla
    connection = CarlaConnection()
    if not connection.connect():
        print("Failed to connect to Carla")
        return
    
    try:
        # Setup world manager
        world_manager = CarlaWorldManager(connection.world)
        world_manager.set_synchronous_mode(True)
        world_manager.set_weather("ClearNoon")
        
        # Setup vehicle manager
        vehicle_manager = CarlaVehicleManager(connection.world)
        
        # Spawn a vehicle
        vehicle = vehicle_manager.spawn_vehicle()
        if vehicle:
            print(f"Spawned vehicle: {vehicle.id}")
            
            # Get vehicle state
            state = vehicle_manager.get_vehicle_state(vehicle)
            print(f"Vehicle state: {state}")
            
            # Clean up
            vehicle_manager.destroy_all_vehicles()
        
        # Restore settings
        world_manager.restore_original_settings()
        
    finally:
        connection.disconnect()

if __name__ == "__main__":
    main()
