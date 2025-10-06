"""
Day 10-11 - PID Controller for Vehicle Lane Following
Implements PID control for steering and throttle/brake control
"""

import carla
import numpy as np
import time
import csv
import os
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
import yaml

@dataclass
class PIDGains:
    """PID controller gains"""
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0

class PIDController:
    """Generic PID controller implementation"""
    
    def __init__(self, gains: PIDGains, output_limits: Tuple[float, float] = (-1.0, 1.0)):
        self.gains = gains
        self.output_limits = output_limits
        
        # State variables
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None
        
    def update(self, error: float, current_time: float) -> float:
        """Update PID controller with current error"""
        if self.previous_time is None:
            self.previous_time = current_time
            dt = 0.0
        else:
            dt = current_time - self.previous_time
            
        if dt <= 0.0:
            dt = 1e-6  # Prevent division by zero
            
        # Proportional term
        proportional = self.gains.kp * error
        
        # Integral term
        self.integral += error * dt
        integral = self.gains.ki * self.integral
        
        # Derivative term
        derivative = self.gains.kd * (error - self.previous_error) / dt
        
        # Calculate output
        output = proportional + integral + derivative
        
        # Apply output limits
        output = max(min(output, self.output_limits[1]), self.output_limits[0])
        
        # Update state
        self.previous_error = error
        self.previous_time = current_time
        
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None

class VehiclePIDController:
    """PID-based vehicle controller for path following"""
    
    def __init__(self, vehicle: carla.Vehicle, 
                 steering_gains: PIDGains = PIDGains(1.0, 0.0, 0.1),
                 speed_gains: PIDGains = PIDGains(0.5, 0.02, 0.1),
                 target_speed: float = 30.0):  # km/h
        
        self.vehicle = vehicle
        self.target_speed = target_speed / 3.6  # Convert to m/s
        
        # PID controllers
        self.steering_controller = PIDController(steering_gains, (-1.0, 1.0))
        self.speed_controller = PIDController(speed_gains, (-1.0, 1.0))
        
        # Path following
        self.path_waypoints = []
        self.current_waypoint_index = 0
        self.lookahead_distance = 10.0  # meters
        
        # Logging
        self.control_log = []
        self.start_time = time.time()
        
    def set_path(self, waypoints: List[Tuple[float, float, float]]):
        """Set the path waypoints to follow"""
        self.path_waypoints = waypoints
        self.current_waypoint_index = 0
        
    def get_current_target_waypoint(self) -> Optional[Tuple[float, float, float]]:
        """Get the current target waypoint based on lookahead distance"""
        if not self.path_waypoints:
            return None
            
        vehicle_location = self.vehicle.get_location()
        vehicle_pos = (vehicle_location.x, vehicle_location.y, vehicle_location.z)
        
        # Find waypoint within lookahead distance
        for i in range(self.current_waypoint_index, len(self.path_waypoints)):
            wp = self.path_waypoints[i]
            distance = np.sqrt((wp[0] - vehicle_pos[0])**2 + 
                             (wp[1] - vehicle_pos[1])**2)
            
            if distance >= self.lookahead_distance:
                self.current_waypoint_index = max(0, i - 1)
                return wp
                
        # If no waypoint found within lookahead, return the last one
        if self.path_waypoints:
            return self.path_waypoints[-1]
        
        return None
    
    def calculate_steering_error(self, target_waypoint: Tuple[float, float, float]) -> float:
        """Calculate cross-track error for steering control"""
        vehicle_transform = self.vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_rotation = vehicle_transform.rotation
        
        # Calculate vector to target
        target_vector = np.array([
            target_waypoint[0] - vehicle_location.x,
            target_waypoint[1] - vehicle_location.y
        ])
        
        # Calculate vehicle forward vector
        yaw_rad = np.radians(vehicle_rotation.yaw)
        forward_vector = np.array([
            np.cos(yaw_rad),
            np.sin(yaw_rad)
        ])
        
        # Calculate cross-track error using cross product
        cross_product = np.cross(forward_vector, target_vector)
        distance_to_target = np.linalg.norm(target_vector)
        
        if distance_to_target > 0:
            cross_track_error = cross_product / distance_to_target
        else:
            cross_track_error = 0.0
            
        return cross_track_error
    
    def calculate_speed_error(self) -> float:
        """Calculate speed error for throttle/brake control"""
        velocity = self.vehicle.get_velocity()
        current_speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        return self.target_speed - current_speed
    
    def update_control(self) -> carla.VehicleControl:
        """Update vehicle control based on PID controllers"""
        current_time = time.time()
        
        # Get target waypoint
        target_waypoint = self.get_current_target_waypoint()
        
        if target_waypoint is None:
            # No target, stop the vehicle
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0
            return control
        
        # Calculate errors
        steering_error = self.calculate_steering_error(target_waypoint)
        speed_error = self.calculate_speed_error()
        
        # Update PID controllers
        steering_output = self.steering_controller.update(steering_error, current_time)
        speed_output = self.speed_controller.update(speed_error, current_time)
        
        # Create vehicle control
        control = carla.VehicleControl()
        control.steer = float(np.clip(steering_output, -1.0, 1.0))
        
        if speed_output > 0:
            control.throttle = float(np.clip(speed_output, 0.0, 1.0))
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = float(np.clip(-speed_output, 0.0, 1.0))
        
        # Log control data
        self.log_control_data(current_time, steering_error, speed_error, 
                            steering_output, speed_output, control)
        
        return control
    
    def log_control_data(self, timestamp: float, steering_error: float, 
                        speed_error: float, steering_output: float, 
                        speed_output: float, control: carla.VehicleControl):
        """Log control data for analysis"""
        vehicle_location = self.vehicle.get_location()
        velocity = self.vehicle.get_velocity()
        current_speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        log_entry = {
            'timestamp': timestamp - self.start_time,
            'vehicle_x': vehicle_location.x,
            'vehicle_y': vehicle_location.y,
            'vehicle_z': vehicle_location.z,
            'current_speed': current_speed,
            'target_speed': self.target_speed,
            'steering_error': steering_error,
            'speed_error': speed_error,
            'steering_output': steering_output,
            'speed_output': speed_output,
            'steer_command': control.steer,
            'throttle_command': control.throttle,
            'brake_command': control.brake,
            'waypoint_index': self.current_waypoint_index
        }
        
        self.control_log.append(log_entry)
    
    def save_control_log(self, output_dir: str = "output_data/day10_pid_logs"):
        """Save control log to CSV file"""
        os.makedirs(output_dir, exist_ok=True)
        
        if not self.control_log:
            print("No control data to save")
            return
            
        filename = f"{output_dir}/pid_control_log_{int(time.time())}.csv"
        
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = self.control_log[0].keys()
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for entry in self.control_log:
                writer.writerow(entry)
        
        print(f"Control log saved to {filename}")
    
    def get_performance_metrics(self) -> Dict[str, float]:
        """Calculate performance metrics from control log"""
        if not self.control_log:
            return {}
            
        steering_errors = [abs(entry['steering_error']) for entry in self.control_log]
        speed_errors = [abs(entry['speed_error']) for entry in self.control_log]
        
        metrics = {
            'avg_steering_error': np.mean(steering_errors),
            'max_steering_error': np.max(steering_errors),
            'avg_speed_error': np.mean(speed_errors),
            'max_speed_error': np.max(speed_errors),
            'total_time': self.control_log[-1]['timestamp'],
            'total_distance': self._calculate_total_distance()
        }
        
        return metrics
    
    def _calculate_total_distance(self) -> float:
        """Calculate total distance traveled"""
        if len(self.control_log) < 2:
            return 0.0
            
        total_distance = 0.0
        for i in range(1, len(self.control_log)):
            prev_pos = (self.control_log[i-1]['vehicle_x'], 
                       self.control_log[i-1]['vehicle_y'])
            curr_pos = (self.control_log[i]['vehicle_x'], 
                       self.control_log[i]['vehicle_y'])
            
            distance = np.sqrt((curr_pos[0] - prev_pos[0])**2 + 
                             (curr_pos[1] - prev_pos[1])**2)
            total_distance += distance
            
        return total_distance

def load_pid_gains(config_file: str = "configs/pid_gains.yaml") -> Tuple[PIDGains, PIDGains]:
    """Load PID gains from configuration file"""
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
            
        steering_gains = PIDGains(**config['steering_pid'])
        speed_gains = PIDGains(**config['speed_pid'])
        
        return steering_gains, speed_gains
    except:
        print("Could not load PID gains, using defaults")
        return PIDGains(1.0, 0.0, 0.1), PIDGains(0.5, 0.02, 0.1)

def main():
    """Main function to test PID controller"""
    try:
        # Connect to Carla
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        # Spawn vehicle
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        
        spawn_points = world.get_map().get_spawn_points()
        spawn_point = spawn_points[0]
        
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        
        try:
            # Load PID gains
            steering_gains, speed_gains = load_pid_gains()
            
            # Create PID controller
            controller = VehiclePIDController(vehicle, steering_gains, speed_gains)
            
            # Create a simple test path (straight line)
            start_loc = spawn_point.location
            test_path = []
            for i in range(20):
                x = start_loc.x + i * 5.0
                y = start_loc.y
                z = start_loc.z
                test_path.append((x, y, z))
            
            controller.set_path(test_path)
            
            # Run control loop
            print("Starting PID control test...")
            for _ in range(1000):  # Run for ~50 seconds at 20Hz
                control = controller.update_control()
                vehicle.apply_control(control)
                time.sleep(0.05)  # 20Hz control loop
            
            # Save results
            controller.save_control_log()
            metrics = controller.get_performance_metrics()
            print(f"Performance metrics: {metrics}")
            
        finally:
            vehicle.destroy()
            
        print("PID controller test completed successfully!")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
