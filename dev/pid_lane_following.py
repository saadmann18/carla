import carla
import math
import numpy as np
import time
import argparse
import random
import os
import csv
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from enum import Enum

class PIDController:
    def __init__(self, kp, ki, kd, max_output=1.0, min_output=-1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
    
    def update(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Proportional term
        p = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        d = self.kd * derivative
        
        # Calculate output
        output = p + i + d
        
        # Clamp output
        output = max(self.min_output, min(self.max_output, output))
        
        self.prev_error = error
        return output

def get_lane_center(vehicle, waypoints=None):
    """Get the center of the current lane in world coordinates using CARLA's waypoint API"""
    # Get the map and the vehicle's current transform
    world = vehicle.get_world()
    vehicle_transform = vehicle.get_transform()
    
    # Get the nearest waypoint on the road
    waypoint = world.get_map().get_waypoint(
        vehicle_transform.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    
    if waypoint is None:
        # If no waypoint found, return vehicle's current location as fallback
        return vehicle_transform.location, None
        
    # Return the waypoint and its location
    return waypoint.transform.location, waypoint

def get_lateral_error(vehicle, waypoints=None):
    """Calculate lateral error from lane center"""
    # Get vehicle's current transform
    vehicle_transform = vehicle.get_transform()
    
    # Get the lane center and waypoint
    lane_center, current_waypoint = get_lane_center(vehicle, waypoints)
    
    # If we have a valid waypoint and we're at a junction, randomly choose a direction
    if current_waypoint and current_waypoint.is_junction:
        next_waypoints = current_waypoint.next_until_lane_end(5.0)
        if next_waypoints:
            global target_waypoint
            target_waypoint = random.choice(next_waypoints)
    
    # Calculate the vector from vehicle to lane center (in 2D)
    vehicle_loc = vehicle_transform.location
    vehicle_to_center = carla.Location(
        lane_center.x - vehicle_loc.x,
        lane_center.y - vehicle_loc.y,
        0  # Ignore height difference
    )
    
    # Get the right vector of the vehicle (perpendicular to forward vector)
    forward_vector = vehicle_transform.get_forward_vector()
    right_vector = carla.Vector3D(
        -forward_vector.y,
        forward_vector.x,
        0  # 2D vector
    )
    
    # Normalize the right vector
    right_vector_length = math.sqrt(right_vector.x**2 + right_vector.y**2)
    if right_vector_length > 0:
        right_vector.x /= right_vector_length
        right_vector.y /= right_vector_length
    
    # Calculate lateral error (positive if vehicle is to the right of center)
    lateral_error = (
        vehicle_to_center.x * right_vector.x +
        vehicle_to_center.y * right_vector.y
    )
    
    # Debug visualization
    debug = vehicle.get_world().debug
    debug.draw_point(
        lane_center,
        size=0.1,
        color=carla.Color(0, 255, 0),
        life_time=0.1
    )
    
    return lateral_error

def setup_output_directory():
    """Create output directory with timestamp"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join('output_day6', f'run_{timestamp}')
    os.makedirs(output_dir, exist_ok=True)
    return output_dir

def save_spawn_log(output_dir, spawn_point, vehicle_type):
    """Save spawn information to a log file"""
    log_file = os.path.join(output_dir, 'spawn_log.txt')
    with open(log_file, 'w') as f:
        f.write(f"Spawn Time: {datetime.now()}\n")
        f.write(f"Vehicle Type: {vehicle_type}\n")
        f.write(f"Spawn Location: {spawn_point.location}\n")
        f.write(f"Spawn Rotation: {spawn_point.rotation}\n")
    return log_file

def save_trajectory_plot(output_dir, trajectory, lane_centers):
    """Save trajectory visualization"""
    plt.figure(figsize=(12, 8))
    
    # Plot trajectory
    x = [p.x for p in trajectory]
    y = [p.y for p in trajectory]
    plt.plot(x, y, 'b-', label='Vehicle Trajectory')
    
    # Plot lane centers if available
    if lane_centers:
        cx = [p.x for p in lane_centers if p is not None]
        cy = [p.y for p in lane_centers if p is not None]
        plt.scatter(cx, cy, c='g', s=5, alpha=0.5, label='Lane Centers')
    
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Vehicle Trajectory')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    
    # Save plot
    plot_path = os.path.join(output_dir, 'trajectory_plot.png')
    plt.savefig(plot_path)
    plt.close()
    return plot_path

def save_error_plot(output_dir, errors, timestamps):
    """Save error plot over time"""
    plt.figure(figsize=(12, 6))
    plt.plot(timestamps, errors, 'r-')
    plt.xlabel('Time (s)')
    plt.ylabel('Lateral Error (m)')
    plt.title('Lateral Error Over Time')
    plt.grid(True)
    
    # Save plot
    plot_path = os.path.join(output_dir, 'error_plot.png')
    plt.savefig(plot_path)
    plt.close()
    return plot_path

def save_speed_plot(output_dir, speeds, target_speeds, timestamps):
    """Save speed plot over time"""
    plt.figure(figsize=(12, 6))
    
    # Plot actual speed
    plt.plot(timestamps, [s * 3.6 for s in speeds], 'b-', label='Actual Speed')
    
    # Plot target speed if available
    if target_speeds:
        plt.plot(timestamps, [s * 3.6 for s in target_speeds], 'r--', label='Target Speed')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (km/h)')
    plt.title('Vehicle Speed Over Time')
    plt.legend()
    plt.grid(True)
    
    # Save plot
    plot_path = os.path.join(output_dir, 'speed_plot.png')
    plt.close()
    return plot_path

def cleanup_actors(world):
    """Destroy all existing vehicles and other controllable actors"""
    print("\n=== Cleaning up existing actors ===")
    
    # Get all actors
    actors = world.get_actors()
    
    # Filter for vehicles and controllable actors
    vehicles = [actor for actor in actors if 'vehicle' in actor.type_id]
    
    # Destroy all found vehicles
    for vehicle in vehicles:
        try:
            print(f"Destroying {vehicle.type_id} (ID: {vehicle.id})")
            vehicle.destroy()
        except Exception as e:
            print(f"Error destroying {vehicle.type_id}: {e}")
    
    # Wait a bit to allow cleanup to complete
    time.sleep(0.5)
    print("Cleanup complete\n")

def main():
    # Parse arguments
    # Define weather presets
    class WeatherPreset(Enum):
        ClearNoon = carla.WeatherParameters.ClearNoon
        CloudyNoon = carla.WeatherParameters.CloudyNoon
        WetNoon = carla.WeatherParameters.WetNoon
        WetCloudyNoon = carla.WeatherParameters.WetCloudyNoon
        MidRainyNoon = carla.WeatherParameters.MidRainyNoon
        HardRainNoon = carla.WeatherParameters.HardRainNoon
        SoftRainNoon = carla.WeatherParameters.SoftRainNoon
        ClearSunset = carla.WeatherParameters.ClearSunset
        CloudySunset = carla.WeatherParameters.CloudySunset
        WetSunset = carla.WeatherParameters.WetSunset
        WetCloudySunset = carla.WeatherParameters.WetCloudySunset
        MidRainSunset = carla.WeatherParameters.MidRainSunset
        HardRainSunset = carla.WeatherParameters.HardRainSunset
        SoftRainSunset = carla.WeatherParameters.SoftRainSunset

    parser = argparse.ArgumentParser(description='PID Lane Following with Speed Control')
    parser.add_argument('--host', default='127.0.0.1', help='IP of the host server')
    parser.add_argument('--port', default=2000, type=int, help='TCP port to listen to')
    parser.add_argument('--duration', default=60, type=int, help='Duration to run (seconds)')
    parser.add_argument('--target-speed', default=10.0, type=float, 
                       help='Target speed in m/s (default: 10.0 m/s ~ 36 km/h)')
    parser.add_argument('--max-throttle', default=1.0, type=float, 
                       help='Maximum throttle value (0.0 to 1.0)')
    parser.add_argument('--max-brake', default=0, type=float, 
                       help='Maximum brake value (0.0 to 1.0)')
    parser.add_argument('--min-speed', default=1.0, type=float,
                       help='Minimum speed before applying more throttle (m/s)')
    parser.add_argument('--speed-Kp', default=2.0, type=float, 
                       help='Proportional gain for speed control (default: 2.0)')
    parser.add_argument('--speed-Ki', default=0.5, type=float, 
                       help='Integral gain for speed control (default: 0.5)')
    parser.add_argument('--speed-Kd', default=0.2, type=float, 
                       help='Derivative gain for speed control (default: 0.2)')
    parser.add_argument('--steer-Kp', default=0.2, type=float,
                       help='Proportional gain for steering control (default: 0.2)')
    parser.add_argument('--steer-Ki', default=0.01, type=float,
                       help='Integral gain for steering control (default: 0.01)')
    parser.add_argument('--steer-Kd', default=0.2, type=float,
                       help='Derivative gain for steering control (default: 0.2)')
    parser.add_argument('--sync', action='store_true',
                       help='Run in synchronous mode (use when running with traffic)')
    parser.add_argument('--fps', default=20, type=int,
                       help='Frames per second (only used in synchronous mode)')
    parser.add_argument('--weather', default='ClearNoon',
                      choices=[w.name for w in WeatherPreset],
                      help='Weather preset to use (e.g., ClearNoon, HardRainNoon, WetCloudyNoon)')
    parser.add_argument('--output-dir', default=None,
                      help='Directory to save output (default: auto-generated with timestamp)')
    
    # Parse arguments and setup output directory
    args = parser.parse_args()
    output_dir = args.output_dir if args.output_dir else setup_output_directory()
    print(f"Saving results to: {os.path.abspath(output_dir)}")
    
    # Save parameters to spawn log
    with open(os.path.join(output_dir, 'spawn_log.txt'), 'a') as f:
        f.write(f"\nTarget Speed: {args.target_speed} m/s\n")
        f.write(f"Max Throttle: {args.max_throttle}\n")
        f.write(f"Max Brake: {args.max_brake}\n")
        f.write(f"Speed Kp: {args.speed_Kp}\n")
        f.write(f"Speed Ki: {args.speed_Ki}\n")
        f.write(f"Speed Kd: {args.speed_Kd}\n")
        f.write(f"Sync Mode: {args.sync}\n")
        if args.sync:
            f.write(f"FPS: {args.fps}\n")
    
    # Steering PID parameters
    steer_Kp = 1.0  # Start with a moderate P value
    steer_Ki = 0.0  # Start with I=0
    steer_Kd = 0.2  # Small D term to reduce oscillation
    
    # Speed PID parameters
    speed_Kp = 0.5   # Proportional gain for speed control
    speed_Ki = 0.1   # Integral gain for speed control
    speed_Kd = 0.05  # Derivative gain for speed control
    
    try:
        # Connect to CARLA with increased timeout
        client = carla.Client(args.host, args.port)
        client.set_timeout(30.0)  # Increased timeout for busy servers
        
        # Wait for the server to be ready
        print("Waiting for server to be ready...")
        world = client.get_world()
        
        # Apply weather preset
        try:
            weather_preset = WeatherPreset[args.weather].value
            world.set_weather(weather_preset)
            print(f"Weather set to: {args.weather}")
        except KeyError:
            print(f"Warning: Weather preset '{args.weather}' not found. Using default weather.")
        
        # Get all possible spawn points
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points found in the map!")
            
        # Select a random spawn point
        spawn_point = random.choice(spawn_points)
        print(f"Selected spawn point at {spawn_point.location}")
        
        # Get traffic manager
        traffic_manager = client.get_trafficmanager()
        
        # Set up synchronization mode
        settings = world.get_settings()
        
        if args.sync:
            print(f"Running in synchronous mode at {args.fps} FPS")
            
            # Enable synchronous mode
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 1.0 / args.fps
            settings.no_rendering_mode = False
            
            # Configure traffic manager for synchronous mode
            traffic_manager.set_synchronous_mode(True)
            traffic_manager.set_random_device_seed(0)  # For reproducibility
            
            # Apply settings
            world.apply_settings(settings)
            
            # Set spectator to follow the ego vehicle
            spectator = world.get_spectator()
            
            # Set traffic manager to sync mode
            traffic_manager.set_synchronous_mode(True)
            
            # Set all traffic lights to green
            for actor in world.get_actors():
                if actor.type_id.startswith('traffic.traffic_light'):
                    actor.set_state(carla.TrafficLightState.Green)
                    actor.freeze(True)
        else:
            print("Running in asynchronous mode")
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(False)
        
        # Give everything time to initialize
        time.sleep(2.0)
        
        # Get blueprint for the vehicle - using the same approach as manual_control.py
        blueprint_library = world.get_blueprint_library()
        
        # Get all vehicle blueprints
        blueprints = blueprint_library.filter('vehicle.*')
        
        # Filter out bicycles and motorcycles
        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
        blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
        blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('t2')]
        blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
        blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]
        
        # Sort blueprints by ID for consistency
        blueprints = sorted(blueprints, key=lambda bp: bp.id)
        
        # Print available vehicles
        print("\nAvailable vehicles:")
        for i, bp in enumerate(blueprints):
            print(f"{i}: {bp.id}")
        
        # Select first available vehicle
        if not blueprints:
            raise RuntimeError("No suitable vehicles found!")
            
        vehicle_bp = blueprints[0]
        print(f"\nUsing vehicle: {vehicle_bp.id}")
        
        # Get spawn points and select a random one
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points found on the map!")
            
        spawn_point = random.choice(spawn_points)  # Random spawn point
        print(f"\nSelected random spawn point at: {spawn_point.location}")
        
        # Add a highlight marker at the spawn point
        debug = world.debug
        # Draw a red sphere at the spawn point for 30 seconds
        debug.draw_point(spawn_point.location, size=0.2, color=carla.Color(255, 0, 0), life_time=30.0)
        # Draw an arrow pointing in the spawn direction
        debug.draw_arrow(
            spawn_point.location,
            spawn_point.location + spawn_point.get_forward_vector() * 5.0,
            thickness=0.1, arrow_size=0.2, 
            color=carla.Color(0, 255, 0), 
            life_time=30.0
        )
        print(f"Spawn point marked with red sphere and green direction arrow (visible for 30 seconds)", "\n")
        
        # Clean up any existing actors first
        cleanup_actors(world)
        
        # Spawn the vehicle
        try:
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            if vehicle is None:
                raise RuntimeError("Failed to spawn vehicle - spawn_actor returned None")
            print(f'Spawned {vehicle.type_id} at {spawn_point.location}')
        except Exception as e:
            print(f"Error spawning vehicle: {e}")
            # Try one more time after a short delay
            time.sleep(1)
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            if vehicle is None:
                raise RuntimeError("Failed to spawn vehicle on second attempt")
        
        # Debug: Print vehicle state
        print("\n=== Vehicle State After Spawn ===")
        print(f"Vehicle ID: {vehicle.id}")
        print(f"Vehicle Type: {vehicle.type_id}")
        print(f"Location: {vehicle.get_location()}")
        print(f"Velocity: {vehicle.get_velocity()}")
        print(f"Current Control: {vehicle.get_control()}")
        print("================================\n")
        
        # Add a marker to identify the ego vehicle
        def draw_ego_marker():
            if not vehicle.is_alive:
                return False
                
            # Get vehicle transform
            transform = vehicle.get_transform()
            location = transform.location
            
            # Draw a large red arrow above the vehicle
            world.debug.draw_arrow(
                location + carla.Location(z=5.0),  # 5m above the vehicle
                location + carla.Location(z=10.0),  # 10m above the vehicle
                thickness=0.2,
                arrow_size=1.0,
                color=carla.Color(255, 0, 0),  # Red
                life_time=0.1  # Redraw every frame
            )
            
            # Draw a circle around the vehicle
            world.debug.draw_point(
                location + carla.Location(z=0.5),  # Slightly above ground
                size=0.5,
                color=carla.Color(255, 0, 0),  # Red
                life_time=0.1  # Redraw every frame
            )
            
            return True
            
        # Start the marker thread
        import threading
        marker_thread_running = True
        
        def marker_loop():
            while marker_thread_running and vehicle.is_alive:
                if not draw_ego_marker():
                    break
                time.sleep(0.05)  # 20Hz update rate
                
        marker_thread = threading.Thread(target=marker_loop, daemon=True)
        marker_thread.start()
        
        # Initialize PID controllers
        steer_pid = PIDController(
            args.steer_Kp, 
            args.steer_Ki, 
            args.steer_Kd, 
            max_output=1.0, 
            min_output=-1.0
        )
        
        speed_pid = PIDController(
            args.speed_Kp, 
            args.speed_Ki, 
            args.speed_Kd, 
            max_output=args.max_throttle, 
            min_output=-args.max_brake
        )
        
        # Data logging setup
        csv_path = os.path.join(output_dir, 'simulation_data.csv')
        trajectory = []
        lane_centers = []
        errors = []
        speeds = []
        target_speeds = []
        timestamps = []
        
        # Initialize CSV file with headers
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'timestamp', 'x', 'y', 'z', 'speed', 'target_speed',
                'throttle', 'brake', 'steer', 'lateral_error',
                'steering_angle', 'weather'
            ])
        
        # Save spawn log
        save_spawn_log(output_dir, spawn_point, vehicle_bp.id)
        
        # Main loop
        start_time = time.time()
        
        # Debug: Print vehicle physics parameters
        try:
            physics_control = vehicle.get_physics_control()
            print("\nVehicle Physics Parameters:")
            print(f"Mass: {physics_control.mass} kg")
            print(f"Max RPM: {physics_control.max_rpm} RPM")
            # Print available attributes for debugging
            print("Available attributes:", dir(physics_control))
            # Print wheel info if available
            if hasattr(physics_control, 'wheels') and physics_control.wheels:
                print(f"Number of wheels: {len(physics_control.wheels)}")
                print(f"First wheel info: {dir(physics_control.wheels[0])}")
        except Exception as e:
            print(f"Error getting physics control: {e}")
        
        # Set vehicle to autopilot to test if it can move
        print("\nTesting vehicle movement with autopilot...")
        vehicle.set_autopilot(True)
        time.sleep(2)  # Let it try to move
        vehicle.set_autopilot(False)
        print(f"After autopilot test - Velocity: {vehicle.get_velocity()}")
        
        # Reset vehicle control
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
        time.sleep(0.5)
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=False))
        
        # Debug: Print spawn point and initial state
        print(f"\nSpawn Point: {spawn_point}")
        print(f"Initial Location: {vehicle.get_location()}")
        print(f"Initial Velocity: {vehicle.get_velocity()}")
        print(f"Initial Control: {vehicle.get_control()}")
        
        # Initialize target waypoint for navigation
        target_waypoint = None
        
        # Open CSV file for writing
        with open(csv_path, 'w', newline='') as csvfile:
            fieldnames = [
                'timestamp', 'x', 'y', 'z', 'speed', 'target_speed',
                'throttle', 'brake', 'steer', 'lateral_error', 'speed_error'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
        while True:
            # Update current time and check duration
            current_time = time.time() - start_time
            if current_time >= args.duration:
                break
                
            # Get vehicle state
            vehicle_location = vehicle.get_location()
            vehicle_velocity = vehicle.get_velocity()
            speed = math.sqrt(vehicle_velocity.x**2 + vehicle_velocity.y**2 + vehicle_velocity.z**2)
            
            # Get lateral error and lane center
            lane_center, current_waypoint = get_lane_center(vehicle)
            lateral_error = get_lateral_error(vehicle)
            
            # If we have a target waypoint, calculate steering towards it
            if target_waypoint and current_waypoint:
                # Calculate vector to target waypoint
                target_vector = target_waypoint.transform.location - current_waypoint.transform.location
                target_vector = carla.Vector3D(target_vector.x, target_vector.y, 0)
                target_vector = target_vector.make_unit_vector()
                
                # Get forward vector
                forward_vector = vehicle.get_transform().get_forward_vector()
                forward_vector = carla.Vector3D(forward_vector.x, forward_vector.y, 0)
                forward_vector = forward_vector.make_unit_vector()
                
                # Calculate cross product to determine steering direction
                cross = forward_vector.x * target_vector.y - forward_vector.y * target_vector.x
                lateral_error = cross * 10.0  # Scale the error for better control
            
            # Update steering PID
            steer = steer_pid.update(lateral_error)
            
            # Calculate lateral acceleration (m/s²)
            current_steering = abs(steer)  # 0 to 1.0
            current_speed = speed  # m/s
            
            # Calculate maximum safe speed for current steering angle
            # This is a simplified model - you may need to tune these constants
            max_safe_speed = args.target_speed * (1.0 - 0.5 * current_steering**2)
            
            # Calculate speed error with respect to safe speed
            speed_error = min(args.target_speed, max_safe_speed) - speed
            
            # Update speed PID
            speed_output = speed_pid.update(speed_error)
            
            # Calculate throttle and brake based on PID output
            if speed < args.min_speed:
                # If speed is very low, apply more aggressive throttle to get moving
                throttle = min(args.max_throttle * 1.2, 1.0)  # Allow slight over-throttle to start
                brake = 0.0
            elif speed_output > 0:
                # Normal operation - use PID output for throttle
                # Reduce throttle based on steering angle
                throttle_reduction = current_steering * 0.7  # Reduce up to 70% throttle in sharp turns
                throttle = min(speed_output * (1.0 - throttle_reduction), args.max_throttle)
                brake = 0.0
            else:
                # Braking is needed
                throttle = 0.0
                # Increase braking in turns
                brake_boost = 1.0 + (current_steering * 2.0)  # Up to 3x braking in sharp turns
                brake = min(-speed_output * brake_boost, args.max_brake)
                
                # Ensure minimum brake is applied during turns
                if current_steering > 0.2:  # If steering more than 20%
                    brake = max(brake, 0.4)  # Apply at least 40% brake
                
            # Debug output for speed control
            if int(current_time * 10) % 10 == 0:  # Print every second
                print(f"Speed: {speed*3.6:4.1f} km/h | Target: {args.target_speed*3.6:4.1f} km/h | "
                      f"Steer: {steer:5.2f} | Throttle: {throttle:.2f} | Brake: {brake:.2f}")
                print(f"SafeSpeed: {max_safe_speed*3.6:4.1f} km/h | Steering: {current_steering*100:3.0f}% | "
                      f"SpeedError: {speed_error:4.2f} m/s")
                
                # Additional debug for very low speed situations
                if speed < args.min_speed / 2:
                    print(f"WARNING: Very low speed ({speed*3.6:.1f} km/h) - Check for obstacles or spawn issues")
            
            # Check for vehicles in front
            vehicle_in_front = False
            vehicle_in_front_distance = 100.0  # meters
            
            # Get all actors and check for vehicles in front
            for actor in world.get_actors().filter('vehicle.*'):
                if actor.id != vehicle.id:  # Skip self
                    # Calculate distance and angle to other vehicle
                    other_location = actor.get_location()
                    distance = vehicle_location.distance(other_location)
                    
                    # Get vector from vehicle to other vehicle
                    vector_to_other = other_location - vehicle_location
                    vector_to_other = carla.Vector3D(
                        vector_to_other.x,
                        vector_to_other.y,
                        0
                    )
                    
                    # Get forward vector of the vehicle
                    forward_vector = vehicle.get_transform().get_forward_vector()
                    forward_vector = carla.Vector3D(
                        forward_vector.x,
                        forward_vector.y,
                        0
                    )
                    
                    # Calculate angle between forward vector and vector to other vehicle
                    angle = math.degrees(math.acos(
                        (forward_vector.x * vector_to_other.x + forward_vector.y * vector_to_other.y) /
                        (math.sqrt(forward_vector.x**2 + forward_vector.y**2) * 
                         math.sqrt(vector_to_other.x**2 + vector_to_other.y**2))
                    ))
                    
                    # If vehicle is in front (within 45 degrees) and within distance
                    if angle < 45.0 and distance < vehicle_in_front_distance:
                        vehicle_in_front = True
                        vehicle_in_front_distance = distance
            
            # Adjust braking based on vehicle in front
            if vehicle_in_front:
                # Calculate safe following distance (2 seconds gap)
                safe_distance = max(5.0, speed * 2.0)  # At least 5m
                
                if vehicle_in_front_distance < safe_distance:
                    # Apply stronger braking when too close
                    brake_boost = max(brake_boost, 2.0)  # At least 2x braking
                    brake = min(1.0, (safe_distance - vehicle_in_front_distance) / safe_distance)
                    throttle = 0.0
                    if int(current_time * 10) % 10 == 0:
                        print(f"Vehicle in front! Distance: {vehicle_in_front_distance:.1f}m | Applying brake: {brake:.2f}")
            
            # Create and apply control
            control = carla.VehicleControl(
                throttle=float(throttle),
                steer=float(steer),
                brake=float(brake),
                hand_brake=False,
                reverse=False
            )
            vehicle.apply_control(control)
            
            # Debug: Print detailed state every second
            if int(current_time * 10) % 10 == 0:  # Every second
                try:
                    accel = vehicle.get_acceleration()
                    print(f"\n--- Debug Info (t={current_time:.1f}s) ---")
                    print(f"Location: {vehicle_location}")
                    print(f"Velocity: {vehicle_velocity} (Speed: {speed*3.6:.1f} km/h)")
                    print(f"Control: Throttle={throttle:.2f}, Brake={brake:.2f}, Steer={steer:.2f}")
                    print(f"Acceleration: Forward={accel.x:.2f}, Lateral={accel.y:.2f}, Up={accel.z:.2f} m/s²")
                    
                    # Try to get wheel contacts if available
                    try:
                        wheels = vehicle.get_wheel_contacts()
                        if wheels:
                            print(f"Wheel Contacts: {[w.is_in_contact for w in wheels]}")
                    except Exception as e:
                        print(f"Could not get wheel contacts: {e}")
                except Exception as e:
                    print(f"Error in debug info: {e}")
            
            # Log data
            trajectory.append(vehicle_location)
            lane_centers.append(lane_center)
            errors.append(lateral_error)
            speeds.append(speed)
            target_speeds.append(args.target_speed)
            timestamps.append(current_time)
            
            # Log detailed data to CSV
            with open(csv_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    current_time,  # timestamp
                    vehicle_location.x, vehicle_location.y, vehicle_location.z,  # position
                    speed, args.target_speed,  # speed and target
                    throttle, brake, steer,  # control inputs
                    lateral_error,  # lateral error
                    current_steering,  # current steering angle
                    args.weather  # weather condition
                ])
            
            # Write to CSV
            try:
                with open(csv_path, 'a', newline='') as csvfile:
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writerow({
                        'timestamp': current_time,
                        'x': vehicle_location.x,
                        'y': vehicle_location.y,
                        'z': vehicle_location.z,
                        'speed': speed,
                        'target_speed': args.target_speed,
                        'throttle': throttle,
                        'brake': brake,
                        'steer': steer,
                        'lateral_error': lateral_error,
                        'speed_error': speed_error
                    })
            except Exception as e:
                print(f"Error writing to CSV: {e}")
            
            # Print debug info
            if int(current_time * 10) % 10 == 0:  # Print every second
                print(f"Time: {current_time:5.1f}s | "
                      f"Speed: {speed*3.6:4.1f} km/h (target: {args.target_speed*3.6:.1f}) | "
                      f"Throttle: {throttle:.2f} | "
                      f"Brake: {brake:.2f} | "
                      f"Lateral error: {lateral_error:5.2f}m | "
                      f"Steer: {steer:5.2f}")
            
            # Handle simulation tick based on sync mode
            if args.sync:
                try:
                    # In sync mode, we control the tick
                    frame = world.tick()
                    
                    # Update spectator to follow the ego vehicle
                    if 'vehicle' in locals() and vehicle.is_alive:
                        transform = vehicle.get_transform()
                        spectator.set_transform(carla.Transform(
                            transform.location + carla.Location(z=50),  # 50m above
                            carla.Rotation(pitch=-90)  # Look straight down
                        ))
                        
                    # Print frame rate info
                    if int(current_time * 10) % 10 == 0:  # Every second
                        print(f"Frame: {frame} | Time: {current_time:.1f}s | "
                              f"Speed: {speed*3.6:.1f} km/h | "
                              f"Position: {vehicle.get_location().x:.1f}, {vehicle.get_location().y:.1f}")
                except RuntimeError as e:
                    if "time-out" in str(e):
                        print("Warning: Timeout while ticking the world. The simulation might be running too slowly.")
                        continue
                    raise
            else:
                # In async mode, use a small sleep to prevent high CPU usage
                time.sleep(0.01)
            
            # Periodically save plots
            if int(current_time) % 5 == 0 and int(current_time) > 0:  # Every 5 seconds
                save_trajectory_plot(output_dir, trajectory, lane_centers)
                save_error_plot(output_dir, errors, timestamps)
                save_speed_plot(output_dir, speeds, target_speeds, timestamps)
            
    except KeyboardInterrupt:
        print('\nSimulation cancelled by user')
    except Exception as e:
        print(f'\nError: {e}')
    finally:
        try:
            # Save final plots
            if 'trajectory' in locals() and 'lane_centers' in locals():
                save_trajectory_plot(output_dir, trajectory, lane_centers)
            if 'errors' in locals() and 'timestamps' in locals():
                save_error_plot(output_dir, errors, timestamps)
            if 'speeds' in locals() and 'timestamps' in locals():
                save_speed_plot(output_dir, speeds, target_speeds, timestamps)
            
            # Clean up
            if 'marker_thread_running' in locals():
                marker_thread_running = False
                if 'marker_thread' in locals():
                    marker_thread.join(timeout=1.0)
                    
            if 'vehicle' in locals():
                vehicle.destroy()
            if 'world' in locals():
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)
            
            print(f'\nSimulation complete. Results saved to: {os.path.abspath(output_dir)}')
            print(f'- Trajectory plot: {os.path.join(output_dir, "trajectory_plot.png")}')
            print(f'- Error plot: {os.path.join(output_dir, "error_plot.png")}')
            print(f'- Speed plot: {os.path.join(output_dir, "speed_plot.png")}')
            print(f'- Simulation data: {os.path.join(output_dir, "simulation_data.csv")}')
            print(f'- Spawn log: {os.path.join(output_dir, "spawn_log.txt")}')
            
        except Exception as e:
            print(f'Error during cleanup: {e}')
        finally:
            print('Done')

if __name__ == '__main__':
    main()
