"""
CARLA Ego Vehicle with Dual Cameras and GPS Tracking

This script spawns a vehicle in the CARLA simulator, attaches two cameras (front and rear),
and records sensor data to disk. The vehicle operates in autopilot mode while collecting data.

Features:
- Front and rear RGB cameras with customizable resolution and FOV
- GPS tracking with trajectory visualization
- Data saved to timestamped directories

Outputs:
    - RGB images in output_<timestamp>/front/ and output_<timestamp>/rear/
    - GNSS coordinates in output_<timestamp>/gnss_log.txt
    - Trajectory plot in output_<timestamp>/trajectory.png
"""

import carla
import random
import time
import os
import math
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt

def setup_camera(blueprint_library, width=1280, height=720, fov=110, location=carla.Location(x=1.5, z=2.4), rotation=carla.Rotation()):
    """Set up a camera with specified parameters."""
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(width))
    camera_bp.set_attribute("image_size_y", str(height))
    camera_bp.set_attribute("fov", str(fov))
    camera_transform = carla.Transform(location, rotation)
    return camera_bp, camera_transform

def plot_trajectory(gps_data, output_dir):
    """Plot the vehicle's trajectory from GPS data."""
    lats = [point[1] for point in gps_data]
    longs = [point[2] for point in gps_data]
    
    plt.figure(figsize=(12, 8))
    plt.plot(longs, lats, 'b-', linewidth=2)
    plt.scatter(longs[0], lats[0], c='g', s=100, label='Start')
    plt.scatter(longs[-1], lats[-1], c='r', s=100, label='End')
    
    plt.title('Vehicle Trajectory')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    plot_path = os.path.join(output_dir, "trajectory.png")
    plt.savefig(plot_path)
    plt.close()
    print(f"Trajectory plot saved to {plot_path}")

def main():
    # Create output directory structure
    base_dir = "output_day3"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(base_dir, f"run_{timestamp}")
    
    # Create all necessary directories
    os.makedirs(os.path.join(output_dir, "front"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "rear"), exist_ok=True)
    print(f"Output will be saved to: {os.path.abspath(output_dir)}")
    
    # GPS data storage
    gps_data = []
    
    try:
        # Connect to CARLA
        client = carla.Client("localhost", 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        # Set up vehicle
        vehicle_blueprints = blueprint_library.filter('vehicle.*')
        vehicle_bp = random.choice(vehicle_blueprints)
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        vehicle.set_autopilot(True)
        print(f"Using vehicle: {vehicle_bp.id}")

        # Camera settings
        camera_settings = [
            # Front camera
            {
                'width': 1920,  # Higher resolution
                'height': 1080,
                'fov': 90,      # Wider FOV
                'location': carla.Location(x=1.5, z=2.4),
                'rotation': carla.Rotation(pitch=0),
                'dir': 'front'
            },
            # Rear camera
            {
                'width': 1280,
                'height': 720,
                'fov': 120,     # Wider FOV for rear view
                'location': carla.Location(x=-1.5, z=2.4),
                'rotation': carla.Rotation(yaw=180, pitch=0),
                'dir': 'rear'
            }
        ]

        # Set up cameras
        cameras = []
        for setting in camera_settings:
            camera_bp, camera_transform = setup_camera(
                blueprint_library,
                width=setting['width'],
                height=setting['height'],
                fov=setting['fov'],
                location=setting['location'],
                rotation=setting['rotation']
            )
            
            # Create save function with directory
            def make_save_function(cam_dir):
                def save_image(image, cam_dir=cam_dir):
                    filename = os.path.join(output_dir, cam_dir, f"{image.frame:08d}.png")
                    image.save_to_disk(filename)
                return save_image
            
            # Spawn and attach camera
            camera = world.spawn_actor(
                camera_bp,
                camera_transform,
                attach_to=vehicle
            )
            camera.listen(make_save_function(setting['dir']))
            cameras.append(camera)
            print(f"Added {setting['dir']} camera: {setting['width']}x{setting['height']}, FOV: {setting['fov']}°")

        # Set up GNSS
        gnss_bp = blueprint_library.find("sensor.other.gnss")
        gnss_transform = carla.Transform(carla.Location(x=1.0, z=2.0))
        gnss = world.spawn_actor(gnss_bp, gnss_transform, attach_to=vehicle)
        
        def log_gnss(event):
            gps_data.append((event.timestamp, event.latitude, event.longitude, event.altitude))
        
        gnss.listen(lambda event: log_gnss(event))

        # Record data
        print("\nRecording data for 20 seconds... (Press Ctrl+C to stop early)")
        try:
            for i in range(20):
                print(f"\rRecording... {i+1}/20 seconds", end="", flush=True)
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopped by user")

    finally:
        # Clean up
        print("\nCleaning up...")
        for camera in cameras:
            camera.stop()
            camera.destroy()
        if 'gnss' in locals():
            gnss.stop()
            gnss.destroy()
        if 'vehicle' in locals():
            vehicle.destroy()
        
        # Save and plot trajectory
        if gps_data:
            with open(os.path.join(output_dir, "gnss_log.txt"), "w") as f:
                for point in gps_data:
                    f.write(f"{point[0]}, {point[1]}, {point[2]}, {point[3]}\n")
            plot_trajectory(gps_data, output_dir)
        
        print(f"\nDone! Data saved to {output_dir}/")

if __name__ == "__main__":
    main()