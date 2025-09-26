"""
CARLA Ego Vehicle with Sensor Suite (Front/Rear Cameras, GNSS, LiDAR)

This script spawns a vehicle in the CARLA simulator, attaches:
- Front and rear RGB cameras
- GNSS sensor
- LiDAR sensor

The vehicle operates in autopilot mode while collecting sensor data.

Outputs:
    - Images in output_<timestamp>/front/ and output_<timestamp>/rear/
    - GNSS coordinates in output_<timestamp>/gnss_log.txt
    - Trajectory plot in output_<timestamp>/trajectory.png
    - LiDAR point clouds in output_<timestamp>/lidar/ (saved as .npy)
"""

import carla
import random
import time
import os
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt

# =====================================================
# Helpers
# =====================================================
def setup_camera(blueprint_library, width=1280, height=720, fov=110, location=carla.Location(x=1.5, z=2.4), rotation=carla.Rotation()):
    """Set up a camera with specified parameters."""
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(width))
    camera_bp.set_attribute("image_size_y", str(height))
    camera_bp.set_attribute("fov", str(fov))
    camera_transform = carla.Transform(location, rotation)
    return camera_bp, camera_transform

def plot_trajectory(gps_data, output_dir):
    """Plot the vehicle's trajectory from GPS data and save to disk.
    This function is resilient to short runs (few points)."""
    if not gps_data:
        print("No GPS data collected; skipping trajectory plot.")
        return

    lats = [point[1] for point in gps_data]
    longs = [point[2] for point in gps_data]

    plt.figure(figsize=(12, 8))
    if len(gps_data) > 1:
        plt.plot(longs, lats, 'b-', linewidth=2, label='Path')
    plt.scatter(longs[0], lats[0], c='g', s=100, label='Start')
    plt.scatter(longs[-1], lats[-1], c='r', s=100, label='End')

    plt.title('Vehicle Trajectory')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.legend()
    plt.grid(True)
    try:
        plt.axis('equal')
    except Exception:
        # Fallback in case equal scaling fails for degenerate ranges
        pass
    plt.tight_layout()

    plot_path = os.path.join(output_dir, "trajectory.png")
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Trajectory plot saved to {plot_path}")

# =====================================================
# Main
# =====================================================
def main():
    # Create output directory
    base_dir = "output_suite"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(base_dir, f"run_{timestamp}")
    os.makedirs(os.path.join(output_dir, "front"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "rear"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "lidar"), exist_ok=True)
    print(f"Output will be saved to: {os.path.abspath(output_dir)}")

    gps_data = []

    try:
        # Connect to CARLA
        client = carla.Client("localhost", 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        # Spawn ego vehicle
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        vehicle.set_autopilot(True)
        print(f"Using vehicle: {vehicle_bp.id}")

        # Compute dynamic camera placement from vehicle dimensions
        extent = vehicle.bounding_box.extent  # x: half-length, y: half-width, z: half-height
        veh_len = extent.x
        veh_h = extent.z
        is_long_vehicle = (veh_len > 3.5) or ('fire' in vehicle_bp.id.lower())
        print(f"Vehicle extents -> length/2: {veh_len:.2f} m, height/2: {veh_h:.2f} m; long_vehicle={is_long_vehicle}")

        # -------------------------
        # Cameras
        # -------------------------
        # Dynamic, vehicle-size-aware camera positions
        front_x = veh_len + 1.0   # push a bit in front of bumper
        front_z = veh_h + 0.5     # above roofline for clear view
        rear_x = -(veh_len + 1.0) # behind rear bumper
        rear_z = veh_h + 0.5
        if is_long_vehicle:
            # Raise and move the rear cam farther back to avoid obstructions (e.g., ladders)
            rear_x = -(veh_len + 1.5)
            rear_z = veh_h + 1.3

        camera_settings = [
            # Front
            {
                'width': 1920,
                'height': 1080,
                'fov': 90,
                'location': carla.Location(x=front_x, y=0.0, z=front_z),
                'rotation': carla.Rotation(pitch=-3.0),  # slight downward tilt
                'dir': 'front'
            },
            # Rear
            {
                'width': 1280,
                'height': 720,
                'fov': 110,
                'location': carla.Location(x=rear_x, y=0.0, z=rear_z),
                'rotation': carla.Rotation(yaw=180.0, pitch=-5.0),  # slight downward tilt
                'dir': 'rear'
            }
        ]

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
            
            def make_save_function(cam_dir):
                def save_image(image, cam_dir=cam_dir):
                    filename = os.path.join(output_dir, cam_dir, f"{image.frame:08d}.png")
                    image.save_to_disk(filename)
                return save_image
            
            camera = world.spawn_actor(
                camera_bp,
                camera_transform,
                attach_to=vehicle
            )
            camera.listen(make_save_function(setting['dir']))
            cameras.append(camera)
            print(f"Added {setting['dir']} camera")

        # -------------------------
        # GNSS
        # -------------------------
        gnss_bp = blueprint_library.find("sensor.other.gnss")
        # Ensure periodic GNSS updates even in async mode
        gnss_bp.set_attribute("sensor_tick", "0.2")
        gnss_transform = carla.Transform(carla.Location(x=1.0, z=2.0))
        gnss = world.spawn_actor(gnss_bp, gnss_transform, attach_to=vehicle)
        def _on_gnss(event):
            gps_data.append((event.timestamp, event.latitude, event.longitude, event.altitude))
            # occasional progress output
            if len(gps_data) % 50 == 0:
                print(f"GNSS samples collected: {len(gps_data)}")
        gnss.listen(lambda event: _on_gnss(event))
        print("GNSS sensor added")

        # -------------------------
        # LiDAR
        # -------------------------
        lidar_bp = blueprint_library.find("sensor.lidar.ray_cast")
        lidar_bp.set_attribute("range", "50.0")
        lidar_bp.set_attribute("rotation_frequency", "10")
        lidar_bp.set_attribute("channels", "64")
        lidar_bp.set_attribute("points_per_second", "100000")
        lidar_transform = carla.Transform(carla.Location(x=0, z=2.5))
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)

        def save_lidar(data):
            pts = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
            filename = os.path.join(output_dir, "lidar", f"{data.frame:08d}.npy")
            np.save(filename, pts)

        lidar.listen(lambda data: save_lidar(data))
        print("LiDAR sensor added")

        # -------------------------
        # Run recording
        # -------------------------
        print("\nRecording data for 20 seconds... (Press Ctrl+C to stop early)")
        try:
            for i in range(20):
                print(f"\rRecording... {i+1}/20 seconds", end="", flush=True)
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopped by user")

    finally:
        # Cleanup
        print("\nCleaning up...")
        for cam in cameras:
            cam.stop()
            cam.destroy()
        if 'gnss' in locals():
            gnss.stop()
            gnss.destroy()
        if 'lidar' in locals():
            lidar.stop()
            lidar.destroy()
        if 'vehicle' in locals():
            vehicle.destroy()

        # Save GNSS log + trajectory
        if gps_data:
            with open(os.path.join(output_dir, "gnss_log.txt"), "w") as f:
                for point in gps_data:
                    f.write(f"{point[0]}, {point[1]}, {point[2]}, {point[3]}\n")
            plot_trajectory(gps_data, output_dir)

        print(f"\nDone! Data saved to {output_dir}/")

# =====================================================
if __name__ == "__main__":
    main()
