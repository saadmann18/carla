"""
CARLA Ego Vehicle with Sensors

This script spawns a vehicle in the CARLA simulator, attaches sensors (RGB camera and GNSS),
and records sensor data to disk. The vehicle operates in autopilot mode while collecting data.

Outputs:
    - RGB images in output_day3/images/
    - GNSS coordinates in output_day3/gnss_log.txt
"""

import carla
import random
import time
import os
from datetime import datetime

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    # Get available vehicles and print them
    vehicle_blueprints = blueprint_library.filter('vehicle.*')
    print("\nAvailable vehicle blueprints:")
    for bp in vehicle_blueprints:
        print(f"- {bp.id}")
    
    # Try to find a Tesla Model 3, or use the first available vehicle
    vehicle_bp = None
    for bp in vehicle_blueprints:
        if 'model3' in bp.id.lower():
            vehicle_bp = bp
            break
    
    if vehicle_bp is None:
        print("\nTesla Model 3 not found, using first available vehicle")
        vehicle_bp = vehicle_blueprints[0]
    
    print(f"\nUsing vehicle: {vehicle_bp.id}")
    
    # Spawn the vehicle
    spawn_point = random.choice(world.get_map().get_spawn_points())
    try:
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        vehicle.set_autopilot(True)
        print(f"Successfully spawned {vehicle_bp.id}")
    except Exception as e:
        print(f"Failed to spawn vehicle: {e}")
        print("Available spawn points:", len(world.get_map().get_spawn_points()))
        return  # Exit if we can't spawn the vehicle

    # Output folders
    os.makedirs("output_day3/images", exist_ok=True)

    # RGB Camera
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    if camera_bp is None:
        print("Error: Could not find RGB camera blueprint")
        vehicle.destroy()
        return
        
    camera_bp.set_attribute("image_size_x", "800")
    camera_bp.set_attribute("image_size_y", "600")
    camera_bp.set_attribute("fov", "90")
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    def save_image(image):
        filename = f"output_day3/images/{datetime.now().strftime('%H%M%S%f')}.png"
        image.save_to_disk(filename)

    camera.listen(lambda image: save_image(image))

    # GNSS
    gnss_bp = blueprint_library.find("sensor.other.gnss")
    if gnss_bp is None:
        print("Error: Could not find GNSS blueprint")
        camera.destroy()
        vehicle.destroy()
        return
        
    gnss_transform = carla.Transform(carla.Location(x=1.0, z=2.0))
    gnss = world.spawn_actor(gnss_bp, gnss_transform, attach_to=vehicle)
    gps_file = open("output_day3/gnss_log.txt", "w")

    def log_gnss(event):
        gps_file.write(f"{event.timestamp}, {event.latitude}, {event.longitude}, {event.altitude}\n")

    gnss.listen(lambda event: log_gnss(event))

    # Run for 20 seconds
    print("Recording data...")
    try:
        time.sleep(20)
    except KeyboardInterrupt:
        print("Stopped by user")

    # Cleanup
    print("Cleaning up...")
    camera.stop()
    gnss.stop()
    gps_file.close()
    camera.destroy()
    gnss.destroy()
    vehicle.destroy()
    print("Done, data saved to output_day3/")

if __name__ == "__main__":
    main()