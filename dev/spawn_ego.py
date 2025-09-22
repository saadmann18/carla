import carla
import random
import time

def main():
    # Connect to CARLA server
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Get blueprint library and list all available vehicle blueprints
    blueprint_library = world.get_blueprint_library()
    vehicle_blueprints = blueprint_library.filter('vehicle.*')
    
    # Print available vehicle blueprints for debugging
    print("\nAvailable vehicle blueprints:")
    for bp in vehicle_blueprints:
        print(f"- {bp.id}")
    
    # Try to find a Tesla Model 3, or fall back to any car if not found
    vehicle_bp = None
    for bp in vehicle_blueprints:
        if 'model3' in bp.id.lower():
            vehicle_bp = bp
            break
    
    if vehicle_bp is None:
        print("\nTesla Model 3 not found, using random vehicle")
        vehicle_bp = random.choice(vehicle_blueprints)

    # Choose random spawn point
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = random.choice(spawn_points)

    # Spawn vehicle
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    print("Ego vehicle spawned:", vehicle.type_id)

    # Enable autopilot
    vehicle.set_autopilot(True)

    # Run for 20 seconds
    time.sleep(80)

    # Clean up
    vehicle.destroy()
    print("Ego vehicle destroyed.")

if __name__ == "__main__":
    main()
