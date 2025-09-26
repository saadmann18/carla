"""
Day 5 – Lane Following Framework (No PID)

This script demonstrates how to:
- Spawn an ego vehicle in CARLA.
- Extract the nearest lane center waypoint from the map.
- Compute lane errors:
  * Lateral error: signed distance from ego position to lane center (left negative, right positive)
  * Heading error: difference between ego yaw and lane yaw (degrees, in [-180, 180])
- Drive forward with constant throttle (no steering/PID yet).
- Log the errors over time to a CSV file and print them to the console.

Outputs:
- output_day5/run_<timestamp>/lane_errors.csv

Usage:
- Ensure CARLA server is running.
- python dev/day5_lane_following.py
"""

import os
import time
from datetime import datetime
import csv
import math
import random

import carla
import matplotlib.pyplot as plt


def normalize_angle_deg(angle: float) -> float:
    """Normalize an angle in degrees to [-180, 180]."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def signed_lateral_error(ego_loc: carla.Location, wp_transform: carla.Transform) -> float:
    """Compute signed lateral error (meters) from ego to the waypoint centerline.
    Sign convention: left of lane center is negative, right is positive.
    """
    # Waypoint location and forward unit vector on the lane center
    wp_loc = wp_transform.location
    fwd = wp_transform.get_forward_vector()  # carla.Vector3D

    # Vector from waypoint to ego
    vec = ego_loc - wp_loc

    # Convert to 2D for lateral computation
    fwd_xy = carla.Vector3D(fwd.x, fwd.y, 0.0)
    vec_xy = carla.Vector3D(vec.x, vec.y, 0.0)

    # Perpendicular left vector (rotate forward by +90 deg)
    left_xy = carla.Vector3D(-fwd_xy.y, fwd_xy.x, 0.0)

    # Normalize left vector
    left_len = math.hypot(left_xy.x, left_xy.y)
    if left_len > 1e-6:
        left_xy.x /= left_len
        left_xy.y /= left_len

    # Signed projection of vec onto left direction gives lateral offset
    lat = vec_xy.x * left_xy.x + vec_xy.y * left_xy.y
    return lat


def main():
    # Output directory
    base_dir = "output_day5"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(base_dir, f"run_{timestamp}")
    os.makedirs(output_dir, exist_ok=True)

    # CSV for logging
    csv_path = os.path.join(output_dir, "lane_errors.csv")

    client = None
    world = None
    vehicle = None

    try:
        # Connect to CARLA
        client = carla.Client("localhost", 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        carla_map = world.get_map()
        blueprint_library = world.get_blueprint_library()

        # Spawn ego vehicle
        vehicle_bp = random.choice(blueprint_library.filter("vehicle.*"))
        spawn_point = random.choice(carla_map.get_spawn_points())
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        # Disable autopilot; we'll apply a constant throttle
        vehicle.set_autopilot(False)
        print(f"Spawned vehicle: {vehicle_bp.id}")

        # Prepare logging
        with open(csv_path, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "time_s",
                "x",
                "y",
                "yaw_deg",
                "lane_x",
                "lane_y",
                "lane_yaw_deg",
                "lateral_error_m",
                "heading_error_deg",
            ])

        print("\nDriving straight with constant throttle and logging lane errors for 20 seconds...")
        start_time = time.time()
        duration_s = 20.0
        throttle_value = 0.35  # constant throttle; adjust as desired
        tick_dt = 0.05         # 20 Hz logging
        # Histories for plotting
        t_hist = []
        lat_hist = []
        head_hist = []

        while True:
            now = time.time()
            elapsed = now - start_time
            if elapsed >= duration_s:
                break

            # Apply constant throttle (no steering)
            vehicle.apply_control(carla.VehicleControl(throttle=throttle_value, steer=0.0))

            # Get ego transform
            ego_transform = vehicle.get_transform()
            ego_loc = ego_transform.location
            ego_rot = ego_transform.rotation
            ego_yaw = ego_rot.yaw

            # Nearest driving-lane waypoint
            waypoint = carla_map.get_waypoint(ego_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            wp_tf = waypoint.transform
            lane_loc = wp_tf.location
            lane_yaw = wp_tf.rotation.yaw

            # Errors
            lat_err = signed_lateral_error(ego_loc, wp_tf)
            head_err = normalize_angle_deg(ego_yaw - lane_yaw)

            # Log
            with open(csv_path, mode="a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    f"{elapsed:.3f}",
                    f"{ego_loc.x:.3f}", f"{ego_loc.y:.3f}", f"{ego_yaw:.2f}",
                    f"{lane_loc.x:.3f}", f"{lane_loc.y:.3f}", f"{lane_yaw:.2f}",
                    f"{lat_err:.3f}", f"{head_err:.2f}",
                ])

            print(
                f"t={elapsed:5.2f}s | lateral={lat_err:6.3f} m | heading={head_err:6.2f} deg"
            )

            # Append histories for plotting
            t_hist.append(elapsed)
            lat_hist.append(lat_err)
            head_hist.append(head_err)

            time.sleep(tick_dt)

        print(f"\nSaved log: {csv_path}")

        # Create error plots
        if len(t_hist) >= 2:
            fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
            axes[0].plot(t_hist, lat_hist, 'b-', linewidth=2)
            axes[0].set_ylabel('Lateral error (m)')
            axes[0].grid(True)
            axes[0].axhline(0.0, color='k', linestyle='--', linewidth=1)

            axes[1].plot(t_hist, head_hist, 'r-', linewidth=2)
            axes[1].set_ylabel('Heading error (deg)')
            axes[1].set_xlabel('Time (s)')
            axes[1].grid(True)
            axes[1].axhline(0.0, color='k', linestyle='--', linewidth=1)

            fig.suptitle('Lane Following Errors (No PID)')
            fig.tight_layout(rect=[0, 0.03, 1, 0.95])
            plot_path = os.path.join(output_dir, 'lane_errors.png')
            fig.savefig(plot_path, dpi=150)
            plt.close(fig)
            print(f"Saved error plot: {plot_path}")
        else:
            print("Not enough samples to plot errors.")

    finally:
        # Cleanup
        print("\nCleaning up actors...")
        try:
            if vehicle is not None:
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                time.sleep(0.2)
                vehicle.destroy()
        except Exception as e:
            print(f"Cleanup error: {e}")

        print("Done.")


if __name__ == "__main__":
    main()
