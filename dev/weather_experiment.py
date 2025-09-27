import os
import time
import argparse
import subprocess
import json
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime


def run_experiment(weather_preset, output_dir):
    """Run the PID controller with specified weather conditions"""
    print(f"\n=== Starting experiment: {weather_preset} ===")
    
    # Create output directory for this run
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(output_dir, f"{weather_preset}_{timestamp}")
    os.makedirs(run_dir, exist_ok=True)
    
    # Build command
    cmd = [
        "python", "pid_lane_following.py",
        "--output-dir", run_dir,
        "--weather", weather_preset,
        "--target-speed", "5.0",
        "--max-throttle", "1.0",
        "--max-brake", "0.5",
        "--min-speed", "0.5",
        "--speed-Kp", "1.0",
        "--speed-Ki", "0.5",
        "--speed-Kd", "0.2",
        "--steer-Kp", "0.5",
        "--steer-Ki", "0.01",
        "--steer-Kd", "0.2",
        "--sync",
        "--fps", "20"
    ]
    
    print(f"Running: {' '.join(cmd)}")
    
    # Run the command
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print(f"Experiment completed successfully")
        return run_dir
    except subprocess.CalledProcessError as e:
        print(f"Experiment failed with error: {e}")
        print(f"STDERR: {e.stderr}")
        return None

def analyze_results(run_dirs, output_dir):
    """Analyze and plot results from multiple runs"""
    plt.figure(figsize=(15, 10))
    
    # Plot 1: Lane deviation over time
    plt.subplot(2, 1, 1)
    for name, data in run_dirs.items():
        try:
            with open(os.path.join(data['dir'], 'simulation_data.csv'), 'r') as f:
                lines = f.readlines()
                timestamps = []
                errors = []
                for line in lines[1:]:  # Skip header
                    parts = line.strip().split(',')
                    timestamps.append(float(parts[0]))
                    errors.append(abs(float(parts[5])))  # Absolute lateral error
                
                plt.plot(timestamps, errors, label=name)
                data['errors'] = errors
                data['timestamps'] = timestamps
        except Exception as e:
            print(f"Error processing {name}: {e}")
    
    plt.title('Lane Deviation Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Absolute Lateral Error (m)')
    plt.legend()
    plt.grid(True)
    
    # Plot 2: Steering command over time
    plt.subplot(2, 1, 2)
    for name, data in run_dirs.items():
        try:
            with open(os.path.join(data['dir'], 'simulation_data.csv'), 'r') as f:
                lines = f.readlines()
                timestamps = []
                steering = []
                for line in lines[1:]:  # Skip header
                    parts = line.strip().split(',')
                    timestamps.append(float(parts[0]))
                    steering.append(float(parts[8]))  # Steering command
                
                plt.plot(timestamps, steering, label=name)
                data['steering'] = steering
        except Exception as e:
            print(f"Error processing {name}: {e}")
    
    plt.title('Steering Command Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Steering Command (-1 to 1)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    
    # Save the figure
    plot_path = os.path.join(output_dir, 'weather_comparison.png')
    plt.savefig(plot_path)
    plt.close()
    
    print(f"\nAnalysis complete. Results saved to: {plot_path}")

def main():
    parser = argparse.ArgumentParser(description='Run PID controller under different weather conditions')
    parser.add_argument('--output-dir', default='weather_experiments', help='Directory to save results')
    args = parser.parse_args()
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Define weather presets to test
    weather_conditions = {
        'clear': 'ClearNoon',
        'rain': 'HardRainNoon',
        'fog': 'WetCloudyNoon',
        'sun_glare': 'ClearSunset'
    }
    
    # Run experiments
    results = {}
    for name, preset in weather_conditions.items():
        run_dir = run_experiment(preset, args.output_dir)
        if run_dir:
            results[name] = {'dir': run_dir, 'weather': preset}
    
    # Analyze and plot results
    if results:
        analyze_results(results, args.output_dir)
    else:
        print("No successful runs to analyze")

if __name__ == '__main__':
    main()
