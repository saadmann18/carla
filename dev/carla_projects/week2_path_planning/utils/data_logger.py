"""
Data logging utilities for CSV/JSON logging of path planning experiments
"""

import csv
import json
import os
import time
from typing import Dict, List, Any, Optional, Union
from datetime import datetime
import pickle
import pandas as pd
from pathlib import Path

class DataLogger:
    """Base class for data logging"""
    
    def __init__(self, output_dir: str = "logs", experiment_name: str = None):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        if experiment_name is None:
            experiment_name = f"experiment_{int(time.time())}"
        
        self.experiment_name = experiment_name
        self.start_time = time.time()
        self.log_entries = []
        
    def log(self, data: Dict[str, Any], timestamp: Optional[float] = None):
        """Log a data entry"""
        if timestamp is None:
            timestamp = time.time() - self.start_time
            
        entry = {
            'timestamp': timestamp,
            'datetime': datetime.now().isoformat(),
            **data
        }
        
        self.log_entries.append(entry)
    
    def save(self, filename: Optional[str] = None):
        """Save logged data - to be implemented by subclasses"""
        raise NotImplementedError
    
    def clear(self):
        """Clear logged data"""
        self.log_entries.clear()
        self.start_time = time.time()
    
    def get_summary(self) -> Dict[str, Any]:
        """Get summary statistics of logged data"""
        if not self.log_entries:
            return {}
        
        return {
            'total_entries': len(self.log_entries),
            'experiment_name': self.experiment_name,
            'start_time': self.start_time,
            'duration': time.time() - self.start_time,
            'first_entry': self.log_entries[0]['datetime'],
            'last_entry': self.log_entries[-1]['datetime']
        }

class CSVLogger(DataLogger):
    """CSV data logger for structured data"""
    
    def __init__(self, output_dir: str = "logs", experiment_name: str = None, 
                 fieldnames: Optional[List[str]] = None):
        super().__init__(output_dir, experiment_name)
        self.fieldnames = fieldnames
        self.auto_fieldnames = fieldnames is None
        
    def log(self, data: Dict[str, Any], timestamp: Optional[float] = None):
        """Log data entry and update fieldnames if needed"""
        super().log(data, timestamp)
        
        # Auto-update fieldnames if not specified
        if self.auto_fieldnames and self.log_entries:
            all_keys = set()
            for entry in self.log_entries:
                all_keys.update(entry.keys())
            self.fieldnames = sorted(list(all_keys))
    
    def save(self, filename: Optional[str] = None) -> str:
        """Save data to CSV file"""
        if not self.log_entries:
            print("No data to save")
            return ""
        
        if filename is None:
            filename = f"{self.experiment_name}.csv"
        
        filepath = self.output_dir / filename
        
        with open(filepath, 'w', newline='', encoding='utf-8') as csvfile:
            if self.fieldnames is None:
                # Use keys from first entry if no fieldnames specified
                self.fieldnames = list(self.log_entries[0].keys())
            
            writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
            writer.writeheader()
            
            for entry in self.log_entries:
                # Only write fields that exist in fieldnames
                filtered_entry = {k: v for k, v in entry.items() if k in self.fieldnames}
                writer.writerow(filtered_entry)
        
        print(f"CSV data saved to {filepath}")
        return str(filepath)
    
    def load_from_csv(self, filepath: str) -> List[Dict[str, Any]]:
        """Load data from CSV file"""
        data = []
        try:
            with open(filepath, 'r', encoding='utf-8') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    # Convert numeric strings back to numbers
                    converted_row = {}
                    for key, value in row.items():
                        try:
                            # Try to convert to float
                            converted_row[key] = float(value)
                        except ValueError:
                            # Keep as string if conversion fails
                            converted_row[key] = value
                    data.append(converted_row)
            
            print(f"Loaded {len(data)} entries from {filepath}")
            return data
            
        except Exception as e:
            print(f"Error loading CSV file {filepath}: {e}")
            return []

class JSONLogger(DataLogger):
    """JSON data logger for flexible data structures"""
    
    def __init__(self, output_dir: str = "logs", experiment_name: str = None,
                 pretty_print: bool = True):
        super().__init__(output_dir, experiment_name)
        self.pretty_print = pretty_print
    
    def save(self, filename: Optional[str] = None) -> str:
        """Save data to JSON file"""
        if not self.log_entries:
            print("No data to save")
            return ""
        
        if filename is None:
            filename = f"{self.experiment_name}.json"
        
        filepath = self.output_dir / filename
        
        data_to_save = {
            'experiment_info': {
                'name': self.experiment_name,
                'start_time': self.start_time,
                'total_entries': len(self.log_entries)
            },
            'data': self.log_entries
        }
        
        with open(filepath, 'w', encoding='utf-8') as jsonfile:
            if self.pretty_print:
                json.dump(data_to_save, jsonfile, indent=2, default=str)
            else:
                json.dump(data_to_save, jsonfile, default=str)
        
        print(f"JSON data saved to {filepath}")
        return str(filepath)
    
    def load_from_json(self, filepath: str) -> Dict[str, Any]:
        """Load data from JSON file"""
        try:
            with open(filepath, 'r', encoding='utf-8') as jsonfile:
                data = json.load(jsonfile)
            
            if 'data' in data:
                self.log_entries = data['data']
                print(f"Loaded {len(self.log_entries)} entries from {filepath}")
                return data
            else:
                print(f"Invalid JSON format in {filepath}")
                return {}
                
        except Exception as e:
            print(f"Error loading JSON file {filepath}: {e}")
            return {}

class PickleLogger(DataLogger):
    """Pickle logger for complex Python objects"""
    
    def save(self, filename: Optional[str] = None) -> str:
        """Save data to pickle file"""
        if not self.log_entries:
            print("No data to save")
            return ""
        
        if filename is None:
            filename = f"{self.experiment_name}.pkl"
        
        filepath = self.output_dir / filename
        
        data_to_save = {
            'experiment_info': {
                'name': self.experiment_name,
                'start_time': self.start_time,
                'total_entries': len(self.log_entries)
            },
            'data': self.log_entries
        }
        
        with open(filepath, 'wb') as pklfile:
            pickle.dump(data_to_save, pklfile)
        
        print(f"Pickle data saved to {filepath}")
        return str(filepath)
    
    def load_from_pickle(self, filepath: str) -> Dict[str, Any]:
        """Load data from pickle file"""
        try:
            with open(filepath, 'rb') as pklfile:
                data = pickle.load(pklfile)
            
            if 'data' in data:
                self.log_entries = data['data']
                print(f"Loaded {len(self.log_entries)} entries from {filepath}")
                return data
            else:
                print(f"Invalid pickle format in {filepath}")
                return {}
                
        except Exception as e:
            print(f"Error loading pickle file {filepath}: {e}")
            return {}

class PathPlanningLogger:
    """Specialized logger for path planning experiments"""
    
    def __init__(self, output_dir: str = "logs", experiment_name: str = None):
        self.csv_logger = CSVLogger(output_dir, experiment_name)
        self.json_logger = JSONLogger(output_dir, experiment_name)
        self.pickle_logger = PickleLogger(output_dir, experiment_name)
        
        # Planning-specific data
        self.planning_results = []
        self.vehicle_states = []
        self.control_commands = []
        self.performance_metrics = {}
    
    def log_planning_result(self, algorithm: str, scenario: str, 
                           path: List[Tuple[float, float]], 
                           planning_time: float, success: bool,
                           additional_data: Dict[str, Any] = None):
        """Log path planning result"""
        result = {
            'algorithm': algorithm,
            'scenario': scenario,
            'path_length': self._calculate_path_length(path),
            'path_points': len(path),
            'planning_time': planning_time,
            'success': success,
            'timestamp': time.time()
        }
        
        if additional_data:
            result.update(additional_data)
        
        self.planning_results.append(result)
        self.csv_logger.log(result)
        self.json_logger.log(result)
    
    def log_vehicle_state(self, vehicle_id: int, position: Tuple[float, float, float],
                         velocity: Tuple[float, float, float], 
                         control: Dict[str, float],
                         additional_data: Dict[str, Any] = None):
        """Log vehicle state information"""
        state = {
            'vehicle_id': vehicle_id,
            'pos_x': position[0],
            'pos_y': position[1],
            'pos_z': position[2],
            'vel_x': velocity[0],
            'vel_y': velocity[1],
            'vel_z': velocity[2],
            'speed': (velocity[0]**2 + velocity[1]**2 + velocity[2]**2)**0.5,
            'timestamp': time.time()
        }
        
        state.update(control)
        
        if additional_data:
            state.update(additional_data)
        
        self.vehicle_states.append(state)
        self.csv_logger.log(state)
    
    def log_control_command(self, vehicle_id: int, steering: float, 
                           throttle: float, brake: float,
                           target_speed: float = None,
                           cross_track_error: float = None):
        """Log control command"""
        command = {
            'vehicle_id': vehicle_id,
            'steering': steering,
            'throttle': throttle,
            'brake': brake,
            'timestamp': time.time()
        }
        
        if target_speed is not None:
            command['target_speed'] = target_speed
        if cross_track_error is not None:
            command['cross_track_error'] = cross_track_error
        
        self.control_commands.append(command)
        self.csv_logger.log(command)
    
    def log_performance_metrics(self, algorithm: str, metrics: Dict[str, float]):
        """Log performance metrics for an algorithm"""
        if algorithm not in self.performance_metrics:
            self.performance_metrics[algorithm] = []
        
        metrics_entry = {
            'timestamp': time.time(),
            **metrics
        }
        
        self.performance_metrics[algorithm].append(metrics_entry)
        
        # Also log to general loggers
        log_entry = {'algorithm': algorithm, **metrics_entry}
        self.csv_logger.log(log_entry)
        self.json_logger.log(log_entry)
    
    def _calculate_path_length(self, path: List[Tuple[float, float]]) -> float:
        """Calculate total path length"""
        if len(path) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            total_length += (dx**2 + dy**2)**0.5
        
        return total_length
    
    def save_all(self, base_filename: Optional[str] = None):
        """Save all logged data"""
        if base_filename is None:
            base_filename = f"path_planning_experiment_{int(time.time())}"
        
        # Save individual data types
        self.csv_logger.save(f"{base_filename}_detailed.csv")
        self.json_logger.save(f"{base_filename}_detailed.json")
        
        # Save specialized data
        self._save_planning_results(f"{base_filename}_planning_results.csv")
        self._save_vehicle_states(f"{base_filename}_vehicle_states.csv")
        self._save_control_commands(f"{base_filename}_control_commands.csv")
        self._save_performance_metrics(f"{base_filename}_performance_metrics.json")
    
    def _save_planning_results(self, filename: str):
        """Save planning results to CSV"""
        if not self.planning_results:
            return
        
        filepath = self.csv_logger.output_dir / filename
        df = pd.DataFrame(self.planning_results)
        df.to_csv(filepath, index=False)
        print(f"Planning results saved to {filepath}")
    
    def _save_vehicle_states(self, filename: str):
        """Save vehicle states to CSV"""
        if not self.vehicle_states:
            return
        
        filepath = self.csv_logger.output_dir / filename
        df = pd.DataFrame(self.vehicle_states)
        df.to_csv(filepath, index=False)
        print(f"Vehicle states saved to {filepath}")
    
    def _save_control_commands(self, filename: str):
        """Save control commands to CSV"""
        if not self.control_commands:
            return
        
        filepath = self.csv_logger.output_dir / filename
        df = pd.DataFrame(self.control_commands)
        df.to_csv(filepath, index=False)
        print(f"Control commands saved to {filepath}")
    
    def _save_performance_metrics(self, filename: str):
        """Save performance metrics to JSON"""
        if not self.performance_metrics:
            return
        
        filepath = self.csv_logger.output_dir / filename
        with open(filepath, 'w') as f:
            json.dump(self.performance_metrics, f, indent=2, default=str)
        print(f"Performance metrics saved to {filepath}")
    
    def generate_summary_report(self) -> Dict[str, Any]:
        """Generate summary report of all logged data"""
        report = {
            'experiment_summary': {
                'total_planning_results': len(self.planning_results),
                'total_vehicle_states': len(self.vehicle_states),
                'total_control_commands': len(self.control_commands),
                'algorithms_tested': list(self.performance_metrics.keys())
            }
        }
        
        # Planning results summary
        if self.planning_results:
            df = pd.DataFrame(self.planning_results)
            report['planning_summary'] = {
                'success_rate': df['success'].mean(),
                'avg_planning_time': df['planning_time'].mean(),
                'avg_path_length': df['path_length'].mean(),
                'algorithms': df['algorithm'].unique().tolist(),
                'scenarios': df['scenario'].unique().tolist()
            }
        
        # Performance metrics summary
        if self.performance_metrics:
            report['performance_summary'] = {}
            for algorithm, metrics_list in self.performance_metrics.items():
                if metrics_list:
                    df = pd.DataFrame(metrics_list)
                    numeric_columns = df.select_dtypes(include=[float, int]).columns
                    report['performance_summary'][algorithm] = {
                        col: {
                            'mean': df[col].mean(),
                            'std': df[col].std(),
                            'min': df[col].min(),
                            'max': df[col].max()
                        }
                        for col in numeric_columns if col != 'timestamp'
                    }
        
        return report

def create_test_logger():
    """Create test logger with sample data"""
    logger = PathPlanningLogger("test_logs", "sample_experiment")
    
    # Log some sample planning results
    logger.log_planning_result(
        algorithm="A*",
        scenario="simple",
        path=[(0, 0), (10, 10), (20, 20)],
        planning_time=0.5,
        success=True,
        additional_data={'iterations': 100}
    )
    
    logger.log_planning_result(
        algorithm="RRT",
        scenario="simple", 
        path=[(0, 0), (5, 8), (15, 18), (20, 20)],
        planning_time=1.2,
        success=True,
        additional_data={'iterations': 500}
    )
    
    # Log some vehicle states
    for i in range(10):
        logger.log_vehicle_state(
            vehicle_id=1,
            position=(i*2, i*2, 0),
            velocity=(2, 2, 0),
            control={'steering': 0.1, 'throttle': 0.5, 'brake': 0.0}
        )
    
    # Log performance metrics
    logger.log_performance_metrics("A*", {
        'success_rate': 0.95,
        'avg_planning_time': 0.8,
        'avg_path_length': 25.5
    })
    
    # Save all data
    logger.save_all()
    
    # Generate report
    report = logger.generate_summary_report()
    print("Summary Report:")
    print(json.dumps(report, indent=2, default=str))
    
    return logger

def main():
    """Test data logging functionality"""
    print("Testing data logging utilities...")
    test_logger = create_test_logger()
    print("Data logging test completed!")

if __name__ == "__main__":
    main()
