"""
PID Controller module for reuse across different path planning components
"""

import time
import numpy as np
from typing import Tuple, Optional, Dict, Any
from dataclasses import dataclass

@dataclass
class PIDGains:
    """PID controller gains configuration"""
    kp: float = 1.0  # Proportional gain
    ki: float = 0.0  # Integral gain
    kd: float = 0.0  # Derivative gain
    
    def __post_init__(self):
        """Validate gains after initialization"""
        if self.kp < 0:
            raise ValueError("Proportional gain (kp) must be non-negative")
        if self.ki < 0:
            raise ValueError("Integral gain (ki) must be non-negative") 
        if self.kd < 0:
            raise ValueError("Derivative gain (kd) must be non-negative")
    
    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary"""
        return {'kp': self.kp, 'ki': self.ki, 'kd': self.kd}
    
    @classmethod
    def from_dict(cls, data: Dict[str, float]) -> 'PIDGains':
        """Create from dictionary"""
        return cls(kp=data.get('kp', 1.0), 
                  ki=data.get('ki', 0.0), 
                  kd=data.get('kd', 0.0))

class PIDController:
    """Generic PID controller implementation with advanced features"""
    
    def __init__(self, gains: PIDGains, 
                 output_limits: Tuple[float, float] = (-1.0, 1.0),
                 integral_limits: Optional[Tuple[float, float]] = None,
                 derivative_filter_alpha: float = 0.1,
                 sample_time: Optional[float] = None):
        """
        Initialize PID controller
        
        Args:
            gains: PID gains configuration
            output_limits: Min and max output values
            integral_limits: Min and max integral term values (anti-windup)
            derivative_filter_alpha: Low-pass filter coefficient for derivative term
            sample_time: Expected sample time in seconds (None for variable)
        """
        self.gains = gains
        self.output_limits = output_limits
        self.integral_limits = integral_limits
        self.derivative_filter_alpha = derivative_filter_alpha
        self.sample_time = sample_time
        
        # Internal state
        self.reset()
        
        # Statistics
        self.update_count = 0
        self.total_error = 0.0
        self.max_error = 0.0
        
    def reset(self):
        """Reset PID controller internal state"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None
        self.filtered_derivative = 0.0
        
        # Reset statistics
        self.update_count = 0
        self.total_error = 0.0
        self.max_error = 0.0
    
    def update(self, error: float, current_time: Optional[float] = None) -> float:
        """
        Update PID controller with current error
        
        Args:
            error: Current error value
            current_time: Current timestamp (None to use system time)
            
        Returns:
            PID controller output
        """
        if current_time is None:
            current_time = time.time()
        
        # Calculate time delta
        if self.previous_time is None:
            dt = self.sample_time if self.sample_time else 0.01  # Default 10ms
        else:
            dt = current_time - self.previous_time
            if self.sample_time and abs(dt - self.sample_time) > self.sample_time * 0.1:
                # Warn about irregular sampling
                pass
        
        # Prevent division by zero
        if dt <= 0.0:
            dt = 1e-6
        
        # Proportional term
        proportional = self.gains.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        if self.integral_limits:
            self.integral = np.clip(self.integral, 
                                  self.integral_limits[0] / (self.gains.ki + 1e-6),
                                  self.integral_limits[1] / (self.gains.ki + 1e-6))
        
        integral = self.gains.ki * self.integral
        
        # Derivative term with filtering
        derivative_raw = (error - self.previous_error) / dt
        self.filtered_derivative = (self.derivative_filter_alpha * derivative_raw + 
                                  (1 - self.derivative_filter_alpha) * self.filtered_derivative)
        derivative = self.gains.kd * self.filtered_derivative
        
        # Calculate total output
        output = proportional + integral + derivative
        
        # Apply output limits
        output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        # Integral windup prevention (back-calculation)
        if self.gains.ki > 0:
            # If output is saturated, reduce integral term
            if output == self.output_limits[0] or output == self.output_limits[1]:
                # Calculate what integral should be to achieve unsaturated output
                desired_integral = (output - proportional - derivative) / self.gains.ki
                self.integral = desired_integral
        
        # Update state
        self.previous_error = error
        self.previous_time = current_time
        
        # Update statistics
        self.update_count += 1
        self.total_error += abs(error)
        self.max_error = max(self.max_error, abs(error))
        
        return output
    
    def get_components(self, error: float, current_time: Optional[float] = None) -> Dict[str, float]:
        """
        Get individual PID components without updating state
        
        Args:
            error: Current error value
            current_time: Current timestamp
            
        Returns:
            Dictionary with P, I, D components and total output
        """
        if current_time is None:
            current_time = time.time()
        
        # Calculate time delta
        if self.previous_time is None:
            dt = self.sample_time if self.sample_time else 0.01
        else:
            dt = current_time - self.previous_time
        
        if dt <= 0.0:
            dt = 1e-6
        
        # Calculate components
        proportional = self.gains.kp * error
        integral = self.gains.ki * self.integral
        derivative = self.gains.kd * self.filtered_derivative
        
        total = proportional + integral + derivative
        clamped_output = np.clip(total, self.output_limits[0], self.output_limits[1])
        
        return {
            'proportional': proportional,
            'integral': integral,
            'derivative': derivative,
            'total': total,
            'output': clamped_output,
            'error': error,
            'dt': dt
        }
    
    def tune_gains(self, new_gains: PIDGains, reset_state: bool = False):
        """
        Update PID gains
        
        Args:
            new_gains: New PID gains
            reset_state: Whether to reset internal state
        """
        self.gains = new_gains
        if reset_state:
            self.reset()
    
    def get_statistics(self) -> Dict[str, float]:
        """Get controller performance statistics"""
        if self.update_count == 0:
            return {
                'update_count': 0,
                'avg_error': 0.0,
                'max_error': 0.0,
                'current_integral': 0.0
            }
        
        return {
            'update_count': self.update_count,
            'avg_error': self.total_error / self.update_count,
            'max_error': self.max_error,
            'current_integral': self.integral,
            'current_derivative': self.filtered_derivative
        }
    
    def set_output_limits(self, min_output: float, max_output: float):
        """Set new output limits"""
        self.output_limits = (min_output, max_output)
    
    def set_integral_limits(self, min_integral: float, max_integral: float):
        """Set integral term limits for anti-windup"""
        self.integral_limits = (min_integral, max_integral)

class CascadePIDController:
    """Cascade PID controller with inner and outer loops"""
    
    def __init__(self, outer_gains: PIDGains, inner_gains: PIDGains,
                 outer_limits: Tuple[float, float] = (-10.0, 10.0),
                 inner_limits: Tuple[float, float] = (-1.0, 1.0)):
        """
        Initialize cascade PID controller
        
        Args:
            outer_gains: Outer loop PID gains (position/speed control)
            inner_gains: Inner loop PID gains (acceleration/force control)
            outer_limits: Outer loop output limits
            inner_limits: Inner loop output limits (final output)
        """
        self.outer_controller = PIDController(outer_gains, outer_limits)
        self.inner_controller = PIDController(inner_gains, inner_limits)
    
    def update(self, outer_error: float, inner_error: float, 
               current_time: Optional[float] = None) -> float:
        """
        Update cascade controller
        
        Args:
            outer_error: Outer loop error (e.g., position error)
            inner_error: Inner loop error (e.g., velocity error)
            current_time: Current timestamp
            
        Returns:
            Final control output
        """
        # Outer loop generates setpoint for inner loop
        inner_setpoint = self.outer_controller.update(outer_error, current_time)
        
        # Inner loop uses outer loop output as setpoint
        # The inner_error should be (inner_setpoint - current_inner_value)
        final_output = self.inner_controller.update(inner_error, current_time)
        
        return final_output
    
    def reset(self):
        """Reset both controllers"""
        self.outer_controller.reset()
        self.inner_controller.reset()
    
    def get_statistics(self) -> Dict[str, Dict[str, float]]:
        """Get statistics for both controllers"""
        return {
            'outer_loop': self.outer_controller.get_statistics(),
            'inner_loop': self.inner_controller.get_statistics()
        }

class AdaptivePIDController(PIDController):
    """PID controller with adaptive gain tuning"""
    
    def __init__(self, initial_gains: PIDGains, 
                 adaptation_rate: float = 0.01,
                 **kwargs):
        """
        Initialize adaptive PID controller
        
        Args:
            initial_gains: Initial PID gains
            adaptation_rate: Rate of gain adaptation
        """
        super().__init__(initial_gains, **kwargs)
        self.adaptation_rate = adaptation_rate
        self.error_history = []
        self.max_history_length = 100
        
    def update(self, error: float, current_time: Optional[float] = None) -> float:
        """Update with adaptive gain tuning"""
        # Store error history
        self.error_history.append(abs(error))
        if len(self.error_history) > self.max_history_length:
            self.error_history.pop(0)
        
        # Adapt gains based on error characteristics
        if len(self.error_history) >= 10:
            self._adapt_gains()
        
        return super().update(error, current_time)
    
    def _adapt_gains(self):
        """Adapt PID gains based on error history"""
        recent_errors = self.error_history[-10:]
        error_trend = np.mean(recent_errors[-5:]) - np.mean(recent_errors[:5])
        error_variance = np.var(recent_errors)
        
        # Simple adaptation rules
        if error_variance > 0.1:  # High variance - reduce derivative gain
            self.gains.kd *= (1 - self.adaptation_rate)
        
        if error_trend > 0.05:  # Increasing error - increase proportional gain
            self.gains.kp *= (1 + self.adaptation_rate)
        elif error_trend < -0.05:  # Decreasing error - might reduce proportional gain
            self.gains.kp *= (1 - self.adaptation_rate * 0.5)
        
        # Ensure gains stay within reasonable bounds
        self.gains.kp = np.clip(self.gains.kp, 0.1, 10.0)
        self.gains.ki = np.clip(self.gains.ki, 0.0, 2.0)
        self.gains.kd = np.clip(self.gains.kd, 0.0, 1.0)

def create_steering_pid() -> PIDController:
    """Create a PID controller optimized for steering control"""
    gains = PIDGains(kp=1.2, ki=0.05, kd=0.15)
    return PIDController(
        gains=gains,
        output_limits=(-1.0, 1.0),
        integral_limits=(-0.5, 0.5),
        derivative_filter_alpha=0.2
    )

def create_speed_pid() -> PIDController:
    """Create a PID controller optimized for speed control"""
    gains = PIDGains(kp=0.8, ki=0.1, kd=0.05)
    return PIDController(
        gains=gains,
        output_limits=(-1.0, 1.0),
        integral_limits=(-0.3, 0.3),
        derivative_filter_alpha=0.1
    )

def create_position_pid() -> PIDController:
    """Create a PID controller optimized for position control"""
    gains = PIDGains(kp=2.0, ki=0.02, kd=0.3)
    return PIDController(
        gains=gains,
        output_limits=(-10.0, 10.0),
        integral_limits=(-5.0, 5.0),
        derivative_filter_alpha=0.15
    )

def test_pid_controller():
    """Test PID controller functionality"""
    print("Testing PID Controller...")
    
    # Create controller
    gains = PIDGains(kp=1.0, ki=0.1, kd=0.05)
    controller = PIDController(gains, output_limits=(-1.0, 1.0))
    
    # Simulate step response
    setpoint = 1.0
    current_value = 0.0
    dt = 0.01
    
    print("Step Response Test:")
    print("Time\tError\tOutput\tValue")
    
    for i in range(100):
        error = setpoint - current_value
        output = controller.update(error)
        
        # Simple plant model (first-order system)
        current_value += output * dt * 2.0  # Gain of 2
        
        if i % 10 == 0:
            print(f"{i*dt:.2f}\t{error:.3f}\t{output:.3f}\t{current_value:.3f}")
    
    # Print statistics
    stats = controller.get_statistics()
    print(f"\nController Statistics:")
    print(f"Updates: {stats['update_count']}")
    print(f"Average Error: {stats['avg_error']:.3f}")
    print(f"Max Error: {stats['max_error']:.3f}")
    
    print("PID Controller test completed!")

def main():
    """Test PID module functionality"""
    test_pid_controller()

if __name__ == "__main__":
    main()
