"""
Unit tests for PID controller stability and performance
"""

import unittest
import numpy as np
import time
from unittest.mock import Mock, patch
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils.pid_module import PIDController, PIDGains, CascadePIDController, AdaptivePIDController

class TestPIDStability(unittest.TestCase):
    """Test PID controller stability under various conditions"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.gains = PIDGains(kp=1.0, ki=0.1, kd=0.05)
        self.controller = PIDController(self.gains, output_limits=(-1.0, 1.0))
    
    def test_step_response_stability(self):
        """Test PID response to step input"""
        # Simulate step response
        setpoint = 1.0
        current_value = 0.0
        dt = 0.01
        
        values = []
        outputs = []
        
        # Simulate for 5 seconds
        for i in range(500):
            error = setpoint - current_value
            output = self.controller.update(error, time.time())
            
            # Simple first-order plant model
            current_value += output * dt * 2.0  # Plant gain = 2
            
            values.append(current_value)
            outputs.append(output)
        
        # Check stability criteria
        final_value = values[-1]
        self.assertAlmostEqual(final_value, setpoint, delta=0.1)  # Reaches setpoint
        
        # Check for oscillations in final 1 second
        final_values = values[-100:]
        oscillation = np.std(final_values)
        self.assertLess(oscillation, 0.05)  # Low oscillation
    
    def test_overshoot_control(self):
        """Test that PID doesn't have excessive overshoot"""
        gains = PIDGains(kp=0.8, ki=0.05, kd=0.2)  # Well-tuned gains
        controller = PIDController(gains, output_limits=(-1.0, 1.0))
        
        setpoint = 1.0
        current_value = 0.0
        dt = 0.01
        max_value = 0.0
        
        # Simulate step response
        for i in range(300):
            error = setpoint - current_value
            output = controller.update(error)
            
            current_value += output * dt * 2.0
            max_value = max(max_value, current_value)
        
        # Overshoot should be less than 20%
        overshoot = (max_value - setpoint) / setpoint
        self.assertLess(overshoot, 0.2)
    
    def test_disturbance_rejection(self):
        """Test PID response to disturbances"""
        controller = PIDController(self.gains, output_limits=(-1.0, 1.0))
        
        setpoint = 1.0
        current_value = 1.0  # Start at setpoint
        dt = 0.01
        
        # Let system settle
        for i in range(100):
            error = setpoint - current_value
            output = controller.update(error)
            current_value += output * dt * 2.0
        
        # Apply disturbance
        current_value += 0.5  # Step disturbance
        
        # Measure recovery
        recovery_values = []
        for i in range(200):
            error = setpoint - current_value
            output = controller.update(error)
            current_value += output * dt * 2.0
            recovery_values.append(current_value)
        
        # Should recover to within 5% of setpoint
        final_error = abs(recovery_values[-1] - setpoint)
        self.assertLess(final_error, 0.05)
    
    def test_integral_windup_prevention(self):
        """Test integral windup prevention"""
        gains = PIDGains(kp=0.5, ki=0.5, kd=0.0)  # High integral gain
        controller = PIDController(gains, 
                                 output_limits=(-0.5, 0.5),  # Limited output
                                 integral_limits=(-1.0, 1.0))
        
        # Large step input that will saturate output
        large_error = 10.0
        
        # Apply large error for extended time
        for i in range(100):
            output = controller.update(large_error)
            # Output should be saturated
            self.assertLessEqual(abs(output), 0.5)
        
        # Check that integral term is bounded
        stats = controller.get_statistics()
        self.assertLessEqual(abs(stats['current_integral']), 2.0)  # Should be bounded
    
    def test_derivative_kick_prevention(self):
        """Test derivative kick prevention with setpoint changes"""
        gains = PIDGains(kp=1.0, ki=0.1, kd=0.5)  # High derivative gain
        controller = PIDController(gains, derivative_filter_alpha=0.1)
        
        current_value = 0.0
        setpoint = 1.0
        
        # First update - should not have huge derivative kick
        error = setpoint - current_value
        output1 = controller.update(error)
        
        # Output should be reasonable (not huge due to derivative kick)
        self.assertLess(abs(output1), 5.0)
        
        # Sudden setpoint change
        setpoint = 2.0
        error = setpoint - current_value
        output2 = controller.update(error)
        
        # Should not have excessive derivative response
        derivative_component = abs(output2 - output1)
        self.assertLess(derivative_component, 3.0)

class TestPIDPerformance(unittest.TestCase):
    """Test PID controller performance characteristics"""
    
    def test_rise_time(self):
        """Test system rise time with different gains"""
        test_cases = [
            PIDGains(kp=0.5, ki=0.1, kd=0.05),  # Conservative
            PIDGains(kp=2.0, ki=0.2, kd=0.1),   # Aggressive
        ]
        
        for gains in test_cases:
            controller = PIDController(gains, output_limits=(-1.0, 1.0))
            
            setpoint = 1.0
            current_value = 0.0
            dt = 0.01
            rise_time = None
            
            for i in range(500):
                error = setpoint - current_value
                output = controller.update(error)
                current_value += output * dt * 2.0
                
                # Check if reached 90% of setpoint
                if rise_time is None and current_value >= 0.9 * setpoint:
                    rise_time = i * dt
                    break
            
            # Rise time should be reasonable
            self.assertIsNotNone(rise_time)
            self.assertLess(rise_time, 2.0)  # Less than 2 seconds
    
    def test_settling_time(self):
        """Test system settling time"""
        gains = PIDGains(kp=1.0, ki=0.1, kd=0.1)
        controller = PIDController(gains, output_limits=(-1.0, 1.0))
        
        setpoint = 1.0
        current_value = 0.0
        dt = 0.01
        
        values = []
        for i in range(500):
            error = setpoint - current_value
            output = controller.update(error)
            current_value += output * dt * 2.0
            values.append(current_value)
        
        # Find settling time (within 2% of setpoint)
        settling_time = None
        tolerance = 0.02 * setpoint
        
        for i in range(len(values) - 50, -1, -1):  # Check from end backwards
            if abs(values[i] - setpoint) > tolerance:
                settling_time = (i + 50) * dt
                break
        
        if settling_time is None:
            settling_time = 0  # Settled immediately
        
        # Settling time should be reasonable
        self.assertLess(settling_time, 3.0)  # Less than 3 seconds
    
    def test_steady_state_error(self):
        """Test steady-state error elimination"""
        gains = PIDGains(kp=1.0, ki=0.2, kd=0.05)  # Non-zero integral gain
        controller = PIDController(gains, output_limits=(-1.0, 1.0))
        
        setpoint = 1.0
        current_value = 0.0
        dt = 0.01
        
        # Simulate with constant disturbance
        disturbance = 0.1
        
        for i in range(1000):  # Long simulation
            error = setpoint - current_value
            output = controller.update(error)
            # Plant with disturbance
            current_value += (output * 2.0 - disturbance) * dt
        
        # Final steady-state error should be small
        final_error = abs(setpoint - current_value)
        self.assertLess(final_error, 0.02)  # Less than 2% error

class TestCascadePIDController(unittest.TestCase):
    """Test cascade PID controller functionality"""
    
    def test_cascade_controller_creation(self):
        """Test cascade controller creation"""
        outer_gains = PIDGains(kp=2.0, ki=0.1, kd=0.2)
        inner_gains = PIDGains(kp=1.0, ki=0.05, kd=0.1)
        
        cascade = CascadePIDController(outer_gains, inner_gains)
        
        self.assertIsNotNone(cascade.outer_controller)
        self.assertIsNotNone(cascade.inner_controller)
    
    def test_cascade_control_update(self):
        """Test cascade controller update"""
        outer_gains = PIDGains(kp=1.0, ki=0.1, kd=0.05)
        inner_gains = PIDGains(kp=0.8, ki=0.05, kd=0.02)
        
        cascade = CascadePIDController(outer_gains, inner_gains,
                                     outer_limits=(-5.0, 5.0),
                                     inner_limits=(-1.0, 1.0))
        
        # Test update
        outer_error = 2.0  # Position error
        inner_error = 1.0  # Velocity error
        
        output = cascade.update(outer_error, inner_error)
        
        # Output should be within inner limits
        self.assertGreaterEqual(output, -1.0)
        self.assertLessEqual(output, 1.0)
    
    def test_cascade_reset(self):
        """Test cascade controller reset"""
        outer_gains = PIDGains(kp=1.0, ki=0.1, kd=0.05)
        inner_gains = PIDGains(kp=0.8, ki=0.05, kd=0.02)
        
        cascade = CascadePIDController(outer_gains, inner_gains)
        
        # Update controllers to build up internal state
        cascade.update(1.0, 0.5)
        cascade.update(0.8, 0.3)
        
        # Reset
        cascade.reset()
        
        # Check that both controllers are reset
        outer_stats = cascade.outer_controller.get_statistics()
        inner_stats = cascade.inner_controller.get_statistics()
        
        self.assertEqual(outer_stats['update_count'], 0)
        self.assertEqual(inner_stats['update_count'], 0)

class TestAdaptivePIDController(unittest.TestCase):
    """Test adaptive PID controller functionality"""
    
    def test_adaptive_controller_creation(self):
        """Test adaptive controller creation"""
        initial_gains = PIDGains(kp=1.0, ki=0.1, kd=0.05)
        adaptive = AdaptivePIDController(initial_gains, adaptation_rate=0.01)
        
        self.assertEqual(adaptive.gains.kp, 1.0)
        self.assertEqual(adaptive.adaptation_rate, 0.01)
        self.assertEqual(len(adaptive.error_history), 0)
    
    def test_gain_adaptation(self):
        """Test that gains adapt based on error characteristics"""
        initial_gains = PIDGains(kp=1.0, ki=0.1, kd=0.5)
        adaptive = AdaptivePIDController(initial_gains, adaptation_rate=0.1)
        
        original_kd = adaptive.gains.kd
        
        # Create high-variance error pattern (should reduce derivative gain)
        high_variance_errors = [1.0, -0.8, 1.2, -0.9, 1.1, -0.7, 1.3, -1.0, 0.9, -0.8]
        
        for error in high_variance_errors:
            adaptive.update(error)
        
        # Derivative gain should be reduced due to high variance
        self.assertLess(adaptive.gains.kd, original_kd)
    
    def test_error_history_management(self):
        """Test error history management"""
        initial_gains = PIDGains(kp=1.0, ki=0.1, kd=0.05)
        adaptive = AdaptivePIDController(initial_gains, adaptation_rate=0.01)
        adaptive.max_history_length = 5  # Small history for testing
        
        # Add more errors than max history length
        for i in range(10):
            adaptive.update(float(i))
        
        # History should be limited to max length
        self.assertEqual(len(adaptive.error_history), 5)
        
        # Should contain the most recent errors
        expected_recent = [5.0, 6.0, 7.0, 8.0, 9.0]  # abs values of recent errors
        self.assertEqual(adaptive.error_history, expected_recent)

class TestPIDEdgeCases(unittest.TestCase):
    """Test PID controller edge cases and error handling"""
    
    def test_zero_time_delta(self):
        """Test handling of zero time delta"""
        gains = PIDGains(kp=1.0, ki=0.1, kd=0.1)
        controller = PIDController(gains)
        
        current_time = time.time()
        
        # First update
        output1 = controller.update(1.0, current_time)
        
        # Second update with same timestamp (zero delta)
        output2 = controller.update(0.5, current_time)
        
        # Should not crash and should produce reasonable output
        self.assertIsInstance(output2, float)
        self.assertFalse(np.isnan(output2))
        self.assertFalse(np.isinf(output2))
    
    def test_negative_time_delta(self):
        """Test handling of negative time delta"""
        gains = PIDGains(kp=1.0, ki=0.1, kd=0.1)
        controller = PIDController(gains)
        
        current_time = time.time()
        
        # First update
        controller.update(1.0, current_time)
        
        # Second update with earlier timestamp
        output = controller.update(0.5, current_time - 1.0)
        
        # Should handle gracefully
        self.assertIsInstance(output, float)
        self.assertFalse(np.isnan(output))
        self.assertFalse(np.isinf(output))
    
    def test_large_error_values(self):
        """Test handling of very large error values"""
        gains = PIDGains(kp=1.0, ki=0.1, kd=0.1)
        controller = PIDController(gains, output_limits=(-10.0, 10.0))
        
        # Very large error
        large_error = 1e6
        output = controller.update(large_error)
        
        # Output should be clamped to limits
        self.assertGreaterEqual(output, -10.0)
        self.assertLessEqual(output, 10.0)
    
    def test_invalid_gains(self):
        """Test handling of invalid gain values"""
        # Negative gains should raise ValueError
        with self.assertRaises(ValueError):
            PIDGains(kp=-1.0, ki=0.1, kd=0.1)
        
        with self.assertRaises(ValueError):
            PIDGains(kp=1.0, ki=-0.1, kd=0.1)
        
        with self.assertRaises(ValueError):
            PIDGains(kp=1.0, ki=0.1, kd=-0.1)

def run_tests():
    """Run all PID stability tests"""
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test cases
    test_suite.addTest(unittest.makeSuite(TestPIDStability))
    test_suite.addTest(unittest.makeSuite(TestPIDPerformance))
    test_suite.addTest(unittest.makeSuite(TestCascadePIDController))
    test_suite.addTest(unittest.makeSuite(TestAdaptivePIDController))
    test_suite.addTest(unittest.makeSuite(TestPIDEdgeCases))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    return result.wasSuccessful()

if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
