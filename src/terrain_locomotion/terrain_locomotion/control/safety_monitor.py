"""Safety monitoring node for quadruped robot control.

This module monitors robot state for safety violations and can trigger
emergency stops or warnings.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, PoseStamped
from std_msgs.msg import Bool, String
import numpy as np
from typing import Dict, List
import math


class SafetyMonitor(Node):
    """ROS2 node for monitoring robot safety.
    
    Monitors:
    - Joint position limits
    - Joint velocity limits
    - Joint effort (torque) limits
    - Robot stability (CoM position)
    - Excessive tilt angles
    """
    
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Joint names (ANYmal convention)
        self.joint_names = [
            'lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint',  # Left Front
            'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint',  # Right Front
            'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',  # Left Hind
            'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint'   # Right Hind
        ]
        
        # Parameters
        self.declare_parameter('enable_safety_checks', True)
        self.declare_parameter('emergency_stop_on_violation', False)
        self.declare_parameter('max_joint_velocity', 10.0)  # rad/s
        self.declare_parameter('max_joint_torque', 80.0)    # Nm
        self.declare_parameter('max_tilt_angle', 45.0)      # degrees
        self.declare_parameter('check_frequency', 50.0)     # Hz
        
        # Safety thresholds
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        self.max_joint_torque = self.get_parameter('max_joint_torque').value
        self.max_tilt_angle_rad = math.radians(self.get_parameter('max_tilt_angle').value)
        
        # Joint limits (radians) - ANYmal-like limits
        self.joint_position_limits = {
            'haa': (-0.5, 0.5),      # Hip abduction
            'hfe': (-1.8, 1.8),      # Hip flexion
            'kfe': (-2.8, -0.2)      # Knee flexion
        }
        
        # Current state
        self.current_joint_positions = {}
        self.current_joint_velocities = {}
        self.current_joint_efforts = {}
        self.robot_orientation = None
        
        # Safety status
        self.is_safe = True
        self.emergency_stop_triggered = False
        self.violations = []
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Vector3,
            '/imu/orientation',
            self.imu_callback,
            10
        )
        
        # Publishers
        self.safety_status_pub = self.create_publisher(
            Bool,
            '/safety/status',
            10
        )
        
        self.safety_violation_pub = self.create_publisher(
            String,
            '/safety/violations',
            10
        )
        
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/safety/emergency_stop',
            10
        )
        
        # Safety check timer
        check_rate = self.get_parameter('check_frequency').value
        self.safety_timer = self.create_timer(1.0 / check_rate, self.safety_check_callback)
        
        self.get_logger().info('Safety Monitor initialized')
        self.get_logger().info(f'Max joint velocity: {self.max_joint_velocity} rad/s')
        self.get_logger().info(f'Max joint torque: {self.max_joint_torque} Nm')
        self.get_logger().info(f'Max tilt angle: {self.get_parameter("max_tilt_angle").value}°')
    
    def joint_state_callback(self, msg: JointState):
        """Process joint state updates."""
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_names:
                if i < len(msg.position):
                    self.current_joint_positions[joint_name] = msg.position[i]
                
                if i < len(msg.velocity):
                    self.current_joint_velocities[joint_name] = msg.velocity[i]
                
                if i < len(msg.effort):
                    self.current_joint_efforts[joint_name] = msg.effort[i]
    
    def imu_callback(self, msg: Vector3):
        """Process IMU orientation data."""
        self.robot_orientation = np.array([msg.x, msg.y, msg.z])
    
    def safety_check_callback(self):
        """Main safety checking loop."""
        if not self.get_parameter('enable_safety_checks').value:
            return
        
        if self.emergency_stop_triggered:
            return  # Don't check if already in emergency stop
        
        # Clear previous violations
        self.violations = []
        
        # Run all safety checks
        self.check_joint_position_limits()
        self.check_joint_velocity_limits()
        self.check_joint_effort_limits()
        self.check_robot_stability()
        
        # Update safety status
        self.is_safe = len(self.violations) == 0
        
        # Publish status
        status_msg = Bool()
        status_msg.data = self.is_safe
        self.safety_status_pub.publish(status_msg)
        
        # Publish violations if any
        if not self.is_safe:
            violation_msg = String()
            violation_msg.data = '; '.join(self.violations)
            self.safety_violation_pub.publish(violation_msg)
            
            self.get_logger().warn(f'Safety violations detected: {violation_msg.data}')
            
            # Trigger emergency stop if configured
            if self.get_parameter('emergency_stop_on_violation').value:
                self.trigger_emergency_stop()
    
    def check_joint_position_limits(self):
        """Check if any joints are outside position limits."""
        for joint_name, position in self.current_joint_positions.items():
            # Determine joint type (haa, hfe, or kfe)
            joint_type = joint_name.split('_')[-2]  # Extract 'haa', 'hfe', or 'kfe'
            
            if joint_type in self.joint_position_limits:
                min_limit, max_limit = self.joint_position_limits[joint_type]
                
                if position < min_limit or position > max_limit:
                    violation = f'{joint_name} position {math.degrees(position):.1f}° outside limits [{math.degrees(min_limit):.1f}°, {math.degrees(max_limit):.1f}°]'
                    self.violations.append(violation)
    
    def check_joint_velocity_limits(self):
        """Check if any joints exceed velocity limits."""
        for joint_name, velocity in self.current_joint_velocities.items():
            if abs(velocity) > self.max_joint_velocity:
                violation = f'{joint_name} velocity {velocity:.2f} rad/s exceeds limit {self.max_joint_velocity:.2f} rad/s'
                self.violations.append(violation)
    
    def check_joint_effort_limits(self):
        """Check if any joints exceed torque limits."""
        for joint_name, effort in self.current_joint_efforts.items():
            if abs(effort) > self.max_joint_torque:
                violation = f'{joint_name} torque {effort:.1f} Nm exceeds limit {self.max_joint_torque:.1f} Nm'
                self.violations.append(violation)
    
    def check_robot_stability(self):
        """Check robot orientation for excessive tilt."""
        if self.robot_orientation is None:
            return
        
        # Calculate tilt angle from vertical
        # Assuming orientation is roll, pitch, yaw
        roll, pitch, yaw = self.robot_orientation
        
        # Check roll
        if abs(roll) > self.max_tilt_angle_rad:
            violation = f'Roll angle {math.degrees(roll):.1f}° exceeds limit {math.degrees(self.max_tilt_angle_rad):.1f}°'
            self.violations.append(violation)
        
        # Check pitch
        if abs(pitch) > self.max_tilt_angle_rad:
            violation = f'Pitch angle {math.degrees(pitch):.1f}° exceeds limit {math.degrees(self.max_tilt_angle_rad):.1f}°'
            self.violations.append(violation)
    
    def trigger_emergency_stop(self):
        """Trigger emergency stop."""
        if self.emergency_stop_triggered:
            return  # Already triggered
        
        self.emergency_stop_triggered = True
        
        # Publish emergency stop signal
        estop_msg = Bool()
        estop_msg.data = True
        self.emergency_stop_pub.publish(estop_msg)
        
        self.get_logger().error('EMERGENCY STOP TRIGGERED!')
        self.get_logger().error(f'Reason: {"; ".join(self.violations)}')
    
    def reset_emergency_stop(self):
        """Reset emergency stop (callable via service in full implementation)."""
        self.emergency_stop_triggered = False
        self.violations = []
        
        estop_msg = Bool()
        estop_msg.data = False
        self.emergency_stop_pub.publish(estop_msg)
        
        self.get_logger().info('Emergency stop reset')
    
    def get_safety_report(self) -> Dict:
        """Get current safety status report."""
        return {
            'is_safe': self.is_safe,
            'emergency_stop': self.emergency_stop_triggered,
            'violations': self.violations,
            'joint_positions': len(self.current_joint_positions),
            'joint_velocities': len(self.current_joint_velocities),
            'joint_efforts': len(self.current_joint_efforts)
        }


def main(args=None):
    """Run safety monitor node."""
    rclpy.init(args=args)
    node = SafetyMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
