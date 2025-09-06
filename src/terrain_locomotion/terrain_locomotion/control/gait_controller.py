# File: ~/terrain_locomotion_ws/src/terrain_locomotion/terrain_locomotion/control/gait_controller.py

import rclpy
from rclpy.node import Node
import numpy as np
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import math
import threading

class GaitControllerNode(Node):
    def __init__(self):
        super().__init__('gait_controller')
        
        # Joint names (matching ANYmal convention)
        self.joint_names = [
            'lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint',  # Left Front
            'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint',  # Right Front  
            'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',  # Left Hind
            'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint'   # Right Hind
        ]
        
        # Initialize joint states
        self.current_joint_positions = [0.0] * len(self.joint_names)
        self.current_joint_velocities = [0.0] * len(self.joint_names)
        self.target_joint_positions = [0.0] * len(self.joint_names)
        
        # Control parameters
        self.kp = 50.0  # Proportional gain
        self.kd = 5.0   # Derivative gain
        
        # Safety limits
        self.max_joint_velocity = 10.0  # rad/s
        self.max_joint_torque = 80.0    # Nm
        
        # Current trajectory
        self.current_trajectory = None
        self.trajectory_start_time = None
        self.trajectory_lock = threading.Lock()
        
        # Execution state
        self.is_executing = False
        self.current_terrain = 'flat'
        
        # Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/gait_trajectory',
            self.trajectory_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.terrain_sub = self.create_subscription(
            String,
            '/terrain/class',
            self.terrain_callback,
            10
        )
        
        # Publishers
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_effort_controller/commands',
            10
        )
        
        self.controller_state_pub = self.create_publisher(
            JointTrajectoryControllerState,
            '/gait_controller/state',
            10
        )
        
        # Control loop timer
        self.control_timer = self.create_timer(0.01, self.control_callback)  # 100Hz
        
        self.get_logger().info('Gait Controller initialized')
    
    def trajectory_callback(self, msg):
        """Receive new trajectory to execute"""
        with self.trajectory_lock:
            self.current_trajectory = msg
            self.trajectory_start_time = self.get_clock().now()
            self.is_executing = True
            
        self.get_logger().info(f'Received trajectory with {len(msg.points)} points')
    
    def joint_state_callback(self, msg):
        """Update current joint states"""
        try:
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in msg.name:
                    joint_idx = msg.name.index(joint_name)
                    self.current_joint_positions[i] = msg.position[joint_idx]
                    if len(msg.velocity) > joint_idx:
                        self.current_joint_velocities[i] = msg.velocity[joint_idx]
        except Exception as e:
            self.get_logger().error(f'Error processing joint states: {str(e)}')
    
    def terrain_callback(self, msg):
        """Update current terrain type"""
        self.current_terrain = msg.data
    
    def control_callback(self):
        """Main control loop"""
        if not self.is_executing or self.current_trajectory is None:
            return
        
        try:
            with self.trajectory_lock:
                # Calculate time since trajectory started
                current_time = self.get_clock().now()
                elapsed_time = (current_time - self.trajectory_start_time).nanoseconds / 1e9
                
                # Get target positions from trajectory
                target_positions = self.interpolate_trajectory(elapsed_time)
                
                if target_positions is None:
                    # Trajectory finished
                    self.is_executing = False
                    self.get_logger().info('Trajectory execution completed')
                    return
                
                # Calculate control commands
                joint_commands = self.calculate_joint_commands(target_positions)
                
                # Apply safety limits
                joint_commands = self.apply_safety_limits(joint_commands)
                
                # Publish commands
                self.publish_joint_commands(joint_commands)
                
                # Publish controller state
                self.publish_controller_state(target_positions, elapsed_time)
                
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')
    
    def interpolate_trajectory(self, elapsed_time):
        """Interpolate target joint positions from trajectory"""
        if not self.current_trajectory.points:
            return None
        
        # Find the appropriate trajectory segment
        for i, point in enumerate(self.current_trajectory.points):
            point_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            
            if elapsed_time <= point_time:
                if i == 0:
                    # Before first point - use first point
                    return point.positions
                else:
                    # Interpolate between previous and current point
                    prev_point = self.current_trajectory.points[i-1]
                    prev_time = prev_point.time_from_start.sec + prev_point.time_from_start.nanosec / 1e9
                    
                    # Linear interpolation
                    alpha = (elapsed_time - prev_time) / (point_time - prev_time)
                    alpha = max(0.0, min(1.0, alpha))  # Clamp to [0,1]
                    
                    interpolated_positions = []
                    for j in range(len(point.positions)):
                        pos = (1.0 - alpha) * prev_point.positions[j] + alpha * point.positions[j]
                        interpolated_positions.append(pos)
                    
                    return interpolated_positions
        
        # After last point - trajectory finished
        return None
    
    def calculate_joint_commands(self, target_positions):
        """Calculate joint torque commands using PD control"""
        joint_commands = []
        
        for i in range(len(self.joint_names)):
            # Position error
            pos_error = target_positions[i] - self.current_joint_positions[i]
            
            # Velocity error (target velocity is 0 for position control)
            vel_error = 0.0 - self.current_joint_velocities[i]
            
            # PD control
            torque_command = self.kp * pos_error + self.kd * vel_error
            
            # Terrain-specific adjustments
            torque_command = self.apply_terrain_adjustments(torque_command, i)
            
            joint_commands.append(torque_command)
        
        return joint_commands
    
    def apply_terrain_adjustments(self, torque_command, joint_index):
        """Apply terrain-specific control adjustments"""
        terrain_factors = {
            'flat': 1.0,
            'slope': 1.2,    # Higher torques for slopes
            'rubble': 1.5,   # Much higher torques for rough terrain
            'stairs': 1.3    # Moderately higher torques for stairs
        }
        
        factor = terrain_factors.get(self.current_terrain, 1.0)
        return torque_command * factor
    
    def apply_safety_limits(self, joint_commands):
        """Apply safety limits to joint commands"""
        limited_commands = []
        
        for i, command in enumerate(joint_commands):
            # Torque limits
            limited_command = max(-self.max_joint_torque, 
                                min(self.max_joint_torque, command))
            
            # Velocity-based torque limiting
            if abs(self.current_joint_velocities[i]) > self.max_joint_velocity * 0.8:
                # Reduce torque if approaching velocity limit
                limited_command *= 0.5
            
            # Position limits (basic safety check)
            joint_pos = self.current_joint_positions[i]
            if abs(joint_pos) > math.pi:  # Basic joint limit
                if (joint_pos > 0 and limited_command > 0) or (joint_pos < 0 and limited_command < 0):
                    limited_command = 0.0  # Stop moving further into limit
            
            limited_commands.append(limited_command)
        
        return limited_commands
    
    def publish_joint_commands(self, joint_commands):
        """Publish joint torque commands"""
        command_msg = Float64MultiArray()
        command_msg.data = joint_commands
        self.joint_command_pub.publish(command_msg)
    
    def publish_controller_state(self, target_positions, elapsed_time):
        """Publish controller state for monitoring"""
        state_msg = JointTrajectoryControllerState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.joint_names = self.joint_names
        
        # Current state
        state_msg.actual.positions = self.current_joint_positions
        state_msg.actual.velocities = self.current_joint_velocities
        
        # Desired state  
        state_msg.desired.positions = target_positions
        state_msg.desired.velocities = [0.0] * len(self.joint_names)
        
        # Errors
        state_msg.error.positions = [target_positions[i] - self.current_joint_positions[i] 
                                   for i in range(len(self.joint_names))]
        state_msg.error.velocities = [0.0 - self.current_joint_velocities[i] 
                                    for i in range(len(self.joint_names))]
        
        self.controller_state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GaitControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()