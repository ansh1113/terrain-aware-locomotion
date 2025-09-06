# File: ~/terrain_locomotion_ws/src/terrain_locomotion/terrain_locomotion/planning/footstep_planner.py

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
from builtin_interfaces.msg import Duration

class FootstepPlannerNode(Node):
    def __init__(self):
        super().__init__('footstep_planner')
        
        # Robot parameters (ANYmal-like dimensions)
        self.body_length = 0.6  # meters
        self.body_width = 0.4
        self.default_step_length = 0.3
        self.default_step_height = 0.05
        self.default_step_duration = 0.5
        
        # Terrain adaptation parameters
        self.terrain_params = {
            'flat': {
                'step_length_factor': 1.0,
                'step_height_factor': 1.0, 
                'step_duration_factor': 1.0,
                'stance_width_factor': 1.0
            },
            'slope': {
                'step_length_factor': 0.7,   # Shorter steps on slopes
                'step_height_factor': 1.2,   # Higher clearance
                'step_duration_factor': 1.3,  # Slower gait
                'stance_width_factor': 1.2   # Wider stance for stability
            },
            'rubble': {
                'step_length_factor': 0.5,   # Very short steps
                'step_height_factor': 2.0,   # High clearance over obstacles
                'step_duration_factor': 1.8,  # Much slower
                'stance_width_factor': 1.3   # Wide stance
            },
            'stairs': {
                'step_length_factor': 0.4,   # Precise small steps
                'step_height_factor': 1.5,   # Moderate clearance
                'step_duration_factor': 2.0,  # Very slow and precise
                'stance_width_factor': 1.1   # Slightly wider
            }
        }
        
        # Current state
        self.current_terrain = 'flat'
        self.terrain_confidence = 1.0
        self.is_planning = False
        self.current_gait_phase = 0
        
        # Foot positions (relative to body center)
        self.foot_positions = {
            'front_left': np.array([self.body_length/2, self.body_width/2, 0]),
            'front_right': np.array([self.body_length/2, -self.body_width/2, 0]),
            'rear_left': np.array([-self.body_length/2, self.body_width/2, 0]),
            'rear_right': np.array([-self.body_length/2, -self.body_width/2, 0])
        }
        
        # Declare parameters
        self.declare_parameter('confidence_threshold', 0.8)
        self.declare_parameter('planning_frequency', 5.0)  # Hz
        
        # Subscribers
        self.terrain_class_sub = self.create_subscription(
            String,
            '/terrain/class',
            self.terrain_callback,
            10
        )
        
        self.terrain_confidence_sub = self.create_subscription(
            Float32,
            '/terrain/confidence',
            self.confidence_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers
        self.footstep_plan_pub = self.create_publisher(
            PoseArray,
            '/footstep_plan',
            10
        )
        
        self.gait_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/gait_trajectory',
            10
        )
        
        # Planning timer
        self.planning_timer = self.create_timer(
            1.0 / self.get_parameter('planning_frequency').value,
            self.planning_callback
        )
        
        self.desired_velocity = Twist()
        
        self.get_logger().info('Footstep Planner initialized')
    
    def terrain_callback(self, msg):
        """Update current terrain type"""
        self.current_terrain = msg.data
        self.get_logger().info(f'Terrain updated: {self.current_terrain}')
    
    def confidence_callback(self, msg):
        """Update terrain classification confidence"""
        self.terrain_confidence = msg.data
        
        # Warn if confidence is low
        threshold = self.get_parameter('confidence_threshold').value
        if self.terrain_confidence < threshold:
            self.get_logger().warn(f'Low terrain confidence: {self.terrain_confidence:.2f}')
    
    def cmd_vel_callback(self, msg):
        """Update desired velocity command"""
        self.desired_velocity = msg
    
    def planning_callback(self):
        """Main planning loop"""
        if not self.is_planning and (abs(self.desired_velocity.linear.x) > 0.01 or 
                                   abs(self.desired_velocity.linear.y) > 0.01 or
                                   abs(self.desired_velocity.angular.z) > 0.01):
            self.plan_footsteps()
    
    def plan_footsteps(self):
        """Generate footstep plan based on current terrain and desired motion"""
        try:
            # Get terrain-adapted parameters
            terrain_config = self.terrain_params.get(self.current_terrain, self.terrain_params['flat'])
            
            # Apply uncertainty handling
            confidence_factor = 1.0
            threshold = self.get_parameter('confidence_threshold').value
            if self.terrain_confidence < threshold:
                # Reduce aggressiveness if uncertain
                confidence_factor = 0.7
                self.get_logger().info('Applying conservative gait due to low confidence')
            
            # Calculate adapted step parameters
            step_length = (self.default_step_length * 
                         terrain_config['step_length_factor'] * 
                         confidence_factor)
            
            step_height = (self.default_step_height * 
                         terrain_config['step_height_factor'])
            
            step_duration = (self.default_step_duration * 
                           terrain_config['step_duration_factor'] / 
                           confidence_factor)
            
            stance_width = (self.body_width * 
                          terrain_config['stance_width_factor'])
            
            # Generate footstep sequence
            footsteps = self.generate_footstep_sequence(
                step_length, step_height, step_duration, stance_width
            )
            
            # Publish footstep plan
            self.publish_footstep_plan(footsteps)
            
            # Generate and publish joint trajectory
            joint_trajectory = self.generate_joint_trajectory(footsteps, step_duration)
            self.publish_joint_trajectory(joint_trajectory)
            
        except Exception as e:
            self.get_logger().error(f'Error in footstep planning: {str(e)}')
    
    def generate_footstep_sequence(self, step_length, step_height, step_duration, stance_width):
        """Generate sequence of footstep poses"""
        footsteps = []
        
        # Trot gait sequence: [FL+RR, FR+RL]
        gait_sequence = [
            ['front_left', 'rear_right'],
            ['front_right', 'rear_left']
        ]
        
        # Calculate number of steps based on desired velocity
        forward_vel = self.desired_velocity.linear.x
        lateral_vel = self.desired_velocity.linear.y
        angular_vel = self.desired_velocity.angular.z
        
        # Determine step direction and number of steps
        num_steps = max(1, int(abs(forward_vel) * 2))  # At least 1 step
        
        for step_idx in range(num_steps):
            phase_idx = step_idx % len(gait_sequence)
            moving_feet = gait_sequence[phase_idx]
            
            for foot_name in moving_feet:
                # Calculate target position
                base_pos = self.foot_positions[foot_name].copy()
                
                # Adjust stance width
                if 'left' in foot_name:
                    base_pos[1] = stance_width / 2
                else:
                    base_pos[1] = -stance_width / 2
                
                # Add forward motion
                forward_offset = forward_vel * step_length
                lateral_offset = lateral_vel * step_length
                
                target_pos = base_pos + np.array([forward_offset, lateral_offset, 0])
                
                # Create footstep pose
                footstep = PoseStamped()
                footstep.header.frame_id = 'base_link'
                footstep.header.stamp = self.get_clock().now().to_msg()
                
                footstep.pose.position.x = target_pos[0]
                footstep.pose.position.y = target_pos[1]
                footstep.pose.position.z = target_pos[2]
                
                # Orientation (flat for now)
                footstep.pose.orientation.w = 1.0
                
                footsteps.append({
                    'pose': footstep,
                    'foot_name': foot_name,
                    'step_height': step_height,
                    'duration': step_duration,
                    'phase': phase_idx
                })
        
        return footsteps
    
    def publish_footstep_plan(self, footsteps):
        """Publish footstep plan for visualization"""
        pose_array = PoseArray()
        pose_array.header.frame_id = 'base_link'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        for footstep in footsteps:
            pose_array.poses.append(footstep['pose'].pose)
        
        self.footstep_plan_pub.publish(pose_array)
        
        self.get_logger().info(f'Published {len(footsteps)} footsteps for terrain: {self.current_terrain}')
    
    def generate_joint_trajectory(self, footsteps, base_duration):
        """Generate joint trajectory from footstep plan"""
        trajectory = JointTrajectory()
        trajectory.header.frame_id = 'base_link'
        trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Joint names (ANYmal-like)
        trajectory.joint_names = [
            'lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint',  # Left Front
            'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint',  # Right Front  
            'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',  # Left Hind
            'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint'   # Right Hind
        ]
        
        # Generate trajectory points
        time_from_start = 0.0
        
        for i, footstep in enumerate(footsteps):
            point = JointTrajectoryPoint()
            
            # Simple inverse kinematics (placeholder)
            # In a real implementation, you'd use proper IK
            joint_angles = self.simple_ik(footstep)
            
            point.positions = joint_angles
            point.velocities = [0.0] * len(trajectory.joint_names)
            point.accelerations = [0.0] * len(trajectory.joint_names)
            
            # Set timing
            time_from_start += footstep['duration']
            point.time_from_start = Duration(sec=int(time_from_start), nanosec=int((time_from_start % 1) * 1e9))
            
            trajectory.points.append(point)
        
        return trajectory
    
    def publish_joint_trajectory(self, trajectory):
        """Publish joint trajectory"""
        self.gait_trajectory_pub.publish(trajectory)
        self.get_logger().info(f'Published trajectory with {len(trajectory.points)} points')
    
    def simple_ik(self, footstep):
        """Simple inverse kinematics (placeholder implementation)"""
        # This is a simplified IK - in practice you'd use proper kinematics
        # For now, return neutral joint positions with small variations
        
        foot_name = footstep['foot_name']
        target_pos = footstep['pose'].pose.position
        
        # Base joint angles for neutral stance
        base_angles = {
            'front_left': [0.1, -0.5, 1.0],   # HAA, HFE, KFE
            'front_right': [-0.1, -0.5, 1.0],
            'rear_left': [0.1, 0.5, -1.0],
            'rear_right': [-0.1, 0.5, -1.0]
        }
        
        # Get base angles for this foot
        if foot_name in base_angles:
            foot_angles = base_angles[foot_name]
        else:
            foot_angles = [0.0, 0.0, 0.0]
        
        # Adjust based on target position (simplified)
        foot_angles[0] += target_pos.y * 0.1  # Lateral adjustment
        foot_angles[1] += target_pos.x * 0.1  # Forward adjustment
        
        # Return all 12 joint angles
        all_angles = []
        for foot in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            if foot == foot_name:
                all_angles.extend(foot_angles)
            else:
                all_angles.extend(base_angles.get(foot, [0.0, 0.0, 0.0]))
        
        return all_angles

def main(args=None):
    rclpy.init(args=args)
    node = FootstepPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()