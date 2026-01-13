#!/usr/bin/env python3
"""
Simple walking demo for ANYmal quadruped robot.
Generates basic trotting gait and publishes joint trajectories.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import numpy as np


class SimpleWalkingDemo(Node):
    """Node that generates a simple walking gait for the quadruped."""
    
    def __init__(self):
        super().__init__('simple_walking_demo')
        
        # Joint names in order
        self.joint_names = [
            'lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint',  # Left Front
            'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint',  # Right Front
            'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',  # Left Hind
            'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint'   # Right Hind
        ]
        
        # Publisher for joint trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Gait parameters
        self.gait_frequency = 0.8  # Hz (slower for stability)
        self.gait_period = 1.0 / self.gait_frequency
        self.phase = 0.0
        
        # Standing pose (neutral position with bent knees for stability)
        self.standing_pose = {
            'haa': 0.0,   # Hip abduction (0 = neutral)
            'hfe': 0.5,   # Hip flexion (positive = forward)
            'kfe': -1.2   # Knee flexion (negative = bent)
        }
        
        # Gait parameters
        self.step_length = 0.15  # Forward step length (m)
        self.step_height = 0.08  # Step height (m)
        self.lateral_splay = 0.05  # Small lateral movement
        
        # Control loop timer (20 Hz for smooth motion)
        self.control_timer = self.create_timer(0.05, self.control_callback)
        
        # Wait a bit before starting
        self.start_time = self.get_clock().now()
        self.wait_duration = 2.0  # Wait 2 seconds for controllers to load
        
        self.get_logger().info('Simple Walking Demo initialized')
        self.get_logger().info(f'Waiting {self.wait_duration}s for controllers to stabilize...')
    
    def control_callback(self):
        """Main control loop - generates and publishes trajectory."""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        if elapsed < self.wait_duration:
            # Still waiting - send standing pose
            self.publish_standing_pose()
            return
        
        # Update gait phase
        self.phase = ((elapsed - self.wait_duration) * self.gait_frequency) % 1.0
        
        # Generate joint positions for current gait phase
        joint_positions = self.compute_trot_gait(self.phase)
        
        # Publish trajectory
        self.publish_trajectory(joint_positions)
    
    def publish_standing_pose(self):
        """Publish a trajectory to the standing pose."""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = []
        
        # All legs in standing position
        for _ in range(4):  # 4 legs
            point.positions.extend([
                self.standing_pose['haa'],
                self.standing_pose['hfe'],
                self.standing_pose['kfe']
            ])
        
        point.time_from_start = Duration(sec=1, nanosec=0)
        trajectory.points = [point]
        
        self.trajectory_pub.publish(trajectory)
    
    def compute_trot_gait(self, phase):
        """
        Compute joint positions for trot gait.
        
        Trot gait: Diagonal legs move together
        - LF and RH swing together (phase 0.0 - 0.5)
        - RF and LH swing together (phase 0.5 - 1.0)
        
        Args:
            phase: Current gait phase [0, 1)
        
        Returns:
            List of joint positions for all 12 joints
        """
        positions = []
        
        # Define leg pairs for trot
        # Phase offset for each leg (0 = LF/RH group, 0.5 = RF/LH group)
        leg_phases = [0.0, 0.5, 0.5, 0.0]  # LF, RF, LH, RH
        
        for i, leg_phase_offset in enumerate(leg_phases):
            # Calculate this leg's phase
            leg_phase = (phase + leg_phase_offset) % 1.0
            
            # Determine if leg is in swing (0.0-0.5) or stance (0.5-1.0)
            if leg_phase < 0.5:
                # Swing phase
                swing_progress = leg_phase / 0.5  # 0 to 1
                haa, hfe, kfe = self.swing_leg_trajectory(swing_progress, i)
            else:
                # Stance phase
                stance_progress = (leg_phase - 0.5) / 0.5  # 0 to 1
                haa, hfe, kfe = self.stance_leg_trajectory(stance_progress, i)
            
            positions.extend([haa, hfe, kfe])
        
        return positions
    
    def swing_leg_trajectory(self, progress, leg_index):
        """
        Generate joint angles for swing phase.
        
        Args:
            progress: Progress through swing phase [0, 1]
            leg_index: Which leg (0=LF, 1=RF, 2=LH, 3=RH)
        
        Returns:
            Tuple of (haa, hfe, kfe) joint angles
        """
        # Base standing position
        haa = self.standing_pose['haa']
        hfe = self.standing_pose['hfe']
        kfe = self.standing_pose['kfe']
        
        # Add lateral splay for front legs (helps with stability)
        if leg_index < 2:  # Front legs
            lateral = 1.0 if leg_index == 0 else -1.0  # LF positive, RF negative
            haa += lateral * self.lateral_splay * math.sin(progress * math.pi)
        
        # Forward swing motion
        # Move from back position to front position
        forward_motion = self.step_length * (progress - 0.5)
        hfe += forward_motion * 1.0  # Scale for joint angles
        
        # Lift leg during swing (parabolic trajectory)
        lift = self.step_height * 4 * progress * (1 - progress)
        # To lift the foot, we need to extend the knee (less negative kfe)
        kfe += lift * 3.0  # Convert height to knee angle
        
        # Also adjust hip to maintain foot trajectory
        hfe += lift * 0.5
        
        return haa, hfe, kfe
    
    def stance_leg_trajectory(self, progress, leg_index):
        """
        Generate joint angles for stance phase.
        
        Args:
            progress: Progress through stance phase [0, 1]
            leg_index: Which leg (0=LF, 1=RF, 2=LH, 3=RH)
        
        Returns:
            Tuple of (haa, hfe, kfe) joint angles
        """
        # Base standing position
        haa = self.standing_pose['haa']
        hfe = self.standing_pose['hfe']
        kfe = self.standing_pose['kfe']
        
        # During stance, leg moves backward relative to body
        # (body moves forward over the leg)
        backward_motion = self.step_length * (0.5 - progress)
        hfe += backward_motion * 1.0
        
        # Slight compression/extension for shock absorption
        compression = 0.1 * math.sin(progress * math.pi)
        kfe -= compression
        
        return haa, hfe, kfe
    
    def publish_trajectory(self, joint_positions):
        """Publish joint trajectory."""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * len(joint_positions)
        
        # Time to reach this point (small for continuous motion)
        point.time_from_start = Duration(sec=0, nanosec=50000000)  # 50ms
        
        trajectory.points = [point]
        self.trajectory_pub.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleWalkingDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
