"""Inverse kinematics for quadruped robots.

This module provides analytical inverse kinematics solutions for 3-DOF
quadruped legs (HAA, HFE, KFE joints).
"""

import numpy as np
from typing import Tuple, Optional, Dict


class QuadrupedIK:
    """Inverse kinematics solver for quadruped robot leg.

    This class provides analytical IK for a 3-DOF leg with:
    - HAA (Hip Abduction/Adduction): rotation about z-axis
    - HFE (Hip Flexion/Extension): rotation about y-axis
    - KFE (Knee Flexion/Extension): rotation about y-axis

    Assumes leg configuration similar to ANYmal/A1 robots.
    """

    def __init__(
        self,
        hip_offset_x: float = 0.0,
        hip_offset_y: float = 0.0,
        hip_offset_z: float = 0.0,
        upper_leg_length: float = 0.25,
        lower_leg_length: float = 0.25
    ):
        """Initialize IK solver with leg geometry.

        Args:
            hip_offset_x: X offset of hip from body center (m)
            hip_offset_y: Y offset of hip from body center (m)
            hip_offset_z: Z offset of hip from body center (m)
            upper_leg_length: Length of upper leg link (m)
            lower_leg_length: Length of lower leg link (m)
        """
        self.hip_offset = np.array([hip_offset_x, hip_offset_y, hip_offset_z])
        self.upper_leg_length = upper_leg_length
        self.lower_leg_length = lower_leg_length

        # Joint limits (radians)
        self.joint_limits = {
            'haa': (-0.4, 0.4),      # Hip abduction: ±23 degrees
            'hfe': (-1.5, 1.5),      # Hip flexion: ±86 degrees
            'kfe': (-2.7, -0.3)      # Knee flexion: -155 to -17 degrees
        }

    def solve(
        self,
        foot_position: np.ndarray,
        leg_side: str = 'left'
    ) -> Optional[Tuple[float, float, float]]:
        """Solve inverse kinematics for a foot position.

        Args:
            foot_position: Target foot position [x, y, z] in body frame (m)
            leg_side: 'left' or 'right' (affects hip abduction direction)

        Returns:
            Tuple of (haa_angle, hfe_angle, kfe_angle) in radians, or None if unreachable
        """
        # Adjust foot position relative to hip
        foot_rel = foot_position - self.hip_offset

        # Extract coordinates
        x, y, z = foot_rel

        # Solve HAA (Hip Abduction/Adduction)
        # Projects onto y-z plane
        r_yz = np.sqrt(y**2 + z**2)

        if r_yz < 1e-6:
            return None  # Singularity

        haa_angle = np.arctan2(y, -z)

        # Adjust sign based on leg side
        if leg_side == 'right':
            haa_angle = -haa_angle

        # Check HAA limits
        if not (self.joint_limits['haa'][0] <=
                haa_angle <= self.joint_limits['haa'][1]):
            return None

        # Distance from HAA joint to foot in x-r plane
        r = r_yz  # Distance in y-z plane

        # Distance in x-r plane (after HAA rotation)
        d = np.sqrt(x**2 + r**2)

        # Check reachability
        l1 = self.upper_leg_length
        l2 = self.lower_leg_length

        if d > (l1 + l2) or d < abs(l1 - l2):
            return None  # Out of reach

        # Solve for KFE using law of cosines
        # cos(kfe) = (l1^2 + l2^2 - d^2) / (2 * l1 * l2)
        cos_kfe = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
        cos_kfe = np.clip(cos_kfe, -1.0, 1.0)  # Numerical stability

        kfe_angle = np.pi - np.arccos(cos_kfe)  # Exterior angle

        # Make knee bend backwards (negative angle)
        kfe_angle = -abs(kfe_angle)

        # Check KFE limits
        if not (self.joint_limits['kfe'][0] <=
                kfe_angle <= self.joint_limits['kfe'][1]):
            return None

        # Solve for HFE
        # Angle from horizontal to foot
        alpha = np.arctan2(-r, x)

        # Angle of upper leg using law of cosines
        cos_beta = (l1**2 + d**2 - l2**2) / (2 * l1 * d)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)

        hfe_angle = alpha + beta

        # Check HFE limits
        if not (self.joint_limits['hfe'][0] <=
                hfe_angle <= self.joint_limits['hfe'][1]):
            return None

        return (haa_angle, hfe_angle, kfe_angle)

    def forward_kinematics(
        self,
        haa_angle: float,
        hfe_angle: float,
        kfe_angle: float
    ) -> np.ndarray:
        """Compute forward kinematics to get foot position.

        Args:
            haa_angle: Hip abduction angle (rad)
            hfe_angle: Hip flexion angle (rad)
            kfe_angle: Knee flexion angle (rad)

        Returns:
            Foot position [x, y, z] in body frame (m)
        """
        l1 = self.upper_leg_length
        l2 = self.lower_leg_length

        # HAA rotation
        cy = np.cos(haa_angle)
        sy = np.sin(haa_angle)

        # Position after upper leg
        x1 = l1 * np.cos(hfe_angle)
        z1 = -l1 * np.sin(hfe_angle)

        # Position after lower leg (relative to upper leg end)
        knee_angle = hfe_angle + kfe_angle
        x2 = l2 * np.cos(knee_angle)
        z2 = -l2 * np.sin(knee_angle)

        # Total position
        x = x1 + x2
        y = 0.0  # In leg frame before HAA rotation
        z = z1 + z2

        # Apply HAA rotation
        y_rot = y * cy - z * sy
        z_rot = y * sy + z * cy

        # Add hip offset
        foot_pos = np.array([x, y_rot, z_rot]) + self.hip_offset

        return foot_pos

    def validate_solution(
        self,
        foot_position: np.ndarray,
        joint_angles: Tuple[float, float, float],
        tolerance: float = 0.01
    ) -> bool:
        """Validate IK solution by checking FK result.

        Args:
            foot_position: Desired foot position
            joint_angles: IK solution (haa, hfe, kfe)
            tolerance: Acceptable position error (m)

        Returns:
            True if solution is valid
        """
        if joint_angles is None:
            return False

        computed_pos = self.forward_kinematics(*joint_angles)
        error = np.linalg.norm(computed_pos - foot_position)

        return error < tolerance


class QuadrupedFullIK:
    """Full inverse kinematics for all four legs of a quadruped.

    Manages IK for all legs with proper leg configurations.
    """

    def __init__(
        self,
        body_length: float = 0.6,
        body_width: float = 0.4,
        upper_leg_length: float = 0.25,
        lower_leg_length: float = 0.25
    ):
        """Initialize full quadruped IK.

        Args:
            body_length: Distance between front and rear hips (m)
            body_width: Distance between left and right hips (m)
            upper_leg_length: Length of upper leg (m)
            lower_leg_length: Length of lower leg (m)
        """
        self.body_length = body_length
        self.body_width = body_width

        # Create IK solver for each leg with appropriate hip offsets
        self.leg_ik = {
            'front_left': QuadrupedIK(
                hip_offset_x=body_length / 2,
                hip_offset_y=body_width / 2,
                hip_offset_z=0.0,
                upper_leg_length=upper_leg_length,
                lower_leg_length=lower_leg_length
            ),
            'front_right': QuadrupedIK(
                hip_offset_x=body_length / 2,
                hip_offset_y=-body_width / 2,
                hip_offset_z=0.0,
                upper_leg_length=upper_leg_length,
                lower_leg_length=lower_leg_length
            ),
            'rear_left': QuadrupedIK(
                hip_offset_x=-body_length / 2,
                hip_offset_y=body_width / 2,
                hip_offset_z=0.0,
                upper_leg_length=upper_leg_length,
                lower_leg_length=lower_leg_length
            ),
            'rear_right': QuadrupedIK(
                hip_offset_x=-body_length / 2,
                hip_offset_y=-body_width / 2,
                hip_offset_z=0.0,
                upper_leg_length=upper_leg_length,
                lower_leg_length=lower_leg_length
            )
        }

        # Leg sides for HAA direction
        self.leg_sides = {
            'front_left': 'left',
            'front_right': 'right',
            'rear_left': 'left',
            'rear_right': 'right'
        }

    def solve_all_legs(
        self,
        foot_positions: Dict[str, np.ndarray]
    ) -> Dict[str, Optional[Tuple[float, float, float]]]:
        """Solve IK for all legs.

        Args:
            foot_positions: Dictionary mapping leg names to target positions

        Returns:
            Dictionary mapping leg names to joint angles (or None if unreachable)
        """
        solutions = {}

        for leg_name, foot_pos in foot_positions.items():
            leg_side = self.leg_sides[leg_name]
            ik_solver = self.leg_ik[leg_name]

            solution = ik_solver.solve(foot_pos, leg_side=leg_side)
            solutions[leg_name] = solution

        return solutions

    def get_neutral_stance(self, height: float = -
                           0.4) -> Dict[str, np.ndarray]:
        """Get neutral standing position for all feet.

        Args:
            height: Height of feet below body (negative value, m)

        Returns:
            Dictionary mapping leg names to foot positions
        """
        foot_positions = {}

        for leg_name, ik_solver in self.leg_ik.items():
            # Foot directly below hip
            foot_pos = ik_solver.hip_offset.copy()
            foot_pos[2] = height  # Set height
            foot_positions[leg_name] = foot_pos

        return foot_positions


def test_ik():
    """Test IK solver with some example positions."""
    print("Testing Quadruped IK\n")
    print("=" * 60)

    # Create IK solver
    ik = QuadrupedIK(
        hip_offset_x=0.3,
        hip_offset_y=0.2,
        hip_offset_z=0.0,
        upper_leg_length=0.25,
        lower_leg_length=0.25
    )

    # Test positions
    test_positions = [
        np.array([0.3, 0.2, -0.4]),   # Straight down
        np.array([0.5, 0.2, -0.3]),   # Forward
        np.array([0.1, 0.2, -0.4]),   # Backward
        np.array([0.3, 0.3, -0.4]),   # Outward
    ]

    for i, target_pos in enumerate(test_positions):
        print(f"\nTest {i + 1}: Target position: {target_pos}")

        # Solve IK
        solution = ik.solve(target_pos, leg_side='left')

        if solution is None:
            print("  ❌ No IK solution found (unreachable)")
            continue

        haa, hfe, kfe = solution
        print("  ✓ IK Solution:")
        print(f"    HAA: {np.degrees(haa):6.2f}°")
        print(f"    HFE: {np.degrees(hfe):6.2f}°")
        print(f"    KFE: {np.degrees(kfe):6.2f}°")

        # Validate with FK
        computed_pos = ik.forward_kinematics(haa, hfe, kfe)
        error = np.linalg.norm(computed_pos - target_pos)
        print(f"  Forward kinematics: {computed_pos}")
        print(f"  Position error: {error * 1000:.2f} mm")

        if error < 0.01:
            print("  ✓ Validation passed")
        else:
            print("  ❌ Validation failed")

    print("\n" + "=" * 60)
    print("\nTesting Full Quadruped IK\n")

    full_ik = QuadrupedFullIK(body_length=0.6, body_width=0.4)

    # Get neutral stance
    neutral_stance = full_ik.get_neutral_stance(height=-0.4)
    print("Neutral stance positions:")
    for leg, pos in neutral_stance.items():
        print(f"  {leg:12s}: {pos}")

    # Solve IK for neutral stance
    solutions = full_ik.solve_all_legs(neutral_stance)
    print("\nNeutral stance joint angles:")
    for leg, angles in solutions.items():
        if angles:
            haa, hfe, kfe = angles
            print(
                f"  {
                    leg:12s}: HAA={
                    np.degrees(haa):6.2f}°, HFE={
                    np.degrees(hfe):6.2f}°, KFE={
                    np.degrees(kfe):6.2f}°")
        else:
            print(f"  {leg:12s}: No solution")


if __name__ == '__main__':
    test_ik()
