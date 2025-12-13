"""Gait pattern generator for quadruped locomotion.

This module generates phase-based gait patterns (trot, walk, crawl) for
quadruped robots with terrain-adaptive timing.
"""

import numpy as np
from typing import Dict, List, Tuple
from enum import Enum


class GaitType(Enum):
    """Supported gait types for quadruped locomotion."""
    TROT = "trot"
    WALK = "walk"
    CRAWL = "crawl"
    STAND = "stand"


class GaitGenerator:
    """Generate gait patterns for quadruped robots.
    
    This class generates phase-based leg coordination patterns for different
    gaits. Each leg has a swing/stance phase determined by the gait pattern.
    
    Attributes:
        gait_type: Current gait type
        frequency: Gait cycle frequency in Hz
        duty_factor: Fraction of time each leg is in stance
    """
    
    def __init__(self, gait_type: GaitType = GaitType.TROT, frequency: float = 1.0):
        """Initialize gait generator.
        
        Args:
            gait_type: Type of gait to generate
            frequency: Gait cycle frequency in Hz
        """
        self.gait_type = gait_type
        self.frequency = frequency
        self.cycle_period = 1.0 / frequency
        
        # Leg names
        self.legs = ['front_left', 'front_right', 'rear_left', 'rear_right']
        
        # Define gait patterns (phase offsets and duty factors)
        self._gait_patterns = self._initialize_gait_patterns()
        
        # Current phase (0 to 1 over one cycle)
        self.phase = 0.0
    
    def _initialize_gait_patterns(self) -> Dict:
        """Initialize phase offsets and duty factors for each gait.
        
        Returns:
            Dictionary with gait parameters for each gait type
        """
        return {
            GaitType.TROT: {
                'duty_factor': 0.5,  # 50% stance, 50% swing
                'phase_offsets': {
                    'front_left': 0.0,
                    'front_right': 0.5,
                    'rear_left': 0.5,
                    'rear_right': 0.0
                },
                'description': 'Diagonal legs move together'
            },
            GaitType.WALK: {
                'duty_factor': 0.75,  # 75% stance, 25% swing
                'phase_offsets': {
                    'front_left': 0.0,
                    'front_right': 0.5,
                    'rear_left': 0.75,
                    'rear_right': 0.25
                },
                'description': 'Alternating single leg swing'
            },
            GaitType.CRAWL: {
                'duty_factor': 0.875,  # 87.5% stance, 12.5% swing
                'phase_offsets': {
                    'front_left': 0.0,
                    'front_right': 0.25,
                    'rear_left': 0.75,
                    'rear_right': 0.5
                },
                'description': 'Very stable, slow gait'
            },
            GaitType.STAND: {
                'duty_factor': 1.0,  # Always in stance
                'phase_offsets': {
                    'front_left': 0.0,
                    'front_right': 0.0,
                    'rear_left': 0.0,
                    'rear_right': 0.0
                },
                'description': 'Static standing'
            }
        }
    
    def set_gait(self, gait_type: GaitType, frequency: float = None):
        """Change the current gait pattern.
        
        Args:
            gait_type: New gait type
            frequency: Optional new frequency (Hz)
        """
        self.gait_type = gait_type
        if frequency is not None:
            self.frequency = frequency
            self.cycle_period = 1.0 / frequency
    
    def update_phase(self, dt: float):
        """Update gait phase based on time step.
        
        Args:
            dt: Time step in seconds
        """
        self.phase += dt * self.frequency
        self.phase = self.phase % 1.0  # Keep phase in [0, 1)
    
    def get_leg_phase(self, leg_name: str) -> float:
        """Get current phase for a specific leg.
        
        Args:
            leg_name: Name of the leg
            
        Returns:
            Phase value in [0, 1) for this leg
        """
        pattern = self._gait_patterns[self.gait_type]
        offset = pattern['phase_offsets'][leg_name]
        leg_phase = (self.phase + offset) % 1.0
        return leg_phase
    
    def is_stance_phase(self, leg_name: str) -> bool:
        """Check if leg is in stance phase.
        
        Args:
            leg_name: Name of the leg
            
        Returns:
            True if leg is in stance, False if in swing
        """
        pattern = self._gait_patterns[self.gait_type]
        duty_factor = pattern['duty_factor']
        leg_phase = self.get_leg_phase(leg_name)
        
        return leg_phase < duty_factor
    
    def get_leg_states(self) -> Dict[str, Dict]:
        """Get current state of all legs.
        
        Returns:
            Dictionary with state info for each leg
        """
        states = {}
        pattern = self._gait_patterns[self.gait_type]
        
        for leg in self.legs:
            leg_phase = self.get_leg_phase(leg)
            in_stance = self.is_stance_phase(leg)
            
            # Calculate normalized position within current phase
            if in_stance:
                # Stance phase: 0 to 1 over stance duration
                phase_progress = leg_phase / pattern['duty_factor']
            else:
                # Swing phase: 0 to 1 over swing duration
                phase_progress = (leg_phase - pattern['duty_factor']) / (1.0 - pattern['duty_factor'])
            
            states[leg] = {
                'phase': leg_phase,
                'stance': in_stance,
                'progress': phase_progress
            }
        
        return states
    
    def get_swing_trajectory_point(self, leg_name: str, start_pos: np.ndarray, 
                                  end_pos: np.ndarray, max_height: float = 0.05) -> np.ndarray:
        """Generate point on swing trajectory for a leg.
        
        Uses a parabolic trajectory for foot swing.
        
        Args:
            leg_name: Name of the leg
            start_pos: Starting position [x, y, z]
            end_pos: Ending position [x, y, z]
            max_height: Maximum height of swing trajectory
            
        Returns:
            Current position on swing trajectory
        """
        leg_state = self.get_leg_states()[leg_name]
        
        if leg_state['stance']:
            # In stance, maintain contact
            return start_pos
        
        # Swing phase - use parabolic trajectory
        s = leg_state['progress']  # 0 to 1 during swing
        
        # Linear interpolation in x-y plane
        pos = start_pos + s * (end_pos - start_pos)
        
        # Parabolic height profile: z = h * 4 * s * (1 - s)
        # Maximum at s = 0.5
        height_offset = max_height * 4 * s * (1 - s)
        pos[2] += height_offset
        
        return pos
    
    def select_gait_for_terrain(self, terrain_type: str, velocity: float = 0.0) -> GaitType:
        """Select appropriate gait based on terrain and velocity.
        
        Args:
            terrain_type: Type of terrain (flat, slope, rubble, stairs)
            velocity: Desired forward velocity (m/s)
            
        Returns:
            Recommended gait type
        """
        # Gait selection heuristics
        if velocity < 0.05:
            return GaitType.STAND
        
        if terrain_type == 'flat':
            if velocity > 0.5:
                return GaitType.TROT
            else:
                return GaitType.WALK
        
        elif terrain_type == 'slope':
            # Slower, more stable gait for slopes
            return GaitType.WALK
        
        elif terrain_type == 'rubble':
            # Very stable gait for rough terrain
            return GaitType.CRAWL
        
        elif terrain_type == 'stairs':
            # Careful, deliberate gait for stairs
            return GaitType.CRAWL
        
        else:
            # Default to walk
            return GaitType.WALK
    
    def get_contact_sequence(self, num_steps: int = 4) -> List[List[str]]:
        """Get sequence of leg contacts for planning.
        
        Args:
            num_steps: Number of steps to plan
            
        Returns:
            List of contact sets (which legs are in stance) over time
        """
        pattern = self._gait_patterns[self.gait_type]
        contact_sequence = []
        
        # Sample at regular intervals
        for i in range(num_steps):
            phase = i / num_steps
            contacts = []
            
            for leg in self.legs:
                leg_phase = (phase + pattern['phase_offsets'][leg]) % 1.0
                if leg_phase < pattern['duty_factor']:
                    contacts.append(leg)
            
            contact_sequence.append(contacts)
        
        return contact_sequence


def get_gait_frequency_for_terrain(terrain_type: str, base_frequency: float = 1.0) -> float:
    """Get recommended gait frequency for terrain type.
    
    Args:
        terrain_type: Type of terrain
        base_frequency: Base frequency for flat terrain (Hz)
        
    Returns:
        Recommended frequency (Hz)
    """
    frequency_factors = {
        'flat': 1.0,
        'slope': 0.7,   # Slower on slopes
        'rubble': 0.5,  # Much slower on rubble
        'stairs': 0.4   # Very slow on stairs
    }
    
    factor = frequency_factors.get(terrain_type, 0.8)
    return base_frequency * factor


if __name__ == '__main__':
    # Test gait generator
    print("Testing Gait Generator\n")
    
    generator = GaitGenerator(GaitType.TROT, frequency=1.0)
    
    print(f"Gait: {generator.gait_type.value}")
    print(f"Frequency: {generator.frequency} Hz")
    print(f"Period: {generator.cycle_period:.2f} s\n")
    
    # Simulate one gait cycle
    dt = 0.1  # 10 Hz update
    num_steps = int(generator.cycle_period / dt)
    
    print("Leg states over one cycle:")
    print("-" * 60)
    
    for step in range(num_steps):
        states = generator.get_leg_states()
        
        print(f"Time: {step * dt:.2f}s | Phase: {generator.phase:.2f}")
        for leg, state in states.items():
            phase_str = "STANCE" if state['stance'] else "SWING "
            print(f"  {leg:12s}: {phase_str} (phase: {state['phase']:.2f}, progress: {state['progress']:.2f})")
        print()
        
        generator.update_phase(dt)
    
    # Test gait selection
    print("\nGait selection for terrains:")
    print("-" * 60)
    terrains = ['flat', 'slope', 'rubble', 'stairs']
    velocities = [0.0, 0.3, 0.8]
    
    for terrain in terrains:
        for vel in velocities:
            gait = generator.select_gait_for_terrain(terrain, vel)
            freq = get_gait_frequency_for_terrain(terrain)
            print(f"{terrain:8s} @ {vel:.1f} m/s -> {gait.value:6s} ({freq:.1f} Hz)")
