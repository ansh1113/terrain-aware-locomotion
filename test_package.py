#!/usr/bin/env python3
"""
Quick test script to verify package installation and basic functionality.
Run this after building the workspace to ensure everything is set up correctly.
"""

import sys
import os

def test_imports():
    """Test if all Python modules can be imported."""
    print("Testing Python module imports...")
    print("-" * 60)
    
    tests = [
        ("rclpy", "ROS2 Python client library"),
        ("numpy", "NumPy"),
        ("cv2", "OpenCV (cv2)"),
        ("scipy", "SciPy"),
        ("yaml", "PyYAML"),
    ]
    
    failed = []
    for module, name in tests:
        try:
            __import__(module)
            print(f"✓ {name:40s} OK")
        except ImportError as e:
            print(f"✗ {name:40s} FAILED: {e}")
            failed.append(name)
    
    print()
    
    # Test project modules
    print("Testing project modules...")
    print("-" * 60)
    
    project_modules = [
        ("terrain_locomotion.perception.elevation_mapper", "Elevation Mapper"),
        ("terrain_locomotion.perception.terrain_classifier", "Terrain Classifier"),
        ("terrain_locomotion.planning.gait_generator", "Gait Generator"),
        ("terrain_locomotion.planning.footstep_planner", "Footstep Planner"),
        ("terrain_locomotion.control.gait_controller", "Gait Controller"),
        ("terrain_locomotion.control.inverse_kinematics", "Inverse Kinematics"),
        ("terrain_locomotion.simple_walk_demo", "Simple Walk Demo"),
    ]
    
    for module, name in project_modules:
        try:
            __import__(module)
            print(f"✓ {name:40s} OK")
        except ImportError as e:
            print(f"✗ {name:40s} FAILED: {e}")
            failed.append(name)
    
    print()
    
    if failed:
        print(f"✗ {len(failed)} modules failed to import")
        print("Failed modules:", ", ".join(failed))
        return False
    else:
        print("✓ All modules imported successfully!")
        return True


def test_ros2_packages():
    """Test if ROS2 packages are available."""
    print("\nTesting ROS2 packages...")
    print("-" * 60)
    
    import subprocess
    
    packages = [
        "terrain_description",
        "terrain_locomotion",
    ]
    
    failed = []
    for pkg in packages:
        try:
            result = subprocess.run(
                ["ros2", "pkg", "prefix", pkg],
                capture_output=True,
                text=True,
                check=True
            )
            print(f"✓ {pkg:40s} OK")
        except subprocess.CalledProcessError:
            print(f"✗ {pkg:40s} NOT FOUND")
            failed.append(pkg)
    
    print()
    
    if failed:
        print(f"✗ {len(failed)} ROS2 packages not found")
        print("Failed packages:", ", ".join(failed))
        print("\nMake sure you have:")
        print("  1. Built the workspace: colcon build --symlink-install")
        print("  2. Sourced the workspace: source install/setup.bash")
        return False
    else:
        print("✓ All ROS2 packages found!")
        return True


def test_ik_solver():
    """Test the IK solver with a simple case."""
    print("\nTesting IK solver...")
    print("-" * 60)
    
    try:
        from terrain_locomotion.control.inverse_kinematics import QuadrupedIK
        import numpy as np
        
        # Create IK solver
        ik = QuadrupedIK(
            hip_offset_x=0.3,
            hip_offset_y=0.2,
            hip_offset_z=0.0,
            upper_leg_length=0.3,
            lower_leg_length=0.3
        )
        
        # Test position (foot directly below hip)
        target = np.array([0.3, 0.2, -0.5])
        solution = ik.solve(target, leg_side='left')
        
        if solution is None:
            print("✗ IK solver failed to find solution")
            return False
        
        # Verify with FK
        computed = ik.forward_kinematics(*solution)
        error = np.linalg.norm(computed - target)
        
        if error < 0.01:  # 1cm tolerance
            print(f"✓ IK solver working (error: {error*1000:.2f}mm)")
            return True
        else:
            print(f"✗ IK solver error too large: {error*1000:.2f}mm")
            return False
            
    except Exception as e:
        print(f"✗ IK solver test failed: {e}")
        return False


def test_gait_generator():
    """Test the gait generator."""
    print("\nTesting gait generator...")
    print("-" * 60)
    
    try:
        from terrain_locomotion.planning.gait_generator import GaitGenerator, GaitType
        
        # Create gait generator
        generator = GaitGenerator(GaitType.TROT, frequency=1.0)
        
        # Update phase
        generator.update_phase(0.1)
        
        # Get leg states
        states = generator.get_leg_states()
        
        if len(states) == 4:
            print(f"✓ Gait generator working (phase: {generator.phase:.2f})")
            return True
        else:
            print("✗ Gait generator returned wrong number of legs")
            return False
            
    except Exception as e:
        print(f"✗ Gait generator test failed: {e}")
        return False


def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("Terrain-Aware Locomotion - Package Verification")
    print("=" * 60)
    print()
    
    results = []
    
    # Run tests
    results.append(("Imports", test_imports()))
    results.append(("ROS2 Packages", test_ros2_packages()))
    results.append(("IK Solver", test_ik_solver()))
    results.append(("Gait Generator", test_gait_generator()))
    
    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    
    for test_name, passed in results:
        status = "✓ PASSED" if passed else "✗ FAILED"
        print(f"{test_name:30s} {status}")
    
    all_passed = all(result[1] for result in results)
    
    print()
    if all_passed:
        print("✓ All tests passed! The package is ready to use.")
        print()
        print("To launch the simulation, run:")
        print("  ros2 launch terrain_locomotion simulation.launch.py")
        return 0
    else:
        print("✗ Some tests failed. Please check the output above.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
