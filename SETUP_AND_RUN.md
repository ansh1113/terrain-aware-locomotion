# Terrain-Aware Locomotion - Setup and Run Guide

## Complete Installation Guide for ROS2 Humble

This guide provides step-by-step instructions to set up and run the terrain-aware locomotion project on your local system.

### Prerequisites

- Ubuntu 22.04 (for ROS2 Humble)
- ROS2 Humble installed
- Python 3.10+
- Gazebo Classic (version 11)

### Step 1: Install ROS2 Humble (if not already installed)

```bash
# Set up sources
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop

# Install additional ROS2 packages
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Step 2: Install Python Dependencies

```bash
# Install Python packages
pip3 install numpy opencv-python scipy PyYAML transforms3d torch torchvision
```

### Step 3: Set Up Workspace

```bash
# Create workspace directory
mkdir -p ~/terrain_locomotion_ws/src
cd ~/terrain_locomotion_ws/src

# Clone or copy the project
# Option 1: If you have the project as a zip, extract it here
# Option 2: Clone from repository
# git clone <repository-url> .

# For this example, we assume the project files are already in this directory
```

### Step 4: Build the Project

```bash
# Navigate to workspace root
cd ~/terrain_locomotion_ws

# Install dependencies using rosdep
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Step 5: Verify Installation

```bash
# Check if packages are available
ros2 pkg list | grep terrain

# You should see:
# terrain_description
# terrain_locomotion
```

## Running the Simulation

### Launch the Complete Simulation

This will launch Gazebo with the comprehensive terrain world, spawn the ANYmal robot, load controllers, and start the walking demo:

```bash
# Source workspace (do this in every new terminal)
source ~/terrain_locomotion_ws/install/setup.bash

# Launch simulation with walking demo
ros2 launch terrain_locomotion simulation.launch.py
```

### What to Expect

1. **Gazebo GUI opens** with a comprehensive terrain world containing:
   - Flat starting platform
   - Gentle slope (10°)
   - Moderate slope (15°)
   - Rubble terrain with obstacles
   - Stairs (5 steps)
   - Final platform (goal)

2. **Robot spawns** on the flat platform at the start

3. **Controllers load** automatically:
   - Joint state broadcaster
   - Joint trajectory controller

4. **Walking demo starts** after 3 seconds:
   - Robot will stand up with a stable pose
   - After 2 seconds, it will start trotting in place
   - You can watch the leg coordination

### Alternative Launch Options

#### Launch without walking demo (just spawn robot):

```bash
# Edit simulation.launch.py to comment out the walking_demo node, or:
ros2 launch terrain_locomotion simulation.launch.py
# Then in another terminal, manually start walking:
source ~/terrain_locomotion_ws/install/setup.bash
ros2 run terrain_locomotion simple_walk_demo
```

#### Launch with different world:

```bash
# Use the simpler terrain world
ros2 launch terrain_locomotion simulation.launch.py world:=terrain_world.world
```

#### Launch without GUI (headless):

```bash
ros2 launch terrain_locomotion simulation.launch.py gui:=false
```

## Monitoring the Robot

### View Joint States

```bash
# In a new terminal
source ~/terrain_locomotion_ws/install/setup.bash
ros2 topic echo /joint_states
```

### View Camera Feed

```bash
# RGB camera
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# Depth camera
ros2 run rqt_image_view rqt_image_view /camera/depth/image_raw
```

### View Robot Model in RViz

```bash
source ~/terrain_locomotion_ws/install/setup.bash
rviz2
```

Then add displays for:
- RobotModel (use /robot_description topic)
- TF
- Camera images

## Testing Individual Components

### Test Elevation Mapper

```bash
source ~/terrain_locomotion_ws/install/setup.bash
ros2 run terrain_locomotion elevation_mapper
```

### Test Terrain Classifier

```bash
source ~/terrain_locomotion_ws/install/setup.bash
ros2 run terrain_locomotion terrain_classifier
```

## Troubleshooting

### Gazebo doesn't start

```bash
# Check if Gazebo is properly installed
gazebo --version

# If not installed:
sudo apt install gazebo
```

### Robot falls through ground

- Check that the world file loaded correctly
- Verify physics parameters in the world file
- Try spawning at higher z position (already set to 0.7m)

### Controllers don't load

```bash
# Check controller manager
ros2 control list_controllers

# Manually load controllers if needed:
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller joint_trajectory_controller
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state joint_trajectory_controller active
```

### Robot is unstable

The URDF has been tuned for stability with:
- Proper inertial properties
- High foot friction (μ = 1.5)
- Damping and friction in joints
- Realistic joint limits

If still unstable, try:
- Increasing foot friction in URDF
- Lowering the gait frequency in `simple_walk_demo.py`
- Adjusting standing pose parameters

### Build errors

```bash
# Clean and rebuild
cd ~/terrain_locomotion_ws
rm -rf build install log
colcon build --symlink-install
```

## Project Structure

```
~/terrain_locomotion_ws/
└── src/
    ├── terrain_description/          # Robot URDF and configuration
    │   ├── urdf/
    │   │   └── anymal_simple.urdf.xacro
    │   └── config/
    │       └── anymal_controllers.yaml
    │
    └── terrain_locomotion/           # Main locomotion package
        ├── terrain_locomotion/       # Python package
        │   ├── perception/          # Elevation mapping, classification
        │   ├── planning/            # Footstep planning, gait generation
        │   ├── control/             # Gait controller, IK, safety
        │   └── simple_walk_demo.py  # Simple walking demo
        ├── launch/
        │   └── simulation.launch.py # Main launch file
        ├── worlds/
        │   ├── comprehensive_terrain.world  # Full terrain course
        │   └── terrain_world.world          # Simple terrain
        └── config/                  # Configuration files
```

## Next Steps

1. **Observe the trotting gait** - The robot demonstrates a basic trot pattern
2. **Tune parameters** - Adjust gait frequency, step length in `simple_walk_demo.py`
3. **Add forward motion** - Modify the demo to move forward through terrain
4. **Integrate perception** - Connect elevation mapper to gait controller
5. **Add terrain classification** - Use CNN to adapt gait to terrain
6. **Implement footstep planning** - Use MoveIt for optimal foot placement

## Performance Metrics

This implementation achieves:
- ✅ Stable standing pose
- ✅ Coordinated trotting gait
- ✅ Proper leg swing trajectories
- ✅ Real-time control at 20 Hz
- ✅ Smooth joint motions

## Advanced Features (To Be Integrated)

The project includes modules for:
- Elevation mapping from depth camera
- CNN-based terrain classification
- Footstep planning with MoveIt2
- Adaptive gait parameters
- Safety monitoring

See individual node files for implementation details.

## Contributing

To extend this project:
1. Add terrain-specific gait adaptations in `gait_generator.py`
2. Implement forward velocity control
3. Add obstacle avoidance
4. Integrate with perception pipeline
5. Tune for real hardware

## References

- ANYmal robot: https://www.anybotics.com/
- ROS2 Control: https://control.ros.org/
- Gazebo: http://gazebosim.org/
- MoveIt2: https://moveit.ros.org/

---

**Author**: Ansh Bhansali  
**Email**: anshbhansali5@gmail.com  
**Institution**: University of Illinois Urbana-Champaign
