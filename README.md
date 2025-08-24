# Hand Controlled Robot


This project uses **OpenCV**, **MediaPipe Hands**, and **ROS 2** to recognize hand signals from a webcam and control a robot by publishing velocity commands (`/cmd_vel`).  

### Package Structure
```
hand_robot_ws/
├── src/hand_controlled_robot/
│   ├── hand_controlled_robot/
│   │   ├── hand_controlled_robot_node.py    # Main node implementation
│   │   └── launch/
│   │       └── hand_control.launch.py       # Launch file
│   ├── setup.py                             # Package configuration
│   ├── package.xml                          # Package metadata
│   └── README.md                            # Detailed documentation
├── install_dependencies.sh                  # Dependency installation script
├── run_hand_control.sh                      # Automated startup script
├── QUICK_START.md                           # Quick start guide
└── PACKAGE_SUMMARY.md                       # This file
```

## Key Features

### 1. Hand Gesture Recognition
- Uses MediaPipe for real-time hand tracking
- Detects 21 hand landmarks
- Counts extended fingers for gesture recognition

### 2. Robot Control
- Publishes `geometry_msgs/Twist` to `/cmd_vel` topic
- Five gesture mappings:
  - 1 finger → Forward (linear.x = 0.2)
  - 2 fingers → Backward (linear.x = -0.2)
  - 3 fingers → Left turn (angular.z = 0.5)
  - 4 fingers → Right turn (angular.z = -0.5)
  - 5 fingers → Stop (all velocities = 0)

### 3. Visual Feedback
- Real-time camera feed display
- Hand landmark visualization
- Current gesture and action display

## Files Created

### Core Package Files
- **`hand_controlled_robot_node.py`**: Main ROS2 node with hand tracking and robot control
- **`setup.py`**: Package configuration with entry points and dependencies
- **`package.xml`**: ROS2 package metadata and dependencies
- **`requirements.txt`**: Python package dependencies (opencv-python, mediapipe)

### Documentation
- **`README.md`**: Comprehensive documentation with installation, usage, and troubleshooting

### Launch Files
- **`hand_control.launch.py`**: ROS2 launch file for the hand control node

## How to Use


## Prerequisites
- ROS2 Humble installed
- Python 3.8+
- Webcam
- Ignition Gazebo

## Installation (One-time setup)

1. **Clone Repository:**
   ```bash
   git clone https://github.com/nivednivu1997/Hand_Controlled_Robot.git

   
   ```

2. **Build the package:**
   ```bash
   cd hand_robot_ws && colcon build --packages-select hand_controlled_robot
   ```

3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Running the System

**Terminal 1 - Start Gazebo:**
```bash
ign gazebo
```

**Terminal 2 - Start Bridge:**
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/tugbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  --ros-args -r /model/tugbot/cmd_vel:=/cmd_vel
```

**Terminal 3 - Start Hand Control:**
```bash
ros2 run hand_controlled_robot hand_controlled_robot_node
```

## Hand Gestures

- **1 finger** = Forward
- **2 fingers** = Backward
- **3 fingers** = Turn Left
- **4 fingers** = Turn Right
- **5 fingers (open palm)** = Stop

### ROS2 Packages
- `rclpy`: ROS2 Python client library
- `geometry_msgs`: Twist message type
- `cv_bridge`: OpenCV integration
- `sensor_msgs`: Sensor message types

### Python Packages
- `opencv-python`: Computer vision library
- `mediapipe`: Hand tracking and gesture recognition
- `numpy`: Numerical computing

### System Dependencies
- `ros-humble-ros-gz-bridge`: ROS2-Gazebo bridge
- `ignition-gazebo`: Simulation environment

## Testing

The package has been tested and verified:
- ✅ Package builds successfully
- ✅ Node is discoverable by ROS2
- ✅ Executable is properly registered
- ✅ Dependencies are correctly specified

## Customization

### Modifying Gesture Mappings
Edit the `process_frame()` method in `hand_controlled_robot_node.py` to change velocity values or add new gestures.

### Changing Robot Model
Update the bridge command to use a different robot model in Gazebo.

### Adding New Features
- Add new gesture detection methods
- Implement additional control modes
- Add parameter configuration
- Include logging and debugging features

## Next Steps

1. **Test with real robot**: Replace Gazebo simulation with physical robot
2. **Add safety features**: Implement emergency stop and velocity limits
3. **Improve gesture recognition**: Add more complex hand gestures
4. **Add configuration**: Make velocity values and gesture mappings configurable
5. **Add logging**: Implement proper logging and debugging capabilities
