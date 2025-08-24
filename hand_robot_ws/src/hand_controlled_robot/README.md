# Hand Controlled Robot

A ROS2 Python package that enables hand gesture control of a robot using MediaPipe for hand tracking and computer vision. The package provides a ROS2 node that publishes velocity commands to the `/cmd_vel` topic based on detected hand gestures.

## Features

- **Real-time Hand Gesture Recognition**: Uses MediaPipe to detect and track hand landmarks
- **Five Gesture Controls**:
  - 1 finger: Move Forward
  - 2 fingers: Move Backward  
  - 3 fingers: Turn Left
  - 4 fingers: Turn Right
  - 5 fingers (open palm): Stop
- **ROS2 Integration**: Publishes `geometry_msgs/Twist` messages to `/cmd_vel` topic
- **Visual Feedback**: Displays the detected gesture and control signal in real-time

## Prerequisites

- ROS2 (tested with Humble)
- Python 3.8+
- Webcam
- Ignition Gazebo (for simulation)
- ros_gz_bridge (for ROS2-Gazebo communication)

## Installation

### 1. Install Python Dependencies

```bash
pip3 install opencv-python mediapipe
```

### 2. Install ROS2 Dependencies

```bash
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs ros-humble-ros-gz-bridge
```

## Building the Package

1. Navigate to your workspace:
```bash
cd /home/thedush/hand_robot_ws
```

2. Build the package:
```bash
colcon build --packages-select hand_controlled_robot
```

3. Source the workspace:
```bash
source install/setup.bash
```

## How It Works

### Hand Gesture Detection

The node uses MediaPipe's hand tracking solution to detect hand landmarks in real-time. It processes each frame from the webcam and:

1. **Converts BGR to RGB**: MediaPipe requires RGB format
2. **Detects Hand Landmarks**: Identifies 21 key points on the hand
3. **Counts Extended Fingers**: Analyzes finger positions to determine gesture
4. **Maps Gestures to Commands**: Converts finger count to robot movement

### Finger Counting Algorithm

The algorithm counts extended fingers by comparing landmark positions:
- **Thumb**: Compares x-coordinates of tip (4) and PIP joint (3)
- **Other Fingers**: Compares y-coordinates of tip and PIP joint
- **Extended Finger**: Tip is above PIP joint (lower y-coordinate)

### Velocity Mapping

| Gesture | Fingers | Linear X | Angular Z | Action |
|---------|---------|----------|-----------|---------|
| Forward | 1 | 0.2 | 0.0 | Move forward |
| Backward | 2 | -0.2 | 0.0 | Move backward |
| Left | 3 | 0.0 | 0.5 | Turn left |
| Right | 4 | 0.0 | -0.5 | Turn right |
| Stop | 5 | 0.0 | 0.0 | Stop |

## Running the System

### Step 1: Launch Ignition Gazebo

Open a new terminal and run:
```bash
ign gazebo
```

This will start the Gazebo simulator with the default world.

### Step 2: Start the ROS-Gazebo Bridge

In another terminal, run the parameter bridge to connect ROS2 topics to Gazebo:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/tugbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  --ros-args -r /model/tugbot/cmd_vel:=/cmd_vel
```

This command:
- Bridges the `/cmd_vel` topic from ROS2 to Gazebo
- Maps it to the tugbot model's velocity command
- Remaps the topic name for compatibility

### Step 3: Run the Hand Control Node

In a third terminal, run the hand-controlled robot node:
```bash
ros2 run hand_controlled_robot hand_controlled_robot_node
```

This will:
- Open your webcam
- Start hand gesture detection
- Display the camera feed with gesture overlay
- Publish velocity commands to `/cmd_vel`

## Usage Instructions

1. **Position your hand** in front of the camera
2. **Show gestures** by extending fingers:
   - 1 finger = Forward
   - 2 fingers = Backward
   - 3 fingers = Left turn
   - 4 fingers = Right turn
   - 5 fingers (open palm) = Stop
3. **Monitor the display** to see detected gestures
4. **Press ESC** to quit the application

## Troubleshooting

### Common Issues

1. **Camera not found**:
   - Ensure webcam is connected and working
   - Try changing camera index in code (currently set to 0)

2. **Hand not detected**:
   - Ensure good lighting
   - Keep hand clearly visible in camera frame
   - Check MediaPipe confidence thresholds

3. **Bridge connection issues**:
   - Verify Gazebo is running
   - Check if tugbot model exists in Gazebo world
   - Ensure ros_gz_bridge is properly installed

4. **Package not found**:
   - Make sure to source the workspace: `source install/setup.bash`
   - Verify package is built: `colcon build --packages-select hand_controlled_robot`

### Performance Tips

- **Reduce frame rate** if experiencing lag by changing timer frequency
- **Adjust confidence thresholds** in MediaPipe setup for better detection
- **Use good lighting** for optimal hand tracking
- **Keep hand steady** for more reliable gesture recognition

## Customization

### Modifying Gesture Mappings

Edit the `process_frame()` method in `hand_controlled_robot_node.py` to change velocity values or add new gestures.

### Adding New Gestures

1. Modify the `count_fingers()` method to detect new patterns
2. Add new conditions in `process_frame()` method
3. Update velocity mappings accordingly

### Changing Robot Model

If using a different robot model in Gazebo, update the bridge command:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/YOUR_ROBOT_NAME/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  --ros-args -r /model/YOUR_ROBOT_NAME/cmd_vel:=/cmd_vel
```

## License

This project is licensed under the MIT License.

## Contributing

Feel free to submit issues and enhancement requests!
