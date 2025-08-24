# 🖐️ Hand Signal Based Robot Control using ROS 2

This project uses **OpenCV**, **MediaPipe Hands**, and **ROS 2** to recognize hand signals from a webcam and control a robot by publishing velocity commands (`/cmd_vel`).  

---

## 🚀 Features
- Detects **hand landmarks** using [MediaPipe Hands](https://developers.google.com/mediapipe/solutions/vision/hand_landmarker).  
- Counts the number of **raised fingers**.  
- Maps raised finger counts to movement commands.  
- Publishes ROS 2 `Twist` messages to `/cmd_vel`.  
- Supports **palm stop signal (5 fingers)** for safety.  
- Default behavior: robot **stops** if no hand is detected.

---

## ✋ Hand Signal Mapping
| Fingers Raised | Command    | Action                                  |
|----------------|-----------|------------------------------------------|
| 1             | Forward   | `linear.x = +0.2`                        |
| 2             | Backward  | `linear.x = -0.2`                        |
| 3             | Left      | `angular.z = +0.5`                       |
| 4             | Right     | `angular.z = -0.5`                       |
| 5 (Palm)      | Stop      | `linear.x = 0`, `angular.z = 0`          |
| None detected | Stop      | Safety fallback                          |

---

## ⚙️ How It Works
1. **Capture Webcam Feed**  
   - OpenCV reads frames from the webcam (`cv2.VideoCapture`).  
   - Each frame is flipped for a *selfie-view*.  

2. **Hand Detection with MediaPipe**  
   - MediaPipe extracts **21 landmarks per hand** (tip, joint, wrist, etc.).  
   - These landmarks are used to determine which fingers are raised.  

3. **Finger Counting**  
   - For each finger:  
     - If the **tip** landmark is above the **PIP joint** → finger is raised.  
   - For the thumb: compare **x coordinates** of landmarks to check if raised.  

4. **Signal Recognition**  
   - Based on the number of raised fingers, a command is selected.  

5. **Publishing ROS 2 Command**  
   - The node publishes a `geometry_msgs/Twist` message to `/cmd_vel`.  
   - This message controls the robot's linear and angular velocity.  

6. **Fail-Safe Mechanism**  
   - If **no hand is detected**, a **Stop** command is published.  
   - If **palm (5 fingers)** is shown, a **Stop** command is published immediately.  

---

## 🛠️ Requirements
Install dependencies:
```bash
pip install opencv-python mediapipe
