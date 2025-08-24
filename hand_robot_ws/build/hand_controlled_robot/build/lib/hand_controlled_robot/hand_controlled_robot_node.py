import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class HandSignalNode(Node):
    def __init__(self):
        super().__init__("hand_signal_node")

        # Publisher to /cmd_vel
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Mediapipe hands setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7,
                                         min_tracking_confidence=0.7)
        self.mp_drawing = mp.solutions.drawing_utils

        # OpenCV webcam
        self.cap = cv2.VideoCapture(0)

        # Timer for processing frames
        self.timer = self.create_timer(0.05, self.process_frame)  # ~20 FPS

    def count_fingers(self, hand_landmarks):
        finger_tips = [8, 12, 16, 20]  # Index, Middle, Ring, Pinky tips
        finger_pips = [6, 10, 14, 18]  # PIP joints

        count = 0

        # Thumb (check x instead of y for left hand detection)
        if hand_landmarks.landmark[4].x < hand_landmarks.landmark[3].x:
            count += 1

        # Other fingers
        for tip, pip in zip(finger_tips, finger_pips):
            if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[pip].y:
                count += 1

        return count

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)

        twist = Twist()
        direction = "Stop"  # default = Stop

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(frame, hand_landmarks,
                                               self.mp_hands.HAND_CONNECTIONS)

                fingers = self.count_fingers(hand_landmarks)

                if fingers == 1:
                    direction = "Forward"
                    twist.linear.x = 0.2
                elif fingers == 2:
                    direction = "Backward"
                    twist.linear.x = -0.2
                elif fingers == 3:
                    direction = "Left"
                    twist.angular.z = 0.5
                elif fingers == 4:
                    direction = "Right"
                    twist.angular.z = -0.5
                elif fingers == 5:  # Palm open
                    direction = "Stop"
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

        # Publish command every cycle (default = Stop if no signal)
        self.pub.publish(twist)

        # Show detected signal on the window
        cv2.putText(frame, f"Signal: {direction}", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Hand Signal Control", frame)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HandSignalNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
