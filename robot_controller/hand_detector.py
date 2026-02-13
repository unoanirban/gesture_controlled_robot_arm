# robot_controller/robot_controller/hand_detector.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import cv2
import mediapipe as mp
import numpy as np

class HandDetectorNode(Node):
    def __init__(self):
        super().__init__('hand_detector')
        self.publisher_ = self.create_publisher(Int32, 'finger_count', 10)
        self.get_logger().info('HandDetector node started, opening camera...')
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera (index 0).')
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False,
                                         max_num_hands=1,
                                         min_detection_confidence=0.5,
                                         min_tracking_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils

        # Timer to run detection at ~15-30 Hz
        self.timer = self.create_timer(1.0/20.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Camera frame not read.')
            return

        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)

        finger_count = 0

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            h, w, _ = frame.shape
            # convert to list of (x,y) in pixel coords
            lm = [(int(pt.x * w), int(pt.y * h)) for pt in hand_landmarks.landmark]

            # Fingers tips indexes in Mediapipe: thumb=4, index=8, middle=12, ring=16, pinky=20
            tips = [4, 8, 12, 16, 20]
            pip_ids = {4: 2, 8:6, 12:10, 16:14, 20:18}

            # Determine hand orientation to correctly interpret thumb direction
            # Use x difference between wrist(0) and middle_finger_mcp(9)
            wrist_x = hand_landmarks.landmark[0].x
            mcp9_x = hand_landmarks.landmark[9].x
            is_right = wrist_x < mcp9_x  # approximate

            for tip in tips:
                tip_x = hand_landmarks.landmark[tip].x
                tip_y = hand_landmarks.landmark[tip].y
                pip = pip_ids[tip]
                pip_x = hand_landmarks.landmark[pip].x
                pip_y = hand_landmarks.landmark[pip].y

                # Thumb uses x comparison (left/right); fingers use y (up/down)
                if tip == 4:
                    # Thumb: consider it open if the tip is to the left/right of pip depending on handedness
                    if is_right:
                        if tip_x > pip_x:
                            finger_count += 1
                    else:
                        if tip_x < pip_x:
                            finger_count += 1
                else:
                    # Other fingers: tip y lower than pip y means finger extended (remember origin top-left)
                    if tip_y < pip_y:
                        finger_count += 1

            # draw annotations
            self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        # publish
        msg = Int32()
        msg.data = int(finger_count)
        self.publisher_.publish(msg)
        # debug log (throttled)
        self.get_logger().debug(f'Published finger_count: {finger_count}')
        # show frame
        cv2.putText(frame, f'Fingers: {finger_count}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.imshow('Hand Detector', frame)
        key = cv2.waitKey(1)
        if key == 27:  # ESC to quit
            self.get_logger().info('ESC received, shutting down camera node.')
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = HandDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
