import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import requests
import time
import numpy as np

# Adjust this if your drone ID or server IP/port changes
DRONE_ID = 1
SERVER_URL = 'http://172.17.0.1:5000/detections' # host.docker.internal works in Docker Desktop

class DroneDetector(Node):
    def __init__(self):
        super().__init__('drone_detector_node')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # Use any trained model you want

        self.get_logger().info("YOLOv8 detector initialized and subscribed to /camera/image_raw")

    def listener_callback(self, msg):
        try:
            # Convert ROS2 image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Run YOLO detection
        results = self.model(frame)

        # Parse and send detections
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                # Send detection
                detection_data = {
                    'timestamp': time.time(),
                    'drone_id': DRONE_ID,
                    'class_id': class_id,
                    'confidence': conf,
                    'bbox': [x1, y1, x2, y2]
                }

                try:
                    requests.post(SERVER_URL, json=detection_data, timeout=0.5)
                except requests.RequestException as e:
                    self.get_logger().warn(f"Failed to send detection: {e}")

                # Draw detection
                label = f"{self.model.names[class_id]}: {conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Show output
        cv2.imshow("YOLO Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DroneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down DroneDetector")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
