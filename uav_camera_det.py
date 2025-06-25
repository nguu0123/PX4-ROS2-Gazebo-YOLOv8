import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import requests
from ultralytics import YOLO
import time

# Set this to your actual server URL
SERVER_URL = 'http://172.17.0.1:5000/detections'

class UAVCameraDetector(Node):
    def __init__(self):
        super().__init__('uav_camera_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # or change to match your topic
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # or path to custom model
        self.drone_id = 1

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Run detection with timing
        start_time = time.time()
        results = self.model(cv_image)
        end_time = time.time()

        speed_info = results[0].speed  # {'preprocess': x, 'inference': y, 'postprocess': z}
        detections = results[0].boxes

        # Format detections
        if detections and len(detections.xyxy) > 0:
            formatted = []
            for i, box in enumerate(detections.xyxy):
                x1, y1, x2, y2 = map(float, box[:4])
                conf = float(detections.conf[i])
                cls = int(detections.cls[i])
                formatted.append({
                    "class_id": cls,
                    "confidence": conf,
                    "bbox": [x1, y1, x2, y2]
                })
        else:
            formatted = []

        payload = {
            "drone_id": self.drone_id,
            "timestamp": time.time(),
            "inference_time_ms": round((end_time - start_time) * 1000, 2),
            "speed": {
                "preprocess": round(speed_info['preprocess'], 2),
                "inference": round(speed_info['inference'], 2),
                "postprocess": round(speed_info['postprocess'], 2)
            },
            "detections": formatted
        }

        print("📤 Sending detection data to server:")
        print(payload)

        try:
            response = requests.post(SERVER_URL, json=payload, timeout=1)
            print(f"✅ Server responded: {response.status_code} - {response.text}")
        except Exception as e:
            print(f"❌ Failed to send detection: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UAVCameraDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
