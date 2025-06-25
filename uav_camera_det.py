import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import requests
from ultralytics import YOLO
import time

SERVER_URL = 'http://172.17.0.1:5000/detections'  # change if needed

class UAVCameraDetector(Node):
    def __init__(self):
        super().__init__('uav_camera_detector')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # use correct model path
        self.subscription = self.create_subscription(
            Image,
            'camera',  # adjust if using /camera/compressed
            self.image_callback,
            10)
        self.get_logger().info("✅ YOLO node initialized and subscribed to camera")

    def image_callback(self, msg):
        self.get_logger().info("📸 Image received")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"❌ cv_bridge failed: {e}")
            return

        try:
            start_time = time.time()
            results = self.model(cv_image)
            end_time = time.time()
            self.get_logger().info("✅ YOLO inference done")
        except Exception as e:
            self.get_logger().error(f"❌ YOLO failed: {e}")
            return

        speed_info = results[0].speed
        detections = results[0].boxes

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
            "drone_id": 1,
            "timestamp": time.time(),
            "inference_time_ms": round((end_time - start_time) * 1000, 2),
            "speed": {
                "preprocess": round(speed_info['preprocess'], 2),
                "inference": round(speed_info['inference'], 2),
                "postprocess": round(speed_info['postprocess'], 2)
            },
            "detections": formatted
        }

        print("\n📤 SENDING PAYLOAD TO SERVER:")
        print(payload)

        try:
            response = requests.post(SERVER_URL, json=payload, timeout=2)
            print(f"✅ Server responded: {response.status_code} - {response.text}")
        except Exception as e:
            print(f"❌ Failed to POST to server: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UAVCameraDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
