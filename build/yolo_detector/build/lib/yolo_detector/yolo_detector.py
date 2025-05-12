#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.model = YOLO('/home/muhammad/ros2_ws/src/vehicle_detection.pt')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.br = CvBridge()
        self.get_logger().info('YOLO Detector Node has been started')

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run YOLOv8 inference
        results = self.model(frame)
        
        # Draw bounding boxes and labels on the image
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()  # Bounding box coordinates
            confidences = result.boxes.conf.cpu().numpy()  # Confidence scores
            classes = result.boxes.cls.cpu().numpy()  # Class IDs
            
            for box, conf, cls in zip(boxes, confidences, classes):
                if conf > 0.5:  # Confidence threshold
                    x1, y1, x2, y2 = map(int, box)
                    label = f'{self.model.names[int(cls)]} {conf:.2f}'
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Display the image with detections
        cv2.imshow('YOLOv8 Detection', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down YOLO Detector Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()