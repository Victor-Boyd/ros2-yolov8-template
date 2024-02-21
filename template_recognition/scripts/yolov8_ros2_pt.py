#!/usr/bin/env python3
# THIS FILE NEEDS TO BE CHANGED TO WORK
# look through the file to find the lines with comments above them to change based of your workspace setup
# This file also includes topic names that you may want to change look for the lines with comments above them for this

# Import necessary libraries
from ultralytics import YOLO  # Import the YOLO object detection class
import rclpy  # ROS2 Python library
from rclpy.node import Node  # Base class for ROS2 nodes
from cv_bridge import CvBridge  # Provides an interface between ROS messages and OpenCV
from sensor_msgs.msg import Image  # ROS2 message type for images
from yolov8_msgs.msg import Yolov8Inference, InferenceResult  # Custom message types for YOLOv8 inference results

# Initialize the CvBridge
cv_bridge = CvBridge()

# Define a class for the ROS2 node
class camera_sub(Node):
    
    def __init__(self):
        # Initialize the node with the name 'camera_subscriber'
        super().__init__('camera_subscriber')

        # Load the YOLO model; CHANGE THIS PATH to the location of your YOLOv8 model file
        self.model = YOLO('~/PATH/TO/YOUR/WORKSPACE/yolov8n.pt')

        # Initialize an object to store YOLOv8 inference results
        self.yolov8_inference = Yolov8Inference()

        # Create a subscription to the camera image topic
        # Note: Change 'camera/image_raw' if your camera publishes on a different topic
        self.subscription = self.create_subscription(
            Image,
            # This is where you can change the name of the subscription topic do this if the name of the topic your camera publishes on is different than the one given keep in mind that if you change it here you must also change it in yolov8_ros2_subscriber.py
            'camera/image_raw',
            self.camera_callback,
            10) # The queue size is set to 10
        
        # Create publishers for publishing inference results and annotated images
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    # Callback function for processing images received from the camera
    def camera_callback(self, data):

        # Convert ROS Image message to OpenCV format
        img = cv_bridge.imgmsg_to_cv2(data, "bgr8")
        # Perform object detection using YOLOv8
        results = self.model(img)

        # Prepare the header of the inference message
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

        # Process detection results
        for r in results:
            boxes = r.boxes # Get detected bounding boxes
            for box in boxes:
                # Prepare an InferenceResult message for each detected object
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # Box coordinates
                c = box.cls  # Detected class
                # Assign detection results to the message
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                # Append to the list of detections
                self.yolov8_inference.yolov8_inference.append(self.inference_result)


        # Generate an annotated image with detection results
        annotated_frame = results[0].plot()
        # Convert the annotated image back to a ROS Image message
        img_msg = cv_bridge.cv2_to_imgmsg(annotated_frame)  

        # Publish the annotated image and inference results
        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        # Clear the inference results for the next callback execution
        self.yolov8_inference.yolov8_inference.clear()

# Main function to initialize the ROS2 node and start processing
if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = camera_sub()
    rclpy.spin(camera_subscriber)  # Keep the node alive to continue processing callbacks
    rclpy.shutdown()  # Shutdown ROS2 cleanly
