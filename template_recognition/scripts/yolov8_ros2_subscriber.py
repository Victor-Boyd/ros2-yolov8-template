#!/usr/bin/env python3
#NO NEED TO CHANGE ANYTHING IN THIS FILE unless you have more than one camera
#This file defines the nodes camera_subcriber and yolo_subcriber
#If you need to change the names of a topic look for the lines with comments above them keep in mind that if you change the name of a topic here you may need to change the name of the topic in yolov8_ros2_pt.py as well

# Import necessary libraries
import cv2 # cv2 libraries
import threading # allows for the simultanious running of multiple parts of the process
import rclpy  # ROS2 Python library
from rclpy.node import Node  # Base class for ROS2 nodes
from cv_bridge import CvBridge  # Provides an interface between ROS messages and OpenCV
from sensor_msgs.msg import Image  # ROS2 message type for images
from yolov8_template_msgs.msg import Yolov8Inference  # Custom message types for YOLOv8 inference results

# Initialize the CvBridge
cv_bridge = CvBridge()

# Define a class for the ROS2 node
class camera_sub(Node):

    def __init__(self):
        # Initialize the node with the name 'camera_subscriber'
        super().__init__('camera_subscriber')

        # Create a subscription to the camera image topic
        # Note: Change 'camera/image_raw' if your camera publishes on a different topic
        self.subscription = self.create_subscription(
            Image,
            # This is where you can change the name of the subscription topic do this if the name of the topic your camera publishes on is different than the one given keep in mind that if you change it here you must also change it in yolov8_ros2_subscriber.py
            'camera/image_raw',
            self.camera_callback,
            10)  # The queue size is set to 10


    # Callback function that converts ROS image messages to OpenCV images.
    def camera_callback(self, data):
        global img
        img = cv_bridge.imgmsg_to_cv2(data, "bgr8")

# Defines the 'yolo_sub' node for subscribing to YOLOv8 inference results.
class yolo_sub(Node):

    def __init__(self):
        super().__init__('yolo_subscriber')

        # Subscribe to the YOLOv8 inference results topic.
        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',  # Topic for receiving YOLOv8 inference data.
            self.yolo_callback,
            10)  # The queue size is set to 10

        self.cnt = 0  # Counter for logging and debugging purposes.
        self.img_pub = self.create_publisher(Image, "/inference_result_cv2", 1) # Publisher for annotated images.

    # Callback function to process inference results and annotate images.
    def yolo_callback(self, data):
        global img
        for r in data.yolov8_inference:
            # Extract and log bounding box details for each detected object.
            class_name = r.class_name
            top = r.top
            left = r.left
            bottom = r.bottom
            right = r.right
            yolo_subscriber.get_logger().info(f"{self.cnt} {class_name} : {top}, {left}, {bottom}, {right}")
            # Draw bounding boxes on the image.
            cv2.rectangle(img, (top, left), (bottom, right), (255, 255, 0))
            self.cnt += 1

        self.cnt = 0  # Reset counter after processing all detections.
        img_msg = cv_bridge.cv2_to_imgmsg(img)  # Convert annotated OpenCV image back to ROS image message.
        self.img_pub.publish(img_msg)  # Publish the annotated image.

if __name__ == '__main__':
    rclpy.init(args=None)
    yolo_subscriber = yolo_sub()
    camera_subscriber = camera_sub()

    # Convert annotated OpenCV image back to ROS image message.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(yolo_subscriber)
    executor.add_node(camera_subscriber)

    # Start executor in a separate thread.
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Create a loop to keep the program running, allowing for periodic tasks or shutdown.
    rate = yolo_subscriber.create_rate(2)  # Define a rate to control the loop frequency.
    try:
        while rclpy.ok():
            rate.sleep()  # Sleep to maintain the loop at the specified rate.
    except KeyboardInterrupt:
        # Allow for clean shutdown on interrupt.
        pass

    rclpy.shutdown()  # Shutdown ROS2 nodes cleanly.
    executor_thread.join()  # Ensure the executor thread is also cleanly shut down.
