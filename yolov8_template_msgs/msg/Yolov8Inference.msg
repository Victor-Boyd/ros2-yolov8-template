# DO NOT CHANGE ANYTHING IN THS FOLDER unless you know exactly what you're doing

# The 'header' includes metadata for the message, such as timestamp and coordinate frame ID, to track when and where the detection occurred.
std_msgs/Header header

# An array of 'InferenceResult' to hold the results from YOLOv8 detection, including detected objects' classes, confidence scores, and locations.
InferenceResult[] yolov8_inference
