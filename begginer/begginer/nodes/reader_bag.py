#!/usr/bin/env python3

# Import necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import cv_bridge

# Define a ROS node class for reading from a ROS bag
class ReaderBag(Node):
    def __init__(self):
        # Initialize the node with the name 'reader_bag'
        super().__init__('reader_bag')
        self.bridge = cv_bridge.CvBridge()
        # Create a publisher for publishing Image messages
        self.image_publisher = self.create_publisher(Image, '/image/rosbag', 10)
        print("ReaderBag Node Initialized")  # Log node initialization
        # Read data from the bag file
        self.read_bag()

    def read_bag(self):
        print("Starting to read bag file...")  # Log the start of bag file processing
        # Open the ROS bag file for reading
        with Reader('../rosbag2_2023_10_25-09_13_26') as reader:
            # Iterate through each message in the bag file
            for connection, _, rawdata in reader.messages():
                # Filter messages from the topic '/cam_front/image_raw'
                if connection.topic == '/cam_front/image_raw':
                    # print(f"Processing message from topic: {connection.topic}")  # Log message processing
                    # Deserialize the message data
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    img = Image()
                    # Copy relevant data to a new Image message
                    img.data = msg.data.data
                    img.encoding = msg.encoding
                    img.height = msg.height
                    img.width = msg.width
                    img.step = msg.step
                    # Publish the Image message
                    self.image_publisher.publish(img)
                    # print("Published image message")  # Log the publication of an image message

# Main function to initialize and run the ROS node
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS client library
    print("Initializing ReaderBag Node...")  # Log the start of the node
    reader_bag = ReaderBag()  # Create an instance of the ReaderBag node
    rclpy.spin(reader_bag)  # Keep the node running
    reader_bag.destroy_node()  # Clean up the node
    rclpy.shutdown()  # Shutdown the ROS client library
    print("ReaderBag Node shutdown")  # Log node shutdown

# Entry point of the script
if __name__ == '__main__':
    main()
