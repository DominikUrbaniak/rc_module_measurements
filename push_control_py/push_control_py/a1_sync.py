# Basic ROS 2 program to publish real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
import sys
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
#from std_msgs.msg import Empty # Image is the message type
#from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
#import cv2 # OpenCV library
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import Sync
from push_control_py.qos_profiles import qos_profile_R1_deadline
from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles = {'R1':qos_profile_R1,'R10':qos_profile_R10,'B1':qos_profile_B1,'B10':qos_profile_B10}

class SyncPublisher1(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('sync_publisher_1')

    self.counter = 0
    self.qos_profile = "R10" #default profile
    if len(sys.argv)>1:
        self.qos_profile = sys.argv[1]
    self.get_logger().info(f'Starting measurement with qos_profile: {self.qos_profile}')
    self.publisher_ = self.create_publisher(Sync, 'sync/pub1', qos_profiles[self.qos_profile])#,event_callbacks=self.publisher_callbacks
    fps = 200
    # We will publish a message every 0.1 seconds
    timer_period = 1.0/fps  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.


    start_time = self.get_clock().now().nanoseconds


    msg = Sync()
    msg.stamp1 = start_time
    msg.stamp2 = start_time
    msg.id = self.counter
    self.publisher_.publish(msg)
    self.counter = self.counter + 1
      #end_time = time.time()
      #computation_time = end_time - start_time
      #self.get_logger().info(f'Sending image #{self.counter}')



def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  image_publisher = SyncPublisher1()

  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
