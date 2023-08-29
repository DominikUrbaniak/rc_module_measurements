import sys
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import Sync
from geometry_msgs.msg import Pose
from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles = {'R1':qos_profile_R1,'R10':qos_profile_R10,'B1':qos_profile_B1,'B10':qos_profile_B10}

class SyncSubPub(Node):
    def __init__(self):
        super().__init__('sync_sub_pub')

        self.qos_profile = "R10" #default profile
        self.file_note = ""
        
        if len(sys.argv)>1:
            self.qos_profile = sys.argv[1]
        self.get_logger().info(f'Starting measurement with qos_profile: {self.qos_profile}')

        self.subscription = self.create_subscription(
            Sync,
            'sync/pub1',
            self.sub_callback,
            qos_profiles[self.qos_profile]#,event_callbacks=self.subscription_callbacks
        )
        self.publisher_2 = self.create_publisher(Sync, 'sync/pub2', qos_profile=qos_profiles[self.qos_profile])


    def sub_callback(self, msg):
        #start_time = time.time()
        start_time = self.get_clock().now().nanoseconds

        msg2 = Sync()
        msg2.stamp1 = msg.stamp1
        msg2.stamp2 = start_time
        msg2.id = msg.id

        self.publisher_2.publish(msg2)


def main(args=None):
    rclpy.init(args=args)
    node = SyncSubPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
