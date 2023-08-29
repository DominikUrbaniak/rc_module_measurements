import sys
import rclpy
from rclpy.node import Node
import numpy as np
import time

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import Sync
from geometry_msgs.msg import Pose
from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles = {'R1':qos_profile_R1,'R10':qos_profile_R10,'B1':qos_profile_B1,'B10':qos_profile_B10}

class SyncSub(Node):
    def __init__(self):
        super().__init__('sync_sub')

        self.qos_profile = "R10" #default profile
        self.counter = 0
        self.acc_latency_p1 = 0
        self.acc_latency_p2 = 0

        if len(sys.argv)>1:
            self.qos_profile = sys.argv[1]
        self.get_logger().info(f'Starting measurement with qos_profile: {self.qos_profile}')

        self.subscription = self.create_subscription(
            Sync,
            'sync/pub2',
            self.sub_callback,
            qos_profiles[self.qos_profile]#,event_callbacks=self.subscription_callbacks
        )

    def sub_callback(self, msg):

        if self.counter < 1000:
            end_time = self.get_clock().now().nanoseconds
            total_latency = end_time - msg.stamp1
            expected_one_way_latency = total_latency/2
            measured_latency_pub1 = msg.stamp2-msg.stamp1
            measured_latency_pub2 = end_time-msg.stamp2
            latency_diff = expected_one_way_latency - measured_latency_pub1
            self.acc_latency_p1 += measured_latency_pub1
            self.acc_latency_p2 += measured_latency_pub2

        elif self.counter == 1000:
            avg_latency_p1 = self.acc_latency_p1/self.counter
            avg_latency_p2 = self.acc_latency_p2/self.counter
            diff = avg_latency_p2-avg_latency_p1
            self.get_logger().info(f'average latency p1: {avg_latency_p1}, average latency p2: {avg_latency_p2}, diff: {diff}')
        else:
            self.counter = 0
            self.acc_latency_p1 = 0
            self.acc_latency_p2 = 0



        self.counter += 1




def main(args=None):
    rclpy.init(args=args)
    node = SyncSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
