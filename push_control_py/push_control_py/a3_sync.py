import sys
import rclpy
from rclpy.node import Node
import numpy as np
import time
import logging
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import Sync
from geometry_msgs.msg import Pose
from push_control_py.qos_profiles import qos_profile_R1, qos_profile_R10, qos_profile_B1, qos_profile_B10

qos_profiles = {'R1':qos_profile_R1,'R10':qos_profile_R10,'B1':qos_profile_B1,'B10':qos_profile_B10}

logging.basicConfig(
    level=logging.INFO,
    format='%(message)s',
    filename='measurement_log.txt',  # Specify the filename
    filemode='a'  # 'w' to overwrite, 'a' to append
)
delimiter = ';'
output_dir = "docs/data/qos_tests/_sync/"

log_messages = []

class SyncSub(Node):
    def __init__(self):
        super().__init__('sync_sub')

        self.qos_profile = "R10" #default profile
        self.counter = 0
        self.counts = 1000
        self.latencies_p1 = np.zeros(self.counts)
        self.latencies_p2 = np.zeros(self.counts)
        self.first_log = True

        if len(sys.argv)>1:
            self.file_note = sys.argv[1]
            if len(sys.argv)>2:
                self.qos_profile = sys.argv[2]#
        self.get_logger().info(f'Starting measurement with qos_profile: {self.qos_profile}')
        self.output_log_filename = output_dir + self.file_note + "_" + self.qos_profile + ".csv"
        self.subscription = self.create_subscription(
            Sync,
            'sync/pub2',
            self.sub_callback,
            qos_profiles[self.qos_profile]#,event_callbacks=self.subscription_callbacks
        )

    def sub_callback(self, msg):
        end_time = self.get_clock().now().nanoseconds
        if self.counter < self.counts:
            total_latency = end_time - msg.stamp1
            expected_one_way_latency = total_latency/2
            measured_latency_pub1 = msg.stamp2-msg.stamp1
            measured_latency_pub2 = end_time-msg.stamp2
            latency_diff = expected_one_way_latency - measured_latency_pub1
            self.latencies_p1[self.counter] = measured_latency_pub1
            self.latencies_p2[self.counter] = measured_latency_pub2

        elif self.counter == self.counts:
            mean_diff = np.mean(self.latencies_p2)-np.mean(self.latencies_p1)
            median_diff = np.median(self.latencies_p2)-np.median(self.latencies_p1)
            #avg_latency_p2 = self.acc_latency_p2/self.counter
            #diff = avg_latency_p2-avg_latency_p1
            self.get_logger().info(f'mean latency p1: {np.mean(self.latencies_p1)}, mean latency p2: {np.mean(self.latencies_p2)}, mean diff: {mean_diff}')
            log_message = (
                f"{self.counts};{mean_diff:.0f};{median_diff:.0f};"
                f"{np.mean(self.latencies_p1):.0f};{np.median(self.latencies_p1):.0f};{np.std(self.latencies_p1):.0f};{np.min(self.latencies_p1)};{np.max(self.latencies_p1)};"
                f"{np.mean(self.latencies_p2):.0f};{np.median(self.latencies_p2):.0f};{np.std(self.latencies_p2):.0f};{np.min(self.latencies_p2)};{np.max(self.latencies_p2)}"
            )
            log_messages.append(log_message)
            if self.first_log:
                self.first_log = False
                with open(self.output_log_filename, 'w') as log_file:
                    log_file.write('Counts; Mean Diff P2 P1; Median Diff P2 P1; Mean p1; Median p1; Std Deviation p1; Min p1; Max p1; Mean p2; Median p2; Std Deviation p2; Min p2; Max p2\n')
                    log_file.write(log_message + '\n')
            else:
                with open(self.output_log_filename, 'a') as log_file:
                    log_file.write(log_message + '\n')

        else:
            self.counter = 0
            self.acc_latency_p1 = 0
            self.acc_latency_p2 = 0

        if self.counter % 100 == 0:
            self.get_logger().info(f'counter / counts: {self.counter} / {self.counts}')


        self.counter += 1




def main(args=None):
    rclpy.init(args=args)
    node = SyncSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
