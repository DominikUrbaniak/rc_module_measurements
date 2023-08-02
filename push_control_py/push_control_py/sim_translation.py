import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from geometry_msgs.msg import Pose#, TransformStamped
from geometry_msgs.msg import Quaternion

from custom_interfaces.srv import RobPose
#from custom_interfaces.srv import SetRandomization
from custom_interfaces.srv import ResetPoses
from custom_interfaces.msg import PoseSensing
from custom_interfaces.srv import PoseSensingSettings

import time
import math
import random
import numpy as np

##************************* argv *********************
# 1: latency from file, e.g. private5g
# 2: simulated computation latency in ms
# 3: number of runs
# 4: max change in velocities, e.g. 0.002
##*****************************************************

cube_pose = Pose()
cube_y_init = 0.6
push_distance = 0.12
v_init = 0.00
v_z_offset = 0.008#0.011
manipulability_index = 1
accuracy = 0.0
execution_time = 0.0
timeout_ms = 1000 #for low-level control and backup



#from the construct
def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to Euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Below should be replaced when porting for ROS2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

import math

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return Quaternion(w=w, x=x, y=y, z=z)



class SimTranslation(Node):

    def __init__(self):
        super().__init__('sim_translation')
        random.seed()
        self.control_rate = 60
        self.control_rate_ms = float(1)/self.control_rate
        self.start = True
        self.n_episode = int(sys.argv[3])
        self.steps_per_episode = 500
        self.current_episode = 0
        self.sensing_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(PoseSensing, '/pose_sensing/delayed_pose', self.sensing_callback, 10, callback_group=self.sensing_cb_group)
        self.cube_pose = Pose()
        self.latency_sensing = 0
        self.time_stamp_cube_origin = 0
        self.time_stamp_cube_delayed = 0

        self.controller = RobotControl()

        self.cube_y_init_sensed = cube_y_init

        self.cube_y_goal = cube_y_init+push_distance;

        self.back_up_counter = 0
        self.back_up_counter_limit = self.control_rate / 2

        self.timeout = False
        self.action = 0 #velocity in one direction
        self.previous_action = self.action
        self.distance_to_goal = push_distance
        self.max_velocity = 3.0
        self.max_velocity_change = float(sys.argv[4])
        self.start_time = 0.0
        self.end_time = 0.0

        self.controller.reset()
        self.reset = True
        time.sleep(2)


    #def _timer_cb(self):
    def sensing_callback(self, msg):
        #self.get_logger().info(f'current episode/step: {self.current_episode}/{self.current_step}')
        self.cube_pose = msg.pose
        _, _, yaw = euler_from_quaternion(self.cube_pose.orientation)

        self.latency_sensing = msg.latency_mms
        self.time_stamp_cube_origin = msg.time_stamp_origin
        self.time_stamp_cube_delayed = msg.time_stamp_delayed

        if self.current_episode < self.n_episode:
            if self.reset:
                self.cube_y_init_sensed = self.cube_pose.position.y
                self.reset = False

            if self.timeout:
                global execution_time
                execution_time = self.end_time - self.start_time
                global accuracy
                accuracy = self.cube_pose.position.y - self.cube_y_goal
                self.get_logger().info(f'Episode {self.current_episode} complete, final accuracy: {accuracy}')
                self.get_logger().info('Execution time: {:.2f} ms'.format(execution_time * 1000))
                self.current_episode += 1
                self.back_up_counter = 0
                self.action = 0
                self.previous_action = self.action
                self.timeout = False
                self.controller.reset()
                self.reset = True
                time.sleep(2)


            elif self.cube_pose.position.y == self.cube_y_init_sensed: #in simulation thius should always be the same, in real world add small deviations
                self.action = -v_init
                self.controller.control(self.action)
                self.start_time = time.time()
            elif self.cube_pose.position.y < self.cube_y_goal:
                self.end_time = time.time()

                self.distance_to_goal = self.cube_y_goal-self.cube_pose.position.y
                self.previous_action = self.action
                self.action = -self.get_action(self.distance_to_goal, -self.previous_action, self.max_velocity, self.max_velocity_change)
                self.controller.control(self.action)
                #self.get_logger().info(f'pushing at v: {self.action}')
            else:
                #self.back_up_counter = self.back_up_counter + 1
                self.action = v_init
                self.controller.control(self.action)
                #self.get_logger().info(f'time: {time.time()-self.end_time}')
                if 1000*(time.time()-self.end_time) > timeout_ms:
                    self.timeout = True

        else:
            self.get_logger().info(f'Episodes finished, saving docs (todo)...')
            self.cleanup()


    def get_action(self, distance_to_goal, previous_velocity, max_velocity, max_velocity_change):
        #
        # Calculate the distance factor as a value between 0 and 1
        distance_factor = min(1.0, distance_to_goal / push_distance)

        # Calculate the sigmoid function to smoothen the velocity increase and decrease
        sigmoid_factor = 1 / (1 + math.exp(-12 * (distance_factor - 0.5)))

        # Calculate the desired velocity at the current distance factor
        desired_velocity = max_velocity * sigmoid_factor

        # Limit the change in velocity to make the transition smooth
        #max_velocity_change = 0.1  # Tweak this value for a faster/slower transition
        velocity_change = max(-max_velocity_change, min(desired_velocity - previous_velocity, max_velocity_change))

        # Calculate and return the new velocity
        new_velocity = previous_velocity + velocity_change
        return new_velocity

    def cleanup(self):
        self.destroy_node()
        self.controller.destroy_node()
        rclpy.shutdown()


class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        random.seed()
        self.control_rate = 60
        self.control_rate_ms = float(1)/self.control_rate
        self.start = True
        self.control_cb_group = MutuallyExclusiveCallbackGroup()
        #self.control2_cb_group = MutuallyExclusiveCallbackGroup()
        self.sensing_cb_group = MutuallyExclusiveCallbackGroup()
        self.cli_control = self.create_client(RobPose, '/velocity_controller/set_desired_rob_pose',callback_group=self.control_cb_group)
        self.cli_reset = self.create_client(ResetPoses, '/world_setup/reset_world',callback_group=self.control_cb_group)

        self.x_offset = -0.027
        self.pose_eef_init = Pose()
        self.pose_cube_init = Pose()
        self.pose_eef_init.position.x = 0.1 + self.x_offset
        self.pose_eef_init.position.y = -0.39#,-0.49
        self.pose_eef_init.position.z = 0.08#,0.112
        self.pose_eef_init.orientation = euler_to_quaternion(0.8,np.pi,0)#,euler_to_quaternion(0,np.pi,0)
        self.pose_cube_init.position.x = -0.1
        self.pose_cube_init.position.y = cube_y_init
        self.pose_cube_init.orientation = euler_to_quaternion(0,np.pi,0)
        self.pose_gripper = 0.6
        #self.cli_sensing = self.create_client(PoseSensing, '/pose_sensing/get_delayed_pose',callback_group=self.sensing_cb_group)

        self.cli_sensing_settings = self.create_client(PoseSensingSettings, '/pose_sensing/set_settings')
        while not self.cli_sensing_settings.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PoseSensingSettings.Request()
        self.req.objid = 5
        self.req.computation_ms = int(sys.argv[2])
        self.req.network = sys.argv[1]
        self.req.n_freshness_samples = 10
        #self.eef_pose = TransformStamped()
        self.time_stamp_eef = 0

        self.cube_rel_x_y_yaw = np.zeros(3) #x,y,yaw
        self.eef_rel_x_y_yaw = np.zeros(3) #x,y,yaw

        self.future = self.cli_sensing_settings.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f'pose settings success: {self.future.result().success}, control rate: {self.control_rate_ms}')


    def reset(self):
        while not self.cli_reset.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for reset service...')
        request = ResetPoses.Request()
        request.pose_eef = self.pose_eef_init
        request.pose_cube = self.pose_cube_init
        request.pose_gripper = self.pose_gripper
        future = self.cli_reset.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'World is successfully reset: {future.result().success}')
        else:
            self.get_logger().warning('Failed to reset')


    def control(self,a):
        while not self.cli_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for control service...')
        #self.get_logger().info('Connected to control service!')
        request = RobPose.Request()
        request.timeout_ms = timeout_ms
        request.cart_pose = 0

        request.goal_dir = [0.0,a,v_z_offset,0.0,0.0,0.0,self.pose_gripper]

        future = self.cli_control.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            #self.get_logger().info('request successful!')

            self.time_stamp_eef = future.result().time_stamp
            manipulability_index = future.result().manipulability_index

        else:
            self.get_logger().warning('Failed to control')


    def cleanup(self):
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SimTranslation()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')

    executor.shutdown()


if __name__ == '__main__':
    main()
