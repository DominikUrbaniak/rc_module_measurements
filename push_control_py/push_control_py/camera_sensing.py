import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from custom_interfaces.msg import ImageStampId

# Define QoS profiles
qos_profile_R10 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_ALL,         # Keep all messages KEEP_ALL
    depth=10                                      # Keep 10 messages in history 10
)
qos_profile_R1 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1                                      # Keep one message in history 1
)
qos_profile_B10 = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Best effort delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_ALL,         # Keep all messages KEEP_ALL
    depth=10                                      # Keep 10 messages in history 10
)
qos_profile_B1 = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1                                      # Keep one message in history 1
)

current_profile = qos_profile_B10

class ArUcoTagDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        self.get_logger().info(f'Starting camera...')
        self.subscription = self.create_subscription(
            ImageStampId,
            'camera/image_raw',
            self.image_callback,
            current_profile
        )
        self.publisher_ = self.create_publisher(Image, 'camera/image_arucos', 10)
        self.cv_bridge = CvBridge()
        self.cameraMatrix = 1000*np.array([[1.6695,0.0,0.9207],[0.0,1.6718,0.5518],[0,0,0.0010]]) #Logitech Desktop webcam
        self.distortionCoeffs = np.array([0.0772,-0.2883,0.0,0.0]) #k1,k2,p1,p2
        self.tag_position_camera = np.zeros(3)
        self.euler_angles = np.zeros(3)
        self.communication_duration = 120
        self.n_data_points = 30*self.communication_duration
        self.latencies = np.zeros([self.n_data_points,4])
        self.counter = 0
        self.tag_found = 0
        #found_aruco = False

    def image_callback(self, msg):
        start_time = time.time()
        start_time_ros2 = self.get_clock().now()
        #self.get_logger().info(f'python time: {start_time} vs. ROS2 time: {start_time_ros2} vs. time stamp: {msg.stamp_ns}')
        #msg = ImageStampId()
        # Convert ROS Image message to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg.image)

        #pub_time_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

        latency = start_time_ros2.nanoseconds - msg.stamp_ns#(msg.header.stamp.seconds * 1e9 + msg.header.stamp.nanoseconds)
        #latency = latency_stamp.sec * 1e9 + latency_stamp.nanosec
        counter_id = msg.id

        # Convert the image to grayscale for ArUco detection
        #gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Define the dictionary of ArUco tags
        aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # Create the ArUco detector
        aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Detect ArUco tags
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, aruco_dictionary, parameters=aruco_parameters)
        #self.get_logger().info(f'corners: {corners}, ids: {ids}, K: {self.cameraMatrix.shape}')
        if ids is not None:
            # Find the index of the ArUco tag with ID 0
            tag_index = np.where(ids == 0)[0]

            # Calculate the pose of the detected tag
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.034, self.cameraMatrix, self.distortionCoeffs)

            if len(tag_index) > 0:

                # Publish the pose of the tag
                # Get the rotation vector and translation vector of the tag
                rvec = rvecs[tag_index]
                tvec = tvecs[tag_index]
                if rvec is not None and tvec is not None:

                    self.tag_found = 1
                    #self.get_logger().info(f'ArUco tag found, ids: {ids}')
                    #for i in enumerate(ids):
                        #cv2.aruco.drawAxis(cv_image,self.cameraMatrix,self.distortionCoeffs,rvecs[i],tvecs[i],0.1)
                    cv2.aruco.drawAxis(cv_image,self.cameraMatrix,self.distortionCoeffs,rvec,tvec,0.1)
                    rmat, _ = cv2.Rodrigues(rvec)
                    # Transform the tag's position to the camera frame
                    self.tag_position_camera = -np.dot(rmat, np.transpose(tvec[0]))
                    # Convert the rotation matrix to Euler angles
                    self.euler_angles = Rotation.from_matrix(rmat).as_euler('xyz', degrees=True)
                    # Transform the tag's position to the world frame
                    #tag_position_world = np.dot(rmat_world_camera, tag_position_camera)

                    # Publish the pose information using ROS
                    # Replace 'tag_pose_topic' with the actual topic name for publishing pose data
                    # Replace 'camera_frame' and 'tag_frame' with the actual frame names
                    # Publish rvecs and tvecs
        else:
            self.tag_found = 0

        #else:
            #found_aruco = False

        end_time = time.time()
        computation_time = int((end_time - start_time)*1e9) #nanosecs
        if self.counter < self.n_data_points:
            self.latencies[self.counter,:] = [counter_id, latency, self.tag_found, computation_time]
        else:
            np.savetxt('docs/data/aruco_latency_measurement.csv', self.latencies, delimiter=',')
            self.get_logger().info(f'Successful measurement!')
            rclpy.shutdown()
        self.get_logger().info(f'Counter id: {counter_id}, latency: {latency}')
        #self.get_logger().info(f'Tag position: {self.tag_position_camera}, tag orientation: {self.euler_angles}')
        #self.get_logger().info('ArUco tag computation time: {:.2f} ms'.format(computation_time * 1000))
        self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(cv_image))
        self.counter = self.counter + 1


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoTagDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

'''import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ArUcoTagDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        self.cv_bridge = CvBridge()
        self.publisher_ = self.create_publisher(Pose, 'aruco_pose', 10)

    def image_callback(self, msg):
        start_time = time.time()

        # Convert ROS Image message to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Convert the image to grayscale for ArUco detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Define the dictionary of ArUco tags
        aruco_dictionary = cv2.aruco.aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        # Create the ArUco detector
        aruco_parameters = cv2.aruco.DetectorParameters()
        aruco_detector = aruco.ArucoDetector(aruco_dictionary, aruco_parameters)

        # Detect ArUco tags
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            # Calculate the pose of the detected tag
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distortionCoeffs)

            # Publish the pose of the tag
            if rvecs is not None and tvecs is not None:
                # Extract the rotation vector and translation vector
                rvec = rvecs[0]
                tvec = tvecs[0]

                # Create a Pose message
                pose_msg = Pose()

                # Set the position (translation)
                #pose_msg.position = Point()
                pose_msg.pose.position.x = tvec[0][0]
                pose_msg.pose.position.y = tvec[0][1]
                pose_msg.pose.position.z = tvec[0][2]


                # Convert the rotation vector to a quaternion
                rvec_norm = np.linalg.norm(rvec)
                rotation = cv2.Rodrigues(rvec / rvec_norm)[0]
                quaternion = tf.transformations.quaternion_from_matrix(rotation)
                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1]
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]

                # Publish the pose information using ROS
                # Replace 'tag_pose_topic' with the actual topic name for publishing pose data
                # Replace 'camera_frame' and 'tag_frame' with the actual frame names
                # Publish rvecs and tvecs

        end_time = time.time()
        computation_time = end_time - start_time
        self.get_logger().info('ArUco tag computation time: {:.2f} ms'.format(computation_time * 1000))


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoTagDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()'''
