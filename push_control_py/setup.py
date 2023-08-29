from setuptools import setup

package_name = 'push_control_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dominik',
    maintainer_email='dominik.urbaniak@upc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_translation = push_control_py.sim_translation:main',
            'sim_translation_ur3e = push_control_py.sim_translation_ur3e:main',
            'real_pose_sim_push = push_control_py.real_pose_sim_push:main',
            'qos_test_service = push_control_py.qos_test_service:main',
            'qos_test_client = push_control_py.qos_test_client:main',
            'qos_test_publisher = push_control_py.qos_test_publisher:main',
            'qos_test_subscriber = push_control_py.qos_test_subscriber:main',
            'a2_aruco_detection = push_control_py.a2_aruco_detection:main',
            'a3_pose_subscriber = push_control_py.a3_pose_subscriber:main',
            'a1_image_publisher = push_control_py.a1_image_publisher:main',
            'a1_sync = push_control_py.a1_sync:main',
            'a2_sync = push_control_py.a2_sync:main',
            'a3_sync = push_control_py.a3_sync:main',
            'webcam_pub_raw = push_control_py.webcam_pub_raw:main',
            'aruco_detection_srv = push_control_py.aruco_detection_srv:main',
            'aruco_detection_client = push_control_py.aruco_detection_client:main',
            'webcam_srv = push_control_py.webcam_srv:main',
            'rc_april_tag_detect_client = push_control_py.rc_tagdetect:rc_april_tag_detect_client',
            'rc_itempick_onboard_client = push_control_py.rc_itempick_onboard_client:main',
            'rc_boxpick_onboard_client = push_control_py.rc_boxpick_onboard_client:main',
        ],
    },
)
