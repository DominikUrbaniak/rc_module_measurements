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
            'qos_test_service = push_control_py.qos_test_service:main',
            'qos_test_client = push_control_py.qos_test_client:main',
            'qos_test_publisher = push_control_py.qos_test_publisher:main',
            'qos_test_subscriber = push_control_py.qos_test_subscriber:main',
            'camera_sensing = push_control_py.camera_sensing:main',
            'webcam_pub = push_control_py.webcam_pub:main',
            'camera_sensing_client = push_control_py.camera_sensing_client:main',
            'webcam_srv = push_control_py.webcam_srv:main',
            'rc_april_tag_detect_client = push_control_py.rc_tagdetect:rc_april_tag_detect_client',
            'rc_itempick_onboard_client = push_control_py.rc_itempick_onboard_client:main',
            'rc_boxpick_onboard_client = push_control_py.rc_boxpick_onboard_client:main',
        ],
    },
)
