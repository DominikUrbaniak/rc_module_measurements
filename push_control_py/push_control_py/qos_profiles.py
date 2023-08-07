import rclpy.qos as qos
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Define QoS profiles
qos_profile_R10 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=10                                      # Keep 10 messages in history 10
)
qos_profile_R1 = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1,                                     # Keep one message in history 1
)
qos_profile_R1_deadline = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1,                                     # Keep one message in history 1
    #deadline=qos.Duration(seconds=0, nanoseconds=1000000),
    #lifespan=qos.Duration(seconds=20, nanoseconds=0),
)
qos_profile_B10 = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Best effort delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep last messages not KEEP_ALL
    depth=10                                      # Keep 10 messages in history 10
)
qos_profile_B1 = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Reliable delivery RELIABLE
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep only the last message KEEP_LAST
    depth=1                                      # Keep one message in history 1
)
