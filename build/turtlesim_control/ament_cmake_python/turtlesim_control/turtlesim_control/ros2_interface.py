#!/usr/bin/python3

from typing import Callable

# RCLPY libraries, classes, functions
from rclpy.node import Node

# ROS Packages
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim_interfaces.srv import SendPosition
from std_msgs.msg import Bool, Int16
from std_srvs.srv import Empty

class ControllerROS2Interface:
    def __init__(self, enable_srv_callback: Callable, set_goal_srv_callback: Callable) -> None:
        """
        Initialize the ControllerROS2Interface class.

        Args:
            enable_srv_callback (Callable): Callback for enabling the controller.
            set_goal_srv_callback (Callable): Callback for setting a goal.
        """
        self.node = Node(node_name='controller')
        self.pub_cmd_vel = self.node.create_publisher(
            msg_type=Twist,
            topic='cmd_vel',
            qos_profile=10
        )
        self.pub_enable_status = self.node.create_publisher(
            msg_type=Bool,
            topic='enable_status',
            qos_profile=10
        )
        self.node.create_subscription(
            msg_type=Pose,
            topic='pose',
            callback=self.pose_sub_callback,
            qos_profile=10
        )
        self.node.create_service(
            srv_type=Empty,
            srv_name='enable',
            callback=enable_srv_callback
        )
        self.node.create_service(
            srv_type=SendPosition,
            srv_name='set_goal',
            callback=set_goal_srv_callback
        )
        self.notify_arrival_client = self.node.create_client(Empty, 'notify_arrival')
        self.node.declare_parameter(name='linear_gain', value=1.0)
        self.node.declare_parameter(name='angular_gain', value=5.0)
        self.node.declare_parameter(name='tolerance', value=0.1)
        self.variable = {'pose': None}
        self.timer = None
        self.client_futures = []

    def pose_sub_callback(self, msg: Pose) -> None:
        """
        Callback function for processing pose data.

        Args:
            msg (Pose): Pose data received from the subscription.
        """
        self.variable['pose'] = [msg.x, msg.y, msg.theta]

class SchedulerROS2Interface:
    def __init__(self, notify_arrival_srv_callback: Callable) -> None:
        """
        Initialize the SchedulerROS2Interface class.

        Args:
            notify_arrival_srv_callback (Callable): Callback for notifying arrival.
        """
        self.node = Node(node_name='scheduler')
        
        self.pub_scheduler_index = self.node.create_publisher(
            msg_type=Int16,
            topic='~/index',
            qos_profile=10
        )
        self.node.create_service(
            srv_type=Empty,
            srv_name='notify_arrival',
            callback=notify_arrival_srv_callback
        )
        self.enable_client = self.node.create_client(Empty, 'enable')
        self.set_goal_client = self.node.create_client(SendPosition, 'set_goal')
        
        self.client_futures = []