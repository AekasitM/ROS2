#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtlesim_plus_interfaces.srv import GivePosition
from my_controller_interfaces.srv import SetTarget
import math

class DummyNode(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.pub_cmd_vel = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.create_timer(0.01, self.timer_callback)
        self.create_subscription(Point, "mouse_position", self.target_pose_callback, 10)
        self.create_subscription(Pose, "turtle1/pose", self.turtle_pose_callback, 10)
        self.spawn_pizza_client = self.create_client(GivePosition, "spawn_pizza")
        self.eat_pizza_client = self.create_client(Empty, "turtle1/eat")
        self.turtle_current_pose = [0.0, 0.0, 0.0]
        self.turtle_target_pose = [0.0, 0.0]
        self.kp_dis = 0.5
        self.kp_ori = 1.0
        self.isEnableController = False
        self.create_service(SetTarget, "my_controller/p2p_eat", self.p2p_eat_callback)
    
    def p2p_eat_callback(self, request, response):
        if self.isEnableController == True:
            response.result = False
        else:
            self.turtle_target_pose = [request.target.x, request.target.y]
            self.spawn_pizza(self.turtle_target_pose)
            self.isEnableController = True
            response.result = True
        return response

    def target_pose_callback(self, msg):
        self.turtle_target_pose = [msg.x, msg.y]
        self.spawn_pizza(self.turtle_target_pose)
        self.isEnableController = True
        
    def turtle_pose_callback(self, msg):
        self.turtle_current_pose = [msg.x, msg.y, msg.theta]
        # print(self.turtle_current_pose)
        
    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)
    
    def spawn_pizza(self, position):
        position_request = GivePosition.Request()
        position_request.x = position[0]
        position_request.y = position[1]
        self.spawn_pizza_client.call_async(position_request)

    def cmd_vel(self, vx, w):
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = w
        self.pub_cmd_vel.publish(cmd_vel)

    def timer_callback(self):
        if self.isEnableController:
            self.controller()

    def controller(self):
        dx = self.turtle_target_pose[0] - self.turtle_current_pose[0]
        dy = self.turtle_target_pose[1] - self.turtle_current_pose[1]
        alpha = math.atan2(dy, dx)
        e_dis = math.hypot( dx, dy )
        e_ori = alpha - self.turtle_current_pose[2]
        e_ori = math.atan2( math.sin(e_ori), math.cos(e_ori) )
        u_dis = e_dis * self.kp_dis
        u_ori = e_ori * self.kp_ori
        if e_dis <= 0.05:
            self.isEnableController = False
            self.cmd_vel(0.0, 0.0)
            self.eat_pizza()
            print("stop")
        else:    
            print("run")
            self.cmd_vel(u_dis, u_ori)
        
def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
