#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtlesim_plus_interfaces.srv import GivePosition
from my_controller_interfaces.srv import SetTarget
import math
import yaml, argparse
import sys


from turtlesim_interfaces.srv import SendPosition

def read_yaml_file(path:str):
    
    with open(path,'r') as file:
        data = yaml.load(file,Loader=yaml.SafeLoader)
    return data

via_points_Foxy = read_yaml_file('/home/tuchapong1234/FRA501_6431_6475_WS/src/turtlesim_control/via_point/via_point_Foxy.yaml')
values_list_Foxy = list(via_points_Foxy.values())
via_points_Noetic = read_yaml_file('/home/tuchapong1234/FRA501_6431_6475_WS/src/turtlesim_control/via_point/via_point_Noetic.yaml')
values_list_Noetic = list(via_points_Noetic.values())
via_points_Humble = read_yaml_file('/home/tuchapong1234/FRA501_6431_6475_WS/src/turtlesim_control/via_point/via_point_Humble.yaml')
values_list_Humble = list(via_points_Humble.values())
via_points_Iron = read_yaml_file('/home/tuchapong1234/FRA501_6431_6475_WS/src/turtlesim_control/via_point/via_point_Iron.yaml')
values_list_Iron = list(via_points_Iron.values())

class DummyNode(Node):
    def __init__(self, namespace:str,file_path:str):
        super().__init__('dummy_node')

        self.scheduler_client = self.create_client(SendPosition, "{namespace}/set_goal",)
        self.create_timer(0.1, self.timer_callback)
        self.create_service(Empty, '{namespace}/notify_arrival', self.notify_arrival_callback)
        self.count = 0
        self.enable = False
        self.name = 'Foxy'
        















        

        with open(file= file_path, mode='r') as file:
            data = yaml.load(file, Loader=yaml.SafeLoader)
        via_point = data['via_point']
        self.values_list_Foxy = list(via_point.values())

        
    def scheduler(self, count):
        position_request = SendPosition.Request()

        values_list = self.values_list_Foxy

        if self.count < 20:
            position_request.position.x = values_list[0][count][0]
            position_request.position.y = values_list[0][count][1]
            self.count = count+1
            self.scheduler_client.call_async(position_request)
        else:
            position_request.position.x = 10.0
            position_request.position.y = 10.0
            self.scheduler_client.call_async(position_request)

    def notify_arrival_callback(self, request:Empty.Request, response:Empty.Response):
        self.scheduler(self.count)
        print("arrive")
        return response

    def timer_callback(self):
        print(self.name)
        if self.enable == False:
            self.scheduler(self.count)
        self.enable = True

def main(args=None):
    parser = argparse.ArgumentParser(description='schedule via points')
    parser.add_argument('-f', '--file', help='path to the YAML file of via points')
    parsed_args, remaining_args = parser.parse_known_args(args=args)

    rclpy.init(args=remaining_args)
    node = DummyNode(parsed_args.file)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
