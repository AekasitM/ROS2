import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtlesim_plus_interfaces.srv import GivePosition
from my_controller_interfaces.srv import SetTarget
import math
import yaml
from turtlesim_interfaces.srv import SendPosition

def read_yaml_file(path:str):
    
    with open(path,'r') as file:
        data = yaml.load(file,Loader=yaml.SafeLoader)
    return data
via_points_Iron = read_yaml_file('/home/tuchapong1234/FRA501_6431_6475_WS/src/turtlesim_control/via_point/via_point_Iron.yaml')
values_list_Iron = list(via_points_Iron.values())

class DummyNode(Node):
    def __init__(self):
        super().__init__('pizza_spawner_Iron')
        self.create_subscription(Pose, "/Iron/pose", self.turtle_pose_callback, 10)
        self.spawn_pizza_client = self.create_client(GivePosition, "spawn_pizza")
        self.turtle_current_pose = [0.0, 0.0, 0.0]
        self.turtle_target_pose = [0.0, 0.0]
        self.kp_dis = 2.5
        self.kp_ori = 5.0
        self.count = 0
        self.isEnableController = False
        self.create_service(SendPosition, 'set_goal', self.set_goal_callback)
        self.create_notify_arrival_client = self.create_client(Empty, 'notify_arrival')

 

    def set_goal_callback(self, request:SendPosition.Request, response:SendPosition.Response):
        if self.isEnableController == True:
            print("Not Done")
        else:
            self.turtle_target_pose = [request.position.x,request.position.y]
            self.isEnableController = True
            print("Done")
        return response
    
    def turtle_pose_callback(self, msg):
        self.turtle_current_pose = [msg.x, msg.y, msg.theta]
        
    def spawn_pizza(self, position):
        position_request = GivePosition.Request()
        position_request.x = position[0]
        position_request.y = position[1]
        self.spawn_pizza_client.call_async(position_request)

    def pizza_path(self,count):
        if self.count < 20:
            self.turtle_target_pose[0] = values_list_Iron[0][count][0]
            self.turtle_target_pose[1] = values_list_Iron[0][count][1]
            self.count = count+1

    def timer_callback(self):
        #print(self.turtle_current_pose[0])
        if self.count == 0:
            self.pizza_path(self.count)
            self.count += 1
        elif self.isEnableController == True:
            #print("Run")
            pass

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
        
        if e_dis <= 0.5:
            self.isEnableController = False
            self.spawn_pizza(self.turtle_current_pose)  
            self.create_notify_arrival_client.call_async(Empty.Request())
            # self.cmd_vel(0.0, 0.0)
            self.pizza_path(self.count)
            print("stop")
            
        else:    
            self.isEnableController = True

        
def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()