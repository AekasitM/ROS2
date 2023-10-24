#!/usr/bin/python3

#other libraries
from typing import List, Tuple

# RCLPY libraries, classes, functions
import rclpy

# ROS Packages
from geometry_msgs.msg import Twist
from turtlesim_interfaces.srv import SendPosition
from std_msgs.msg import Bool
from std_srvs.srv import Empty

# Package module
from turtlesim_control.ros2_interface import ControllerROS2Interface
from turtlesim_control.algorithm import Controller

class ControllerROS2:
    def __init__(self) -> None:
        """
        Initialize the ControllerROS2 class.
        """
        # Create instances of Controller and ControllerROS2Interface
        self.controller = Controller()
        self.ros2_interface = ControllerROS2Interface(
            self.enable_srv_callback,
            self.set_goal_srv_callback
        )
        # Set data interface for the controller
        self.controller.set_data_interface(self.send_cmd_vel, self.get_pose, self.notify_arrival, self.update_parameters)
        # Create a timer to execute controller behavior
        self.ros2_interface.timer = self.ros2_interface.node.create_timer(
            timer_period_sec=0.1,
            callback=self.controller.behavior
        )
        # Create a heartbeat timer
        self.heartbeat_timer = self.ros2_interface.node.create_timer(
            timer_period_sec=1.0,
            callback=self.heartbeat
        )
    
    def heartbeat(self) -> None:
        """
        Publish heartbeat status.
        """
        msg = Bool()
        msg.data = self.controller.enable
        self.ros2_interface.pub_enable_status.publish(msg)
    
    def send_cmd_vel(self, v:float, w:float) -> None:
        """
        Publish linear and angular velocities.
        
        Args:
            v (float): Linear velocity.
            w (float): Angular velocity.
        """
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.ros2_interface.pub_cmd_vel.publish(msg)
    
    def get_pose(self) -> List[float]:
        """
        Retrieve current pose.
        
        Returns:
            List[float] | None: Current pose [x, y, theta] or None if not available.
        """
        return self.ros2_interface.variable['pose']
    
    def update_parameters(self) -> Tuple[List[float],float]:
        """
        Update control parameters.

        Returns:
            Tuple[List[float], float]: Linear and angular gains, and tolerance.
        """
        linear_gain = self.ros2_interface.node.get_parameter('linear_gain').value
        angular_gain = self.ros2_interface.node.get_parameter('angular_gain').value
        tolerance = self.ros2_interface.node.get_parameter('tolerance').value
        return [linear_gain, angular_gain], tolerance
    
    def enable_srv_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        """
        Callback to enable the controller.

        Args:
            request (Empty.Request): Service request.
            response (Empty.Response): Service response.

        Returns:
            Empty.Response: Response indicating the controller is enabled.
        """
        self.controller.enable = True
        return response
    
    def set_goal_srv_callback(self, request: SendPosition.Request, response: SendPosition.Response) -> SendPosition.Response:
        """
        Callback to set a goal.

        Args:
            request (SendPosition.Request): Service request.
            response (SendPosition.Response): Service response.

        Returns:
            SendPosition.Response: Response indicating the goal is set.
        """
        self.controller.goal = [request.position.x, request.position.y]
        return response
    
    def notify_arrival(self) -> None:
        """
        Notify about arrival at the goal.
        """
        self.ros2_interface.client_futures.append(
            self.ros2_interface.notify_arrival_client.call_async(Empty.Request())
        )
        self.ros2_interface.node.get_logger().info('Arrived at the goal !')

# Define the main function that will run when this script is execute
def main(args=None):
    """
    Main function to run the ROS2 controller node.
    """
    rclpy.init(args=args)
    system = ControllerROS2()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(system.ros2_interface.node)
            incomplete_futures = []
            for f in system.ros2_interface.client_futures:
                if f.done():
                    res = f.result()
                else:
                    incomplete_futures.append(f)
            system.ros2_interface.client_futures = incomplete_futures
            
    except KeyboardInterrupt:
        system.ros2_interface.node.get_logger().info('Node has stopped cleanly.')
    except SystemExit:
        system.ros2_interface.node.get_logger().info('Node is complete.')
    except BaseException as exc:
        type = exc.__class__.__name__
        system.ros2_interface.node.get_logger().error(f'{type} exception in node has occurred.')
        raise
    finally:
        system.ros2_interface.node.destroy_node()
        rclpy.shutdown()

# If this script is being run as the main program, call the main function
if __name__ == '__main__':
    main()