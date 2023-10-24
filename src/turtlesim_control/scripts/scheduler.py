#!/usr/bin/python3

# other libraries
import sys
import math
import yaml, argparse

# RCLPY libraries, classes, functions
import rclpy

# ROS packages
from turtlesim_interfaces.srv import SendPosition
from std_msgs.msg import Int16
from std_srvs.srv import Empty

# Package module
from turtlesim_control.ros2_interface import SchedulerROS2Interface
from turtlesim_control.algorithm import Scheduler


class SchedulerROS2:
    def __init__(self, file_path: str) -> None:
        """
        Initialize the SchedulerROS2 class.

        Args:
            file_path (str): Path to the YAML file containing via points.
        """
        self.scheduler = Scheduler()
        self.ros2_interface = SchedulerROS2Interface(self.notify_arrival_srv_callback)
        self.scheduler.set_data_interface(self.send_goal, self.complete)
        
        wait_time = 10  # seconds
        time_out = 1.0  # seconds
        wait_counter = 0
        
        while (wait_counter > math.floor(wait_time / time_out)) or (not self.ros2_interface.set_goal_client.wait_for_service(timeout_sec=time_out)):
            self.ros2_interface.node.get_logger().info('set_goal service not available, waiting again')
            wait_counter += 1
        
        if wait_counter > 0:
            self.ros2_interface.node.get_logger().info(f'The node waited for too long. Shutting down {self.ros2_interface.node.get_name()} in process...')
            sys.exit()
        else:
            self.heartbeat_timer = self.ros2_interface.node.create_timer(
                timer_period_sec=1.0,
                callback=self.heartbeat
            )
            
            with open(file=file_path, mode='r') as file:
                data = yaml.load(file, Loader=yaml.SafeLoader)
            via_point = data['via_point']
            self.scheduler.set_via_point(via_point)
            self.scheduler.send_goal()

    def heartbeat(self) -> None:
        """
        Publish the current index as heartbeat.
        """
        msg = Int16()
        msg.data = self.scheduler.idx
        self.ros2_interface.pub_scheduler_index.publish(msg)
    
    def notify_arrival_srv_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        """
        Callback to notify arrival.

        Args:
            request (Empty.Request): Service request.
            response (Empty.Response): Service response.

        Returns:
            Empty.Response: Response indicating arrival notification.
        """
        self.scheduler.update()
        return response
    
    def send_goal(self) -> None:
        """
        Send the current goal to the robot.
        """
        x, y = self.scheduler.via_points[self.scheduler.idx]
        request = SendPosition.Request()
        request.position.x = x
        request.position.y = y
        self.ros2_interface.client_futures.append(
            self.ros2_interface.set_goal_client.call_async(request)
        )
        self.ros2_interface.client_futures.append(
            self.ros2_interface.enable_client.call_async(Empty.Request())
        )
        self.ros2_interface.node.get_logger().info('Goal has been sent !')
    
    def complete(self) -> None:
        """
        Complete the scheduling and shut down the node.
        """
        self.ros2_interface.node.get_logger().info('All via points have been reached.')
        self.ros2_interface.node.get_logger().info('Shutting down Scheduler...')
        sys.exit()

# Define the main function that will run when this script is execute
def main(args=None):
    """
    Main function to run the ROS2 scheduler node.
    """
    parser = argparse.ArgumentParser(description='schedule via points')
    parser.add_argument('-f', '--file', help='path to the YAML file of via points')
    parsed_args, remaining_args = parser.parse_known_args(args=args)

    rclpy.init(args=remaining_args)
    system = SchedulerROS2(parsed_args.file)
    
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
