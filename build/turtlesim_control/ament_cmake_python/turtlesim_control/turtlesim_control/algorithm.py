#!/usr/bin/python3

import math
from numbers import Number
from typing import Callable, List, Tuple

class Controller:
    def __init__(self):
        """
        Initialize the Controller class.
        """
        self.enable = False
        self.gain = [1.0, 5.0]
        self.goal = None
        self.send_cmd_vel = None
        self.get_pose = None
        self.notify_arrival = None
        self.update_parameters = None

    def set_data_interface(self, send_cmd_vel: Callable, get_pose: Callable,
                           notify_arrival: Callable, update_parameters: Callable) -> None:
        """
        Set data interface callables for the Controller.

        Args:
            send_cmd_vel (Callable): Callable for sending velocity commands.
            get_pose (Callable): Callable for retrieving the current pose.
            notify_arrival (Callable): Callable to be invoked upon arrival.
            update_parameters (Callable): Callable to update control parameters.
        """
        self.send_cmd_vel = send_cmd_vel
        self.get_pose = get_pose
        self.notify_arrival = notify_arrival
        self.update_parameters = update_parameters

    def control_law(self, goal_position: List[Number], pose: List[Number], gain: List[Number]) -> Tuple[Number, Number, Number]:
        """
        Calculate control commands based on goal, pose, and gains.

        Args:
            goal_position (List[Number]): Desired goal position [x, y].
            pose (List[Number]): Current pose [x, y, theta].
            gain (List[Number]): Control gains [linear_gain, angular_gain].

        Returns:
            Tuple[Number, Number, Number]: Linear velocity, angular velocity, distance to goal.
        """
        position_diff = [goal_position[0] - pose[0], goal_position[1] - pose[1]]
        theta_diff = math.atan2(position_diff[1], position_diff[0]) - pose[2]
        orientation_diff = math.atan2(math.sin(theta_diff), math.cos(theta_diff))
        angular_vel = gain[1] * orientation_diff
        distance = math.sqrt(position_diff[0] ** 2 + position_diff[1] ** 2)
        linear_vel = gain[0] * distance
        return linear_vel, angular_vel, distance

    def behavior(self) -> None:
        """
        Define the behavior of the Controller.
        """
        if self.enable:
            gain, tolerance = self.update_parameters()
            pose = self.get_pose()
            if pose and self.goal:
                linear_vel, angular_vel, distance = self.control_law(
                    goal_position=self.goal,
                    pose=pose,
                    gain=gain
                )
                if abs(distance) <= tolerance:
                    linear_vel = 0.0
                    angular_vel = 0.0
                    self.enable = False
                    self.notify_arrival()
                self.send_cmd_vel(linear_vel, angular_vel)

class Scheduler:
    def __init__(self):
        """
        Initialize the Scheduler class.
        """
        self.send_goal = None
        self.complete = None
        self.via_points = None
        self.num_via_points = None
        self.idx = 0

    def set_data_interface(self, send_goal: Callable, complete: Callable) -> None:
        """
        Set data interface callables for the Scheduler.

        Args:
            send_goal (Callable): Callable for sending a goal.
            complete (Callable): Callable to indicate task completion.
        """
        self.send_goal = send_goal
        self.complete = complete

    def set_via_point(self, via_point: List[Number]) -> None:
        """
        Set a via point for the Scheduler.

        Args:
            via_point (List[Number]): Via point coordinates [x, y].
        """
        self.via_points = via_point
        self.num_via_points = len(self.via_points)

    def update(self) -> None:
        """
        Update the Scheduler's state and perform actions.
        """
        if self.idx < self.num_via_points - 1:
            self.idx += 1
            self.send_goal()
        else:
            self.complete()