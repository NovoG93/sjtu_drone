#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import sys
import time
from enum import Enum
from math import pi

import rclpy
from rclpy import utilities
from rclpy.exceptions import ROSInterruptException
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity

from geometry_msgs.msg import Vector3
from sjtu_drone_control.drone_utils.drone_object import DroneObject


class TaskState(Enum):
    INIT = 'INIT'
    TAKEOFF = 'TAKEOFF'
    MOVE_FORWARD = 'MOVE_FORWARD'
    TURN_LEFT = 'TURN_LEFT'
    TURN_RIGHT = 'TURN_RIGHT'
    LAND = 'LAND'


class Task(Enum):
    SQUARE = 'square'
    TRIANGLE = 'triangle'
    FORWARD = 'forward'
    BACKWARD = 'backward'
    LEFT = 'left'
    RIGHT = 'right'
    UP = 'up'
    DOWN = 'down'

    @property
    def states(self):
        if self in [Task.SQUARE]:
            return [TaskState.INIT, TaskState.TAKEOFF, TaskState.MOVE_FORWARD,
                    TaskState.TURN_LEFT, TaskState.LAND]
        elif self in [Task.TRIANGLE]:
            return [TaskState.INIT, TaskState.TAKEOFF, TaskState.MOVE_FORWARD,
                    TaskState.TURN_RIGHT, TaskState.LAND]
        else:
            return [TaskState.INIT, TaskState.TAKEOFF, TaskState.MOVE_FORWARD, TaskState.LAND]


class DroneController(DroneObject):
    def __init__(self, node_name: str, args=None):
        super().__init__(node_name)
        self.task_logger = rclpy.logging.get_logger('task_logger')
        self.task = Task[args.task.upper()]
        self.number = args.number
        self.distance = args.distance
        self.logger.info(f"Drone will perform {self.task} {self.number} times")
        self.max_vel = 1.0
        self.max_yaw = 1.0

        self.hz = 30
        self.duration = 0.0
        self.possible_states = self.task.states
        self.task_state = TaskState.INIT

        self.clock = self.get_clock()
        self.task_timer = self.create_timer(1.0 / self.hz, self.task_logging)
        self.timer = self.create_timer(1.0 / self.hz, self.task_execution)

        self.current_side = 0
        self.current_turn = 0
        self.straight_duration = (self.distance / self.max_vel) * 1e9
        self.turn_duration = (pi / 2 / self.max_yaw) * 1e9
        self.start_time = None

        self.logger.info("Drone controller initialized. Taking off")
        self.takeOff()
        self.change_state(TaskState.TAKEOFF)
        time.sleep(1)

    # Task logging method
    def task_logging(self):
        self.task_logger.info(
            f"Task: {self.task}\nState: {self.task_state}\nDuration: {self.duration} \
                \nStart Time: {self.start_time}", throttle_duration_sec=1)

    # State change method
    def change_state(self, state):
        if state in self.possible_states:
            self.task_state = state
        else:
            self.logger.error(f"Cannot change to state {state}")
            self.shutdown()

    # ------- #
    #  Tasks  #
    def task_execution(self):
        current_time = self.clock.now().nanoseconds

        # Common actions for all tasks
        if self.task_state == TaskState.INIT:
            self.takeOff()
            self.change_state(TaskState.TAKEOFF)

        elif self.task_state == TaskState.TAKEOFF:
            if self.isFlying:
                self.start_time = current_time
                self.change_state(TaskState.MOVE_FORWARD)

        # Task-specific actions
        if self.task in [Task.SQUARE, Task.TRIANGLE]:
            if self.task_state == TaskState.MOVE_FORWARD:
                if current_time - self.start_time > self.straight_duration:
                    self.current_side += 1
                    turn_state = TaskState.TURN_LEFT if self.task == Task.SQUARE else \
                        TaskState.TURN_RIGHT
                    self.change_state(turn_state)
                    self.start_time = current_time
                else:
                    self.go_straight()

            elif self.task_state in [TaskState.TURN_LEFT, TaskState.TURN_RIGHT]:
                turn_duration = self.turn_duration if self.task == Task.SQUARE else \
                    (2 * pi / 3 / self.max_yaw) * 1e9
                if current_time - self.start_time > turn_duration:
                    if self.current_side < 4 if self.task == Task.SQUARE else \
                            self.current_side < 3:
                        self.change_state(TaskState.MOVE_FORWARD)
                        self.start_time = current_time
                    else:
                        self.change_state(TaskState.LAND)
                else:
                    self.go_turn(left=self.task_state == TaskState.TURN_LEFT)

        else:  # For FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN
            if self.task_state == TaskState.MOVE_FORWARD:
                if current_time - self.start_time > self.straight_duration:
                    self.change_state(TaskState.LAND)
                else:
                    direction = {
                        Task.FORWARD: (True, False, False, False, False, False),
                        Task.BACKWARD: (False, True, False, False, False, False),
                        Task.LEFT: (False, False, True, False, False, False),
                        Task.RIGHT: (False, False, False, True, False, False),
                        Task.UP: (False, False, False, False, True, False),
                        Task.DOWN: (False, False, False, False, False, True)
                    }
                    self.go_in_direction(*direction[self.task])

        if self.task_state == TaskState.LAND:
            self.land()
            self.shutdown(f"Completed {self.task.name}", 0)

    def go_in_direction(self, forward=False, backward=False,
                        left=False, right=False, up=False, down=False):
        if forward:
            self.go_straight(forward=True)
        elif backward:
            self.go_straight(forward=False)
        elif left:
            self.go_horizontal(left=True)
        elif right:
            self.go_horizontal(left=False)
        elif up:
            self.go_vertical(up=True)
        elif down:
            self.go_vertical(up=False)

    # ----------------- #
    #  Basic Movements  #
    def go_straight(self, forward: bool = True):
        self.logger.info("Moving straight forward" if forward else "Moving straight backward")
        v_linear = Vector3()
        v_linear.x = self.max_vel if forward else -self.max_vel
        self.move(v_linear=v_linear)

    def go_horizontal(self, left: bool = True):
        self.logger.info('Moving left' if left else 'Moving right')
        v_linear = Vector3()
        v_linear.y = self.max_vel if left else -self.max_vel
        self.move(v_linear=v_linear)

    def go_vertical(self, up: bool = True):
        self.logger.info('Moving up' if up else 'Moving down')
        v_linear = Vector3()
        v_linear.z = self.max_vel if up else -self.max_vel
        self.move(v_linear=v_linear)

    def go_turn(self, left: bool = True):
        self.logger.info('Rotating left' if left else 'Rotating right')
        v_angular = Vector3()
        v_angular.z = self.max_yaw if left else -self.max_yaw
        self.move(v_angular=v_angular)

    def shutdown(self, reason: str = "Unknown", exit_code: int = 1):
        if exit_code == 0:
            self.logger.info("Shutting down: {}".format(reason))
        else:
            self.logger.error("Shutting down: {}".format(reason))
        self.hover()
        self.land()
        self.destroy_node()
        rclpy.shutdown(context=self.context)
        sys.exit(exit_code)


def main(args=None):
    rclpy.init(args=args)
    args = utilities.remove_ros_args(sys.argv[1:])
    parser = argparse.ArgumentParser(description='Open Loop Controller')
    choices = ['square', 'triangle', 'forward', 'backward', 'left', 'right', 'up', 'down']
    parser.add_argument('--task', '-t', type=str, help='Side length of the square',
                        default='backward', choices=choices)
    parser.add_argument('--number', '-n', type=int, help='Number of times to perform task',
                        default=1, choices=range(1, 3))
    parser.add_argument('--distance', '-d', type=float, help='Distance to travel', default=1,
                        choices=range(1, 5))
    args = parser.parse_args(args)

    node_name = 'drone'
    drone_ctrl = DroneController(node_name=node_name, args=args)
    drone_ctrl.logger.set_level(LoggingSeverity.DEBUG)

    executor = MultiThreadedExecutor()
    executor.add_node(drone_ctrl)

    try:
        executor.spin()
    except (KeyboardInterrupt, ROSInterruptException) as e:
        drone_ctrl.get_logger().error(f'Error: {e}')
    finally:
        sys.exit()


if __name__ == '__main__':
    main()
