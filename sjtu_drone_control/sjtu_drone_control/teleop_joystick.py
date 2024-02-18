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

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('teleop_node')

        # Subscribers
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.takeoff_publisher = self.create_publisher(Empty, 'takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, 'land', 10)

        # Velocity parameters
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.linear_increment = 0.05
        self.angular_increment = 0.05
        self.max_linear_velocity = 1.0
        self.max_angular_velocity = 1.0

        self.joy_axes = []
        self.joy_buttons = []

        # Start a timer to listen to keyboard inputs
        self.create_timer((0.005), self.read_joystick_input)

    def clip(self, value: float, max_val: float) -> float:
        return -max_val if value < -max_val else max_val if value > max_val else value

    def get_velocity_msg(self) -> str:
        return f"Linear Velocity: {self.linear_velocity}\nAngular Velocity: {self.angular_velocity}\n"

    def read_joystick_input(self) -> None:
        """
        Read keyboard inputs and publish corresponding commands
        """
        while rclpy.ok():
            # Print the instructions
            print(self.get_velocity_msg())
            # Handle velocity changes
            self.linear_velocity = self.clip(self.linear_velocity + (self.joy_axes[3] * self.linear_increment),
                                       self.max_linear_velocity)
            self.angular_velocity = self.clip(self.angular_velocity + (self.joy_axes[3] * self.angular_increment),
                                        self.max_angular_velocity)

            linear_vec = Vector3()
            linear_vec.x = self.joy_axes[1] * self.linear_velocity
            linear_vec.y = self.joy_axes[0] * self.linear_velocity
            linear_vec.z = self.linear_velocity

            angular_vec = Vector3()
            angular_vec.z = self.joy_axes[2] * self.angular_velocity

            self.publish_cmd_vel(linear_vec, angular_vec)

            # Handle other keys for different movements
            if self.joy_buttons[0] == 1:
                # Takeoff
                self.takeoff_publisher.publish(Empty())
            elif self.joy_buttons[1] == 1:
                # Land
                self.publish_cmd_vel()
                self.land_publisher.publish(Empty())

    def joy_callback(self, msg: Joy) -> None:
        """
        Callback for joystick input

        BUTTONS
        -------
        0 	A (CROSS)
        1 	B (CIRCLE)
        2 	X (SQUARE)
        3 	Y (TRIANGLE)
        4 	BACK (SELECT)
        5 	GUIDE (Middle/Manufacturer Button)
        6 	START
        7 	LEFTSTICK
        8 	RIGHTSTICK
        9 	LEFTSHOULDER
        10 	RIGHTSHOULDER
        11 	DPAD_UP
        12 	DPAD_DOWN
        13 	DPAD_LEFT
        14 	DPAD_RIGHT
        15 	MISC1 (Depends on the controller manufacturer, but is usually at a similar location on the controller as back/start)
        16 	PADDLE1 (Upper left, facing the back of the controller if present)
        17 	PADDLE2 (Upper right, facing the back of the controller if present)
        18 	PADDLE3 (Lower left, facing the back of the controller if present)
        19 	PADDLE4 (Lower right, facing the back of the controller if present)
        20 	TOUCHPAD (If present. Button status only)

        AXES
        ----
        0 	LEFTX
        1 	LEFTY
        2 	RIGHTX
        3 	RIGHTY
        4 	TRIGGERLEFT
        5 	TRIGGERRIGHT
        """
        self.joy_axes = msg.axes
        self.joy_buttons = msg.buttons

    def publish_cmd_vel(self, linear_vec: Vector3, angular_vec: Vector3) -> None:
        """
        Publish a Twist message to cmd_vel topic
        """
        twist = Twist(linear=linear_vec, angular=angular_vec)
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
