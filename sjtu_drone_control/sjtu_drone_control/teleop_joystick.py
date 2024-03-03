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
        self.create_subscription(Joy, 'joy', self.joy_callback, 0)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.takeoff_publisher = self.create_publisher(Empty, 'takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, 'land', 10)

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
        linear_vec = Vector3()
        linear_vec.x = msg.axes[1]
        linear_vec.y = msg.axes[0]
        linear_vec.z = msg.axes[3]

        angular_vec = Vector3()
        angular_vec.z = msg.axes[2]

        self.cmd_vel_publisher.publish(Twist(linear=linear_vec, angular=angular_vec))

        # Handle other keys for different movements
        if msg.buttons[0] == 1:
            # Takeoff
            self.takeoff_publisher.publish(Empty())
        elif msg.buttons[1] == 1:
            # Land
            self.cmd_vel_publisher.publish(Twist())
            self.land_publisher.publish(Empty())


def main(args=None):
    rclpy.init(args=args)

    try:
        teleop_node = TeleopNode()
        rclpy.spin(teleop_node)

    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
