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
from rclpy.node import Node
from std_msgs.msg import Empty, Bool, Int8, String
from geometry_msgs.msg import Twist, Pose, Vector3
from sensor_msgs.msg import Range, Image, Imu

STATES = {
    0: "Landed",
    1: "Flying",
    2: "Taking off",
    3: "Landing",
}

MODES = ["velocity", "position"]


class DroneObject(Node):
    def __init__(self, node_name: str = "drone_node"):
        super().__init__(node_name)
        self._state = STATES[0]
        self._mode = MODES[0]
        self._hover_distance = 0.0
        self.isFlying = False
        self.isPosctrl = False
        self.isVelMode = False

        self.logger = self.get_logger()

        # Publishers
        self.pubTakeOff = self.create_publisher(Empty, '~/takeoff', 1024)
        self.pubLand = self.create_publisher(Empty, '~/land', 1024)
        self.pubReset = self.create_publisher(Empty, '~/reset', 1024)
        self.pubPosCtrl = self.create_publisher(Bool, '~/posctrl', 1024)
        self.pubCmd = self.create_publisher(Twist, '~/cmd_vel', 1024)
        self.pubVelMode = self.create_publisher(Bool, '~/dronevel_mode', 1024)

        # Subscribers
        self.sub_sonar = self.create_subscription(Range, '~/sonar', self.cb_sonar, 1024)
        self.sub_imu = self.create_subscription(Range, '~/imu', self.cb_imu, 1024)
        self.sub_front_img = self.create_subscription(Image, '~/front/image_raw',
                                                      self.cb_front_img, 1024)
        self.sub_bottom_img = self.create_subscription(Image, '~/bottom/image_raw',
                                                       self.cb_bottom_img, 1024)
        self.sub_gt_pose = self.create_subscription(Pose, '~/gt_pose', self.cb_gt_pose, 1024)
        self.sub_state = self.create_subscription(Int8, '~/state', self.cb_state, 1024)
        self.sub_cmd_mode = self.create_subscription(String, '~/cmd_mode', self.cb_cmd_mode, 1024)

        self._sonar = Range()
        self._imu = Imu()
        self._front_img = Image()
        self._bottom_img = Image()
        self._gt_pose = Pose()

        while self.pubTakeOff.get_subscription_count() == 0:
            self.logger.info("Waiting for drone to spawn", throttle_duration_sec=1)

    @property
    def state(self):
        return self._state

    @property
    def mode(self):
        return self._mode

    @property
    def hover_distance(self):
        return self._hover_distance

    @property
    def sonar(self):
        return self._sonar

    @property
    def imu(self):
        return self._imu

    @property
    def front_img(self):
        return self._front_img

    @property
    def bottom_img(self):
        return self._bottom_img

    @property
    def gt_pose(self):
        return self._gt_pose

    @state.setter
    def state(self, value):
        self._state = value

    @mode.setter
    def mode(self, value):
        self._mode = value

    @hover_distance.setter
    def hover_distance(self, value):
        self._hover_distance = value

    @sonar.setter
    def sonar(self, value):
        self._sonar = value

    @imu.setter
    def imu(self, value):
        self._imu = value

    @front_img.setter
    def front_img(self, value):
        self._front_img = value

    @bottom_img.setter
    def bottom_img(self, value):
        self._bottom_img = value

    @gt_pose.setter
    def gt_pose(self, value):
        self._gt_pose = value

    def takeOff(self):
        """
        Take off the drone
        :return: True if the command was sent successfully, False if drone is already flying
        """
        if self.isFlying:
            return False
        self.logger.info("Taking off")
        self.pubTakeOff.publish(Empty())
        self.isFlying = True
        return True

    def land(self):
        """
        Land the drone
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        self.logger.info("Landing")
        self.pubLand.publish(Empty())
        self.isFlying = False
        return True

    def hover(self):
        """
        Hover the drone
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def posCtrl(self, on):
        """
        Turn on/off position control
        :param on: True to turn on position control, False to turn off
        """
        self.isPosctrl = on
        bool_msg = Bool()
        bool_msg.data = on
        self.pubPosCtrl.publish(bool_msg)
        return True

    def velMode(self, on):
        """
        Turn on/off velocity control mode
        :param on: True to turn on velocity control mode, False to turn off
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        self.isVelMode = on
        bool_msg = Bool()
        bool_msg.data = on
        self.pubVelMode.publish(bool_msg)
        return True

    def move(self, v_linear: Vector3 = Vector3(),
             v_angular: Vector3 = Vector3()):
        """
        Move the drone using velocity control along the linear x and z axis and rotation around
        the x, y and z axis
        :param v_linear: Linear velocity in m/s
        :param v_angular: Angular velocity in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist(linear=v_linear, angular=v_angular)
        self.pubCmd.publish(twist_msg)
        return True

    def moveTo(self, x: float, y: float, z: float):
        """
        Move the drone to a specific position
        :param x: X position in m
        :param y: Y position in m
        :param z: Z position in m
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = z
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def pitch(self, speed):
        """
        Pitch the drone
        :param speed: Pitch speed in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 1.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = speed
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def roll(self, speed: float):
        """
        Roll the drone
        :param speed: Roll speed in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 1.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = speed
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def rise(self, speed: float):
        """
        Rise or fall the drone
        :param speed: Rise speed in m/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = speed
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def yaw(self, speed: float):
        """
        Rotate the drone around the z-axis
        :param speed: Rotation speed in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = speed
        self.pubCmd.publish(twist_msg)
        return True

    def cb_sonar(self, msg: Range):
        """Callback for the sonar sensor"""
        self._sonar = msg
        self._hover_distance = msg.min_range

    def cb_imu(self, msg: Imu):
        """Callback for the imu sensor"""
        self._imu = msg

    def cb_front_img(self, msg: Image):
        """Callback for the front camera"""
        self._front_img = msg

    def cb_bottom_img(self, msg: Image):
        """Callback for the rear camera"""
        self._bottom_img = msg

    def cb_gt_pose(self, msg: Pose) -> None:
        """Callback for the ground truth pose"""
        self._gt_pose = msg

    def cb_state(self, msg: Int8):
        """Callback for the drone state"""
        self._state = STATES[msg.data]
        self.logger.info("State: {}".format(self._state), throttle_duration_sec=1)

    def cb_cmd_mode(self, msg: String):
        """Callback for the command mode"""
        if msg.data in MODES:
            self._mode = msg.data
            self.logger.info("Changed command mode to: {}".format(self._mode))
        else:
            self.logger.error("Invalid command mode: {}".format(msg.data))

    def reset(self):
        self.pubReset.publish(Empty())
