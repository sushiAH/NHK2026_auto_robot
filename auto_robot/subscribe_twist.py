"""twistを受け取って、dynamixelモーターに指示値を送信する。
dynamixelモーターからfeedbackデータを受け取って、DynaFeedbackメッセージをpublishする
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from auto_robot.lib.dyna_lib import dxl_controller
import math
import numpy as np
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from my_robot_interfaces.msg import DynaFeedback


class twist_subscriber(Node):
    def __init__(self):
        super().__init__("twist_subscriber")

        self.subscription_twist_joy = self.create_subscription(
            Twist,  # メッセージの型
            "/cmd_vel_joy",  # 購読するトピック名
            self.twist_by_joy_callback,  # 呼び出すコールバック関数
            10,
        )
        self.subscription_twist_joy

        self.dyna_feedback_publisher = self.create_publisher(
            DynaFeedback, "/feedback", 10
        )

        # timer callback setting
        # publish timer
        self.publish_period = 0.01
        self.publish_timer = self.create_timer(
            self.publish_period, self.publish_feedback
        )

        # write_to_dyna timer
        self.write_dyna_period = 0.01
        self.write_dyna_timer = self.create_timer(
            self.write_dyna_period, self.write_to_dyna
        )

        # robot_params
        self.track_width = 0.325  # [m]
        self.wheel_radius = 0.0275  # [m]

        self.joy_straight = 0
        self.joy_w = 0

        # dynamixel velocity gain
        self.dyna_vel_gain = (0.229 * 2.0 * math.pi * self.wheel_radius) / 60.0

        # initialize dynamixel
        self.dxl_1 = dxl_controller("/dev/ttyUSB0", 0, 3)
        self.dxl_2 = dxl_controller("/dev/ttyUSB0", 1, 3)

    def twist_by_joy_callback(self, msg):
        """Joy topicをsubscribeする

        Args:
            msg (Joy): joystick data
        """
        self.joy_straight = msg.linear.x
        self.joy_w = msg.angular.z

    def write_to_dyna(self):
        """一定周期でdynamixelに指示値を送信する"""
        straight = self.joy_straight
        w = self.joy_w

        V_r = int((2 * straight - w * self.track_width) / (2 * self.dyna_vel_gain))
        V_l = -int((2 * straight + w * self.track_width) / (2 * self.dyna_vel_gain))
        # print(V_r)

        self.dxl_1.write_vel(1, V_r)
        self.dxl_2.write_vel(2, V_l)

    def publish_feedback(self):
        """一定間隔で、dynamixelからのfeedback_dataを受取、publishする"""
        V_r = np.int32(self.dxl_1.read_vel(1)) * self.dyna_vel_gain  # rps
        V_l = -(np.int32(self.dxl_2.read_vel(2)) * self.dyna_vel_gain)

        feedback_data = DynaFeedback()

        feedback_data.data[0] = V_r
        feedback_data.data[1] = V_l

        self.dyna_feedback_publisher.publish(feedback_data)


def main():
    rclpy.init()  # rclpyライブラリの初期化

    twist_subscriber_node = twist_subscriber()

    rclpy.spin(twist_subscriber_node)  # ノードをスピンさせる
    twist_subscriber_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
