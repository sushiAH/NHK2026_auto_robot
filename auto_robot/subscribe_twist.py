"""twistを受け取って、dynamixelモーターに指示値を送信する。
dynamixelモーターからfeedbackデータを受け取って、DynaFeedbackメッセージをpublishする

ps4コントローラーの上矢印で、フレームのdcモーターを4つ前側に回す

丸ボタンでフレームの昇降を制御する

1/8
本来であれば、各機構を動かすノードと機構に指示を出すノードを分けたほうがいいが、今回は
joyからのコールバックを受け取って動かすようにする


"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from auto_minicar.lib.dyna_lib import dxl_controller
from auto_minicar.lib.ah_python_can import *
import math
import numpy as np
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from my_robot_interfaces.msg import DynaFeedback
import atexit


bus = can.interface.Bus(
    bustype="socketcan", channel="can0", asynchronous=True, bitrate=1000000
)
SERIAL_DEVICE_NAME = "/dev/ttyUSB3"


def calc_frame_height(dis_cm):
    return int((360 / (45 * math.pi) / 0.088) * (dis_cm))


class FrameDriver:
    """フレーム駆動輪

    Attributes:
        now_button_state: [TODO:attribute]
        last_button_state: [TODO:attribute]
    """

    def __init__(self):
        send_packet_1byte(0x010, 0, 4, bus)
        send_packet_1byte(0x011, 0, 4, bus)
        send_packet_1byte(0x012, 0, 4, bus)
        send_packet_1byte(0x013, 0, 4, bus)

        self.now_button_state = 0  # 上矢印
        self.last_button_state = 0

    def move_frame_drive(self):
        if self.now_button_state == 0:
            send_packet_4byte(0x010, 3, 0, bus)  # set_goal_pos
            send_packet_4byte(0x011, 3, 0, bus)  # set_goal_pos
            send_packet_4byte(0x012, 3, 0, bus)  # set_goal_pos
            send_packet_4byte(0x013, 3, 0, bus)  # set_goal_pos

        elif self.now_button_state == 1:
            send_packet_4byte(0x010, 3, -400, bus)  # set_goal_pos
            send_packet_4byte(0x011, 3, -400, bus)  # set_goal_pos
            send_packet_4byte(0x012, 3, 400, bus)  # set_goal_pos
            send_packet_4byte(0x013, 3, 400, bus)  # set_goal_pos


class FrameController:
    """フレーム昇降

    Attributes:
        dxl_3: [TODO:attribute]
        dxl_4: [TODO:attribute]
        state_counter: [TODO:attribute]
        last_button_state: [TODO:attribute]
        now_button_state: [TODO:attribute]
        state_length: [TODO:attribute]
        state_counter: [TODO:attribute]
        last_button_state: [TODO:attribute]
    """

    def __init__(self):
        self.dxl_3 = dxl_controller(SERIAL_DEVICE_NAME, 4, 4)  # 前
        self.dxl_4 = dxl_controller(SERIAL_DEVICE_NAME, 5, 4)

        self.now_button_state = 0  # 丸ボタン
        self.last_button_state = 0

        self.state_counter = 0

        self.state_length = 4

        self.dxl_3.write_pos(600)
        self.dxl_4.write_pos(600)

    def update_frame_state(self):
        """フレーム昇降機構の状態を更新"""
        if self.now_button_state == 1 and self.last_button_state == 0:
            self.state_counter += 1

        self.state_counter = self.state_counter % self.state_length

        self.last_button_state = self.now_button_state

    def move_frame(self):
        if self.state_counter == 0:
            self.dxl_3.write_pos(600)
            self.dxl_4.write_pos(600)

        elif self.state_counter == 1:
            self.dxl_3.write_pos(calc_frame_height(210))
            self.dxl_4.write_pos(calc_frame_height(210))

        elif self.state_counter == 2:
            self.dxl_3.write_pos(calc_frame_height(-10))
            self.dxl_4.write_pos(calc_frame_height(210))

        elif self.state_counter == 3:
            self.dxl_3.write_pos(calc_frame_height(-10))
            self.dxl_4.write_pos(calc_frame_height(-10))


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

        # joy_callback 消す予定
        self.subscription_joy = self.create_subscription(
            Joy,  # メッセージの型
            "/joy",  # 購読するトピック名
            self.joy_callback,  # 呼び出すコールバック関数
            10,
        )

        # robot_params
        self.track_width = 0.285  # [m]
        self.wheel_radius = 0.041  # [m]

        self.joy_straight = 0
        self.joy_w = 0

        # dynamixel velocity gain
        self.dyna_vel_gain = (0.229 * 2.0 * math.pi * self.wheel_radius) / 60.0

        # initialize dynamixel
        # 差動二輪
        self.dxl_1 = dxl_controller(SERIAL_DEVICE_NAME, 0, 1)
        self.dxl_2 = dxl_controller(SERIAL_DEVICE_NAME, 1, 1)

        # box_arm
        self.dxl_5 = dxl_controller(SERIAL_DEVICE_NAME, 6, 3)

        self.frame_controller = FrameController()
        self.frame_driver = FrameDriver()

    def twist_by_joy_callback(self, msg):
        """Joy topicをsubscribeする

        Args:
            msg (Joy): joystick data
        """
        self.joy_straight = msg.linear.x
        self.joy_w = msg.angular.z

    def joy_callback(self, msg):
        """フレーム上昇機構、フレーム駆動輪を動かす

        Args:
            msg ([TODO:parameter]): [TODO:description]
        """
        self.frame_controller.now_button_state = msg.buttons[1]
        self.frame_driver.now_button_state = msg.axes[7]

        self.frame_controller.update_frame_state()
        self.frame_controller.move_frame()

        self.frame_driver.move_frame_drive()

    def write_to_dyna(self):
        """一定周期でdynamixelに指示値を送信する"""
        straight = -self.joy_straight
        w = self.joy_w

        V_r = int((2 * straight - w * self.track_width) / (2 * self.dyna_vel_gain))
        V_l = -int((2 * straight + w * self.track_width) / (2 * self.dyna_vel_gain))
        # print(V_r)

        self.dxl_1.write_vel(V_r)
        self.dxl_2.write_vel(V_l)

    def publish_feedback(self):
        """一定間隔で、dynamixelからのfeedback_dataを受取、publishする"""
        V_r = np.int32(self.dxl_1.read_vel()) * self.dyna_vel_gain  # rps
        V_l = -(np.int32(self.dxl_2.read_vel()) * self.dyna_vel_gain)

        feedback_data = DynaFeedback()

        feedback_data.data[0] = V_r
        feedback_data.data[1] = V_l

        self.dyna_feedback_publisher.publish(feedback_data)


def stop():

    send_packet_1byte(0x010, 0, 0, bus)
    send_packet_1byte(0x011, 0, 0, bus)
    send_packet_1byte(0x012, 0, 0, bus)
    send_packet_1byte(0x013, 0, 0, bus)

    print("stop")


atexit.register(stop)


def main():
    rclpy.init()  # rclpyライブラリの初期化

    twist_subscriber_node = twist_subscriber()

    rclpy.spin(twist_subscriber_node)  # ノードをスピンさせる
    twist_subscriber_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
