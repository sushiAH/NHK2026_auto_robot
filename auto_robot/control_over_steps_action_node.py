"""段差上り、降り制御フロー
段差上り
    最初は(600,600)の状態
    フレーム上げる(210,210) [mm]
    前方に進める
    tof_forwardの値が小さくなったら(段差が近づいたら)、車体を止めて、前方の足を上げる(-10,210) [cm]
    決まった秒数が立ったら、dcモーターと、dynamixelを前方に動かす
    tof_backwardの値が小さくなったら(段差が近づいたら)、車体を止めて、後方の足を上げる(-10,-10) [cm]
    dcモーターと、dynamixelを前方に動かす
    決まった秒数が立ったら車体を止め、フレームを600,600とする
    決まった秒数が立ったら終了

段差降り(後ろ向き)
    最初は(600,600)の状態
    後方に進める
    後方のtofがしきい値を超えたら、車体を止めて、後方の足を下げる
    決まった秒数が立ったら、dcモーターとdynamixelを後方に動かす
    前方のtofがしきい値を超えたら、車体を止めて、前方の足を下げる
    後方に進める
    決まった秒数が立ったら、車体を停止して、フレームを(600,600)とする
    決まった秒数が立ったら終了
　　
"""

import rclpy
import math
import numpy as np
import time

from std_msgs.msg import UInt8, UInt16, String
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import atexit

import asyncio
from rclpy.action import ActionServer, CancelResponse

#自作ライブラリ
from auto_robot_interfaces.action import OverSteps
from dyna_interfaces.msg import DynaFeedback, DynaTarget
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from ah_python_can import *

CAN_BUS = can.interface.Bus(bustype="socketcan",
                            channel="can0",
                            asynchronous=True,
                            bitrate=1000000)

# ----Config Params -----
TOF_THRESHOLD_CLIMB = 200  # (210 - 200 = 10) < 100[mm]
TOF_THRESHOLD_DESCEND = 500


def calc_frame_height(dis_cm):
    return int((360 / (45 * math.pi) / 0.088) * (dis_cm))


def move_motor(pwm):
    send_packet_4byte(0x010, 3, -pwm, CAN_BUS)
    send_packet_4byte(0x011, 3, -pwm, CAN_BUS)
    send_packet_4byte(0x012, 3, pwm, CAN_BUS)
    send_packet_4byte(0x013, 3, pwm, CAN_BUS)


class OverStepsActionServer(Node):

    def __init__(self):
        super().__init__("over_steps_server")

        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            OverSteps,
            "over_steps",
            self.execute_callback,
            callback_group=self.cb_group,
        )

        self.subscription_tof_forward = self.create_subscription(
            UInt16,
            "/tof_forward",
            self.subscribe_tof_forward,
            10,
            callback_group=self.cb_group,
        )
        self.subscription_tof_forward

        self.subscription_tof_backward = self.create_subscription(
            UInt16,
            "/tof_backward",
            self.subscribe_tof_backward,
            10,
            callback_group=self.cb_group,
        )
        self.subscription_tof_backward

        # publisherの設定
        # dynamixel_extended
        self.dyna_extpos_publisher = self.create_publisher(
            DynaTarget, "/dyna_target_extpos", 10)

        #twistの配信
        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        #内部変数の定義
        self.tof_forward = 0
        self.tof_backward = 0

        send_packet_1byte(0x010, 0, 4, CAN_BUS)  # set_operating
        send_packet_1byte(0x011, 0, 4, CAN_BUS)  # set_operating
        send_packet_1byte(0x012, 0, 4, CAN_BUS)  # set_operating
        send_packet_1byte(0x013, 0, 4, CAN_BUS)  # set_operating

        #init_dc_motor
        move_motor(0)

        #init_dynamixel
        self.publish_dyna_extpos(4, 1700)
        self.publish_dyna_extpos(5, 1700)
        # self.publish_dyna_extpos(4, calc_frame_height(230))
        # self.publish_dyna_extpos(5, calc_frame_height(230))

    def publish_dyna_extpos(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_extpos_publisher.publish(msg)

    def publish_twist(self, straight, w):
        twist = Twist()
        twist.linear.x = float(straight)
        twist.angular.z = float(w)
        self.twist_publisher.publish(twist)

    def subscribe_tof_forward(self, msg):
        self.tof_forward = msg.data

    def subscribe_tof_backward(self, msg):
        self.tof_backward = msg.data

    # ---- Action実行メインロジック
    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        res = OverSteps.Result()
        success = False

        if req.mode == 1:
            success = await self.climb_steps()
        elif req.mode == 2:
            success = await self.descend_steps()

        res.success = success

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return res

    async def climb_steps(self):
        self.get_logger().info("段差超えアクションを開始します。")

        #フレーム上昇
        self.get_logger().info("フレーム上昇")
        self.publish_dyna_extpos(4, calc_frame_height(230))
        self.publish_dyna_extpos(5, calc_frame_height(230))
        time.sleep(4.0)

        #直進
        self.publish_twist(0.1, 0)

        #段差検知待ち
        while self.tof_forward >= TOF_THRESHOLD_CLIMB:
            time.sleep(0.1)

        #停止して前方足上げ
        self.publish_twist(0, 0)
        move_motor(0)
        self.publish_dyna_extpos(4, calc_frame_height(-10))
        time.sleep(4.0)

        #再直進＆後方段差検知待ち
        self.publish_twist(0.1, 0)
        move_motor(400)
        while (self.tof_backward >= TOF_THRESHOLD_CLIMB):
            time.sleep(0.1)

        #停止して後方足上げ
        self.publish_twist(0, 0)
        move_motor(0)
        self.publish_dyna_extpos(5, calc_frame_height(-10))
        time.sleep(4.0)

        #最終直進
        self.publish_twist(0.1, 0)
        move_motor(400)
        time.sleep(2.0)

        #フレームをもとに戻す
        self.publish_twist(0, 0)
        move_motor(0)
        self.publish_dyna_extpos(4, 600)
        self.publish_dyna_extpos(5, 600)
        time.sleep(2.0)

        return True

    async def descend_steps(self):

        self.get_logger().info("段差降りアクションを開始します。")

        #後ろ向きに直進
        self.publish_twist(-0.1, 0)

        #後方tof検知待ち
        while (self.tof_backward <= TOF_THRESHOLD_DESCEND):
            time.sleep(0.1)

        #車体を止めて、後方の足を下げる
        self.publish_twist(0, 0)
        self.publish_dyna_extpos(5, calc_frame_height(230))
        time.sleep(4.0)

        #dcモーターとdynamixelを後方に動かす
        self.publish_twist(-0.1, 0)
        move_motor(-400)

        #前方tof検知待ち
        while (self.tof_forward <= TOF_THRESHOLD_DESCEND):
            time.sleep(0.1)

        #車体を止めて、前方の足を下げる
        self.publish_twist(0, 0)
        move_motor(0)
        self.publish_dyna_extpos(4, calc_frame_height(230))
        time.sleep(4.0)

        #後ろ向きに直進する
        self.publish_twist(-0.1, 0)
        time.sleep(1.0)

        #車体を停止し、フレーム高さをもとに戻す
        self.publish_twist(0, 0)
        self.publish_dyna_extpos(4, 600)
        self.publish_dyna_extpos(5, 600)
        time.sleep(2.0)

        return True


def main():
    rclpy.init()
    node = OverStepsActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
