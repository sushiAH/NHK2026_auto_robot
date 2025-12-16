"""twistをpublishする
dynamixelのフィードバックデータをsubscribeして、odomをpublishする
tfを出力する
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import math
import numpy as np
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from my_robot_interfaces.msg import DynaFeedback


class odom_publisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")
        self.subscription_joy = self.create_subscription(
            Joy,  # メッセージの型
            "/joy",  # 購読するトピック名
            self.joy_callback,  # 呼び出すコールバック関数
            10,
        )
        self.subscription_joy

        self.subscription_dyna = self.create_subscription(
            DynaFeedback, "/feedback", self.feedback_callback, 10
        )
        self.subscription_dyna

        # publisherの設定
        self.odom_publisher_ = self.create_publisher(Odometry, "odom", 10)
        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel_joy", 10)

        self.tf_broadcaster_ = TransformBroadcaster(self)

        # robot parameter
        self.track_width = 0.27  # [m]
        self.wheel_radius = 0.05  # [m]

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        # timer callback の周期設定
        self.dt = 0.01  # [seconds]
        self.timer = self.create_timer(self.dt, self.publish_odometry)

        self.V_r = 0.0
        self.V_l = 0.0

    def joy_callback(self, msg):

        axes_values = msg.axes
        buttons_values = msg.buttons

        straight = axes_values[1] * 0.2
        w = axes_values[0] * 0.5

        twist = Twist()
        twist.linear.x = straight
        twist.angular.z = w

        self.twist_publisher.publish(twist)

    def feedback_callback(self, msg):
        self.V_r = -msg.data[0]
        self.V_l = -msg.data[1]

    def publish_odometry(self):

        current_time = self.get_clock().now()
        current_time_msg = current_time.to_msg()

        df_actual = (current_time - self.last_time).nanoseconds / 1e9

        # x,y,thetaの算出

        v = (self.V_r + self.V_l) / 2.0
        omega = (self.V_r - self.V_l) / self.track_width

        dt_theta = omega * df_actual

        delta_x = v * df_actual * math.cos(self.theta + (dt_theta / 2))
        delta_y = v * df_actual * math.sin(self.theta + (dt_theta / 2))

        self.x += delta_x
        self.y += delta_y

        self.theta += dt_theta

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # print('theta',self.theta)
        # print('x',self.x)
        # print('y',self.y)

        # オドメトリメッセージの作成
        odom = Odometry()
        odom.header.stamp = current_time_msg
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # ロボットの位置と向きを設定
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0

        # クォータニオンへの変換

        qx = 0.0
        qy = 0.0
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(omega)
        # odomをパブリッシュ

        self.odom_publisher_.publish(odom)

        t = TransformStamped()

        t.header.stamp = current_time_msg
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster_.sendTransform(t)

        self.last_time = current_time


def main():
    rclpy.init()  # rclpyライブラリの初期化

    odom_publisher_node = odom_publisher()

    rclpy.spin(odom_publisher_node)  # ノードをスピンさせる
    odom_publisher_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
