import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R
import math


class RvizSplineFollower(Node):
    def __init__(self):
        super().__init__("rviz_spline_follower")

        self.points = []

        self.point_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.point_callback, 10
        )

        self.path_pub = self.create_publisher(Path, "/visual_spline_path", 10)

        self._action_client = ActionClient(self, FollowPath, "follow_path")

        self.get_logger().info("RVizでpublish_pointを3回クリックしてください")

    def point_callback(self, msg):
        """rviz_publish_pointがクリックされると呼び出し

        Args:
            msg
        """

        self.points.append([msg.point.x, msg.point.y])
        self.get_logger().info(f"地点追加: {len(self.points)}点目")

        if len(self.points) == 3:
            self.get_logger().info("3点取得、スプライン経路を生成して実行します")
            path_msg = self.generate_path(self.points)
            self.path_pub.publish(path_msg)
            self.send_goal(path_msg)
            self.points = []

    def generate_path(self, pts):
        """pathをスプライン補完で生成

        Args:
            pts ([]): [list]
        """
        pts = np.array(pts)
        t = np.array([0, 1, 2])
        cs = CubicSpline(t, pts, bc_type="natural")

        t_fine = np.linspace(0, 2, 50)
        smooth_pts = cs(t_fine)

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(smooth_pts)):
            pose = PoseStamped()
            pose.header = path.header
            x, y = float(smooth_pts[i][0]), float(smooth_pts[i][1])
            pose.pose.position.x = x
            pose.pose.position.y = y

            if i < len(smooth_pts) - 1:
                next_x = float(smooth_pts[i + 1][0])
                next_y = float(smooth_pts[i + 1][1])
                yaw = math.atan2(next_y - y, next_x - x)

            quat = R.from_euler("z", yaw).as_quat()

            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = quat

            path.poses.append(pose)

        return path

    def send_goal(self, path_msg):
        self._action_client.wait_for_server()
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller.id = "FollowPath"
        self._aciton_client.send_goal_async(goal_msg)


def main():
    rclpy.init()
    rclpy.spin(RvizSplineFollower())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
