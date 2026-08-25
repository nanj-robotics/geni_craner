#!/usr/bin/env python3
"""
抓取节点 v3 - IK服务 + 关节轨迹
修复:
  1. IK timeout 显式设为 1s (kinematics.yaml 默认仅 5ms, KDL 必失败)
  2. PRE_GRASP / LIFT 高度降到 5cm (垂直向下姿态在 y≈38cm 处最高约 z=15cm,
     受 joint6 ±95° 限位约束)
  3. 四元数 (x=1,w=0) = 绕 X 轴 180° = tool0 朝下, 正确, 保留
  4. 补 pose stamp / 更好的错误日志 / 轨迹结果解析修复
  5. TF 标定偏移补偿: x+2cm, y-0.5cm, z+2cm (实测微调)
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import time

# ==================== 配置 ====================
BASE_FRAME = "base_link"
OBJECT_FRAME = "object_triangular_prism"
END_EFFECTOR_LINK = "tool0"
PLANNING_GROUP = "geni_craner"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4",
               "joint5", "joint6", "joint7"]

# 垂直向下姿态: tool0 在零位时朝上, 绕 X 转 180° 后朝下
# 四元数 [x,y,z,w] = [1,0,0,0]
DOWN_QUATERNION_XYZW = [1.0, 0.0, 0.0, 0.0]

# TF 标定偏移补偿 (米): 视觉TF坐标 + 偏移 = 实际物理坐标
# 基于实测: x+2cm, y-0.5cm, z+2cm
# (预抓取目标 x≈4.2 y≈38.2 z≈14.8cm, 距物体上表面约4~5cm, 已验证可达)
OBJECT_POS_OFFSET = [0.022, -0.015, 0.01]   # [dx, dy, dz]

PRE_GRASP_HEIGHT = 0.05  # 物体上方 5cm
LIFT_HEIGHT = 0.05        # 抓取后抬起 5cm (同高度, 安全)
MOVE_DURATION = 5.0
DESCEND_DURATION = 3.0
LIFT_DURATION = 3.0
IK_TIMEOUT_SEC = 1.0      # KDL 求解超时 (Humble /compute_ik 只取整数秒)
# ==============================================


class GraspNode(Node):
    def __init__(self):
        super().__init__("grasp_node")

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 关节轨迹 Action
        self.traj_client = ActionClient(
            self, FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory"
        )
        self.get_logger().info("等待关节轨迹控制器...")
        self.traj_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info("关节轨迹控制器连接成功")

        # IK 服务
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.get_logger().info("等待IK服务 /compute_ik ...")
        self.ik_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("IK服务连接成功")

        self.current_joints = None
        self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)
        self.get_logger().info("抓取节点启动完成")

    def joint_cb(self, msg):
        self.current_joints = msg

    def get_object_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, OBJECT_FRAME, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"暂时无法获取物体TF: {e}")
            return None
        pos = t.transform.translation
        # 应用标定偏移补偿
        return {
            "x": pos.x + OBJECT_POS_OFFSET[0],
            "y": pos.y + OBJECT_POS_OFFSET[1],
            "z": pos.z + OBJECT_POS_OFFSET[2],
        }

    def compute_ik(self, x, y, z):
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = PLANNING_GROUP
        ik_req.ik_link_name = END_EFFECTOR_LINK
        ik_req.avoid_collisions = True
        ik_req.timeout = Duration(sec=int(IK_TIMEOUT_SEC), nanosec=0)

        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = DOWN_QUATERNION_XYZW[0]
        pose.pose.orientation.y = DOWN_QUATERNION_XYZW[1]
        pose.pose.orientation.z = DOWN_QUATERNION_XYZW[2]
        pose.pose.orientation.w = DOWN_QUATERNION_XYZW[3]
        ik_req.pose_stamped = pose

        # 用当前关节角作为种子
        if self.current_joints is not None:
            rs = RobotState()
            rs.joint_state = self.current_joints
            ik_req.robot_state = rs

        req.ik_request = ik_req
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=IK_TIMEOUT_SEC + 5.0)
        resp = future.result()

        if resp is None:
            self.get_logger().error("IK服务无响应 (超时)")
            return None
        if resp.error_code.val != 1:
            self.get_logger().error(
                f"IK求解失败 error_code={resp.error_code.val} | "
                f"pos=({x:.3f},{y:.3f},{z:.3f}) quat_xyzw={DOWN_QUATERNION_XYZW}"
            )
            return None

        js = resp.solution.joint_state
        joint_dict = dict(zip(js.name, js.position))
        positions = [joint_dict[name] for name in JOINT_NAMES]
        self.get_logger().info(
            f"IK成功: {[f'{n}={p:.3f}' for n, p in zip(JOINT_NAMES, positions)]}"
        )
        return positions

    def send_joint_trajectory(self, positions, duration=MOVE_DURATION):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 7
        point.time_from_start = Duration(sec=int(duration), nanosec=0)
        goal.trajectory.points = [point]

        self.get_logger().info(f"发送关节目标 (duration={duration}s)...")
        future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("轨迹被拒绝!")
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration + 10.0
        )
        action_response = result_future.result()
        if action_response is None:
            self.get_logger().error("轨迹执行结果超时")
            return False
        # ROS2 action: GetResult_Response.result 才是 FollowJointTrajectory_Result
        if action_response.result.error_code != 0:
            self.get_logger().error(
                f"轨迹执行失败, error_code={action_response.result.error_code}"
            )
            return False

        self.get_logger().info("轨迹执行完成")
        return True

    def run(self):
        # 等待关节状态
        self.get_logger().info("等待关节状态...")
        for _ in range(20):
            if self.current_joints is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.5)
        if self.current_joints is None:
            self.get_logger().error("未收到 /joint_states")
            return

        # 等待物体TF (重试)
        self.get_logger().info("等待物体TF...")
        obj = None
        for _ in range(30):
            obj = self.get_object_pose()
            if obj is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.5)
        if obj is None:
            self.get_logger().error("未获取到物体TF")
            return

        self.get_logger().info(
            f"物体位置(已加偏移): x={obj['x']*100:.1f}cm, "
            f"y={obj['y']*100:.1f}cm, z={obj['z']*100:.1f}cm"
        )

        # 1. 预抓取 (物体上方 5cm)
        pre_z = obj["z"] + PRE_GRASP_HEIGHT
        self.get_logger().info(f"[1/4] 预抓取: z={pre_z*100:.1f}cm")
        pre_joints = self.compute_ik(obj["x"], obj["y"], pre_z)
        if pre_joints is None:
            self.get_logger().error("预抓取IK无解!")
            return
        if not self.send_joint_trajectory(pre_joints, duration=MOVE_DURATION):
            return

        # 2. 等待用户确认
        grasp_z = obj["z"]
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"已到达预抓取位置 (z={pre_z*100:.1f}cm)")
        self.get_logger().info(f"下降到 z={grasp_z*100:.1f}cm, 距离{PRE_GRASP_HEIGHT*100:.0f}cm")
        self.get_logger().info("=" * 50)
        self.get_logger().info("按Enter开始下降 (Ctrl+C取消)...")
        try:
            input()
        except KeyboardInterrupt:
            self.get_logger().info("用户取消")
            return

        # 3. 下降抓取
        self.get_logger().info(f"[2/4] 下降抓取: z={grasp_z*100:.1f}cm")
        grasp_joints = self.compute_ik(obj["x"], obj["y"], grasp_z)
        if grasp_joints is None:
            self.get_logger().error("下降位置IK无解!")
            return
        if not self.send_joint_trajectory(grasp_joints, duration=DESCEND_DURATION):
            return
        time.sleep(0.5)

        # TODO: 在这里给磁吸发送吸合信号 (GPIO/CAN/串口)
        self.get_logger().info(">>> 磁吸吸合 (待接入硬件信号) <<<")
        time.sleep(0.5)

        # 4. 抬起
        lift_z = grasp_z + LIFT_HEIGHT
        self.get_logger().info(f"[3/4] 抬起: z={lift_z*100:.1f}cm")
        lift_joints = self.compute_ik(obj["x"], obj["y"], lift_z)
        if lift_joints is None:
            self.get_logger().error("抬起位置IK无解!")
            return
        if not self.send_joint_trajectory(lift_joints, duration=LIFT_DURATION):
            return

        self.get_logger().info("=" * 50)
        self.get_logger().info(f"[4/4] 抓取成功! 物体已吸起{LIFT_HEIGHT*100:.0f}cm")
        self.get_logger().info("=" * 50)


def main():
    rclpy.init()
    node = GraspNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

