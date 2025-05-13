import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from ocs2_ros2_msgs.msg import MpcFlattenedController

from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class TargetTrajectoryMonitor(Node):
    def __init__(self):
        super().__init__("target_trajectory_monitor_plot")

        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            MpcFlattenedController,
            "/g1/mpc_policy",
            self.listener_callback,
            qos,
        )

        self.last_state_traj = None
        self.threshold = 0.05  # Δ超过5cm或0.05rad视为突变

        self.monitor_indices = {"base_z": 2, "base_yaw": 3, "base_left_hip_yaw_q": 6}

        # 缓存轨迹历史用于可视化
        self.time_series = []
        self.z_series = []
        self.yaw_series = []
        self.left_hip_yaw_q = []

        # 启动绘图
        self.fig, (self.ax_z, self.ax_yaw, self.ax_left_hip_yaw_q) = plt.subplots(3, 1)
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=500)
        plt.tight_layout()
        plt.show(block=False)

    def listener_callback(self, msg: MpcFlattenedController):
        traj = [
            np.array(state.value)
            for state in msg.plan_target_trajectories.state_trajectory
        ]
        times = list(msg.plan_target_trajectories.time_trajectory)

        if not traj or not times:
            return

        self.time_series = times
        self.z_series = [s[self.monitor_indices["base_z"]] for s in traj]
        self.yaw_series = [s[self.monitor_indices["base_yaw"]] for s in traj]
        self.left_hip_yaw_q = [
            s[self.monitor_indices["base_left_hip_yaw_q"]] for s in traj
        ]

        # 检查是否跳变
        for name, idx in self.monitor_indices.items():
            diffs = [abs(traj[i + 1][idx] - traj[i][idx]) for i in range(len(traj) - 1)]
            for i, d in enumerate(diffs):
                if d > self.threshold:
                    self.get_logger().warn(
                        f"[Jump Detected] {name}, frame={i}->{i+1}, Δ={d:.4f}"
                    )

    def update_plot(self, frame):
        if not self.time_series:
            return

        # 设置横轴滚动窗口
        max_t = max(self.time_series)
        min_t = max_t - 1.0

        # 筛选时间段内的点
        time_filtered = []
        z_filtered = []
        yaw_filtered = []
        hip_yaw_filtered = []

        for t, z, yaw, hip in zip(
            self.time_series, self.z_series, self.yaw_series, self.left_hip_yaw_q
        ):
            if min_t <= t <= max_t:
                time_filtered.append(t)
                z_filtered.append(z)
                yaw_filtered.append(yaw)
                hip_yaw_filtered.append(hip)

        # 清除并绘制
        self.ax_z.clear()
        self.ax_yaw.clear()
        self.ax_left_hip_yaw_q.clear()

        self.ax_z.plot(time_filtered, z_filtered, label="base_z", color="blue")
        self.ax_yaw.plot(time_filtered, yaw_filtered, label="base_yaw", color="green")
        self.ax_left_hip_yaw_q.plot(
            time_filtered, hip_yaw_filtered, label="hip_yaw_q", color="red"
        )

        # 设置固定纵轴范围
        self.ax_z.set_ylim(0.2, 1.0)
        self.ax_yaw.set_ylim(-0.5, 0.5)
        self.ax_left_hip_yaw_q.set_ylim(-1.0, 1.0)

        # 设置滚动时间窗横轴
        self.ax_z.set_xlim(min_t, max_t)
        self.ax_yaw.set_xlim(min_t, max_t)
        self.ax_left_hip_yaw_q.set_xlim(min_t, max_t)

        # 标注
        self.ax_z.set_ylabel("Base Z [m]")
        self.ax_z.grid(True)
        self.ax_z.legend()

        self.ax_yaw.set_ylabel("Base Yaw [rad]")
        self.ax_yaw.grid(True)
        self.ax_yaw.legend()

        self.ax_left_hip_yaw_q.set_xlabel("Time [s]")
        self.ax_left_hip_yaw_q.set_ylabel("Hip Yaw Q [rad]")
        self.ax_left_hip_yaw_q.grid(True)
        self.ax_left_hip_yaw_q.legend()


def main(args=None):
    rclpy.init(args=args)
    node = TargetTrajectoryMonitor()
    try:
        # 用一个 loop 边刷新边 spin_once
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # 不阻塞
            plt.pause(0.01)  # 更新 GUI 事件循环
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
