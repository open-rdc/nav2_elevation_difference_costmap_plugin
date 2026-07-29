#!/usr/bin/env python3

import argparse
import os
import signal
import subprocess
import sys
import tempfile
import threading
import textwrap
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster


def make_params_file() -> Path:
    params = textwrap.dedent(
        """
        costmap:
          costmap:
            ros__parameters:
              use_sim_time: false
              global_frame: map
              robot_base_frame: base_link
              transform_tolerance: 1.0
              update_frequency: 5.0
              publish_frequency: 5.0
              rolling_window: true
              width: 8
              height: 8
              resolution: 0.1
              robot_radius: 0.2
              plugins: ["elevation_layer"]
              elevation_layer:
                plugin: "nav2_elevation_difference_costmap_plugin::ElevationLayer"
                enabled: true
              always_send_full_costmap: true
        """
    ).strip()
    path = Path(tempfile.gettempdir()) / "elevation_layer_pose_check.yaml"
    path.write_text(params + "\n", encoding="utf-8")
    return path


def start_process(cmd, name):
    env = os.environ.copy()
    env.setdefault("ROS_LOG_DIR", "/tmp")
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        start_new_session=True,
    )
    print(f"[start] {name}: pid={process.pid}")
    return process


def stop_process(process, name):
    if process.poll() is not None:
        return
    print(f"[stop] {name}")
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5.0)


def lifecycle_set(node_name, transition):
    cmd = ["ros2", "lifecycle", "set", node_name, transition]
    result = subprocess.run(cmd, text=True, capture_output=True, timeout=10.0)
    if result.returncode != 0:
        raise RuntimeError(
            f"{' '.join(cmd)} failed\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )
    print(result.stdout.strip())


def lifecycle_get(node_name):
    cmd = ["ros2", "lifecycle", "get", node_name]
    result = subprocess.run(cmd, text=True, capture_output=True, timeout=10.0)
    if result.returncode != 0:
        return None
    output = result.stdout.strip()
    if not output:
        return None
    return output.split()[0]


def wait_for_lifecycle_node(node_name, timeout_sec=10.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        state = lifecycle_get(node_name)
        if state is not None:
            return state
        time.sleep(0.2)
    raise RuntimeError(f"Lifecycle node {node_name} was not found.")


def ensure_costmap_active(node_name):
    state = wait_for_lifecycle_node(node_name)
    print(f"[lifecycle] {node_name}: {state}")

    if state == "unconfigured":
        lifecycle_set(node_name, "configure")
        state = wait_for_lifecycle_node(node_name)
        print(f"[lifecycle] {node_name}: {state}")

    if state == "inactive":
        lifecycle_set(node_name, "activate")
        state = wait_for_lifecycle_node(node_name)
        print(f"[lifecycle] {node_name}: {state}")

    if state != "active":
        raise RuntimeError(f"{node_name} is {state}, expected active.")


class PoseCheckNode(Node):
    def __init__(self, input_topic):
        super().__init__("elevation_layer_pose_check")
        self.br = TransformBroadcaster(self)
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/surestar_points", qos_profile_sensor_data
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, "/costmap/costmap", self.on_costmap, 10
        )
        self.costmap_counts = []
        self.costmap_maxes = []
        self.cloud_count = 0
        self.cloud_frame = ""
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.timer = self.create_timer(0.05, self.publish_tf)

    def on_cloud(self, msg):
        self.cloud_count += 1
        self.cloud_frame = msg.header.frame_id
        self.cloud_pub.publish(msg)

    def on_costmap(self, msg):
        occupied = sum(1 for value in msg.data if value > 0)
        max_cost = max(msg.data) if msg.data else 0
        self.costmap_counts.append(occupied)
        self.costmap_maxes.append(max_cost)
        if len(self.costmap_counts) > 80:
            self.costmap_counts.pop(0)
            self.costmap_maxes.pop(0)

    def publish_tf(self):
        stamp = self.get_clock().now().to_msg()
        transforms = []

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.translation.x = self.offset_x
        map_to_odom.transform.translation.y = self.offset_y
        map_to_odom.transform.rotation.w = 1.0
        transforms.append(map_to_odom)

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_link"
        odom_to_base.transform.rotation.w = 1.0
        transforms.append(odom_to_base)

        base_to_sensor = TransformStamped()
        base_to_sensor.header.stamp = stamp
        base_to_sensor.header.frame_id = "base_link"
        base_to_sensor.child_frame_id = "surestar_frame"
        base_to_sensor.transform.rotation.w = 1.0
        transforms.append(base_to_sensor)

        self.br.sendTransform(transforms)

    def wait_for_cloud(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.cloud_count > 0:
                return True
            time.sleep(0.1)
        return False

    def wait_for_costs(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            time.sleep(0.1)
            if self.costmap_counts and max(self.costmap_counts[-10:]) > 0:
                return (
                    max(self.costmap_counts[-10:]),
                    max(self.costmap_maxes[-10:]),
                )
        return (0, 0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "input_topic",
        help="real-time sensor_msgs/msg/PointCloud2 topic",
    )
    parser.add_argument("--rviz", action="store_true", help="also open RViz2")
    args = parser.parse_args()

    params_file = make_params_file()
    costmap_cmd = [
        "ros2",
        "run",
        "nav2_costmap_2d",
        "nav2_costmap_2d",
        "--ros-args",
        "--params-file",
        str(params_file),
    ]
    costmap = start_process(costmap_cmd, "nav2_costmap_2d")
    rviz = None

    try:
        rclpy.init()
        node = PoseCheckNode(args.input_topic)
        spin_stop = threading.Event()

        def spin_node():
            while rclpy.ok() and not spin_stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)

        spin_thread = threading.Thread(target=spin_node, daemon=True)
        spin_thread.start()

        time.sleep(2.0)
        ensure_costmap_active("/costmap/costmap")

        if args.rviz:
            rviz_config = Path(__file__).with_name("elevation_layer_pose_check.rviz")
            rviz = start_process(["rviz2", "-d", str(rviz_config)], "rviz2")

        try:
            print(f"[input] waiting for PointCloud2: {args.input_topic}")
            if not node.wait_for_cloud(8.0):
                print(f"NG: no PointCloud2 messages received from {args.input_topic}.")
                return 1
            print(
                f"[input] received clouds: {node.cloud_count}, "
                f"frame: {node.cloud_frame}"
            )

            print("[check] initial pose: map->odom = (0, 0)")
            node.offset_x = 0.0
            node.offset_y = 0.0
            initial_count, initial_max = node.wait_for_costs(8.0)
            print(
                f"[result] initial occupied cells: {initial_count}, "
                f"max cost: {initial_max}"
            )

            print("[check] after pose correction: map->odom = (0.5, 0.2)")
            node.costmap_counts.clear()
            node.costmap_maxes.clear()
            node.offset_x = 0.5
            node.offset_y = 0.2
            shifted_count, shifted_max = node.wait_for_costs(8.0)
            print(
                f"[result] shifted occupied cells: {shifted_count}, "
                f"max cost: {shifted_max}"
            )

            if initial_count <= 0:
                print("NG: initial costmap did not receive elevation costs.")
                return 2
            if shifted_count <= 0:
                print("NG: costmap became empty after map->odom correction.")
                return 3

            print("OK: costmap still contains elevation costs after map->odom correction.")
            if args.rviz:
                print("RViz2 is open. Press Ctrl-C here when finished.")
                while rclpy.ok():
                    time.sleep(0.2)
            return 0
        finally:
            spin_stop.set()
            spin_thread.join(timeout=2.0)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    finally:
        if rviz is not None:
            stop_process(rviz, "rviz2")
        stop_process(costmap, "nav2_costmap_2d")
        if costmap.stdout is not None:
            output = costmap.stdout.read()
            if output:
                print("\n[costmap output]")
                print(output[-4000:])


if __name__ == "__main__":
    sys.exit(main())
