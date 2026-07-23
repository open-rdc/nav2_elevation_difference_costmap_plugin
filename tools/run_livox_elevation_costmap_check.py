#!/usr/bin/env python3
"""Check Livox rosbag data with ElevationLayer without launching all of Nav2.

The script starts only nav2_costmap_2d, activates its lifecycle, relays
/livox/lidar to the plugin's hard-coded /surestar_points input, plays the bag
with simulated time, and checks the resulting /costmap/costmap.

Usage:
  python3 run_livox_elevation_costmap_check.py /path/to/bag_directory
"""

from __future__ import annotations

import argparse
import os
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import textwrap
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener


def make_params_file(
    global_frame: str,
    width: int,
    height: int,
    resolution: float,
) -> Path:
    """Create parameters for a standalone nav2_costmap_2d lifecycle node."""
    params = textwrap.dedent(
        f"""
        costmap:
          costmap:
            ros__parameters:
              use_sim_time: true
              global_frame: {global_frame}
              robot_base_frame: base_link
              transform_tolerance: 1.0
              update_frequency: 5.0
              publish_frequency: 5.0
              rolling_window: true
              width: {width}
              height: {height}
              resolution: {resolution}
              robot_radius: 0.2
              plugins: ["elevation_layer"]
              elevation_layer:
                plugin: "nav2_elevation_difference_costmap_plugin::ElevationLayer"
                enabled: true
              always_send_full_costmap: true
        """
    ).strip()
    path = Path(tempfile.gettempdir()) / "livox_elevation_costmap_check.yaml"
    path.write_text(params + "\n", encoding="utf-8")
    return path


def make_rviz_file(
    global_frame: str,
    cloud_topic: str,
    costmap_topic: str,
) -> Path:
    """Create an RViz configuration for the standalone check."""
    costmap_updates_topic = f"{costmap_topic}_updates"
    config = textwrap.dedent(
        f"""
        Panels:
          - Class: rviz_common/Displays
            Name: Displays
        Visualization Manager:
          Class: ""
          Displays:
            - Class: rviz_default_plugins/Grid
              Enabled: true
              Name: Grid
              Plane: XY
              Reference Frame: <Fixed Frame>
            - Class: rviz_default_plugins/TF
              Enabled: true
              Name: TF
              Show Arrows: true
              Show Axes: true
              Show Names: true
            - Alpha: 0.75
              Class: rviz_default_plugins/Map
              Color Scheme: costmap
              Draw Behind: false
              Enabled: true
              Name: Elevation Costmap
              Topic:
                Depth: 5
                Durability Policy: Volatile
                History Policy: Keep Last
                Reliability Policy: Reliable
                Value: {costmap_topic}
              Update Topic:
                Depth: 5
                Durability Policy: Volatile
                History Policy: Keep Last
                Reliability Policy: Reliable
                Value: {costmap_updates_topic}
            - Alpha: 1
              Class: rviz_default_plugins/PointCloud2
              Color Transformer: AxisColor
              Enabled: true
              Name: Livox Points
              Position Transformer: XYZ
              Size (Pixels): 2
              Style: Points
              Topic:
                Depth: 5
                Durability Policy: Volatile
                History Policy: Keep Last
                Reliability Policy: Best Effort
                Value: {cloud_topic}
          Enabled: true
          Global Options:
            Background Color: 48; 48; 48
            Fixed Frame: {global_frame}
            Frame Rate: 30
          Name: root
          Tools:
            - Class: rviz_default_plugins/Interact
            - Class: rviz_default_plugins/MoveCamera
            - Class: rviz_default_plugins/Select
          Views:
            Current:
              Class: rviz_default_plugins/Orbit
              Distance: 20
              Focal Point:
                X: 0
                Y: 0
                Z: 0
              Name: Current View
              Pitch: 0.8
              Target Frame: {global_frame}
              Yaw: 0.8
        Window Geometry:
          Height: 900
          Width: 1400
        """
    ).strip()
    path = Path(tempfile.gettempdir()) / "livox_elevation_costmap_check.rviz"
    path.write_text(config + "\n", encoding="utf-8")
    return path


def start_process(command, name):
    env = os.environ.copy()
    env.setdefault("ROS_LOG_DIR", "/tmp")
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        start_new_session=True,
    )
    print(f"[start] {name}: pid={process.pid}")
    return process


def stop_process(process, name):
    if process is None or process.poll() is not None:
        return
    print(f"[stop] {name}")
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5.0)


def lifecycle_get(node_name):
    try:
        result = subprocess.run(
            ["ros2", "lifecycle", "get", node_name],
            text=True,
            capture_output=True,
            timeout=2.0,
        )
    except subprocess.TimeoutExpired:
        return None
    if result.returncode != 0 or not result.stdout.strip():
        return None
    return result.stdout.strip().split()[0]


def lifecycle_set(node_name, transition):
    command = ["ros2", "lifecycle", "set", node_name, transition]
    result = subprocess.run(
        command,
        text=True,
        capture_output=True,
        timeout=10.0,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"{' '.join(command)} failed\n"
            f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )
    print(result.stdout.strip())


def wait_for_lifecycle_node(node_name, process, timeout_sec=15.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if process.poll() is not None:
            output = process.stdout.read() if process.stdout is not None else ""
            raise RuntimeError(
                f"{node_name} exited before lifecycle activation "
                f"(exit code {process.returncode}).\n{output[-4000:]}"
            )
        state = lifecycle_get(node_name)
        if state is not None:
            return state
        time.sleep(0.2)
    raise RuntimeError(f"Lifecycle node {node_name} was not found.")


def ensure_costmap_configured(node_name, process):
    state = wait_for_lifecycle_node(node_name, process)
    print(f"[lifecycle] {node_name}: {state}")

    if state == "unconfigured":
        lifecycle_set(node_name, "configure")
        state = wait_for_lifecycle_node(node_name, process)
        print(f"[lifecycle] {node_name}: {state}")

    if state not in ("inactive", "active"):
        raise RuntimeError(
            f"{node_name} is {state}, expected inactive or active."
        )
    return state


def activate_costmap(node_name, process):
    state = wait_for_lifecycle_node(node_name, process)
    if state == "inactive":
        lifecycle_set(node_name, "activate")
        state = wait_for_lifecycle_node(node_name, process)
        print(f"[lifecycle] {node_name}: {state}")

    if state != "active":
        raise RuntimeError(f"{node_name} is {state}, expected active.")


def wait_for_bag_tf(node, bag_process, timeout_sec=8.0):
    """Wait until the bag supplies odom->base_link->cloud_frame TF."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if node.tf_ok:
            print(
                f"[tf] {node.global_frame} <- {node.last_frame}: available"
            )
            return
        if bag_process.poll() is not None:
            raise RuntimeError(
                "ros2 bag play finished before the required TF became available. "
                f"Last TF error: {node.tf_error}"
            )
        time.sleep(0.1)
    raise RuntimeError(
        f"Timed out waiting for TF {node.global_frame} <- "
        f"{node.last_frame or 'cloud_frame'}. Last error: {node.tf_error}"
    )


class LivoxCheckNode(Node):
    """Relay the recorded cloud and collect compatibility/costmap results."""

    def __init__(self, input_topic, output_topic, costmap_topic, global_frame):
        super().__init__(
            "livox_elevation_costmap_check",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True)
            ],
        )
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.costmap_topic = costmap_topic
        self.global_frame = global_frame

        self.cloud_count = 0
        self.relay_count = 0
        self.costmap_count = 0
        self.max_positive_cells = 0
        self.max_cost = 0
        self.last_frame = ""
        self.last_point_count = 0
        self.last_xyz_range = None
        self.cloud_error = None
        self.tf_ok = False
        self.tf_error = "No cloud has been received."

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # This matches ElevationLayer's rclcpp::SensorDataQoS subscription.
        self.cloud_pub = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            costmap_topic,
            self.on_costmap,
            10,
        )

    @staticmethod
    def xyz_fields(
        msg: PointCloud2,
    ) -> Tuple[Optional[Dict[str, PointField]], Optional[str]]:
        fields = {field.name: field for field in msg.fields}
        missing = [name for name in ("x", "y", "z") if name not in fields]
        if missing:
            return None, f"Missing fields: {', '.join(missing)}"

        invalid = [
            name
            for name in ("x", "y", "z")
            if fields[name].datatype != PointField.FLOAT32
            or fields[name].count != 1
        ]
        if invalid:
            return (
                None,
                "ElevationLayer requires scalar FLOAT32 fields: "
                + ", ".join(invalid),
            )
        return fields, None

    @staticmethod
    def sampled_xyz_range(msg, fields, max_samples=2000):
        point_count = int(msg.width) * int(msg.height)
        if point_count == 0 or msg.point_step == 0:
            return None

        unpack_float = struct.Struct(
            (">" if msg.is_bigendian else "<") + "f"
        ).unpack_from
        offsets = tuple(fields[name].offset for name in ("x", "y", "z"))
        stride = max(1, (point_count + max_samples - 1) // max_samples)
        data = memoryview(msg.data)
        mins = [float("inf")] * 3
        maxs = [float("-inf")] * 3
        valid = 0

        for index in range(0, point_count, stride):
            base = index * int(msg.point_step)
            xyz = tuple(unpack_float(data, base + offset)[0] for offset in offsets)
            if not all(value == value and abs(value) != float("inf") for value in xyz):
                continue
            valid += 1
            for axis, value in enumerate(xyz):
                mins[axis] = min(mins[axis], value)
                maxs[axis] = max(maxs[axis], value)

        if valid == 0:
            return None
        return (*mins, *maxs)

    def on_cloud(self, msg):
        self.cloud_count += 1
        self.last_frame = msg.header.frame_id
        self.last_point_count = int(msg.width) * int(msg.height)

        fields, error = self.xyz_fields(msg)
        self.cloud_error = error
        if error is not None:
            return

        if self.cloud_count == 1 and fields is not None:
            self.last_xyz_range = self.sampled_xyz_range(msg, fields)

        # Relay without changing timestamp, frame, fields, or point data.
        self.cloud_pub.publish(msg)
        self.relay_count += 1

        # ElevationLayer itself requests the latest available transform.
        try:
            self.tf_buffer.lookup_transform(
                self.global_frame,
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.02),
            )
            self.tf_ok = True
            self.tf_error = ""
        except Exception as exc:
            self.tf_error = str(exc)

    def on_costmap(self, msg):
        self.costmap_count += 1
        positive_cells = sum(value > 0 for value in msg.data)
        self.max_positive_cells = max(self.max_positive_cells, positive_cells)
        if msg.data:
            self.max_cost = max(self.max_cost, max(msg.data))

    def result_ready(self):
        return (
            self.cloud_count > 0
            and self.tf_ok
            and self.costmap_count > 0
            and self.max_positive_cells > 0
        )

    def print_result(self):
        print("\n[rosbag]")
        print(f"received clouds : {self.cloud_count}")
        print(f"relayed clouds  : {self.relay_count}")
        print(f"cloud frame     : {self.last_frame or '(none)'}")
        print(f"points/frame    : {self.last_point_count}")

        if self.last_xyz_range is not None:
            xmin, ymin, zmin, xmax, ymax, zmax = self.last_xyz_range
            print(
                "sampled XYZ     : "
                f"x=[{xmin:.2f}, {xmax:.2f}], "
                f"y=[{ymin:.2f}, {ymax:.2f}], "
                f"z=[{zmin:.2f}, {zmax:.2f}] m"
            )

        print("\n[compatibility]")
        print(f"PointCloud2 XYZ : {'OK' if self.cloud_error is None and self.cloud_count else 'NG'}")
        if self.cloud_error is not None:
            print(f"  reason        : {self.cloud_error}")
        print(
            f"TF {self.global_frame} <- {self.last_frame or 'cloud_frame'}: "
            f"{'OK' if self.tf_ok else 'NG'}"
        )
        if not self.tf_ok:
            print(f"  reason        : {self.tf_error}")

        print("\n[costmap]")
        print(f"received maps   : {self.costmap_count}")
        print(f"positive cells  : {self.max_positive_cells}")
        print(f"maximum cost    : {self.max_cost}")

        if self.result_ready():
            print("\nOK: /livox/lidar produced elevation costs.")
        else:
            print("\nNG: the verification conditions were not all satisfied.")


def validate_bag_path(path: Path) -> Path:
    path = path.expanduser().resolve()
    if path.is_file() and path.name == "metadata.yaml":
        path = path.parent
    if not path.is_dir():
        raise ValueError(f"Bag directory does not exist: {path}")
    if not (path / "metadata.yaml").is_file():
        raise ValueError(
            f"metadata.yaml was not found in bag directory: {path}"
        )
    return path


def main():
    parser = argparse.ArgumentParser(
        description="Run a standalone ElevationLayer check with a Livox rosbag."
    )
    parser.add_argument(
        "bag",
        type=Path,
        help="rosbag2 directory containing metadata.yaml and the db3 file",
    )
    parser.add_argument("--global-frame", default="odom")
    parser.add_argument("--input-topic", default="/livox/lidar")
    parser.add_argument("--output-topic", default="/surestar_points")
    parser.add_argument("--costmap-topic", default="/costmap/costmap")
    # Nav2 Humble declares width and height as integer parameters.
    parser.add_argument("--width", type=int, default=20)
    parser.add_argument("--height", type=int, default=20)
    parser.add_argument("--resolution", type=float, default=0.1)
    parser.add_argument("--timeout", type=float, default=25.0)
    parser.add_argument(
        "--rviz",
        action="store_true",
        help="open RViz2 and keep the check running until RViz is closed",
    )
    args = parser.parse_args()

    try:
        bag_path = validate_bag_path(args.bag)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    params_file = make_params_file(
        args.global_frame,
        args.width,
        args.height,
        args.resolution,
    )
    costmap = None
    bag = None
    rviz = None
    node = None
    spin_stop = threading.Event()
    spin_thread = None

    try:
        costmap = start_process(
            [
                "ros2",
                "run",
                "nav2_costmap_2d",
                "nav2_costmap_2d",
                "--ros-args",
                "--params-file",
                str(params_file),
            ],
            "nav2_costmap_2d",
        )

        rclpy.init()
        node = LivoxCheckNode(
            args.input_topic,
            args.output_topic,
            args.costmap_topic,
            args.global_frame,
        )

        def spin_node():
            while rclpy.ok() and not spin_stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)

        spin_thread = threading.Thread(target=spin_node, daemon=True)
        spin_thread.start()

        # Configure first so the plugin subscriptions and TF listener exist,
        # but do not activate yet. Activation requires odom->base_link, which
        # is supplied by the bag and is unavailable before playback starts.
        ensure_costmap_configured("/costmap/costmap", costmap)

        bag = start_process(
            ["ros2", "bag", "play", str(bag_path), "--clock"],
            "ros2 bag play",
        )
        wait_for_bag_tf(node, bag)
        activate_costmap("/costmap/costmap", costmap)

        if args.rviz:
            rviz_file = make_rviz_file(
                args.global_frame,
                args.output_topic,
                args.costmap_topic,
            )
            rviz = start_process(
                [
                    "rviz2",
                    "-d",
                    str(rviz_file),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                ],
                "rviz2",
            )

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            if node.result_ready():
                # Allow a few additional costmap updates before reporting maxima.
                time.sleep(1.0)
                break
            if bag.poll() is not None and node.cloud_count > 0:
                # Give the costmap time to publish its last update.
                time.sleep(2.0)
                break
            time.sleep(0.1)

        node.print_result()
        result_code = 0 if node.result_ready() else 2

        if rviz is not None and rviz.poll() is None:
            print(
                "\n[rviz] RViz2を閉じるか、このターミナルでCtrl-Cを押すと終了します。"
            )
            while rviz.poll() is None:
                time.sleep(0.2)

        return result_code

    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        spin_stop.set()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        stop_process(rviz, "rviz2")
        stop_process(bag, "ros2 bag play")
        stop_process(costmap, "nav2_costmap_2d")

        if costmap is not None and costmap.stdout is not None:
            output = costmap.stdout.read()
            if output:
                print("\n[costmap output]")
                print(output[-5000:])


if __name__ == "__main__":
    sys.exit(main())
