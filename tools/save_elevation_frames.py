#!/usr/bin/env python3

import csv
from datetime import datetime
from pathlib import Path
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


VALUES_PER_CELL = 6
TARGET_FRAMES = 10


class SaveElevationFrames(Node):
    """指定したフレーム数のセル情報をCSVへ保存するノード。"""

    def __init__(self) -> None:
        super().__init__("save_elevation_frames")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_path = f"elevation_data_{timestamp}.csv"

        self.declare_parameter("output_path", default_path)
        self.declare_parameter("target_frames", TARGET_FRAMES)

        output_path = (
            self.get_parameter("output_path")
            .get_parameter_value()
            .string_value
        )

        self.target_frames = (
            self.get_parameter("target_frames")
            .get_parameter_value()
            .integer_value
        )

        if self.target_frames <= 0:
            raise ValueError("target_frames must be greater than 0")

        self.output_path = Path(output_path).expanduser().resolve()

        self.frame_count = 0
        self.total_cell_count = 0
        self.finished = False

        self.subscription = self.create_subscription(
            Float32MultiArray,
            "/elevation_cell_data",
            self.callback,
            10,
        )

        self.csv_file = None
        self.writer = None

        self.get_logger().info(
            f"Waiting for {self.target_frames} frames "
            "on /elevation_cell_data"
        )
        self.get_logger().info(
            f"Output file: {self.output_path}"
        )

    def open_csv(self) -> None:
        """CSVを最初の有効なフレーム受信時に開く。"""

        self.output_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.csv_file = self.output_path.open(
            mode="w",
            newline="",
            encoding="utf-8",
        )

        self.writer = csv.writer(self.csv_file)

        self.writer.writerow(
            [
                "frame",
                "cell_in_frame",
                "world_x",
                "world_y",
                "z_min",
                "z_max",
                "difference",
                "point_count",
            ]
        )

    def callback(self, msg: Float32MultiArray) -> None:
        if self.finished:
            return

        data: List[float] = list(msg.data)

        if not data:
            self.get_logger().warning(
                "Received an empty message. "
                "This message is not counted as a frame."
            )
            return

        if len(data) % VALUES_PER_CELL != 0:
            self.get_logger().error(
                f"Invalid message length: {len(data)}. "
                f"It must be divisible by {VALUES_PER_CELL}."
            )
            return

        if self.csv_file is None:
            self.open_csv()

        current_frame = self.frame_count + 1
        cells_in_frame = len(data) // VALUES_PER_CELL

        for cell_number, i in enumerate(
            range(0, len(data), VALUES_PER_CELL),
            start=1,
        ):
            wx = data[i]
            wy = data[i + 1]
            z_min = data[i + 2]
            z_max = data[i + 3]
            difference = data[i + 4]
            point_count = int(round(data[i + 5]))

            self.writer.writerow(
                [
                    current_frame,
                    cell_number,
                    wx,
                    wy,
                    z_min,
                    z_max,
                    difference,
                    point_count,
                ]
            )

        self.csv_file.flush()

        self.frame_count += 1
        self.total_cell_count += cells_in_frame

        self.get_logger().info(
            f"Saved frame {self.frame_count}/"
            f"{self.target_frames}: "
            f"{cells_in_frame} cells"
        )

        if self.frame_count >= self.target_frames:
            self.finish()

    def finish(self) -> None:
        if self.finished:
            return

        self.finished = True

        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None

        self.get_logger().info(
            f"Finished saving {self.frame_count} frames, "
            f"{self.total_cell_count} total cell records."
        )
        self.get_logger().info(
            f"Saved to: {self.output_path}"
        )

        rclpy.shutdown()

    def destroy_node(self) -> bool:
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = SaveElevationFrames()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning(
            "Interrupted before all frames were saved."
        )
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
