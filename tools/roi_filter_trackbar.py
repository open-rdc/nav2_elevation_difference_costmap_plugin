#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class RoiFilterNode(Node):

    def __init__(self):

        super().__init__('roi_filter_trackbar')

        self.sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.cloud_callback,
            10)

        self.pub = self.create_publisher(
            PointCloud2,
            '/filtered_points',
            10)

        self.last_msg = None
        self.frame_count = 0

        cv2.namedWindow("ROI")

        # 0.01 m単位

        cv2.createTrackbar("xmin", "ROI", 300, 1000, lambda x: None)
        cv2.createTrackbar("xmax", "ROI", 700, 1000, lambda x: None)

        cv2.createTrackbar("ymin", "ROI", 400, 1000, lambda x: None)
        cv2.createTrackbar("ymax", "ROI", 600, 1000, lambda x: None)

        cv2.createTrackbar("zmin", "ROI", 300, 1000, lambda x: None)
        cv2.createTrackbar("zmax", "ROI", 700, 1000, lambda x: None)

        cv2.createTrackbar("negative", "ROI", 1, 1, lambda x: None)

        self.timer = self.create_timer(
            0.05,
            self.process)

        self.print_timer = self.create_timer(
            1.0,
            self.print_status)

        self.stats = {}

    def slider_to_meter(self, value):
        return (value - 500) / 100.0

    def get_roi(self):

        xmin = self.slider_to_meter(
            cv2.getTrackbarPos("xmin", "ROI"))

        xmax = self.slider_to_meter(
            cv2.getTrackbarPos("xmax", "ROI"))

        ymin = self.slider_to_meter(
            cv2.getTrackbarPos("ymin", "ROI"))

        ymax = self.slider_to_meter(
            cv2.getTrackbarPos("ymax", "ROI"))

        zmin = self.slider_to_meter(
            cv2.getTrackbarPos("zmin", "ROI"))

        zmax = self.slider_to_meter(
            cv2.getTrackbarPos("zmax", "ROI"))

        negative = bool(
            cv2.getTrackbarPos("negative", "ROI"))

        return xmin, xmax, ymin, ymax, zmin, zmax, negative

    def cloud_callback(self, msg):
        self.last_msg = msg

    def process(self):

        cv2.waitKey(1)

        if self.last_msg is None:
            return

        msg = self.last_msg

        points = point_cloud2.read_points(
                 msg,
                 field_names=("x", "y", "z"),
                 skip_nans=True
        )

        # structured array → 通常の(N,3)配列へ変換
        points = np.stack(
          (
                 points["x"],
                 points["y"],
                 points["z"]
          ),
        axis=1
        )

        if points.shape[0] == 0:
            return

        print(points.shape)
        #print(points.dtype)

        if len(points) == 0:
            return

        if points.ndim != 2:
            return

        xmin, xmax, ymin, ymax, zmin, zmax, negative = self.get_roi()

        mask = (
            (points[:, 0] >= xmin) &
            (points[:, 0] <= xmax) &
            (points[:, 1] >= ymin) &
            (points[:, 1] <= ymax) &
            (points[:, 2] >= zmin) &
            (points[:, 2] <= zmax)
        )

        if negative:
            filtered = points[~mask]
        else:
            filtered = points[mask]

        out_msg = point_cloud2.create_cloud_xyz32(
            msg.header,
            filtered.tolist()
        )

        self.pub.publish(out_msg)

        self.stats = {
            "orig": len(points),
            "filtered": len(filtered),
            "removed": len(points) - len(filtered),

            "xmin": xmin,
            "xmax": xmax,
            "ymin": ymin,
            "ymax": ymax,
            "zmin": zmin,
            "zmax": zmax,

            "negative": negative,

            "x_range":
                (float(points[:, 0].min()),
                 float(points[:, 0].max())),

            "y_range":
                (float(points[:, 1].min()),
                 float(points[:, 1].max())),

            "z_range":
                (float(points[:, 2].min()),
                 float(points[:, 2].max()))
        }

    def print_status(self):

        if not self.stats:
            return

        s = self.stats

        print("\n====================")

        print(
            f"Original : {s['orig']}"
        )

        print(
            f"Filtered : {s['filtered']}"
        )

        print(
            f"Removed  : {s['removed']}"
        )

        print(
            f"Mode     : {'REMOVE ROI' if s['negative'] else 'KEEP ROI'}"
        )

        print(
            f"ROI X : {s['xmin']:.2f} ~ {s['xmax']:.2f}"
        )

        print(
            f"ROI Y : {s['ymin']:.2f} ~ {s['ymax']:.2f}"
        )

        print(
            f"ROI Z : {s['zmin']:.2f} ~ {s['zmax']:.2f}"
        )

        print(
            f"Cloud X : {s['x_range'][0]:.2f} ~ {s['x_range'][1]:.2f}"
        )

        print(
            f"Cloud Y : {s['y_range'][0]:.2f} ~ {s['y_range'][1]:.2f}"
        )

        print(
            f"Cloud Z : {s['z_range'][0]:.2f} ~ {s['z_range'][1]:.2f}"
        )


def main():

    rclpy.init()

    node = RoiFilterNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
