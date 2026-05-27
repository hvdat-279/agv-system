import os
from typing import Dict, List, Tuple

from agv_interfaces.msg import DetectedCargo
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml


DEFAULT_RANGES = {
    'red': [
        {'h_min': 0, 'h_max': 10, 's_min': 50, 's_max': 255, 'v_min': 50, 'v_max': 255},
        {'h_min': 160, 'h_max': 180, 's_min': 50, 's_max': 255, 'v_min': 50, 'v_max': 255},
    ],
    'blue': [
        {'h_min': 100, 'h_max': 130, 's_min': 50, 's_max': 255, 'v_min': 50, 'v_max': 255},
    ],
    'yellow': [
        {'h_min': 20, 'h_max': 35, 's_min': 50, 's_max': 255, 'v_min': 50, 'v_max': 255},
    ],
}


class ColorDetector(Node):
    def __init__(self) -> None:
        super().__init__('color_detector')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('config_file', '')
        self.declare_parameter('area_min', 500.0)
        self.declare_parameter('process_every_n', 3)
        self.declare_parameter('debug_view', False)
        self.declare_parameter('object_width_m', 0.2)
        self.declare_parameter('focal_length_px', 550.0)
        self.declare_parameter('angle_per_pixel', 0.002)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.ranges = self._load_color_ranges()

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.publisher = self.create_publisher(DetectedCargo, '/detected_cargo', 10)

        self.get_logger().info(f'Color detector listening on {image_topic}')

    def _load_color_ranges(self) -> Dict[str, List[Dict[str, int]]]:
        config_param = self.get_parameter('config_file').get_parameter_value().string_value
        if config_param:
            config_path = config_param
        else:
            share_dir = get_package_share_directory('agv_perception')
            config_path = os.path.join(share_dir, 'config', 'color_ranges.yaml')

        try:
            with open(config_path, 'r', encoding='utf-8') as file:
                data = yaml.safe_load(file) or {}
        except FileNotFoundError:
            self.get_logger().warning('Color config not found, using defaults')
            return DEFAULT_RANGES

        ranges = data.get('colors', {})
        if not ranges:
            self.get_logger().warning('Color config empty, using defaults')
            return DEFAULT_RANGES

        return ranges

    def image_callback(self, msg: Image) -> None:
        self.frame_count += 1
        process_every_n = (
            self.get_parameter('process_every_n').get_parameter_value().integer_value
        )
        if process_every_n > 1 and (self.frame_count % process_every_n) != 0:
            return

        try:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        best = self._find_best_contour(hsv_image)

        detected_msg = DetectedCargo()
        if best is None:
            detected_msg.detected = False
            self.publisher.publish(detected_msg)
            self._debug_view(bgr_image, None, None, None)
            return

        color_name, contour, bbox, best_mask = best
        x, y, w, h = bbox
        center_x = x + w / 2.0
        center_y = y + h / 2.0
        img_center_x = bgr_image.shape[1] / 2.0

        angle_per_pixel = (
            self.get_parameter('angle_per_pixel').get_parameter_value().double_value
        )
        angle = (center_x - img_center_x) * angle_per_pixel

        object_width_m = (
            self.get_parameter('object_width_m').get_parameter_value().double_value
        )
        focal_length_px = (
            self.get_parameter('focal_length_px').get_parameter_value().double_value
        )
        distance = 0.0
        if w > 0:
            distance = (object_width_m * focal_length_px) / float(w)

        detected_msg.color = color_name
        detected_msg.distance = float(distance)
        detected_msg.angle = float(angle)
        detected_msg.width = float(w)
        detected_msg.detected = True

        self.publisher.publish(detected_msg)

        info = {
            'color': color_name,
            'distance': distance,
            'angle': angle,
            'center': (center_x, center_y),
        }
        self._debug_view(bgr_image, bbox, info, best_mask)

    def _find_best_contour(
        self, hsv_image: np.ndarray
    ) -> Tuple[str, np.ndarray, Tuple[int, int, int, int], np.ndarray] | None:
        area_min = self.get_parameter('area_min').get_parameter_value().double_value
        kernel = np.ones((5, 5), np.uint8)

        best_color = None
        best_contour = None
        best_area = 0.0
        best_mask = None

        for color_name, ranges in self.ranges.items():
            mask = None
            for hsv_range in ranges:
                lower = np.array(
                    [
                        hsv_range['h_min'],
                        hsv_range['s_min'],
                        hsv_range['v_min'],
                    ],
                    dtype=np.uint8,
                )
                upper = np.array(
                    [
                        hsv_range['h_max'],
                        hsv_range['s_max'],
                        hsv_range['v_max'],
                    ],
                    dtype=np.uint8,
                )
                current = cv2.inRange(hsv_image, lower, upper)
                mask = current if mask is None else cv2.bitwise_or(mask, current)

            if mask is None:
                continue

            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < area_min:
                    continue
                if area > best_area:
                    best_area = area
                    best_color = color_name
                    best_contour = contour
                    best_mask = mask

        if best_contour is None or best_color is None or best_mask is None:
            return None

        x, y, w, h = cv2.boundingRect(best_contour)
        return best_color, best_contour, (x, y, w, h), best_mask

    def _debug_view(self, bgr_image: np.ndarray, bbox, info, mask) -> None:
        debug_view = self.get_parameter('debug_view').get_parameter_value().bool_value
        if not debug_view:
            return

        display = bgr_image.copy()
        if bbox is not None:
            x, y, w, h = bbox
            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
        if info is not None:
            label = (
                f"{info['color']} d={info['distance']:.2f}m"
                f" a={info['angle']:.2f}"
            )
            cv2.putText(
                display, label, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2
            )

        cv2.imshow('color_detector', display)
        if mask is not None:
            cv2.imshow('color_mask', mask)
        cv2.waitKey(1)


def main() -> None:
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
