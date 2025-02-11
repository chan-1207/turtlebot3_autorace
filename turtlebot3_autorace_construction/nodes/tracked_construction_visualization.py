#!/usr/bin/env python3
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: ChanHyeong Lee

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from turtlebot3_autorace_msgs.msg import Objects
import random


class TrackedObjectVisualizer(Node):
    def __init__(self):
        super().__init__('tracked_object_visualizer')
        self.subscription = self.create_subscription(
            Objects,
            '/tracked_objects',
            self.objects_callback,
            10
        )
        self.subscription
        self.tracked_objects = []
        self.colors = {}
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("Tracked Object Visualizer node started.")
        self.max_visualization_distance = 1.2

    def objects_callback(self, msg):
        self.tracked_objects = msg.objects

    def get_color_for_id(self, obj_id):
        if obj_id not in self.colors:
            self.colors[obj_id] = (
                random.randint(0, 255),
                random.randint(0, 255),
                random.randint(0, 255)
            )
        return self.colors[obj_id]

    def timer_callback(self):
        img = np.zeros((500, 500, 3), dtype=np.uint8)
        scale = 200.0  # 1m = 100pixel
        center = (img.shape[1] // 2, img.shape[0] // 2)

        # visualization turtlebot
        robot_width_m = 0.12
        robot_height_m = 0.12
        robot_width_px = int(robot_width_m * scale)
        robot_height_px = int(robot_height_m * scale)
        top_left = (center[0] - robot_width_px // 2, center[1] - robot_height_px // 2)
        bottom_right = (center[0] + robot_width_px // 2, center[1] + robot_height_px // 2)
        cv2.rectangle(img, top_left, bottom_right, (255, 0, 0), 2)
        cv2.putText(img, 'Turtlebot', (top_left[0], top_left[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # visualization traked object
        for tracker in self.tracked_objects:
            if (tracker.x**2 + tracker.y**2)**0.5 > self.max_visualization_distance:
                continue

            x = int(center[0] + tracker.x * scale)
            y = int(center[1] - tracker.y * scale)
            color = self.get_color_for_id(tracker.id)
            width_px = int(tracker.width * scale)
            height_px = int(tracker.height * scale)
            top_left_obj = (x - width_px // 2, y - height_px // 2)
            bottom_right_obj = (x + width_px // 2, y + height_px // 2)
            cv2.rectangle(img, top_left_obj, bottom_right_obj, color, 2)
            cv2.putText(img, f'ID:{tracker.id}', (x + 5, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("Tracked Objects", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = TrackedObjectVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
