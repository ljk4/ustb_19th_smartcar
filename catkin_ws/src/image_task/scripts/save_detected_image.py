#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import cv2
from datetime import datetime
from sensor_msgs.msg import Image
from detection_msgs.msg import BoundingBoxes, BoundingBox
from cv_bridge import CvBridge, CvBridgeError
import threading
import sys
import select

class SaveDetectedImageNode:
    def __init__(self):
        rospy.init_node('save_detected_image_node', anonymous=True)

        self.bridge = CvBridge()
        self.latest_image = None          # 最新图像
        self.latest_boxes = BoundingBoxes()  # 最新检测结果
        self.save_count = 0  # 保存计数器

        # 订阅话题
        rospy.Subscriber('/yolov5/image_out', Image, self.img_cb, queue_size=1)
        rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.detect_cb, queue_size=1)

        # 保存目录
        self.save_dir = os.path.join(os.path.dirname(__file__), '..', 'screenshots')
        os.makedirs(self.save_dir, exist_ok=True)

        # 启动键盘监听线程
        self.kill_flag = False
        threading.Thread(target=self.keyboard_thread, daemon=True).start()

    def img_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def detect_cb(self, msg):
        self.latest_boxes = msg

    def keyboard_thread(self):
        """监听键盘输入"""
        print("提示: 输入 's' 并按回车保存图像，输入 'q' 并按回车退出")
        
        while not self.kill_flag and not rospy.is_shutdown():
            # 使用 select 检查是否有输入可用
            if select.select([sys.stdin], [], [], 0.1)[0]:
                line = sys.stdin.readline().strip().lower()
                if line == 's':
                    self.save_once()
                elif line == 'q':
                    print("正在退出...")
                    rospy.signal_shutdown('User quit')
                    break

    def save_once(self):
        if self.latest_image is None:
            print("⚠️  尚未收到图像，无法截图！")
            return

        self.save_count += 1
        
        # 文件名
        filename = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".png"
        full_path = os.path.join(self.save_dir, filename)

        # 保存图像
        cv2.imwrite(full_path, self.latest_image)
        
        # 解析物品
        boxes = self.latest_boxes.bounding_boxes  # list[BoundingBox]
        
        if not boxes:
            print("   检测结果: 未检测到物品")
        else:
            # 按中心 x 排序（从左到右）
            sorted_boxes = sorted(boxes, key=lambda b: (b.xmin + b.xmax) / 2)
            class_names = [b.Class for b in sorted_boxes]
            
            print(f"   检测结果: {len(class_names)} 个物品")
            print(f"   从左到右: {' → '.join(class_names)}")

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = SaveDetectedImageNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
