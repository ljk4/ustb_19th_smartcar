#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
目标检测模块
订阅YOLOv5检测结果，提供查询接口
"""
import rospy
import threading
import math
from detection_msgs.msg import BoundingBoxes
from navigation_controller import NavigationController

class ObjectDetector:
    def __init__(self, navigation_controller):
        self.latest_detections = []
        self.lock = threading.Lock()
        self.nav_controller = navigation_controller
        
        # 订阅检测结果
        self.sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.callback, queue_size=1)
        
        # 目标类别列表
        self.target_classes = [
            'apple', 'banana', 'watermelon', 'potato', 'tomato', 
            'chili', 'cake', 'milk', 'cola'
        ]
        
    def callback(self, msg):
        with self.lock:
            self.latest_detections = msg.bounding_boxes
            
    def get_detected_object(self, retry_on_fail=True):
        """
        获取当前检测到的目标物品
        如果失败，则后退重试一次
        返回: 物品名称(str) 或 None
        """
        # 第一次尝试
        result = self._get_single_detection()
        if result is not None:
            return result
        
        if not retry_on_fail:
            return None

        # 如果失败，执行后退重试逻辑
        rospy.logwarn("初次识别失败，后退10cm重试...")
        
        # 1. 获取当前位置
        original_x, original_y, original_yaw = self.nav_controller.get_current_pose()
        if original_x is None:
            rospy.logerr("无法获取当前位置，无法执行后退重试。")
            return None
            
        # 2. 计算后退目标位置
        backup_distance = 0.10  # 后退10cm
        target_x = original_x - backup_distance * math.cos(original_yaw)
        target_y = original_y - backup_distance * math.sin(original_yaw)
        
        # 3. 执行后退
        rospy.loginfo(f"从 ({original_x:.2f}, {original_y:.2f}) 后退到 ({target_x:.2f}, {target_y:.2f})")
        self.nav_controller.navigate_with_pid(target_x, target_y, original_yaw, timeout=20.0, max_speed=0.3)
        
        # 4. 短暂延迟后再次识别
        rospy.sleep(1.0)
        retry_result = self._get_single_detection()
        rospy.loginfo(f"重试识别结果: {retry_result}")
        
        # 5. 回到原位
        rospy.loginfo(f"返回原位 ({original_x:.2f}, {original_y:.2f})")
        self.nav_controller.navigate_with_pid(original_x, original_y, original_yaw, timeout=20.0, max_speed=0.3)
        
        # 6. 返回重试结果
        return retry_result

    def _get_single_detection(self):
        """
        执行单次检测逻辑
        """
        with self.lock:
            # 清除旧的检测结果，等待新的
            self.latest_detections = []
        
        # 等待1秒以接收新的检测数据
        rospy.sleep(1.0)

        with self.lock:
            if not self.latest_detections:
                return None
            
            # 优先返回置信度最高的目标
            best_detection = None
            max_prob = -1.0
            
            for box in self.latest_detections:
                if box.Class in self.target_classes and box.probability > max_prob:
                    max_prob = box.probability
                    best_detection = box.Class
            
            return best_detection

    def get_all_detected_objects(self):
        """
        获取所有检测到的目标物品
        返回: 物品名称列表
        """
        with self.lock:
            if not self.latest_detections:
                return []
            
            detected = []
            for box in self.latest_detections:
                if box.Class in self.target_classes:
                    detected.append(box.Class)
            return list(set(detected)) # 去重
