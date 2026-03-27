#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
状态机模块 - 管理机器人任务状态转换
"""
import rospy
from enum import Enum

class MissionState(Enum):
    """任务状态枚举"""
    IDLE = 0                    # 等待开始
    NAV_TO_INTERSECTION_1 = 1   # 导航到第一个路口
    QR_SCAN_PICKUP = 3          # 扫描任务领取区二维码
    NAV_TO_PICKUP = 4           # 导航到任务领取区
    VOICE_PICKUP = 5            # 播报任务领取
    LEAVE_PICKUP_AREA = 6       # 离开取货区
    NAV_AND_SCAN_INFO = 8       # 导航并扫描信息区QR
    CALCULATE_ROUTE = 10        # 计算路线
    
    NAV_TO_SHELF_1 = 11         # 导航到第一个货架
    DETECT_OBJECT_1 = 12        # 识别第一个货架物品
    VOICE_OBJECT_1 = 13         # 播报第一个货架物品
    
    NAV_TO_SHELF_2 = 14         # 导航到第二个货架
    DETECT_OBJECT_2 = 15        # 识别第二个货架物品
    VOICE_OBJECT_2 = 16         # 播报第二个货架物品
    
    NAV_TO_INTERSECTION_4 = 17  # 导航到第四个路口
    ROTATE_LEFT_90 = 18         # 左转90度
    NAV_TO_PARKING = 19         # 导航到停车区
    VOICE_COMPLETED = 20        # 播报任务完成
    FINISHED = 21               # 任务完成

class StateMachine:
    """状态机管理类"""
    def __init__(self):
        self.current_state = MissionState.IDLE
        self.previous_state = None
        
        # 任务数据
        self.pickup_items = []      # 需要采购的物品类别 (e.g., ['fruit', 'vegetable'])
        self.shelf_info = {}        # 货架信息 (e.g., {'A': 'fruit', 'B': 'vegetable', 'C': 'dessert'})
        self.target_shelves = []    # 目标货架列表 (e.g., ['A', 'B'])
        self.detected_objects = []  # 已识别到的物品名称
        
        # 重试计数
        self.retry_count = {}
        self.max_retries = 3
        
    def set_state(self, new_state):
        """设置新状态"""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            rospy.loginfo(f"状态转换: {self.previous_state.name} -> {self.current_state.name}")
            
    def get_state(self):
        """获取当前状态"""
        return self.current_state
    
    def reset_retry(self, state):
        """重置重试计数"""
        if state in self.retry_count:
            self.retry_count[state] = 0
            
    def increment_retry(self, state):
        """增加重试计数"""
        if state not in self.retry_count:
            self.retry_count[state] = 0
        self.retry_count[state] += 1
        return self.retry_count[state]
    
    def should_retry(self, state):
        """判断是否应该重试"""
        count = self.retry_count.get(state, 0)
        return count < self.max_retries
    
    def calculate_route(self):
        """根据任务和货架信息计算路线"""
        self.target_shelves = []
        
        # 遍历需要采购的物品类别
        for item_category in self.pickup_items:
            # 在货架信息中查找对应的货架
            for shelf, category in self.shelf_info.items():
                if category == item_category:
                    self.target_shelves.append(shelf)
                    break
        
        rospy.loginfo(f"计算路线: 任务={self.pickup_items}, 货架信息={self.shelf_info}, 目标货架={self.target_shelves}")
        return len(self.target_shelves) > 0
