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
    NAV_TO_PICKUP = 2           # 导航到取货点
    QR_SCAN_CARGO = 3           # 扫描货物QR码
    VOICE_CARGO_PICKED = 4      # 播报已取货
    NAV_TO_INTERSECTION_2 = 5   # 导航到第二个路口
    ROTATE_90 = 6               # 旋转90度
    NAV_TO_QR_UP = 7            # 导航到上方QR识别区（障碍物段）
    QR_SCAN_STATION = 8         # 扫描快递站地址QR码
    NAV_TO_INTERSECTION_3 = 9   # 导航到第三个路口（障碍物段）
    NAV_TO_QR_DOWN = 10         # 导航到下方QR识别区
    QR_SCAN_LOCKER = 11         # 扫描快递柜号码QR码
    CALCULATE_TARGET = 12       # 计算目标柜
    ESCAPE_FRICTION = 13        # 脱离高摩擦力平面
    NAV_TO_INTERSECTION_4 = 14  # 导航到第四个路口
    NAV_TO_CABINET = 15         # 导航到目标快递柜
    VOICE_DELIVERED = 16        # 播报已送达
    NAV_TO_END = 17             # 导航到终点
    VOICE_COMPLETED = 18        # 播报任务完成
    FINISHED = 19               # 任务完成

class StateMachine:
    """状态机管理类"""
    def __init__(self):
        self.current_state = MissionState.IDLE
        self.previous_state = None
        
        # 任务数据
        self.cargo_id = 0           # 货物ID (1-3)
        self.cargo_name = ""        # 货物名称
        self.Q1 = 0                 # 上方QR码值
        self.Q2 = 0                 # 下方QR码值
        self.target_cabinet = 0     # 目标柜号 (1-3)
        self.target_cabinet_name = ""  # 目标柜名 (A/B/C)
        
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
    
    def calculate_target_cabinet(self):
        """计算目标快递柜"""
        self.target_cabinet = (self.Q1 + self.Q2) % 3 + 1
        cabinet_names = {1: "A", 2: "B", 3: "C"}
        self.target_cabinet_name = cabinet_names[self.target_cabinet]
        rospy.loginfo(f"计算结果: Q1={self.Q1}, Q2={self.Q2}, 目标柜={self.target_cabinet_name}")
        
    def get_cargo_name(self, cargo_id):
        """根据货物ID获取货物名称"""
        cargo_names = {
            1: "学习用品",
            2: "娱乐用品", 
            3: "生活用品"
        }
        return cargo_names.get(cargo_id, "未知货物")
