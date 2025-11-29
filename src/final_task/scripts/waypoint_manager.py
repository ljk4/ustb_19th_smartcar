#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路径点管理模块 - 管理所有路径点及其导航模式
"""
import rospy

class Waypoint:
    """路径点类"""
    def __init__(self, name, x, y, yaw, use_pid=True, max_speed=None, position_tolerance=None, angle_tolerance=None):
        """
        Args:
            name: 路径点名称
            x: X坐标
            y: Y坐标
            yaw: 偏航角（弧度）
            use_pid: 是否使用PID控制（True=直线路段，False=障碍物路段使用TEB）
            max_speed: 自定义最大速度（m/s），None表示使用默认值
            position_tolerance: 自定义位置容差，None表示使用默认值
            angle_tolerance: 自定义角度容差，None表示使用默认值
        """
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw
        self.use_pid = use_pid
        self.max_speed = max_speed
        self.position_tolerance = position_tolerance
        self.angle_tolerance = angle_tolerance
    
    def __repr__(self):
        mode = "PID" if self.use_pid else "TEB"
        return f"Waypoint({self.name}, x={float(self.x):.5f}, y={float(self.y):.5f}, yaw={float(self.yaw):.5f}, mode={mode}, pos_tol={self.position_tolerance}, ang_tol={self.angle_tolerance})"

class WaypointManager:
    """路径点管理器"""
    
    def __init__(self):
        # 路径点字典
        self.waypoints = {}
        
        # 从参数服务器加载默认路径点
        self.load_default_waypoints()
        
    def _parse_waypoint_param(self, param_name, default_value):
        """
        解析路径点参数，支持字符串和列表格式
        支持格式: [x, y, yaw] 或 [x, y, yaw, pos_tol, ang_tol]
        """
        param = rospy.get_param(param_name, default_value)
        
        # 如果是字符串，尝试解析
        if isinstance(param, str):
            import ast
            try:
                param = ast.literal_eval(param)
            except:
                rospy.logwarn(f"无法解析参数 {param_name}: {param}，使用默认值")
                param = default_value
        
        # 确保返回列表格式
        if isinstance(param, list):
            if len(param) >= 5:
                return [float(param[0]), float(param[1]), float(param[2]), float(param[3]), float(param[4])]
            elif len(param) >= 3:
                return [float(param[0]), float(param[1]), float(param[2]), None, None]
        
        return default_value
    
    def load_default_waypoints(self):
        """从参数服务器加载默认路径点"""
        try:
            # 加载路径点参数
            intersection_1_data = self._parse_waypoint_param("~intersection_1", [-2.970, 0.054, 0.027])
            pickup_data = self._parse_waypoint_param("~pickup", [-3.148, -0.037, -3.112])
            info_area_data = self._parse_waypoint_param("~info_area", [-3.058, 2.092, 1.448])
            # 备用位置暂时使用原位置占位
            info_area_backup_1_data = self._parse_waypoint_param("~info_area_backup_1", [-3.058, 2.092, 1.448])
            info_area_backup_2_data = self._parse_waypoint_param("~info_area_backup_2", [-3.058, 2.092, 1.448])
            
            intersection_2_data = self._parse_waypoint_param("~intersection_2", [-2.218, 2.358, 1.611])
            intersection_3_data = self._parse_waypoint_param("~intersection_3", [-1.218, 2.387, 1.604])
            
            intersection_ac_left_data = self._parse_waypoint_param("~intersection_ac_left", [-1.202, 1.661, 1.606])
            shelf_a_data = self._parse_waypoint_param("~shelf_a", [-1.899, 1.664, 3.137])
            shelf_c_data = self._parse_waypoint_param("~shelf_c", [-0.467, 1.670, 3.137])
            
            intersection_bd_left_data = self._parse_waypoint_param("~intersection_bd_left", [-1.170, 0.798, 1.580])
            shelf_b_data = self._parse_waypoint_param("~shelf_b", [-1.904, 0.801, 3.141])
            shelf_d_data = self._parse_waypoint_param("~shelf_d", [-0.517, 0.802, 3.137])
            
            intersection_ac_right_data = self._parse_waypoint_param("~intersection_ac_right", [0.171, 1.668, 1.549])
            intersection_bd_right_data = self._parse_waypoint_param("~intersection_bd_right", [0.156, 0.747, 1.554])
            
            intersection_4_data = self._parse_waypoint_param("~intersection_4", [0.171, 2.924, 1.564])
            parking_data = self._parse_waypoint_param("~parking", [-3.230, 3.363, -3.141])
            
            # 创建路径点对象
            # 注意：_parse_waypoint_param 现在返回 5 个元素 [x, y, yaw, pos_tol, ang_tol]
            # 如果只有3个元素，后两个为None
            
            def create_wp(name, data, use_pid):
                # 兼容旧代码，如果data只有3个元素，补全为5个
                if len(data) < 5:
                    data = list(data) + [None, None]
                return Waypoint(name, data[0], data[1], data[2], use_pid, 
                               position_tolerance=data[3], angle_tolerance=data[4])

            self.waypoints = {
                'intersection_1': create_wp('intersection_1', intersection_1_data, True),
                'pickup': create_wp('pickup', pickup_data, True),
                'info_area': create_wp('info_area', info_area_data, False), # TEB
                'info_area_backup_1': create_wp('info_area_backup_1', info_area_backup_1_data, False),
                'info_area_backup_2': create_wp('info_area_backup_2', info_area_backup_2_data, False),
                'intersection_2': create_wp('intersection_2', intersection_2_data, False), # TEB
                'intersection_3': create_wp('intersection_3', intersection_3_data, True),
                
                'intersection_ac_left': create_wp('intersection_ac_left', intersection_ac_left_data, True),
                'shelf_a': create_wp('shelf_a', shelf_a_data, True),
                'shelf_c': create_wp('shelf_c', shelf_c_data, True),
                
                'intersection_bd_left': create_wp('intersection_bd_left', intersection_bd_left_data, True),
                'shelf_b': create_wp('shelf_b', shelf_b_data, True),
                'shelf_d': create_wp('shelf_d', shelf_d_data, True),
                
                'intersection_ac_right': create_wp('intersection_ac_right', intersection_ac_right_data, True),
                'intersection_bd_right': create_wp('intersection_bd_right', intersection_bd_right_data, True),
                
                'intersection_4': create_wp('intersection_4', intersection_4_data, True),
                'parking': create_wp('parking', parking_data, False) # TEB
            }
            
            rospy.loginfo("路径点加载成功:")
            for name, wp in self.waypoints.items():
                rospy.loginfo(f"  {wp}")
                
        except Exception as e:
            rospy.logerr(f"路径点加载失败: {e}")
            rospy.logwarn("使用硬编码的默认路径点")
            self._load_hardcoded_waypoints()
    
    def _load_hardcoded_waypoints(self):
        """加载硬编码的默认路径点（备用）"""
        self.waypoints = {
            'intersection_1': Waypoint('intersection_1', -2.970, 0.054, 0.027, True),
            'pickup': Waypoint('pickup', -3.148, -0.037, -3.112, True),
            'info_area': Waypoint('info_area', -3.058, 2.092, 1.448, False),
            'info_area_backup_1': Waypoint('info_area_backup_1', -3.058, 2.092, 1.448, False),
            'info_area_backup_2': Waypoint('info_area_backup_2', -3.058, 2.092, 1.448, False),
            'intersection_2': Waypoint('intersection_2', -2.218, 2.358, 1.611, False),
            'intersection_3': Waypoint('intersection_3', -1.218, 2.387, 1.604, True),
            
            'intersection_ac_left': Waypoint('intersection_ac_left', -1.202, 1.661, 1.606, True),
            'shelf_a': Waypoint('shelf_a', -1.899, 1.664, 3.137, True),
            'shelf_c': Waypoint('shelf_c', -0.467, 1.670, 3.137, True),
            
            'intersection_bd_left': Waypoint('intersection_bd_left', -1.170, 0.798, 1.580, True),
            'shelf_b': Waypoint('shelf_b', -1.904, 0.801, 3.141, True),
            'shelf_d': Waypoint('shelf_d', -0.517, 0.802, 3.137, True),
            
            'intersection_ac_right': Waypoint('intersection_ac_right', 0.171, 1.668, 1.549, True),
            'intersection_bd_right': Waypoint('intersection_bd_right', 0.156, 0.747, 1.554, True),
            
            'intersection_4': Waypoint('intersection_4', 0.171, 2.924, 1.564, True),
            'parking': Waypoint('parking', -3.230, 3.363, -3.141, False)
        }
    
    def get_waypoint(self, name):
        """
        获取路径点
        
        Args:
            name: 路径点名称
            
        Returns:
            Waypoint: 路径点对象，不存在返回None
        """
        return self.waypoints.get(name)
    
    def add_waypoint(self, name, x, y, yaw, use_pid=True, position_tolerance=None, angle_tolerance=None):
        """
        添加新路径点
        
        Args:
            name: 路径点名称
            x: X坐标
            y: Y坐标
            yaw: 偏航角
            use_pid: 是否使用PID控制
            position_tolerance: 位置容差
            angle_tolerance: 角度容差
        """
        self.waypoints[name] = Waypoint(name, x, y, yaw, use_pid, 
                                      position_tolerance=position_tolerance, 
                                      angle_tolerance=angle_tolerance)
        rospy.loginfo(f"添加路径点: {self.waypoints[name]}")
    
    def update_waypoint(self, name, x=None, y=None, yaw=None, use_pid=None, position_tolerance=None, angle_tolerance=None):
        """
        更新路径点
        
        Args:
            name: 路径点名称
            x: X坐标（可选）
            y: Y坐标（可选）
            yaw: 偏航角（可选）
            use_pid: 是否使用PID控制（可选）
            position_tolerance: 位置容差（可选）
            angle_tolerance: 角度容差（可选）
        """
        if name not in self.waypoints:
            rospy.logerr(f"路径点不存在: {name}")
            return False
        
        wp = self.waypoints[name]
        if x is not None:
            wp.x = x
        if y is not None:
            wp.y = y
        if yaw is not None:
            wp.yaw = yaw
        if use_pid is not None:
            wp.use_pid = use_pid
        if position_tolerance is not None:
            wp.position_tolerance = position_tolerance
        if angle_tolerance is not None:
            wp.angle_tolerance = angle_tolerance
        
        rospy.loginfo(f"更新路径点: {wp}")
        return True
    
    def list_waypoints(self):
        """列出所有路径点"""
        rospy.loginfo("所有路径点:")
        for name, wp in self.waypoints.items():
            rospy.loginfo(f"  {wp}")
