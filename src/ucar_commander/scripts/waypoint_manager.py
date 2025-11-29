#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路径点管理模块 - 管理所有路径点及其导航模式
"""
import rospy

class Waypoint:
    """路径点类"""
    def __init__(self, name, x, y, yaw, use_pid=True, max_speed=None, pid_position_tolerance=None):
        """
        Args:
            name: 路径点名称
            x: X坐标
            y: Y坐标
            yaw: 偏航角（弧度）
            use_pid: 是否使用PID控制（True=直线路段，False=障碍物路段使用TEB）
            max_speed: 自定义最大速度（m/s），None表示使用默认值
            pid_position_tolerance: 自定义PID位置容差（m），None表示使用默认值
        """
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw
        self.use_pid = use_pid
        self.max_speed = max_speed
        self.pid_position_tolerance = pid_position_tolerance
    
    def __repr__(self):
        mode = "PID" if self.use_pid else "TEB"
        return f"Waypoint({self.name}, x={float(self.x):.5f}, y={float(self.y):.5f}, yaw={float(self.yaw):.5f}, mode={mode}, tol={self.pid_position_tolerance})"

class WaypointManager:
    """路径点管理器"""
    
    def __init__(self):
        # 路径点字典
        self.waypoints = {}
        
        # 从参数服务器加载默认路径点
        self.load_default_waypoints()
        
    def _parse_waypoint_param(self, param_name, default_value):
        """解析路径点参数，支持字符串和列表格式"""
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
        if isinstance(param, list) and len(param) >= 3:
            return [float(param[0]), float(param[1]), float(param[2])]
        else:
            return default_value
    
    def load_default_waypoints(self):
        """从参数服务器加载默认路径点"""
        try:
            # 加载路径点参数（从原有代码的格式 [x, y, yaw]）
            intersection_1_data = self._parse_waypoint_param("~intersection_1", [0.5, -1.0, 0.0])
            pickup_data = self._parse_waypoint_param("~pickup", [1.64, -1.69, -1.32])
            intersection_2_data = self._parse_waypoint_param("~intersection_2", [2.0, -1.5, 0.0])
            qr_station_up_data = self._parse_waypoint_param("~qr_station_up", [3.33866, -1.78492, 0.223])
            intersection_3_data = self._parse_waypoint_param("~intersection_3", [3.0, -0.5, 1.57])
            qr_station_down_data = self._parse_waypoint_param("~qr_station_down", [3.14618, 0.52428, 1.808])
            intersection_4_data = self._parse_waypoint_param("~intersection_4", [2.5, 0.5, 3.14])
            station_A_data = self._parse_waypoint_param("~station_A", [2.55107, 0.60859, 0.25])
            station_B_data = self._parse_waypoint_param("~station_B", [2.14873, 0.51149, 0.25])
            station_C_data = self._parse_waypoint_param("~station_C", [1.63901, 0.38807, 0.25])
            end_data = self._parse_waypoint_param("~end", [0.1, 0.15, 0.25])
            
            # 创建路径点对象
            # 根据任务需求设置导航模式：
            # - 直线路段使用PID (use_pid=True)
            # - 障碍物路段使用TEB (use_pid=False)
            
            self.waypoints = {
                'intersection_1': Waypoint('intersection_1', intersection_1_data[0], intersection_1_data[1], intersection_1_data[2], True, max_speed=0.4, pid_position_tolerance=0.05),
                'pickup': Waypoint('pickup', pickup_data[0], pickup_data[1], pickup_data[2], True, max_speed=0.6),  # 上坡路段，提高速度克服重力
                'intersection_2': Waypoint('intersection_2', intersection_2_data[0], intersection_2_data[1], intersection_2_data[2], True, max_speed=0.4),
                'qr_station_up': Waypoint('qr_station_up', qr_station_up_data[0], qr_station_up_data[1], qr_station_up_data[2], False),  # 障碍物段
                'intersection_3': Waypoint('intersection_3', intersection_3_data[0], intersection_3_data[1], intersection_3_data[2], False),  # 障碍物段
                'qr_station_down': Waypoint('qr_station_down', qr_station_down_data[0], qr_station_down_data[1], qr_station_down_data[2], True, max_speed=0.4,pid_position_tolerance=0.1),
                'intersection_4': Waypoint('intersection_4', intersection_4_data[0], intersection_4_data[1], intersection_4_data[2], False, max_speed=0.4),
                'station_A': Waypoint('station_A', station_A_data[0], station_A_data[1], station_A_data[2], False, max_speed=0.4),
                'station_B': Waypoint('station_B', station_B_data[0], station_B_data[1], station_B_data[2], False, max_speed=0.4),
                'station_C': Waypoint('station_C', station_C_data[0], station_C_data[1], station_C_data[2], False, max_speed=0.4),
                'end': Waypoint('end', end_data[0], end_data[1], end_data[2], True, max_speed=0.6)
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
            'intersection_1': Waypoint('intersection_1', 0.5, -1.0, 0.0, use_pid=True),
            'pickup': Waypoint('pickup', 1.64, -1.69, -1.32, use_pid=True),
            'intersection_2': Waypoint('intersection_2', 2.0, -1.5, 0.0, use_pid=True),
            'qr_station_up': Waypoint('qr_station_up', 3.33866, -1.78492, 0.223, use_pid=False),
            'intersection_3': Waypoint('intersection_3', 3.0, -0.5, 1.57, use_pid=False),
            'qr_station_down': Waypoint('qr_station_down', 3.14618, 0.52428, 1.808, use_pid=True),
            'intersection_4': Waypoint('intersection_4', 2.5, 0.5, 3.14, use_pid=True),
            'station_A': Waypoint('station_A', 2.55107, 0.60859, 0.25, use_pid=True),
            'station_B': Waypoint('station_B', 2.14873, 0.51149, 0.25, use_pid=True),
            'station_C': Waypoint('station_C', 1.63901, 0.38807, 0.25, use_pid=True),
            'end': Waypoint('end', 0.1, 0.15, 0.25, use_pid=True)
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
    
    def get_cabinet_waypoint(self, cabinet_number):
        """
        根据柜号获取快递柜路径点
        
        Args:
            cabinet_number: 柜号 (1-3)
            
        Returns:
            Waypoint: 路径点对象
        """
        cabinet_names = {1: 'station_A', 2: 'station_B', 3: 'station_C'}
        cabinet_name = cabinet_names.get(cabinet_number)
        
        if cabinet_name:
            return self.get_waypoint(cabinet_name)
        else:
            rospy.logerr(f"无效的柜号: {cabinet_number}")
            return None
    
    def add_waypoint(self, name, x, y, yaw, use_pid=True):
        """
        添加新路径点
        
        Args:
            name: 路径点名称
            x: X坐标
            y: Y坐标
            yaw: 偏航角
            use_pid: 是否使用PID控制
        """
        self.waypoints[name] = Waypoint(name, x, y, yaw, use_pid)
        rospy.loginfo(f"添加路径点: {self.waypoints[name]}")
    
    def update_waypoint(self, name, x=None, y=None, yaw=None, use_pid=None):
        """
        更新路径点
        
        Args:
            name: 路径点名称
            x: X坐标（可选）
            y: Y坐标（可选）
            yaw: 偏航角（可选）
            use_pid: 是否使用PID控制（可选）
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
        
        rospy.loginfo(f"更新路径点: {wp}")
        return True
    
    def list_waypoints(self):
        """列出所有路径点"""
        rospy.loginfo("所有路径点:")
        for name, wp in self.waypoints.items():
            rospy.loginfo(f"  {wp}")
