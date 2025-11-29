#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主控制节点
实现基于状态机的任务流程控制，支持PID和TEB双模式导航
"""
import rospy
import math
from std_srvs.srv import SetBool, SetBoolResponse

# 导入模块化组件
from state_machine import StateMachine, MissionState
from navigation_controller import NavigationController
from qr_controller import QRController
from voice_controller import VoiceController
from waypoint_manager import WaypointManager

class UCarCommander:
    """主类"""
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('ucar_commander', anonymous=False)
        rospy.loginfo("=" * 60)
        rospy.loginfo("系统初始化中...")
        rospy.loginfo("=" * 60)
        
        # 初始化各个模块
        rospy.loginfo("初始化状态机...")
        self.state_machine = StateMachine()
        
        rospy.loginfo("初始化导航控制器...")
        self.nav_controller = NavigationController()
        
        rospy.loginfo("初始化QR识别控制器...")
        self.qr_controller = QRController()
        
        rospy.loginfo("初始化语音控制器...")
        self.voice_controller = VoiceController()
        
        rospy.loginfo("初始化路径点管理器...")
        self.waypoint_manager = WaypointManager()
        
        # 计时器服务客户端
        rospy.loginfo("连接计时器服务...")
        self.timer_client = rospy.ServiceProxy('/start_stop_service', SetBool)
        
        # 任务控制标志
        self.mission_started = False
        self.mission_completed = False
        
        # 创建启动服务
        self.start_service = rospy.Service('/ucar_commander/start_mission', 
                                          SetBool, 
                                          self.handle_start_mission)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("系统初始化完成！")
        rospy.loginfo("等待启动命令: rosservice call /ucar_commander/start_mission true")
        rospy.loginfo("=" * 60)
    
    def handle_start_mission(self, req):
        """处理任务启动请求"""
        if req.data and not self.mission_started:
            rospy.loginfo("收到任务启动请求")
            
            # 启动计时器
            try:
                timer_resp = self.timer_client(True)
                if timer_resp.success:
                    rospy.loginfo("计时器启动成功")
                    self.mission_started = True
                    self.state_machine.set_state(MissionState.NAV_TO_INTERSECTION_1)
                    return SetBoolResponse(success=True, message="任务已启动")
                else:
                    rospy.logerr("计时器启动失败")
                    return SetBoolResponse(success=False, message="计时器启动失败")
            except rospy.ServiceException as e:
                rospy.logerr(f"计时器服务调用失败: {e}")
                return SetBoolResponse(success=False, message=f"计时器服务错误: {e}")
        else:
            msg = "任务已经在运行" if self.mission_started else "请求数据无效"
            return SetBoolResponse(success=False, message=msg)
    
    def search_and_scan(self, scan_func, is_valid_func):
        """
        执行搜寻并识别
        Args:
            scan_func: 识别函数
            is_valid_func: 判断识别结果是否有效的函数
        Returns:
            识别结果
        """
        # 1. 原地识别
        res = scan_func()
        if is_valid_func(res):
            return res
            
        rospy.loginfo("原地识别失败，开始搜寻模式...")
        
        # 搜寻动作序列 (y_vel, ang_vel, duration)
        # 左右平移 0.5m 范围内 -> +/- 0.25m
        # 旋转 0.5 rad 范围内 -> +/- 0.25 rad
        
        actions = [
            # (vy, wz, time)
            (0.0, 0.3, 0.8),   # 左转 ~0.24 rad
            (0.0, -0.3, 1.6),  # 右转 ~0.48 rad (从左侧到右侧)
            (0.0, 0.3, 0.8),   # 回正
            (0.2, 0.0, 1.0),   # 左移 ~0.2 m
            (-0.2, 0.0, 2.0),  # 右移 ~0.4 m (从左侧到右侧)
            (0.2, 0.0, 1.0),   # 回正
        ]
        
        for vy, wz, duration in actions:
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < duration:
                self.nav_controller.publish_velocity(0, vy, wz)
                res = scan_func()
                if is_valid_func(res):
                    self.nav_controller.stop_robot()
                    rospy.loginfo("搜寻模式下识别成功！")
                    return res
                rospy.sleep(0.1)
            
            self.nav_controller.stop_robot()
            rospy.sleep(0.2)
            res = scan_func()
            if is_valid_func(res):
                rospy.loginfo("搜寻模式下识别成功！")
                return res
                
        rospy.logwarn("搜寻模式结束，仍未识别到目标")
        return res

    def navigate_to_waypoint(self, waypoint_name):
        """
        导航到指定路径点（带容错机制）
        
        Args:
            waypoint_name: 路径点名称
            
        Returns:
            bool: 导航是否成功
        """
        waypoint = self.waypoint_manager.get_waypoint(waypoint_name)
        
        if waypoint is None:
            rospy.logerr(f"路径点不存在: {waypoint_name}")
            return False
        
        rospy.loginfo(f"导航到路径点: {waypoint}")
        
        # 根据路径点的use_pid标志选择导航模式
        if waypoint.use_pid:
            # 使用PID控制（直线路段），传递自定义最大速度
            success = self.nav_controller.navigate_with_pid(
                waypoint.x, waypoint.y, waypoint.yaw, timeout=60.0, max_speed=waypoint.max_speed,
                position_tolerance=waypoint.pid_position_tolerance
            )
            
            # 容错机制：如果PID卡住（返回None），执行震动脱困后重试
            if success is None:
                rospy.logwarn("PID控制卡住，执行震动脱困")
                # 执行震动脱困
                self.nav_controller.vibration_escape(duration=2.0, amplitude=0.3, frequency=2.0)
                # 重试PID控制
                rospy.loginfo("震动脱困完成，重试PID导航")
                success = self.nav_controller.navigate_with_pid(
                    waypoint.x, waypoint.y, waypoint.yaw, timeout=60.0, max_speed=waypoint.max_speed,
                    position_tolerance=waypoint.pid_position_tolerance+0.02  # 增加容差
                )
                if success:
                    rospy.loginfo("震动脱困后PID导航成功")
                else:
                    rospy.logerr("震动脱困后PID导航仍然失败")
        else:
            # 使用TEB planner（障碍物路段）
            success = self.nav_controller.navigate_with_teb(
                waypoint.x, waypoint.y, waypoint.yaw, timeout=60.0
            )
        
        if success:
            rospy.loginfo(f"成功到达: {waypoint_name}")
        else:
            rospy.logwarn(f"导航失败: {waypoint_name}")
        
        return success
    
    def execute_state(self):
        """执行当前状态的任务"""
        state = self.state_machine.get_state()
        
        # 等待开始
        if state == MissionState.IDLE:
            rospy.loginfo_throttle(10, "等待任务启动...")
            return True
        
        # 导航到第一个路口（直线）
        elif state == MissionState.NAV_TO_INTERSECTION_1:
            if self.navigate_to_waypoint('intersection_1'):
                self.state_machine.set_state(MissionState.NAV_TO_PICKUP)
                return True
            return False
        
        # 导航到取货点（直线）
        elif state == MissionState.NAV_TO_PICKUP:
            if self.navigate_to_waypoint('pickup'):
                self.state_machine.set_state(MissionState.QR_SCAN_CARGO)
                return True
            return False
        
        # 扫描货物QR码
        elif state == MissionState.QR_SCAN_CARGO:
            # 使用搜寻模式识别
            cargo_id, cargo_name = self.search_and_scan(
                self.qr_controller.scan_cargo_qr,
                lambda res: res[0] > 0
            )
            
            if cargo_id > 0:
                self.state_machine.cargo_id = cargo_id
                self.state_machine.cargo_name = cargo_name
                self.state_machine.set_state(MissionState.VOICE_CARGO_PICKED)
                return True
            else:
                rospy.logwarn("货物QR码识别失败，重试中...")
                rospy.sleep(1.0)
                return False
        
        # 播报已取货
        elif state == MissionState.VOICE_CARGO_PICKED:
            if self.voice_controller.play_cargo_picked(self.state_machine.cargo_name):
                self.state_machine.set_state(MissionState.NAV_TO_INTERSECTION_2)
                return True
            else:
                rospy.logwarn("语音播报失败，继续任务")
                self.state_machine.set_state(MissionState.NAV_TO_INTERSECTION_2)
                return True
        
        # 导航到第二个路口（直线）
        elif state == MissionState.NAV_TO_INTERSECTION_2:
            if self.navigate_to_waypoint('intersection_2'):
                self.state_machine.set_state(MissionState.ROTATE_90)
                return True
            return False
        
        # 旋转90度
        elif state == MissionState.ROTATE_90:
            # 获取当前位姿
            _, _, current_yaw = self.nav_controller.get_current_pose()
            if current_yaw is None:
                rospy.logwarn("等待位姿数据...")
                return False
            
            # 计算目标角度（旋转90度 = pi/2弧度）
            target_yaw = current_yaw + math.pi / 2
            
            # 归一化角度到 [-pi, pi]
            target_yaw = self.nav_controller.normalize_angle(target_yaw)
            
            if self.nav_controller.rotate_in_place(target_yaw):
                self.state_machine.set_state(MissionState.NAV_TO_QR_UP)
                return True
            return False
        
        # 导航到上方QR识别区（障碍物段）
        elif state == MissionState.NAV_TO_QR_UP:
            if self.navigate_to_waypoint('qr_station_up'):
                self.state_machine.set_state(MissionState.QR_SCAN_STATION)
                return True
            return False
        
        # 扫描快递站地址QR码
        elif state == MissionState.QR_SCAN_STATION:
            # 使用搜寻模式识别
            Q1 = self.search_and_scan(
                self.qr_controller.scan_station_qr,
                lambda res: res > 0
            )
            
            if Q1 > 0:
                self.state_machine.Q1 = Q1
                self.state_machine.set_state(MissionState.NAV_TO_INTERSECTION_3)
                return True
            else:
                rospy.logwarn("快递站QR码识别失败，重试中...")
                rospy.sleep(1.0)
                return False
        
        # 导航到第三个路口（障碍物段）
        elif state == MissionState.NAV_TO_INTERSECTION_3:
            if self.navigate_to_waypoint('intersection_3'):
                self.state_machine.set_state(MissionState.NAV_TO_QR_DOWN)
                return True
            return False
        
        # 导航到下方QR识别区（直线）
        elif state == MissionState.NAV_TO_QR_DOWN:
            if self.navigate_to_waypoint('qr_station_down'):
                self.state_machine.set_state(MissionState.QR_SCAN_LOCKER)
                return True
            return False
        
        # 扫描快递柜号码QR码
        elif state == MissionState.QR_SCAN_LOCKER:
            # 使用搜寻模式识别
            Q2 = self.search_and_scan(
                self.qr_controller.scan_locker_qr,
                lambda res: res > 0
            )
            
            if Q2 > 0:
                self.state_machine.Q2 = Q2
                self.state_machine.set_state(MissionState.CALCULATE_TARGET)
                return True
            else:
                rospy.logwarn("快递柜QR码识别失败，重试中...")
                rospy.sleep(1.0)
                return False
        
        # 计算目标快递柜
        elif state == MissionState.CALCULATE_TARGET:
            self.state_machine.calculate_target_cabinet()
            self.state_machine.set_state(MissionState.NAV_TO_CABINET)
            return True
        
        # # 脱离高摩擦力平面
        # elif state == MissionState.ESCAPE_FRICTION:
        #     rospy.loginfo("执行脱困动作以离开高摩擦力平面")
        #     # 施加大的y方向速度和旋转速度持续3秒
        #     if self.nav_controller.escape_high_friction_surface(
        #         duration=3.0,    # 持续3秒
        #         vel_y=0.5,       # y方向速度 0.5 m/s
        #         angular_z=0.8    # 旋转速度 0.8 rad/s
        #     ):
        #         rospy.loginfo("脱困成功，继续导航")
        #         self.state_machine.set_state(MissionState.NAV_TO_CABINET)
        #         return True
        #     else:
        #         rospy.logwarn("脱困动作执行失败")
        #         return False
        
        # # 导航到第四个路口（直线）
        # elif state == MissionState.NAV_TO_INTERSECTION_4:
        #     if self.navigate_to_waypoint('intersection_4'):
        #         self.state_machine.set_state(MissionState.NAV_TO_CABINET)
        #         return True
        #     return False
        
        # 导航到目标快递柜（直线）
        elif state == MissionState.NAV_TO_CABINET:
            waypoint = self.waypoint_manager.get_cabinet_waypoint(
                self.state_machine.target_cabinet
            )
            if waypoint and self.navigate_to_waypoint(waypoint.name):
                self.state_machine.set_state(MissionState.VOICE_DELIVERED)
                return True
            return False
        
        # 播报货物已送达
        elif state == MissionState.VOICE_DELIVERED:
            if self.voice_controller.play_delivered(
                self.state_machine.target_cabinet_name
            ):
                self.state_machine.set_state(MissionState.NAV_TO_END)
                return True
            else:
                rospy.logwarn("语音播报失败，继续任务")
                self.state_machine.set_state(MissionState.NAV_TO_END)
                return True
        
        # 导航到终点（直线）
        elif state == MissionState.NAV_TO_END:
            if self.navigate_to_waypoint('end'):
                # 停止计时器
                try:
                    timer_resp = self.timer_client(False)
                    if timer_resp.success:
                        rospy.loginfo("计时器停止成功")
                except rospy.ServiceException as e:
                    rospy.logerr(f"计时器服务调用失败: {e}")
                
                self.state_machine.set_state(MissionState.VOICE_COMPLETED)
                return True
            return False
        
        # 播报任务完成
        elif state == MissionState.VOICE_COMPLETED:
            if self.voice_controller.play_mission_completed():
                self.state_machine.set_state(MissionState.FINISHED)
                self.mission_completed = True
                rospy.loginfo("=" * 60)
                rospy.loginfo("任务全部完成！")
                rospy.loginfo("=" * 60)
                return True
            else:
                rospy.logwarn("语音播报失败")
                self.state_machine.set_state(MissionState.FINISHED)
                self.mission_completed = True
                return True
        
        # 任务完成状态
        elif state == MissionState.FINISHED:
            rospy.loginfo_throttle(10, "任务已完成")
            return True
        
        return False
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            try:
                # 执行当前状态的任务
                self.execute_state()
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"主循环错误: {e}")
                import traceback
                traceback.print_exc()
                rospy.sleep(1.0)

def main():
    """主函数"""
    try:
        commander = UCarCommander()
        commander.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr(f"程序运行错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
