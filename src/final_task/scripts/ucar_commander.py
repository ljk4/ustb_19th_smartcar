#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主控制节点
实现基于状态机的任务流程控制，支持PID和TEB双模式导航
"""
import rospy
import math
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Twist

# 导入模块化组件
from state_machine import StateMachine, MissionState
from navigation_controller import NavigationController
from qr_controller import QRController
from voice_controller import VoiceController
from waypoint_manager import WaypointManager
from object_detector import ObjectDetector

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
        
        rospy.loginfo("初始化目标检测器...")
        self.object_detector = ObjectDetector(self.nav_controller)
        
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
    
    def navigate_to_waypoint(self, waypoint_name):
        """
        导航到指定路径点（带容错机制）
        """
        waypoint = self.waypoint_manager.get_waypoint(waypoint_name)
        
        if waypoint is None:
            rospy.logerr(f"路径点不存在: {waypoint_name}")
            return False
        
        rospy.loginfo(f"导航到路径点: {waypoint}")
        
        if waypoint.use_pid:
            success = self.nav_controller.navigate_with_pid(
                waypoint.x, waypoint.y, waypoint.yaw, timeout=60.0, max_speed=waypoint.max_speed,
                position_tolerance=waypoint.position_tolerance, angle_tolerance=waypoint.angle_tolerance
            )
            if success is None:
                rospy.logwarn("PID控制卡住，执行震动脱困")
                self.nav_controller.vibration_escape(duration=2.0, amplitude=0.3, frequency=2.0)
                success = self.nav_controller.navigate_with_pid(
                    waypoint.x, waypoint.y, waypoint.yaw, timeout=60.0, max_speed=waypoint.max_speed,
                    position_tolerance=waypoint.position_tolerance, angle_tolerance=waypoint.angle_tolerance
                )
        else:
            success = self.nav_controller.navigate_with_teb(
                waypoint.x, waypoint.y, waypoint.yaw, timeout=30.0
            )
        
        if success:
            rospy.loginfo(f"成功到达: {waypoint_name}")
        else:
            rospy.logwarn(f"导航失败: {waypoint_name}")
        
        return success
    
    def rotate_relative(self, angle_rad):
        """相对旋转"""
        _, _, current_yaw = self.nav_controller.get_current_pose()
        if current_yaw is None:
            return False
        target_yaw = self.nav_controller.normalize_angle(current_yaw + angle_rad)
        return self.nav_controller.rotate_in_place(target_yaw)

    def navigate_to_shelf(self, shelf_name):
        """
        导航到指定货架
        从对应的路口左侧点，旋转并导航到货架，然后回正
        """
        rospy.loginfo(f"从路口导航到货架 {shelf_name}")
        
        # 旋转并导航到货架
        if shelf_name == 'A':
            self.rotate_relative(math.pi/2) # 左转90
            if not self.navigate_to_waypoint('shelf_a'): return False
            self.rotate_relative(-math.pi/2) # 右转90回正
            
        elif shelf_name == 'C':
            self.rotate_relative(-math.pi/2) # 右转90
            if not self.navigate_to_waypoint('shelf_c'): return False
            self.rotate_relative(math.pi/2) # 左转90回正
            
        elif shelf_name == 'B':
            self.rotate_relative(math.pi/2) # 左转90
            if not self.navigate_to_waypoint('shelf_b'): return False
            self.rotate_relative(-math.pi/2) # 右转90回正
            
        elif shelf_name == 'D':
            self.rotate_relative(-math.pi/2) # 右转90
            if not self.navigate_to_waypoint('shelf_d'): return False
            self.rotate_relative(math.pi/2) # 左转90回正
            
        return True

    def navigate_from_shelf_to_exit(self, last_shelf):
        """从最后一个货架导航到出口路口"""
        rospy.loginfo(f"从货架 {last_shelf} 前往拐点4")
        if last_shelf in ['A', 'C']:
            # 从A,C前往拐点4:先导航到拐点AC右, 再前往拐点4
            self.rotate_relative(-math.pi/2) # 右转90
            if not self.navigate_to_waypoint('intersection_ac_right'): return False
            self.rotate_relative(math.pi/2) # 左转90回正
            
        elif last_shelf in ['B', 'D']:
            # 从B,D前往拐点4:先导航到拐点BD右, 再前往拐点4
            self.rotate_relative(-math.pi/2) # 右转90
            if not self.navigate_to_waypoint('intersection_bd_right'): return False
            self.rotate_relative(math.pi/2) # 左转90回正
            
        return self.navigate_to_waypoint('intersection_4')

    def scan_with_search(self, scan_func, parse_func):
        """
        带搜寻机制的QR码扫描
        """
        # 1. 第一次尝试
        content = scan_func()
        if content:
            result = parse_func(content)
            if result: return result
        
        rospy.logwarn("QR码识别失败，开始搜寻...")
        
        # 2. 后退一点 (10cm)
        rospy.loginfo("搜寻动作1: 后退10cm")
        cmd = Twist()
        cmd.linear.x = -0.1
        self.nav_controller.cmd_vel_pub.publish(cmd)
        rospy.sleep(1.0)
        self.nav_controller.stop_robot()
        rospy.sleep(0.5)
        
        content = scan_func()
        if content:
            result = parse_func(content)
            if result: return result
            
        # 3. 前进一点 (20cm -> 比原位置前10cm)
        rospy.loginfo("搜寻动作2: 前进20cm")
        cmd = Twist()
        cmd.linear.x = 0.2
        self.nav_controller.cmd_vel_pub.publish(cmd)
        rospy.sleep(1.0)
        self.nav_controller.stop_robot()
        rospy.sleep(0.5)
        
        content = scan_func()
        if content:
            result = parse_func(content)
            if result: return result
            
        # 4. 恢复位置 (后退10cm)
        rospy.loginfo("搜寻动作3: 恢复位置")
        cmd = Twist()
        cmd.linear.x = -0.1
        self.nav_controller.cmd_vel_pub.publish(cmd)
        rospy.sleep(1.0)
        self.nav_controller.stop_robot()
        rospy.sleep(0.5)
        
        return None

    def execute_state(self):
        """执行当前状态的任务"""
        state = self.state_machine.get_state()
        
        if state == MissionState.IDLE:
            rospy.loginfo_throttle(10, "等待任务启动...")
            return True
            
        elif state == MissionState.NAV_TO_INTERSECTION_1:
            if self.navigate_to_waypoint('intersection_1'):
                self.state_machine.set_state(MissionState.QR_SCAN_PICKUP)
                return True
            return False
            
        # elif state == MissionState.ROTATE_180:
        #     if self.rotate_relative(math.pi):
        #         self.state_machine.set_state(MissionState.QR_SCAN_PICKUP)
        #         return True
        #     return False
            
        elif state == MissionState.QR_SCAN_PICKUP:
            items = self.scan_with_search(
                self.qr_controller.scan_qr_code,
                self.qr_controller.parse_pickup_qr
            )
            if items:
                self.state_machine.pickup_items = items
                self.state_machine.set_state(MissionState.NAV_TO_PICKUP)
                return True
            rospy.logwarn("重试扫描任务QR码...")
            rospy.sleep(1.0)
            return False
            
        elif state == MissionState.NAV_TO_PICKUP:
            if self.navigate_to_waypoint('pickup'):
                self.state_machine.set_state(MissionState.VOICE_PICKUP)
                return True
            return False
            
        elif state == MissionState.VOICE_PICKUP:
            if self.voice_controller.play_pickup_list(self.state_machine.pickup_items):
                self.state_machine.set_state(MissionState.LEAVE_PICKUP_AREA)
                return True
            return True # 即使播报失败也继续
            
        elif state == MissionState.LEAVE_PICKUP_AREA:
            rospy.loginfo("正在离开任务领取区...")
            # 向后向右移动
            cmd = Twist()
            cmd.linear.x = -0.85
            cmd.linear.y = -0.55
            self.nav_controller.cmd_vel_pub.publish(cmd)
            rospy.sleep(1.0)
            self.nav_controller.stop_robot()
            rospy.sleep(0.5)
            
            # 旋转90度
            if self.rotate_relative(-math.pi/2):
                self.state_machine.set_state(MissionState.NAV_AND_SCAN_INFO) # Change to combined state
                return True
            return False
            
        # elif state == MissionState.ROTATE_RIGHT_90:
        #     if self.rotate_relative(-math.pi/2):
        #         self.state_machine.set_state(MissionState.NAV_TO_INFO_AREA)
        #         return True
        #     return False

        elif state == MissionState.NAV_AND_SCAN_INFO:
            rospy.loginfo("开始导航至信息区并同时扫描QR码...")
            
            # 1. 开始QR码扫描 (非阻塞)
            self.qr_controller.start_scan()
            
            # 2. 导航，同时检查QR结果
            waypoints = ['info_area', 'info_area_backup_1', 'info_area_backup_2']
            nav_success = False
            
            for i, name in enumerate(waypoints):
                rospy.loginfo(f"尝试导航到: {name}")
                waypoint = self.waypoint_manager.get_waypoint(name)
                if not waypoint:
                    rospy.logerr(f"路径点 {name} 不存在，跳过")
                    continue

                # 发送导航目标 (非阻塞)
                self.nav_controller.send_goal(waypoint.x, waypoint.y, waypoint.yaw)
                
                # 循环检查导航状态和QR结果
                start_time = rospy.Time.now()
                timeout = rospy.Duration(30.0) # 导航超时
                
                while (rospy.Time.now() - start_time) < timeout:
                    # 检查QR码
                    qr_content = self.qr_controller.get_qr_data()
                    if qr_content:
                        info = self.qr_controller.parse_info_qr(qr_content)
                        if info:
                            rospy.loginfo(f"在导航到 {name} 途中成功扫描到QR码！")
                            self.nav_controller.cancel_goal() # 取消导航
                            self.qr_controller.stop_scan()
                            self.state_machine.shelf_info = info
                            self.state_machine.set_state(MissionState.CALCULATE_ROUTE)
                            return True
                    
                    # 检查导航是否完成
                    if self.nav_controller.is_goal_reached():
                        rospy.loginfo(f"成功到达 {name}")
                        nav_success = True
                        break # 退出内部循环，导航成功
                    
                    rospy.sleep(0.1) # 短暂休眠
                
                if nav_success:
                    break # 退出外部循环，导航成功
                else:
                    rospy.logwarn(f"导航到 {name} 超时或失败，尝试下一个点...")
                    self.nav_controller.cancel_goal()

            # 3. 如果导航结束仍未扫到码，执行原地搜寻
            self.qr_controller.stop_scan() # 停止之前的扫描
            if not nav_success:
                rospy.logerr("所有信息咨询区位置均无法到达！")
                return False

            rospy.loginfo("导航已到达，但未扫描到QR码，开始原地搜寻...")
            info = self.scan_with_search(
                self.qr_controller.scan_qr_code,
                self.qr_controller.parse_info_qr
            )
            if info:
                self.state_machine.shelf_info = info
                self.state_machine.set_state(MissionState.CALCULATE_ROUTE)
                return True
            
            rospy.logerr("搜寻后仍无法识别信息QR码！")
            return False
            
        elif state == MissionState.CALCULATE_ROUTE:
            rospy.loginfo("计算最优路径...")
            if self.state_machine.calculate_route():
                # 导航到共同路段
                if self.navigate_to_waypoint('intersection_2') and \
                   self.navigate_to_waypoint('intersection_3'):
                    # 左转90度
                    rospy.loginfo("在拐点3左转90度")
                    if self.rotate_relative(math.pi/2):
                        self.state_machine.set_state(MissionState.NAV_TO_SHELF_1)
                        return True
                    else:
                        rospy.logerr("在拐点3左转失败")
                        return False
            else:
                rospy.logerr("无法计算路线！")
            return False
            
        elif state == MissionState.NAV_TO_SHELF_1:
            target_shelf = self.state_machine.target_shelves[0]
            rospy.loginfo(f"开始处理第一个货架: {target_shelf}")
            
            # 1. 导航到对应的路口
            intersection = 'intersection_ac_left' if target_shelf in ['A', 'C'] else 'intersection_bd_left'
            if not self.navigate_to_waypoint(intersection):
                return False
                
            # 2. 从路口导航到货架
            if self.navigate_to_shelf(target_shelf):
                self.state_machine.set_state(MissionState.DETECT_OBJECT_1)
                return True
            return False
            
        elif state == MissionState.DETECT_OBJECT_1:
            rospy.sleep(1.0) # 等待图像稳定
            obj = self.object_detector.get_detected_object(retry_on_fail=True)
            if obj:
                self.state_machine.detected_objects.append(obj)
            else:
                rospy.logerr("物品识别失败，即使在重试后。")
                # 即使失败也继续，但记录一个空值或特定标记
                self.state_machine.detected_objects.append("unknown")

            self.state_machine.set_state(MissionState.VOICE_OBJECT_1)
            return True
            
        elif state == MissionState.VOICE_OBJECT_1:
            obj = self.state_machine.detected_objects[-1]
            self.voice_controller.play_object_detected(obj)
            
            if len(self.state_machine.target_shelves) > 1:
                self.state_machine.set_state(MissionState.NAV_TO_SHELF_2)
            else:
                self.state_machine.set_state(MissionState.NAV_TO_INTERSECTION_4)
            return True
            
        elif state == MissionState.NAV_TO_SHELF_2:
            first_shelf = self.state_machine.target_shelves[0]
            second_shelf = self.state_machine.target_shelves[1]
            rospy.loginfo(f"开始处理第二个货架: {second_shelf}")

            # 1. 返回第一个货架的路口
            prev_intersection = 'intersection_ac_left' if first_shelf in ['A', 'C'] else 'intersection_bd_left'
            if not self.navigate_to_waypoint(prev_intersection):
                rospy.logerr(f"返回路口 {prev_intersection} 失败")
                return False

            # 2. 导航到第二个货架的路口
            next_intersection = 'intersection_ac_left' if second_shelf in ['A', 'C'] else 'intersection_bd_left'
            if prev_intersection != next_intersection:
                if not self.navigate_to_waypoint(next_intersection):
                    rospy.logerr(f"导航到路口 {next_intersection} 失败")
                    return False
            
            # 3. 从路口导航到货架
            if self.navigate_to_shelf(second_shelf):
                self.state_machine.set_state(MissionState.DETECT_OBJECT_2)
                return True
            return False
            
        elif state == MissionState.DETECT_OBJECT_2:
            rospy.sleep(1.0)
            obj = self.object_detector.get_detected_object(retry_on_fail=True)
            if obj:
                self.state_machine.detected_objects.append(obj)
            else:
                rospy.logerr("物品识别失败，即使在重试后。")
                self.state_machine.detected_objects.append("unknown")

            self.state_machine.set_state(MissionState.VOICE_OBJECT_2)
            return True
            
        elif state == MissionState.VOICE_OBJECT_2:
            obj = self.state_machine.detected_objects[-1]
            self.voice_controller.play_object_detected(obj)
            self.state_machine.set_state(MissionState.NAV_TO_INTERSECTION_4)
            return True
            
        elif state == MissionState.NAV_TO_INTERSECTION_4:
            last_shelf = self.state_machine.target_shelves[-1]
            if self.navigate_from_shelf_to_exit(last_shelf):
                self.state_machine.set_state(MissionState.ROTATE_LEFT_90)
                return True
            return False
            
        elif state == MissionState.ROTATE_LEFT_90:
            if self.rotate_relative(math.pi/2):
                self.state_machine.set_state(MissionState.NAV_TO_PARKING)
                return True
            return False
            
        elif state == MissionState.NAV_TO_PARKING:
            if self.navigate_to_waypoint('parking'):
                # 停止计时器
                try:
                    self.timer_client(False)
                except:
                    pass
                self.state_machine.set_state(MissionState.VOICE_COMPLETED)
                return True
            return False
            
        elif state == MissionState.VOICE_COMPLETED:
            self.voice_controller.play_final_summary(self.state_machine.detected_objects)
            self.state_machine.set_state(MissionState.FINISHED)
            self.mission_completed = True
            rospy.loginfo("任务全部完成！")
            return True
            
        elif state == MissionState.FINISHED:
            return True
            
        return False
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.execute_state()
                rate.sleep()
            except Exception as e:
                rospy.logerr(f"主循环错误: {e}")
                rospy.sleep(1.0)

def main():
    try:
        commander = UCarCommander()
        commander.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
