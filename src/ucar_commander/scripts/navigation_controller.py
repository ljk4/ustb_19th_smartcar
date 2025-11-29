#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航控制模块 - 管理PID和TEB导航控制
"""
import rospy
import actionlib
import math
import tf
import tf.transformations as tft
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

class PIDController:
    """PID控制器"""
    def __init__(self, kp, ki, kd, max_output, min_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.previous_error = 0
        self.integral = 0
        self.last_time = None

    def reset(self):
        """重置PID控制器"""
        self.previous_error = 0
        self.integral = 0
        self.last_time = None

    def compute(self, error, dt):
        """计算PID输出"""
        if dt <= 0:
            return 0

        # 计算微分项
        derivative = (error - self.previous_error) / dt
        
        # 计算积分项（带抗饱和）
        self.integral += error * dt
        # 积分限幅
        integral_limit = 5.0
        if self.integral > integral_limit:
            self.integral = integral_limit
        elif self.integral < -integral_limit:
            self.integral = -integral_limit

        # 计算PID输出
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # 输出限幅
        if output > self.max_output:
            output = self.max_output
        elif output < self.min_output:
            output = self.min_output

        self.previous_error = error
        
        return output

class NavigationController:
    """导航控制器 - 支持PID和TEB两种模式"""
    
    def __init__(self):
        # move_base 客户端
        self.ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("等待 move_base 服务器...")
        self.ac.wait_for_server()
        rospy.loginfo("move_base 已连接")
        
        # TF监听器
        self.tf_listener = tf.TransformListener()
        rospy.loginfo("TF监听器已初始化")
        
        # 发布器和订阅器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # 当前位姿
        self.current_odom = None
        
        # PID控制器（用于直线路段的位置控制）
        self.linear_pid = PIDController(kp=0.6, ki=0.03, kd=0.2, 
                                       max_output=0.6, min_output=0.0)  
        self.angular_pid = PIDController(kp=0.8, ki=0.0, kd=0.25,
                                        max_output=0.6, min_output=0.0) 
        
        # PID控制参数
        self.pid_position_tolerance = 0.01  # 位置容差保持1cm
        self.pid_angle_tolerance = 0.1      # 角度容差
        self.pid_control_rate = 20          # 控制频率 (Hz)
        self.angle_control_threshold = 0.35 # 角度误差阈值(~20度)
        
        # 减速控制参数
        self.slow_down_distance = 0.3       # 增加减速距离（更早减速）
        self.min_speed_ratio = 0.2          # 降低最小速度比例（更慢接近）
        
        # 导航成功标志
        self.navigation_success = False
        
    def odom_callback(self, msg):
        """里程计回调"""
        self.current_odom = msg
        
    def get_current_pose(self):
        """获取当前位姿 (x, y, yaw) - 通过TF树读取map到base_footprint的变换"""
        try:
            # 等待TF变换可用
            self.tf_listener.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(0.1))
            
            # 获取变换
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
            
            # 解析位置
            x = trans[0]
            y = trans[1]
            
            # 解析朝向（四元数转欧拉角）
            euler = tft.euler_from_quaternion(rot)
            yaw = euler[2]
            
            return x, y, yaw
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"TF查询失败: {e}")
            return None, None, None
    
    def normalize_angle(self, angle):
        """归一化角度到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def create_goal(self, x, y, yaw):
        """创建导航目标点"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        
        # 创建四元数方向
        q = tft.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        return goal
    
    def navigate_with_pid(self, target_x, target_y, target_yaw, timeout=60.0, max_speed=None, position_tolerance=None):
        """
        使用PID控制导航到目标点（直线路段）
        
        Args:
            target_x: 目标X坐标
            target_y: 目标Y坐标
            target_yaw: 目标朝向
            timeout: 超时时间
            max_speed: 自定义最大速度（m/s），None表示使用默认值
            position_tolerance: 自定义位置容差（m），None表示使用默认值
        """
        # 使用自定义最大速度或默认值
        effective_max_speed = max_speed if max_speed is not None else self.linear_pid.max_output
        # 使用自定义位置容差或默认值
        effective_tolerance = position_tolerance if position_tolerance is not None else self.pid_position_tolerance
        
        rospy.loginfo(f"PID导航: 目标 ({target_x:.2f}, {target_y:.2f}, {target_yaw:.2f}), 最大速度={effective_max_speed:.2f}m/s, 容差={effective_tolerance:.3f}m")
        
        # 重置PID控制器
        self.linear_pid.reset()
        self.angular_pid.reset()
        
        rate = rospy.Rate(self.pid_control_rate)
        start_time = rospy.Time.now()
        last_time = start_time.to_sec()
        last_debug_time = 0  # 用于控制调试输出频率
        
        # 卡住检测参数
        stuck_threshold = 0.02  # 位置变化阈值（2cm）
        stuck_timeout = 15.0     # 卡住判定时间（15秒）
        last_position = None
        last_position_time = rospy.Time.now()
        stuck_detected = False
        
        while not rospy.is_shutdown():
            # 检查超时
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("PID导航超时")
                self.stop_robot()
                return False
            
            # 获取当前位姿
            current_x, current_y, current_yaw = self.get_current_pose()
            if current_x is None:
                rospy.logwarn("等待里程计数据...")
                rate.sleep()
                continue
            
            # 卡住检测：检查位置是否长时间不变
            current_position = (current_x, current_y)
            if last_position is not None:
                # 计算位置变化
                position_change = math.sqrt(
                    (current_x - last_position[0])**2 + 
                    (current_y - last_position[1])**2
                )
                
                if position_change < stuck_threshold:
                    # 位置几乎不变，检查是否超时
                    if (rospy.Time.now() - last_position_time).to_sec() > stuck_timeout:
                        rospy.logwarn("=" * 60)
                        rospy.logwarn("检测到PID控制卡住！")
                        rospy.logwarn(f"位置 {stuck_timeout:.1f}秒内变化 < {stuck_threshold}m")
                        rospy.logwarn("切换到TEB控制以绕过障碍...")
                        rospy.logwarn("=" * 60)
                        stuck_detected = True
                        self.stop_robot()
                        break
                else:
                    # 位置有显著变化，更新记录
                    last_position = current_position
                    last_position_time = rospy.Time.now()
            else:
                # 首次记录位置
                last_position = current_position
                last_position_time = rospy.Time.now()
            
            # 计算位置误差（全局坐标系）
            dx = target_x - current_x
            dy = target_y - current_y
            distance_error = math.sqrt(dx*dx + dy*dy)
            
            # 将目标点转换到机器人本地坐标系（使用TF中得到的current_yaw）
            # 机器人坐标系：x轴向前，y轴向左
            local_dx = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
            local_dy = -dx * math.sin(current_yaw) + dy * math.cos(current_yaw)
            
            # 在机器人坐标系中计算角度误差（相对于机器人前方x轴）
            angle_error = math.atan2(local_dy, local_dx)
            
            # 用于调试显示的全局角度
            target_angle = math.atan2(dy, dx)
            
            # 判断是否到达目标位置
            if distance_error < effective_tolerance:
                # 到达位置后，调整朝向
                final_angle_error = self.normalize_angle(target_yaw - current_yaw)
                if abs(final_angle_error) < self.pid_angle_tolerance:
                    rospy.loginfo("PID导航成功到达目标")
                    self.stop_robot()
                    return True
                
                # 只调整角度
                current_time = rospy.Time.now().to_sec()
                dt = current_time - last_time
                last_time = current_time
                
                angular_output = self.angular_pid.compute(abs(final_angle_error), dt)
                
                cmd = Twist()
                cmd.angular.z = angular_output if final_angle_error > 0 else -angular_output
                self.cmd_vel_pub.publish(cmd)
            else:
                # 计算控制输出
                current_time = rospy.Time.now().to_sec()
                dt = current_time - last_time
                last_time = current_time
                
                linear_output = self.linear_pid.compute(distance_error, dt)
                
                # 应用自定义最大速度限制
                if linear_output > effective_max_speed:
                    linear_output = effective_max_speed
                
                # 根据距离应用减速策略，避免冲过目标点
                if distance_error < self.slow_down_distance:
                    # 在减速区内，线性减速
                    speed_ratio = max(
                        self.min_speed_ratio,
                        distance_error / self.slow_down_distance
                    )
                    linear_output *= speed_ratio
                    rospy.logdebug(f"减速区: distance={distance_error:.3f}, ratio={speed_ratio:.2f}")
                
                # 计算机器人坐标系中的速度分量
                # 利用麦克纳姆轮全向移动能力，不需要转向，直接斜向移动
                local_distance = math.sqrt(local_dx**2 + local_dy**2)
                if local_distance > 0.01:  # 避免除零
                    vx = linear_output * (local_dx / local_distance)
                    vy = linear_output * (local_dy / local_distance)
                else:
                    vx = 0
                    vy = 0
                
                # 发布速度命令（包含x和y方向，不进行角度控制）
                # 麦克纳姆轮可以直接斜向移动，无需转向
                cmd = Twist()
                cmd.linear.x = vx
                cmd.linear.y = vy
                cmd.angular.z = 0  # 移动过程中不转向，保持当前朝向
                
                # 调试输出（每秒输出一次）
                if current_time - last_debug_time >= 1.0:
                    # rospy.loginfo("=" * 60)
                    # rospy.loginfo(f"PID控制调试信息:")
                    # rospy.loginfo(f"当前位置: ({current_x:.3f}, {current_y:.3f}), 朝向: {math.degrees(current_yaw):.1f}°")
                    # rospy.loginfo(f"目标位置: ({target_x:.3f}, {target_y:.3f}), 朝向: {math.degrees(target_yaw):.1f}°")
                    # rospy.loginfo(f"全局误差: dx={dx:.3f}, dy={dy:.3f}, distance={distance_error:.3f}")
                    # rospy.loginfo(f"本地误差: local_dx={local_dx:.3f}, local_dy={local_dy:.3f}")
                    # rospy.loginfo(f"目标角度: {math.degrees(target_angle):.1f}°, 角度误差: {math.degrees(angle_error):.1f}°")
                    # rospy.loginfo(f"PID输出: linear={linear_output:.3f}")
                    # rospy.loginfo(f"速度命令: vx={vx:.3f}, vy={vy:.3f}, wz=0.000 (全向移动，不转向)")
                    # rospy.loginfo("=" * 60)
                    last_debug_time = current_time
                
                self.cmd_vel_pub.publish(cmd)
            
            rate.sleep()
        
        self.stop_robot()
        
        # 如果检测到卡住，返回None表示需要切换控制模式
        if stuck_detected:
            return None
        
        return False
    
    def navigate_with_teb(self, target_x, target_y, target_yaw, timeout=60.0):
        """使用TEB planner导航到目标点（障碍物路段）"""
        rospy.loginfo(f"TEB导航: 目标 ({target_x:.2f}, {target_y:.2f}, {target_yaw:.2f})")
        
        goal = self.create_goal(target_x, target_y, target_yaw)
        self.ac.send_goal(goal)
        
        # 等待结果
        finished = self.ac.wait_for_result(rospy.Duration(timeout))
        
        if finished:
            state = self.ac.get_state()
            success = (state == actionlib.GoalStatus.SUCCEEDED)
            if success:
                rospy.loginfo("TEB导航成功")
            else:
                rospy.logwarn(f"TEB导航失败，状态码: {state}")
            return success
        else:
            rospy.logwarn("TEB导航超时")
            self.ac.cancel_goal()
            return False
    
    def rotate_in_place(self, target_yaw, timeout=30.0):
        """原地旋转到指定角度"""
        rospy.loginfo(f"原地旋转到: {math.degrees(target_yaw):.1f} 度")
        
        self.angular_pid.reset()
        rate = rospy.Rate(self.pid_control_rate)
        start_time = rospy.Time.now()
        last_time = start_time.to_sec()
        
        while not rospy.is_shutdown():
            # 检查超时
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("旋转超时")
                self.stop_robot()
                return False
            
            # 获取当前角度
            _, _, current_yaw = self.get_current_pose()
            if current_yaw is None:
                rate.sleep()
                continue
            
            # 计算角度误差
            angle_error = self.normalize_angle(target_yaw - current_yaw)
            
            # 判断是否到达目标角度
            if abs(angle_error) < self.pid_angle_tolerance:
                rospy.loginfo("旋转完成")
                self.stop_robot()
                return True
            
            # 计算控制输出
            current_time = rospy.Time.now().to_sec()
            dt = current_time - last_time
            last_time = current_time
            
            angular_output = self.angular_pid.compute(abs(angle_error), dt)
            
            # 发布速度命令
            cmd = Twist()
            cmd.angular.z = angular_output if angle_error > 0 else -angular_output
            self.cmd_vel_pub.publish(cmd)
            
            rate.sleep()
        
        self.stop_robot()
        return False
    
    def vibration_escape(self, duration=2.0, amplitude=0.3, frequency=2.0):
        """
        通过x、y方向震动脱困
        
        Args:
            duration: 震动持续时间（秒）
            amplitude: 震动幅度（m/s）
            frequency: 震动频率（Hz）
        """
        rospy.loginfo(f"执行震动脱困: 幅度={amplitude}m/s, 频率={frequency}Hz, 持续{duration}秒")
        
        rate = rospy.Rate(20)  # 20Hz控制频率
        start_time = rospy.Time.now()
        phase = 0.0
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                break
            
            # 计算震动相位
            phase = 2 * math.pi * frequency * elapsed
            
            # 正弦波震动：x和y方向相位差90度
            cmd = Twist()
            cmd.linear.x = amplitude * math.sin(phase)
            cmd.linear.y = amplitude * math.cos(phase)
            cmd.angular.z = 0.0
            
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
        
        # 停止
        self.stop_robot()
        rospy.loginfo("震动脱困完成")
        return True
    
    def escape_high_friction_surface(self, duration=1.0, vel_y=0.5, angular_z=0.8):
        """
        脱离高摩擦力平面
        通过施加大的旋转速度和y方向速度来克服摩擦力
        
        Args:
            duration: 持续时间（秒）
            vel_y: y方向速度（m/s）
            angular_z: 旋转速度（rad/s）
        """
        rospy.loginfo(f"执行脱困动作: vel_y={vel_y}, angular_z={angular_z}, 持续{duration}秒")
        
        cmd = Twist()
        cmd.linear.y = vel_y
        cmd.angular.z = angular_z
        
        rate = rospy.Rate(20)  # 20Hz
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                break
            
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
        
        # 停止
        self.stop_robot()
        rospy.loginfo("脱困动作完成")
        return True
    
    def stop_robot(self):
        """停止机器人"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        rospy.sleep(0.1)
        self.cmd_vel_pub.publish(cmd)

    def publish_velocity(self, vx, vy, wz):
        """发布速度命令"""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
