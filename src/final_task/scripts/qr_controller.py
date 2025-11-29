#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
QR码识别控制模块
调用 /qr/scan_qr 服务进行识别
"""
import rospy
import threading
from qr_msgs.srv import qr

class QRController:
    """QR码识别控制器"""
    
    def __init__(self):
        self.lock = threading.Lock()
        
        # 服务客户端
        rospy.loginfo("等待 /qr/scan_qr 服务...")
        try:
            rospy.wait_for_service('/qr/scan_qr', timeout=5.0)
            self.scan_service = rospy.ServiceProxy('/qr/scan_qr', qr)
            rospy.loginfo("/qr/scan_qr 服务已连接")
        except rospy.ROSException:
            rospy.logerr("连接 /qr/scan_qr 服务超时！")
            self.scan_service = None
        
        # 用于非阻塞扫描的变量
        self.scan_thread = None
        self.scanning = False
        self.qr_data = None
        
        # ID to String Mapping
        self.ID_TO_STRING = {
            1: "fruit;dessert",
            2: "fruit;vegetable",
            3: "vegetable;dessert",
            10: "A:dessert, B:fruit, C:vegetable",
            11: "A:dessert, B:vegetable, C:fruit",
            12: "A:fruit, B:dessert, C:vegetable",
            13: "A:fruit, B:vegetable, C:dessert",
            14: "A:vegetable, B:dessert, C:fruit",
            15: "A:vegetable, B:fruit, C:dessert",
            16: "A:dessert, B:fruit, D:vegetable",
            17: "A:dessert, B:vegetable, D:fruit",
            18: "A:fruit, B:dessert, D:vegetable",
            19: "A:fruit, B:vegetable, D:dessert",
            20: "A:vegetable, B:dessert, D:fruit",
            21: "A:vegetable, B:fruit, D:dessert",
            22: "A:dessert, C:fruit, D:vegetable",
            23: "A:dessert, C:vegetable, D:fruit",
            24: "A:fruit, C:dessert, D:vegetable",
            25: "A:fruit, C:vegetable, D:dessert",
            26: "A:vegetable, C:dessert, D:fruit",
            27: "A:vegetable, C:fruit, D:dessert",
            28: "B:dessert, C:fruit, D:vegetable",
            29: "B:dessert, C:vegetable, D:fruit",
            30: "B:fruit, C:dessert, D:vegetable",
            31: "B:fruit, C:vegetable, D:dessert",
            32: "B:vegetable, C:dessert, D:fruit",
            33: "B:vegetable, C:fruit, D:dessert",
        }
        
        rospy.loginfo("QR控制器初始化完成")
    
    def _scan_thread_func(self):
        """后台扫描线程"""
        rate = rospy.Rate(10) # 10Hz
        rospy.loginfo("后台QR扫描线程启动")
        
        while self.scanning and not rospy.is_shutdown():
            if self.scan_service:
                try:
                    resp = self.scan_service(True)
                    if resp.success:
                        # Access .data because id is std_msgs/Int8
                        qr_str = self.ID_TO_STRING.get(resp.id.data)
                        if qr_str:
                            with self.lock:
                                self.qr_data = qr_str
                            # 扫到后可以降低频率
                            rospy.sleep(0.5)
                except rospy.ServiceException as e:
                    rospy.logwarn(f"Service call failed: {e}")
            
            rate.sleep()
        rospy.loginfo("后台QR扫描线程停止")

    def start_scan(self):
        """开始非阻塞扫描"""
        if not self.scanning:
            rospy.loginfo("启动非阻塞QR扫描")
            self.scanning = True
            self.qr_data = None # 重置数据
            self.scan_thread = threading.Thread(target=self._scan_thread_func)
            self.scan_thread.daemon = True
            self.scan_thread.start()

    def stop_scan(self):
        """停止非阻塞扫描"""
        if self.scanning:
            rospy.loginfo("停止非阻塞QR扫描")
            self.scanning = False
            if self.scan_thread:
                self.scan_thread.join(timeout=1.0)
            self.scan_thread = None

    def get_qr_data(self):
        """获取扫描到的QR数据 (非阻塞)"""
        with self.lock:
            return self.qr_data

    def scan_qr_code(self, timeout=5.0):
        """
        尝试识别QR码 (阻塞式)
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            str: 识别到的QR码内容，失败返回 None
        """
        self.start_scan()
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            data = self.get_qr_data()
            if data:
                rospy.loginfo(f"阻塞式扫描成功: {data}")
                self.stop_scan()
                return data
            rospy.sleep(0.1)
            
        rospy.logwarn("阻塞式扫描超时")
        self.stop_scan()
        return None
    
    def parse_pickup_qr(self, qr_content):
        """
        解析任务领取区QR码
        格式示例: "fruit;vegetable"
        
        Returns:
            list: 物品类别列表，如 ['fruit', 'vegetable']
        """
        if not qr_content:
            return []
            
        try:
            # 去除空白字符并按分号分割
            items = [item.strip() for item in qr_content.split(';') if item.strip()]
            rospy.loginfo(f"解析任务物品: {items}")
            return items
        except Exception as e:
            rospy.logerr(f"解析任务QR码失败: {e}")
            return []
            
    def parse_info_qr(self, qr_content):
            """
            解析信息咨询区QR码
            格式示例: "A:vegetable, B:fruit, D:dessert"
            
            Returns:
                dict: 货架信息字典，如 {'A': 'vegetable', 'B': 'fruit', 'D': 'dessert'}
            """
            if not qr_content:
                return {}
                
            try:
                info = {}
                # 按逗号分割不同的货架信息
                parts = qr_content.split(',')
                for part in parts:
                    # 按冒号分割货架号和类别
                    if ':' in part:
                        shelf, category = part.split(':')
                        info[shelf.strip()] = category.strip()
                
                rospy.loginfo(f"解析货架信息: {info}")
                return info
            except Exception as e:
                rospy.logerr(f"解析信息QR码失败: {e}")
                return {}
