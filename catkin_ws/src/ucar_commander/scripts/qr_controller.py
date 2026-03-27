#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
QR码识别控制模块
"""
import rospy
from qr_msgs.srv import qr, qrRequest

class QRController:
    """QR码识别控制器"""
    
    def __init__(self):
        # QR识别服务客户端
        self.qr_client = rospy.ServiceProxy('/qr/scan_qr', qr)
        rospy.loginfo("等待QR识别服务...")
        try:
            rospy.wait_for_service('/qr/scan_qr', timeout=5.0)
            rospy.loginfo("QR识别服务已连接")
        except rospy.ROSException:
            rospy.logwarn("QR识别服务连接超时，将在使用时重试")
    
    def scan_qr(self, max_retries=3, retry_delay=1.0):
        """
        扫描QR码
        
        Args:
            max_retries: 最大重试次数
            retry_delay: 重试延迟（秒）
            
        Returns:
            int: QR码ID，失败返回-1
        """
        for attempt in range(max_retries):
            try:
                rospy.loginfo(f"尝试识别QR码 (尝试 {attempt + 1}/{max_retries})...")
                
                # 调用QR识别服务
                req = qrRequest()
                req.start_calling = True
                resp = self.qr_client(req)
                
                if resp.success:
                    qr_id = resp.id.data
                    rospy.loginfo(f"QR码识别成功: ID={qr_id}")
                    return qr_id
                else:
                    rospy.logwarn(f"QR码识别失败 (尝试 {attempt + 1}/{max_retries})")
                    
            except rospy.ServiceException as e:
                rospy.logerr(f"QR识别服务调用失败: {e}")
            
            # 如果不是最后一次尝试，等待后重试
            if attempt < max_retries - 1:
                rospy.loginfo(f"等待 {retry_delay} 秒后重试...")
                rospy.sleep(retry_delay)
        
        rospy.logerr("QR码识别最终失败")
        return -1
    
    def scan_cargo_qr(self):
        """
        扫描货物QR码
        
        Returns:
            tuple: (cargo_id, cargo_name)，失败返回(-1, "")
        """
        qr_id = self.scan_qr()
        
        if qr_id > 0:
            # 映射QR码到货物类型 (1-3)
            cargo_id = qr_id
            cargo_names = {
                1: "学习用品",
                2: "娱乐用品",
                3: "生活用品"
            }
            cargo_name = cargo_names.get(cargo_id, "未知货物")
            rospy.loginfo(f"货物类型: {cargo_name} (ID={cargo_id})")
            return cargo_id, cargo_name
        
        return -1, ""
    
    def scan_station_qr(self):
        """
        扫描快递站地址QR码
        
        Returns:
            int: QR码值，失败返回-1
        """
        qr_id = self.scan_qr()
        
        if qr_id > 0:
            rospy.loginfo(f"快递站地址QR码: {qr_id}")
            return qr_id
        
        return -1
    
    def scan_locker_qr(self):
        """
        扫描快递柜号码QR码
        
        Returns:
            int: QR码值，失败返回-1
        """
        qr_id = self.scan_qr()
        
        if qr_id > 0:
            rospy.loginfo(f"快递柜号码QR码: {qr_id}")
            return qr_id
        
        return -1
