#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from tkinter import Tk, Label, Button
import time

class TimerServiceApp:
    def __init__(self):
        rospy.init_node("timer_service_node")

        # 创建服务
        self.service = rospy.Service("/start_stop_service", SetBool, self.handle_start_stop)
        
        # 计时器相关变量
        self.timer_started = False
        self.start_time = None
        
        # 创建可视化界面
        self.create_gui()
        
    def handle_start_stop(self, req):
        """
        服务回调函数，处理启动和停止请求。
        """
        if req.data:  # 如果接收到 True (1)，启动计时器
            if not self.timer_started:
                self.start_timer()
                return SetBoolResponse(success=True, message="Timer started.")
            else:
                return SetBoolResponse(success=False, message="Timer already running.")
        else:  # 如果接收到 False (0)，停止计时器
            if self.timer_started:
                self.stop_timer()
                return SetBoolResponse(success=True, message="Timer stopped.")
            else:
                return SetBoolResponse(success=False, message="Timer is not running.")
    
    def start_timer(self):
        """
        启动计时器。
        """
        self.timer_label.config(text="00:00:000")
        self.timer_started = True
        self.start_time = time.time()
        self.update_timer()
    
    def stop_timer(self):
        """
        停止计时器。
        """
        self.timer_started = False
    
    def update_timer(self):
        """
        更新计时器显示。
        """
        if self.timer_started:
            elapsed_time = time.time() - self.start_time
            formatted_time = self.format_time(elapsed_time)
            self.timer_label.config(text=formatted_time)
            self.root.after(10, self.update_timer)  # 每 10ms 更新一次以提高精度
    
    @staticmethod
    def format_time(seconds):
        """
        将秒数格式化为 MM:SS:mmm。
        """
        minutes = int(seconds // 60)
        seconds_total = int(seconds % 60)
        milliseconds = int((seconds - int(seconds)) * 1000)  # 提取毫秒部分
        return f"{minutes:02}:{seconds_total:02}:{milliseconds:03}"
    
    def create_gui(self):
        """
        创建可视化界面。
        """
        self.root = Tk()
        self.root.title("ROS Race Timer")
        self.root.geometry("850x400")
        
        # 大计时器标签
        self.timer_label = Label(self.root, text="00:00:000", font=("Arial", 60))
        self.timer_label.pack(expand=True)
        
        # 启动/停止按钮
        self.toggle_button = Button(self.root, text="Start/Stop", command=self.toggle_timer)
        self.toggle_button.pack()
        
        # 运行主循环
        self.root.mainloop()
    
    def toggle_timer(self):
        """
        切换计时器状态。
        """
        if self.timer_started:
            self.stop_timer()
        else:
            self.start_timer()

if __name__ == "__main__":
    try:
        app = TimerServiceApp()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass