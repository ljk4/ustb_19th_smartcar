#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
语音播报控制模块
"""
import rospy
import os
import subprocess

class VoiceController:
    """语音播报控制器"""
    
    def __init__(self):
        # 获取语音文件根目录
        self.voice_root = rospy.get_param('/voice_root', '')
        
        if not self.voice_root:
            rospy.logerr("未设置 voice_root 参数，无法播放语音")
        else:
            rospy.loginfo(f"语音文件根目录: {self.voice_root}")
    
    def play_voice(self, relative_path, wait=True):
        """
        播放语音文件
        
        Args:
            relative_path: 相对于voice_root的文件路径
            wait: 是否等待播放完成
            
        Returns:
            bool: 播放是否成功
        """
        if not self.voice_root:
            rospy.logerr("语音根目录未设置，无法播放")
            return False
        
        wav_path = os.path.join(self.voice_root, relative_path)
        
        if not os.path.isfile(wav_path):
            rospy.logerr(f"语音文件不存在: {wav_path}")
            return False
        
        rospy.loginfo(f"播放语音: {relative_path}")
        
        try:
            if wait:
                # 阻塞等待播放完成
                result = subprocess.run(['aplay', wav_path], 
                                      check=True, 
                                      capture_output=True, 
                                      text=True)
                rospy.loginfo("语音播放完成")
                return True
            else:
                # 非阻塞播放
                subprocess.Popen(['aplay', wav_path],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
                return True
                
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"aplay 播放失败 (退出码 {e.returncode}): {e.stderr}")
            return False
        except FileNotFoundError:
            rospy.logerr("未找到 'aplay' 命令")
            return False
        except Exception as e:
            rospy.logerr(f"播放失败: {e}")
            return False
    
    def play_cargo_picked(self, cargo_name):
        """
        播报已取到货物
        
        Args:
            cargo_name: 货物名称 (学习用品/娱乐用品/生活用品)
            
        Returns:
            bool: 播放是否成功
        """
        voice_file = f"voice_library/我已取到{cargo_name}.wav"
        rospy.loginfo(f"播报: 我已取到{cargo_name}")
        return self.play_voice(voice_file, wait=True)
    
    def play_delivered(self, cabinet_name):
        """
        播报货物已送达
        
        Args:
            cabinet_name: 快递柜名称 (A/B/C)
            
        Returns:
            bool: 播放是否成功
        """
        voice_file = f"voice_library/货物已送达至{cabinet_name}号柜 请注意查收.wav"
        rospy.loginfo(f"播报: 货物已送达至{cabinet_name}号柜 请注意查收")
        return self.play_voice(voice_file, wait=True)
    
    def play_mission_completed(self):
        """
        播报任务完成
        
        Returns:
            bool: 播放是否成功
        """
        voice_file = "voice_library/我已完成快递运输 请给个五星好评欧.wav"
        rospy.loginfo("播报: 我已完成快递运输 请给个五星好评欧")
        return self.play_voice(voice_file, wait=True)
