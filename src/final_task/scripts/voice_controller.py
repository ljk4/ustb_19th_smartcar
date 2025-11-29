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
            
        # 物品名称映射
        self.item_map = {
            'apple': '苹果',
            'banana': '香蕉',
            'watermelon': '西瓜',
            'potato': '土豆',
            'tomato': '番茄',
            'chili': '辣椒',
            'cake': '蛋糕',
            'milk': '牛奶',
            'cola': '可乐'
        }
        
        # 类别名称映射
        self.category_map = {
            'fruit': '水果',
            'vegetable': '蔬菜',
            'dessert': '甜点'
        }
    
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
    
    def play_pickup_list(self, items):
        """
        播报采购清单
        Args:
            items: 物品类别列表 (e.g., ['fruit', 'vegetable'])
        """
        if not items:
            return False
            
        # 排序以确保顺序一致
        # items.sort()
        
        # 映射到中文
        cn_items = [self.category_map.get(item, item) for item in items]
        
        # 组合文件名
        # 假设文件名格式为 "本次需要采购的物品为水果和蔬菜.wav"
        filename = f"本次需要采购的物品为{'和'.join(cn_items)}.wav"
        voice_file = f"voice_library/{filename}"
        
        # 容错处理：如果文件不存在且有两个物品，尝试交换顺序
        if self.voice_root:
            full_path = os.path.join(self.voice_root, voice_file)
            if not os.path.isfile(full_path) and len(items) == 2:
                cn_items_rev = cn_items[::-1]
                filename_rev = f"本次需要采购的物品为{'和'.join(cn_items_rev)}.wav"
                voice_file_rev = f"voice_library/{filename_rev}"
                full_path_rev = os.path.join(self.voice_root, voice_file_rev)
                
                if os.path.isfile(full_path_rev):
                    rospy.loginfo(f"播报采购清单(交换顺序): {filename_rev}")
                    return self.play_voice(voice_file_rev, wait=True)
        
        rospy.loginfo(f"播报采购清单: {filename}")
        return self.play_voice(voice_file, wait=True)
    
    def play_object_detected(self, object_name):
        """
        播报已取到物品
        Args:
            object_name: 物品英文名 (e.g., 'apple')
        """
        cn_name = self.item_map.get(object_name, object_name)
        voice_file = f"voice_library/我已取到{cn_name}.wav"
        
        rospy.loginfo(f"播报已取到物品: {cn_name}")
        return self.play_voice(voice_file, wait=True)
        
    def play_final_summary(self, objects):
        """
        播报最终汇总
        Args:
            objects: 物品英文名列表 (e.g., ['tomato', 'cake'])
        """
        if not objects:
            return False
            
        # 映射到中文
        cn_objects = [self.item_map.get(obj, obj) for obj in objects]
        
        # 组合文件名 "番茄和蛋糕.wav"
        # 注意：这里假设顺序是固定的或者文件名覆盖了所有组合
        # 任务描述中只给了一个例子 "番茄和蛋糕.wav"
        # 我们按识别顺序或者字母顺序组合
        
        filename = f"{'和'.join(cn_objects)}.wav"
        voice_file = f"voice_library/{filename}"
        
        # 容错处理：如果文件不存在且有两个物品，尝试交换顺序
        if self.voice_root:
            full_path = os.path.join(self.voice_root, voice_file)
            if not os.path.isfile(full_path) and len(objects) == 2:
                cn_objects_rev = cn_objects[::-1]
                filename_rev = f"{'和'.join(cn_objects_rev)}.wav"
                voice_file_rev = f"voice_library/{filename_rev}"
                full_path_rev = os.path.join(self.voice_root, voice_file_rev)
                
                if os.path.isfile(full_path_rev):
                    rospy.loginfo(f"播报最终汇总(交换顺序): {filename_rev}")
                    return self.play_voice(voice_file_rev, wait=True)
        
        rospy.loginfo(f"播报最终汇总: {filename}")
        return self.play_voice(voice_file, wait=True)
