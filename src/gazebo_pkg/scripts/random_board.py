#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import rospy
import random
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion


# 复用基础模型操作函数
def load_model(model_path):
    """加载模型的SDF文件内容"""
    with open(model_path, 'r') as model_file:
        model_xml = model_file.read()
    return model_xml


def spawn_model(model_name, model_xml, pose, reference_frame="world"):
    """在Gazebo中生成模型"""
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(model_name, model_xml, "", pose, reference_frame)
        rospy.loginfo(f"模型 {model_name} 已成功生成")
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")


def delete_model(model_name):
    """从Gazebo中删除模型"""
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_prox(model_name)
        rospy.loginfo(f"模型 {model_name} 已成功删除")
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")


if __name__ == "__main__":
    rospy.init_node('spawn_boards_by_qrcode_folder')
    timestamp = int(time.time())  # 用于生成唯一模型名
    model_root_path = "/home/ljk/.gazebo/models/"  # 替换为实际模型根路径

    # ---------------------- 1. 生成第一个二维码（可选，保留原有逻辑） ----------------------
    first_qrcode_folders = [
        "fruit;dessert_board", "fruit;vegetable_board", "vegetable;dessert_board"
    ]
    first_selected = random.choice(first_qrcode_folders)
    first_model_path = os.path.join(model_root_path, first_selected, "model.sdf")
    try:
        first_model_xml = load_model(first_model_path)
    except FileNotFoundError:
        rospy.logerr(f"未找到第一个二维码模型: {first_model_path}")
        sys.exit(1)
    # 第一个二维码位置
    first_pose = Pose(
        position=Point(x=-3.65, y=-0.22, z=0.0),
        orientation=Quaternion(x=0, y=0, z=0.7071, w=0.7071)
    )
    spawn_model(
        f"first_qrcode_{first_selected}_{timestamp}",
        first_model_xml,
        first_pose
    )

    # ---------------------- 2. 生成第二个二维码（核心：用文件夹名映射位置） ----------------------
    # 第二个二维码文件夹列表（文件夹名需能体现位置组合，如包含abc、abd等标识）
    second_qrcode_folders = [
        "A_dessert_B_fruit_C_vegetable_board", "A_dessert_B_vegetable_C_fruit_board", "A_fruit_B_dessert_C_vegetable_board", "A_fruit_B_vegetable_C_dessert_board","A_vegetable_B_dessert_C_fruit_board", "A_vegetable_B_fruit_C_dessert_board", "A_dessert_B_fruit_D_vegetable_board", "A_dessert_B_vegetable_D_fruit_board","A_fruit_B_dessert_D_vegetable_board", "A_fruit_B_vegetable_D_dessert_board", "A_vegetable_B_dessert_D_fruit_board", "A_vegetable_B_fruit_D_dessert_board","A_dessert_C_fruit_D_vegetable_board", "A_dessert_C_vegetable_D_fruit_board", "A_fruit_C_dessert_D_vegetable_board", "A_fruit_C_vegetable_D_dessert_board","A_vegetable_C_dessert_D_fruit_board", "A_vegetable_C_fruit_D_dessert_board", "B_dessert_C_fruit_D_vegetable_board", "B_dessert_C_vegetable_D_fruit_board","B_fruit_C_dessert_D_vegetable_board", "B_fruit_C_vegetable_D_dessert_board", "B_vegetable_C_dessert_D_fruit_board", "B_vegetable_C_fruit_D_dessert_board"
    ]
    second_selected = random.choice(second_qrcode_folders)  # 实际场景可改为指定选择
    second_model_path = os.path.join(model_root_path, second_selected, "model.sdf")
    try:
        second_model_xml = load_model(second_model_path)
    except FileNotFoundError:
        rospy.logerr(f"未找到第二个二维码模型: {second_model_path}")
        sys.exit(1)
    # 第二个二维码位置
    second_pose = Pose(
        position=Point(x=-3.06, y=2.4, z=0.0),
        orientation=Quaternion(x=0, y=0, z=0, w=1)
    )
    spawn_model(
        f"second_qrcode_{second_selected}_{timestamp}",
        second_model_xml,
        second_pose
    )

    # ---------------------- 3. 核心映射：文件夹名 → 位置组合（ABCD选3） ----------------------
    # 键：第二个二维码的文件夹名；值：对应的3个位置（A/B/C/D）
    folder_to_positions = {
        "A_fruit_B_vegetable_C_dessert_board": ['A', 'B', 'C'],
        "A_fruit_B_vegetable_D_dessert_board": ['A', 'B', 'D'],
        "A_fruit_B_dessert_C_vegetable_board": ['A', 'C', 'B'],
        "A_fruit_C_vegetable_D_dessert_board": ['A', 'C', 'D'],
        "A_fruit_B_dessert_D_vegetable_board": ['A', 'D', 'B'],
        "A_fruit_C_dessert_D_vegetable_board": ['A', 'D', 'C'],
        "A_vegetable_B_fruit_C_dessert_board": ['B', 'A', 'C'],
        "A_vegetable_B_fruit_D_dessert_board": ['B', 'A', 'D'],
        "A_dessert_B_fruit_C_vegetable_board": ['B', 'C', 'A'],
        "B_fruit_C_vegetable_D_dessert_board": ['B', 'C', 'D'],
        "A_dessert_B_fruit_D_vegetable_board": ['B', 'D', 'A'],
        "B_fruit_C_dessert_D_vegetable_board": ['B', 'D', 'C'],
        "A_vegetable_B_dessert_C_fruit_board": ['C', 'A', 'B'],
        "A_vegetable_C_fruit_D_dessert_board": ['C', 'A', 'D'],
        "A_dessert_B_vegetable_C_fruit_board": ['C', 'B', 'A'],
        "B_vegetable_C_fruit_D_dessert_board": ['C', 'B', 'D'],
        "A_dessert_C_fruit_D_vegetable_board": ['C', 'D', 'A'],
        "B_dessert_C_fruit_D_vegetable_board": ['C', 'D', 'B'],
        "A_vegetable_B_dessert_D_fruit_board": ['D', 'A', 'B'],
        "A_vegetable_C_dessert_D_fruit_board": ['D', 'A', 'C'],
        "A_dessert_B_vegetable_D_fruit_board": ['D', 'B', 'A'],
        "B_vegetable_C_dessert_D_fruit_board": ['D', 'B', 'C'],
        "A_dessert_C_vegetable_D_fruit_board": ['D', 'C', 'A'],
        "B_dessert_C_vegetable_D_fruit_board": ['D', 'C', 'B'],
        # 新增文件夹名时，在此处补充映射关系即可
    }
    # 获取当前选中的位置组合
    target_positions = folder_to_positions.get(second_selected, ['A', 'B', 'C'])  # 默认ABC
    rospy.loginfo(f"第二个二维码文件夹：{second_selected}，对应位置：{target_positions}")

    # ---------------------- 4. 定义ABCD四个位置的坐标 ----------------------
    position_coords = {
        'A': Pose(position=Point(x=-2.07, y=1.8, z=0.0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
        'B': Pose(position=Point(x=-2.07, y=0.9, z=0.0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
        'D': Pose(position=Point(x=-0.67, y=0.9, z=0.0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
        'C': Pose(position=Point(x=-0.67, y=1.8, z=0.0), orientation=Quaternion(x=0, y=0, z=0, w=1))
        # 可根据实际场景调整坐标
    }

    # ---------------------- 5. 定义三类板子的文件夹（按需修改） ----------------------
    board_types = {
        "fruit": ["fruit1_board", "fruit2_board", "fruit3_board"],  # 第一类可选板子
        "vegetable": ["vage1_board", "vage2_board", "vage3_board"],  # 第二类可选板子
        "dessert": ["dessert1_board", "dessert2_board", "dessert3_board"]   # 第三类可选板子
    }

    # ---------------------- 6. 在目标位置生成三类板子 ----------------------
    # 按顺序对应：type1→第一个位置，type2→第二个位置，type3→第三个位置
    for idx, (board_type, folders) in enumerate(board_types.items()):
        if idx >= len(target_positions):
            rospy.logwarn(f"位置数量不足，跳过 {board_type}")
            continue
        # 随机选择该类型下的一个板子
        selected_board = random.choice(folders)
        board_path = os.path.join(model_root_path, selected_board, "model.sdf")
        try:
            board_xml = load_model(board_path)
        except FileNotFoundError:
            rospy.logerr(f"未找到 {board_type} 模型：{board_path}")
            continue
        # 获取目标位置坐标
        target_pose = position_coords[target_positions[idx]]
        # 生成唯一模型名并放置
        board_name = f"{board_type}_{selected_board}_{target_positions[idx]}_{timestamp}"
        spawn_model(board_name, board_xml, target_pose)

    rospy.spin()
