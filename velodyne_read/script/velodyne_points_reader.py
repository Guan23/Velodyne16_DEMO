#!/usr/bin/env python
# Guan_20201121
# -*- coding: utf-8 -*-
# 该例程将订阅/velodyne_points话题，消息类型sensor_msgs/CloudPoint2

# 说明：
# 1、data数据共5列，p[0]~p[4]分别为Y、X、Z(单位均为米)、反射强度(0~255)、激光头(0~15)
# 2、其中Y向前+后-，X向左+右-，Z向上+下-
# 3、假设立方体小车身宽40CM，长60CM，高20CM，激光雷达安装在车头中间部位
# 4、探测的方式有两种，一种是探测矩形范围，一种是探测扇形范围
# 探测矩形则通过约束前后左右边界来实现，探测扇形则通过约束斜率来实现
# 假设汽车左右对称，故此处只需约束一四象限的斜率即可（二三也可）

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

# 重要参数定义

# 六个方向的边界，顺序分别为左、右、前、后、上、下
BOUNDARYS = [0.5, -0.5, 0.5, -0.5, 0.2, -0.2]
# 前后方向和左右方向探测的夹角，单位度
INCLUDED_ANGLE = [90, 60]
#仪器自身的最小测距
NEGATIVE_EDGE = -0.2
POSITIVE_EDGE = 0.2
# 车辆尺寸
VEHICLE_WIDTH = 0.8
VEHICLE_LENGTH = 0.6
VEHICLE_HEIGHT = 0.3


# 将前后夹角转换为斜率
def angle_to_gradient():
    front_slope = np.tan(np.deg2rad(90 - INCLUDED_ANGLE[0]/2))
    side_slope = np.tan(np.deg2rad(90 - INCLUDED_ANGLE[1]/2))
    return front_slope,side_slope


# 障碍物检测，矩形检测
def barriers_detection_rect(data):
    for p in data:
        # 障碍物预警
        if 0 < p[2] < POSITIVE_EDGE:
            # print(round(p[0], 3), round(p[1], 3), round(p[2], 3), p[3], p[4])  # 打印信息
            # print("detecting...")
            # 前方警告
            if NEGATIVE_EDGE < p[1] < POSITIVE_EDGE and 0 < p[0] < BOUNDARYS[2]:
                print("X:", round(p[1], 3), "Y:", round(p[0], 3), "front")
                # break
            # 左侧警告
            if 0 < p[1] < BOUNDARYS[0] and NEGATIVE_EDGE < p[0] < POSITIVE_EDGE:
                print("X:", round(p[1], 3), "Y:", round(p[0], 3), "left")
                # break
            # 右侧警告
            if BOUNDARYS[1] < p[1] < 0 and NEGATIVE_EDGE < p[0] < POSITIVE_EDGE:
                print("X:", round(p[1], 3), "Y:", round(p[0], 3), "right")
                # break
            # 后方警告
            if BOUNDARYS[3] < p[0] < 0 and NEGATIVE_EDGE < p[1] < POSITIVE_EDGE:
                print("X:", round(p[1], 3), "Y:", round(p[0], 3), "back")
                # break


# 障碍物检测，扇形检测
def barriers_detection_sector(data):
    # 获取斜率
    front_slope, side_slope = angle_to_gradient()
    for p in data:
        if 0 < p[2] < POSITIVE_EDGE:
            # 前方警告
            if 0 < p[0] < BOUNDARYS[2] and p[0] > abs(front_slope * p[1]):
                print("X:", round(p[1], 3), "Y:", round(p[0], 3), "front")
            # 右侧警告
            if BOUNDARYS[1] < p[1] < 0 and p[1] < -abs(side_slope * p[0]):
                print("X:", round(p[1], 3), "Y:", round(p[0], 3), "right")
            # 左侧警告
            if 0 < p[1] < BOUNDARYS[0] and p[1] > abs(side_slope * p[0]):
                print("X:", round(p[1], 3), "Y:", round(p[0], 3), "left")


# 检测障碍物大小，扇形检测
def size_detection(data):
    # 获取斜率
    front_slope, side_slope = angle_to_gradient()
    # 三个方向的最小最大xyz值，前方只需xy，左右只需yz
    front_xz = [200,200,-200,-200]  # 顺序为最小x，最小z，最大x，最大z
    left_yz = [200,200,-200,-200]  # 顺序为最小y，最小z，最大y，最大z
    right_yz = [200,200,-200,-200]  # 顺序同上
    # 通过点云xyz坐标来检测障碍物大小
    for p in data:
        # 测量前方障碍物
        if 0 < p[0] < BOUNDARYS[2] and p[0] > abs(front_slope * p[1]):
            if p[1] > front_xz[2]:
                front_xz[2] = p[1]
            if p[1] < front_xz[0]:
                front_xz[0] = p[1]
            if p[2] > front_xz[3]:
                front_xz[3] = p[2]
            if p[2] < front_xz[1]:
                front_xz[1] = p[2]
        # 测量右侧障碍物
        if BOUNDARYS[1] < p[1] <0 and p[1] < -abs(side_slope * p[0]):
            if p[0] > right_yz[2]:
                right_yz[2] = p[0]
            if p[0] < right_yz[0]:
                right_yz[0] = p[0]
            if p[2] > right_yz[3]:
                right_yz[3] = p[2]
            if p[2] < right_yz[1]:
                right_yz[1] = p[2]
        # 测量左侧障碍物
        if 0 < p[1] < BOUNDARYS[0] and p[1] > abs(side_slope * p[0]):
            if p[0] > left_yz[2]:
                left_yz[2] = p[1]
            if p[0] < left_yz[0]:
                left_yz[0] = p[1]
            if p[2] > left_yz[3]:
                left_yz[3] = p[2]
            if p[2] < left_yz[1]:
                left_yz[1] = p[2]
    # 获得三个方向的障碍物的宽和高
    fbw = front_xz[2] - front_xz[0]
    fbh = front_xz[3] - front_xz[1]
    rbw = right_yz[2] - right_yz[0]
    rbh = right_yz[3] - right_yz[1]
    lbw = left_yz[2] - left_yz[0]
    lbh = left_yz[3] - left_yz[1]
    # 打印障碍物宽高信息
    if fbw > -300 or fbh > -300:
        print("front_width:",round(fbw,3),"front_height:",round(fbh,3))
    if rbw > -300 or rbh > -300:
        print("right_width:", round(rbw, 3), "right_height:", round(rbh, 3))
    if lbw > -300 or lbh > -300:
        print("left_width:", round(lbw, 3), "left_height:", round(lbh, 3))

    # return fbw, fbh, rbw, rbh, lbw, lbh


# 判断车辆是否能通过前侧空隙，假设激光雷达安装在车头的正中间，可粗略看做与车身一样高
def is_pass(data):
    LEFT_EDGE = VEHICLE_WIDTH / 2
    RIGHT_EDGE = -LEFT_EDGE
    UP_EDGE = 0
    DOWN_EDGE = -VEHICLE_HEIGHT

    for p in data:
        if 0 < p[0] < BOUNDARYS[2]:
            if RIGHT_EDGE < p[1] < LEFT_EDGE and DOWN_EDGE < p[2] < UP_EDGE:
                print('Can not pass! X = ', round(p[1], 3),' Z = ', round(p[2], 3))
                # break
            else:
                # print("You could pass!")
                pass


# 回调函数
def poseCallback(data):
    # 判断data是否是PointCloud2类型
    assert isinstance(data, PointCloud2) 
    # 读取PointCloud2中的data部分
    gen = point_cloud2.read_points(data)
    # print(type(gen)) # 该类型为<type 'generator'>
    
    # 各项功能的测试
    # barriers_detection_rect(data = gen)  # 障碍物检测，矩形检测
    # barriers_detection_sector(data = gen)  # 障碍物检测，扇形检测
    # size_detection(data = gen)  # 障碍物大小检测
    # is_pass(data = gen)  # 检测能否通过空隙


def velodyne_points_reader():
	# ROS节点初始化
    rospy.init_node('velodyne_points_reader', anonymous=True)
    # 开始信息
    print("----------------Start----------------")
    
	# 创建Subscriber
    # rospy.Rate(10)  # 设置频率10Hz
    rospy.Subscriber("/velodyne_points", PointCloud2, poseCallback)
	# 循环等待回调函数
    rospy.spin()
    # rospy.sleep()
    
    # 结束
    print("----------------End----------------")


if __name__ == '__main__':
    velodyne_points_reader()





