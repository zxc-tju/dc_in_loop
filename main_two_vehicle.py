#!/usr/bin/env python3
# encoding: utf-8
import copy
import rospy
from plusgo_msgs.msg import Cooperate_refPoint
from plusgo_msgs.msg import Cooperate_planList
from plusgo_msgs.msg import Cooperate_refLine
# from plusgo_msgs.msg import MqttToXieTong
from plusgo_msgs.msg import RedisVirtualVehicles
from plusgo_msgs.msg import RedisVirtualVehicle
import numpy as np
from cal_throttle import ThrottleCalculator
from data_transformer import lon_lat_to_xy
from data_transformer import ReferenceLineParser
from planner import MotionPlanner
import csv
from datetime import datetime
import os

global DataFromEgo
global DataFromVeh1
global DataFromVeh2

global hm_pubEgo
global hm_pubVeh1
global hm_pubVeh2
global flag

def callback(data):
    global DataFromEgo, DataFromVeh1, DataFromVehVirtual
    rospy.loginfo("接收到來自Ego車的信息")
    DataFromEgo = copy.deepcopy(data)
    actionEgo, action1 = cal_action(DataFromEgo, DataFromVeh1)
    hm_pubEgo.publish(actionEgo)
    hm_pubVeh1.publish(action1)


def callback1(data):
    global DataFromVeh1
    rospy.loginfo("接收到來自Veh1车的信息")
    DataFromVeh1 = copy.deepcopy(data)

def callback2(data):
    global DataFromVehVirtual
    rospy.loginfo("接收到來自VehVirtual车的信息")
    DataFromVehVirtual = copy.deepcopy(data.RedisVirtualVehicles[0])
    print(data.RedisVirtualVehicles)


def cal_action(data1, data2):
    actionEgo = Cooperate_planList()
    action1 = Cooperate_planList()
    actionEgo.Id = "0"
    action1.Id = "2"
    current_time = rospy.Time.now()
    # actionEgo.timestamp = current_time
    # action1.timestamp = current_time

    # 转换两车的状态信息
    vehicle_data = [data1, data2]  # 先创建包含实车数据的列表
    # if data3:  # 如果有虚拟车数据
    #     vehicle_data.extend(data3)  # 使用extend添加虚拟车列表中的所有元素
    
    state_dict = lon_lat_to_xy(vehicle_data).get_pos()
    # 数据结构
    # {
    #     "0": {"x": 100, "y": 200, "v": 30, "heading": 45, "a": 0},
    #     "1": {"x": 150, "y": 250, "v": 35, "heading": 55, "a": 0}
    # }
    
    # 创建test_log目录（如果不存在）
    log_dir = './test_log'
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # 获取当前最大序号
    existing_logs = [f for f in os.listdir(log_dir) if f.startswith('test_log_') and f.endswith('.csv')]
    current_max = 0
    for log_file in existing_logs:
        try:
            num = int(log_file.replace('test_log_', '').replace('.csv', ''))
            current_max = max(current_max, num)
        except ValueError:
            continue
    
    # 新文件使用下一个序号
    new_file_num = current_max + 1
    csv_filename = os.path.join(log_dir, f'test_log_{new_file_num}.csv')
    
    # 如果是第一次写入该文件，创建文件并写入表头
    if not os.path.exists(csv_filename):
        with open(csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp', 'vehicle_id', 'x', 'y', 'v', 'heading']
            writer.writerow(header)
    
    # 追加当前状态
    with open(csv_filename, 'a', newline='') as f:
        writer = csv.writer(f)
        for vehicle_id, state in state_dict.items():
            row = [
                f"{current_time.to_sec():.3f}",  # 时间戳，保留3位小数（毫秒）
                vehicle_id,             # 车辆ID
                state['x'],            # x坐标
                state['y'],            # y坐标
                state['v'],            # 速度
                state['heading']       # 航向角
            ]
            writer.writerow(row)

    # 获取两车参考线
    # reference_lines的数据结构:
    # {
    #     "轨迹1": {"x": [...], "y": [...], "time": [...]},
    #     "轨迹2": {"x": [...], "y": [...], "time": [...]}
    # }
    parser = ReferenceLineParser()
    reference_lines = parser.read_reference_lines('pre_map.csv')
    # resampled_lines与reference_lines结构相同，但点之间距离更均匀
    resampled_lines = parser.get_reference_line_segments(reference_lines, min_distance=1.0)

    # 计算下一时刻加速度
    # 创建运动规划器
    planner = MotionPlanner(state_dict, resampled_lines)
    
    # 计算第一辆车的规划轨迹和加速度
    ego_acc = planner.plan_ego_vehicle()
    state_dict['0']['a'] = ego_acc
    
    # 其他车辆沿参考线加速至目标速度，然后匀速行驶
    state_dict['1']['a'] = planner.plan_other_vehicle()

    # 计算两车油门量
    calculator = ThrottleCalculator()
    byd_result = calculator.calculate(state_dict['0']['v'], state_dict['0']['a'], 0)
    hongqi_result = calculator.calculate(state_dict['1']['v'], state_dict['1']['a'], 1)

    # 将计算结果(实际上是油门量)赋值给refpoints的speed属性
    data1.refpoints[0].speed = str(byd_result)
    data2.refpoints[0].speed = str(hongqi_result)
    # data1.refpoints[0].speed = str(20)  # TODO: Your thr here
    # data2.refpoints[0].speed = str(20)

    actionEgo.refpoints = data1.refpoints
    action1.refpoints = data2.refpoints
    print('相应的油门量', actionEgo.refpoints[0].speed, action1.refpoints[0].speed)
    # print('相应的油门量', actionEgo.refpoints[0].speed)

    return actionEgo, action1


if __name__ == "__main__":

    DataFromEgo = Cooperate_refLine()
    DataFromVeh1 = Cooperate_refLine()
    DataFromVehVirtual = RedisVirtualVehicles()
    rospy.init_node("fsy_intersection", anonymous=True)
    hm_pubEgo = rospy.Publisher("CooperateOutputMinor", Cooperate_planList, queue_size=5)
    hm_pubVeh1 = rospy.Publisher("CooperateOutputMinor1", Cooperate_planList, queue_size=5)

    hm_subEgo = rospy.Subscriber("CooperateInputFromMinor", Cooperate_refLine, callback, queue_size=3)
    hm_subVeh1 = rospy.Subscriber("CooperateInputFromMinor1", Cooperate_refLine, callback1, queue_size=3)
    hm_subVehVirtual = rospy.Subscriber("redis_virtual_vehicles", RedisVirtualVehicles, callback2, queue_size=3)
    rate = rospy.Rate(100)
    rospy.spin()
