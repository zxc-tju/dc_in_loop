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

# for car
from plusgo_msgs.msg import cooperateRefline
from plusgo_msgs.msg import cooperateControllist
from plusgo_msgs.msg import MqttToXieTong


import numpy as np
from cal_throttle import ThrottleCalculator
from data_transformer import lon_lat_to_xy
from data_transformer import ReferenceLineParser
from planner import MotionPlanner
import csv
from datetime import datetime
import os
import signal

global DataFromEgo
global DataFromVeh1
global DataFromVeh2

global hm_pubEgo
global hm_pubVeh1
global hm_pubVeh2
global flag
global csv_file
global csv_writer


# TODO: 平台选择
platform = "car"  # or "dc"

def callback(data):
    global DataFromEgo, DataFromVeh1, DataFromVehVirtual
    rospy.loginfo("接收到来自Ego的信息")
    DataFromEgo = copy.deepcopy(data)
    actionEgo, action1 = cal_action(DataFromEgo, DataFromVeh1, DataFromVehVirtual)
    # actionEgo, action1 = cal_action(DataFromEgo, None, DataFromVehVirtual)

    hm_pubEgo.publish(actionEgo)
    hm_pubVeh1.publish(action1)
    print("control published!")


def callback1(data):
    global DataFromVeh1
    rospy.loginfo("接收到来自Veh1的信息")
    DataFromVeh1 = copy.deepcopy(data)

def callback2(data):
    global DataFromVehVirtual
    rospy.loginfo("接收到来自VehVirtual的信息")
    DataFromVehVirtual = copy.deepcopy(data.RedisVirtualVehicles[0])


def cal_action(data1, data2=None, data3=None):
    if platform == "car":
        actionEgo = cooperateControllist()
        action1 = cooperateControllist()
        actionEgo.id = "0" #biyadi
        action1.id = "1" #hongqi
    elif platform == "dc":
        actionEgo = Cooperate_planList()
        action1 = Cooperate_planList()
        actionEgo.Id = "0" #域控1
        action1.Id = "2" #域控3


    current_time = rospy.Time.now()

    # 转换状态信息
    vehicle_data = [data1]
    if data2 is not None:
        vehicle_data.append(data2)
    if data3 is not None:
        vehicle_data.append(data3)
    state_dict = lon_lat_to_xy(vehicle_data, platform="car0;l").get_pos()
    
    # 获取两车参考线
    parser = ReferenceLineParser()
    reference_lines = parser.read_reference_lines('map/pre_map.csv')
    reference_lines = parser.read_reference_lines('map/pre_map.csv')
    resampled_lines = parser.get_reference_line_segments(reference_lines, min_distance=1.0)
    
    # 计算主车的规划轨迹和加速度
    planner = MotionPlanner(state_dict, resampled_lines)
    state_dict['0']['a'] = planner.plan_ego_vehicle()
    print('主车加速度', state_dict['0']['a'])
    print('主车加速度', state_dict['0']['a'])
    # 其他车辆沿参考线加速至目标速度，然后匀速行驶
    if data2 is not None:
        state_dict['1']['a'] = planner.plan_other_vehicle()
        print('其他车加速度', state_dict['1']['a'])
    if data2 is not None:
        state_dict['1']['a'] = planner.plan_other_vehicle()
        print('其他车加速度', state_dict['1']['a'])
    # 计算两车油门量
    calculator = ThrottleCalculator()
    byd_result = calculator.calculate(state_dict['0']['v'], state_dict['0']['a'], 0)
    if data2 is not None:
        hongqi_result = calculator.calculate(state_dict['1']['v'], state_dict['1']['a'], 1)
    else:
        hongqi_result = 0
    if data2 is not None:
        hongqi_result = calculator.calculate(state_dict['1']['v'], state_dict['1']['a'], 1)
    else:
        hongqi_result = 0

    if platform == "car":
        actionEgo.controlCommand = str(byd_result)
        print("主车油门量:", actionEgo)
    elif platform == "dc":
        # 将计算结果(实际上是油门量)赋值给refpoints的speed属性
        data1.refpoints[0].speed = str(byd_result)
        actionEgo.refpoints = data1.refpoints
    
    if data2 is not None:
        if platform == "car":
            action1.controlCommand = str(hongqi_result)
            print("其他车油门量:", action1)
        elif platform == "dc":
            data2.refpoints[0].speed = str(hongqi_result)
            action1.refpoints = data2.refpoints

        print('油门量', round(float(byd_result), 2), round(float(hongqi_result), 2))
    else:
        print('油门量', round(float(byd_result), 2))
    
    # 记录车辆状态到CSV
    for vehicle_id, state in state_dict.items():
        # 只有车辆0和1有油门量数据
        throttle = 0.0
        if vehicle_id == '0':
            throttle = float(byd_result)
        elif vehicle_id == '1' and data2 is not None:
            throttle = float(hongqi_result)
        
        row = [
            f"{current_time.to_sec():.3f}",  # 时间戳，精确到毫秒
            vehicle_id,             # 车辆ID
            state['x'],            # x坐标
            state['y'],            # y坐标
            state['v'],            # 速度
            state['heading'],      # 航向角
            state.get('a', 0),     # 加速度
            throttle               # 油门量
        ]
        csv_writer.writerow(row)

    return actionEgo, action1

def signal_handler(signum, frame):
    global csv_file
    if csv_file:
        csv_file.close()
        rospy.loginfo("CSV file closed successfully")
    rospy.signal_shutdown("User requested shutdown")

def setup_csv():
    # 创建test_log目录
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
    
    # 新文件使用下一个序号，并加入时间戳
    current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_filename = os.path.join(log_dir, f'test_log_{current_time}.csv')
    csv_filename = os.path.join(log_dir, f'test_log_{current_time}.csv')
    
    # 创建文件并写入表头
    f = open(csv_filename, 'w', newline='')
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'vehicle_id', 'x', 'y', 'v', 'heading', 'acceleration', 'throttle'])
    
    rospy.loginfo(f"Created new log file: {csv_filename}")
    return f, writer

if __name__ == "__main__":
    # 初始化CSV文件
    csv_file, csv_writer = setup_csv()
    
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    if platform == "dc":
        DataFromEgo = Cooperate_refLine()
        DataFromVeh1 = Cooperate_refLine()
        DataFromVehVirtual = RedisVirtualVehicles()
    elif platform == "car":
        DataFromEgo = cooperateRefline()
        DataFromVeh1 = cooperateRefline()
        DataFromVeh2 = cooperateRefline()
        DataFromVehVirtual = RedisVirtualVehicles()

    
    rospy.init_node("fsy_intersection", anonymous=True)

    if platform == "dc":
        hm_pubEgo = rospy.Publisher("CooperateOutputMinor", Cooperate_planList, queue_size=5)
        hm_pubVeh1 = rospy.Publisher("CooperateOutputMinor1", Cooperate_planList, queue_size=5)

        hm_subEgo = rospy.Subscriber("CooperateInputFromMinor", Cooperate_refLine, callback, queue_size=3)
        hm_subVeh1 = rospy.Subscriber("CooperateInputFromMinor1", Cooperate_refLine, callback1, queue_size=3)
        hm_subVehVirtual = rospy.Subscriber("redis_virtual_vehicles", RedisVirtualVehicles, callback2, queue_size=3)
    
    elif platform == "car":
        hm_pubEgo = rospy.Publisher("CooperateOutputMain", cooperateControllist, queue_size=5)
        hm_pubVeh1 = rospy.Publisher("CooperateOutputMinor0", cooperateControllist, queue_size=5)

        hm_subEgo = rospy.Subscriber("CooperateInputFromMain", cooperateRefline, callback, queue_size=3)
        hm_subVeh1 = rospy.Subscriber("CooperateInputFromMinor0", cooperateRefline, callback1, queue_size=3)
        hm_subVehVirtual = rospy.Subscriber("redis_virtual_vehicles", RedisVirtualVehicles, callback2, queue_size=3)

    rate = rospy.Rate(100)
    rospy.spin()
