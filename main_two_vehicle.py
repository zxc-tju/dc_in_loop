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

def callback(data):
    global DataFromEgo, DataFromVeh1, DataFromVehVirtual
    rospy.loginfo("接收到来自Ego的信息")
    DataFromEgo = copy.deepcopy(data)
    actionEgo, action1 = cal_action(DataFromEgo, DataFromVeh1, DataFromVehVirtual)
    hm_pubEgo.publish(actionEgo)
    hm_pubVeh1.publish(action1)


def callback1(data):
    global DataFromVeh1
    rospy.loginfo("接收到来自Veh1的信息")
    DataFromVeh1 = copy.deepcopy(data)

def callback2(data):
    global DataFromVehVirtual
    rospy.loginfo("接收到来自VehVirtual的信息")
    DataFromVehVirtual = copy.deepcopy(data.RedisVirtualVehicles[0])
    print(data.RedisVirtualVehicles)


def cal_action(data1, data2, data3):
    actionEgo = Cooperate_planList()
    action1 = Cooperate_planList()
    actionEgo.Id = "0"
    action1.Id = "2"
    current_time = rospy.Time.now()

    # 转换状态信息
    vehicle_data = [data1, data2, data3]
    state_dict = lon_lat_to_xy(vehicle_data, platform="dc").get_pos()
    
    # 获取两车参考线
    parser = ReferenceLineParser()
    reference_lines = parser.read_reference_lines('pre_map.csv')
    resampled_lines = parser.get_reference_line_segments(reference_lines, min_distance=1.0)
    
    # 计算主车的规划轨迹和加速度
    planner = MotionPlanner(state_dict, resampled_lines)
    state_dict['0']['a'] = planner.plan_ego_vehicle()
    
    # 其他车辆沿参考线加速至目标速度，然后匀速行驶
    state_dict['1']['a'] = planner.plan_other_vehicle()

    # 计算两车油门量
    calculator = ThrottleCalculator()
    byd_result = calculator.calculate(state_dict['0']['v'], state_dict['0']['a'], 0)
    hongqi_result = calculator.calculate(state_dict['1']['v'], state_dict['1']['a'], 1)

    # 将计算结果(实际上是油门量)赋值给refpoints的speed属性
    data1.refpoints[0].speed = str(byd_result)
    data2.refpoints[0].speed = str(hongqi_result)

    actionEgo.refpoints = data1.refpoints
    action1.refpoints = data2.refpoints
    print('相应的油门量', actionEgo.refpoints[0].speed, action1.refpoints[0].speed)

    # 记录车辆状态到CSV
    for vehicle_id, state in state_dict.items():
        # 只有车辆0和1有油门量数据
        throttle = 0.0
        if vehicle_id == '0':
            throttle = float(data1.refpoints[0].speed)
        elif vehicle_id == '1':
            throttle = float(data2.refpoints[0].speed)
        
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
    new_file_num = current_max + 1
    current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_filename = os.path.join(log_dir, f'test_log_{new_file_num}_{current_time}.csv')
    
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
