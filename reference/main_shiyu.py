#!/usr/bin/env python3
# encoding: utf-8
import copy
import rospy
from plusgo_msgs.msg import Cooperate_refPoint
from plusgo_msgs.msg import Cooperate_planList
from plusgo_msgs.msg import Cooperate_refLine
# from plusgo_msgs.msg import MqttToXieTong
import numpy as np

global DataFromEgo
global DataFromVeh1
global DataFromVeh2

global hm_pubEgo
global hm_pubVeh1
global hm_pubVeh2
global flag

def callback(data):
    global DataFromEgo, DataFromVeh1, DataFromVeh2
    rospy.loginfo("接收到來自Ego車的信息")
    print(data.speed)
    DataFromEgo = copy.deepcopy(data)
    actionEgo, action1, action2 = cal_action(DataFromEgo, DataFromVeh1, DataFromVeh2)
    hm_pubEgo.publish(actionEgo)
    hm_pubVeh1.publish(action1)
    hm_pubVeh2.publish(action2)


def callback1(data):
    global DataFromVeh1
    rospy.loginfo("接收到來自Veh1车的信息")
    DataFromVeh1 = copy.deepcopy(data)
    print(data.speed)


def callback2(data):
    global DataFromVeh2
    rospy.loginfo("接收到來自Veh2车的信息")
    DataFromVeh2 = copy.deepcopy(data)
    print(data.speed)


import data_tranformer
import Environment
from Simulator import Simulator

def cal_action(data1, data2, data3):
    ## 分别获得三辆的车信息，消息格式是Cooperate_refLine
    ## 植入自己的代码，返回三辆车的动作，消息格式是Cooperate_planList
    state_list, direction_list, aggressiveness_list = data_tranformer.lon_lat_to_xy(data1, data2, data3).transform()
    # print('before', state_list[0][2], state_list[1][2], state_list[2][2])
    CPG_simulator = Simulator(state_list, direction_list, aggressiveness_list, pg_type='CPG', if_weight_update='update', if_shapley_update='update', case_id=0, planning_method='with_planning_horizon')
    CPG_simulator.update(data1.timestamp)
    state_list, direction_list, aggressiveness_list = CPG_simulator.output_state_list, CPG_simulator.output_direction, CPG_simulator.vehicle_aggressiveness
    data_from_simulator = data_tranformer.xy_to_lon_lat(state_list, direction_list, aggressiveness_list, [data1, data2, data3]).transform()
    actionEgo_from_simulator, action1_from_simulator, action2_from_simulator = data_from_simulator[0], data_from_simulator[1], data_from_simulator[2]
    # print('pointCount', data2.refpoints[1].latitude, data2.refpoints[1].pointCount, data1.refpoints[1].pointCount, data3.refpoints[1].pointCount)

    actionEgo = Cooperate_planList()
    action1 = Cooperate_planList()
    action2 = Cooperate_planList()
    # actionEgo = copy.deepcopy(data1)
    # action1 = copy.deepcopy(data2)
    # action2 = copy.deepcopy(data3)
    # actionEgo.refpoints = actionEgo_from_simulator.refpoints
    # action1.refpoints = action1_from_simulator.refpoints
    # action2.refpoints = action2_from_simulator.refpoints
    # # return actionEgo, action1, action2
    # return actionEgo, action1, action2
    actionEgo.refpoints = copy.deepcopy(data1.refpoints)
    action1.refpoints = copy.deepcopy(data2.refpoints)
    action2.refpoints = copy.deepcopy(data3.refpoints)
    actionEgo.Id = data1.Id
    action1.Id = "2"
    action2.Id = "3"
    actionEgo.timestamp='0'
    action1.timestamp='0'
    action2.timestamp='0'
    actionEgo.refpoints[0].speed=str(actionEgo_from_simulator.refpoints[0].speed)
    action1.refpoints[0].speed=str(action1_from_simulator.refpoints[0].speed)
    action2.refpoints[0].speed=str(action2_from_simulator.refpoints[0].speed)
    #actionEgo.refpoints[0].speed=str(10.00)
    #action1.refpoints[0].speed=str(10.00)
    #action2.refpoints[0].speed=str(10.00)
    return actionEgo, action1, action2


if __name__ == "__main__":
    DataFromEgo = Cooperate_refLine()
    DataFromVeh1 = Cooperate_refLine()
    DataFromVeh2 = Cooperate_refLine()
    
    rospy.init_node("fsy_intersection", anonymous=True)
    hm_pubEgo = rospy.Publisher("CooperateOutputMain", Cooperate_planList, queue_size=5)
    hm_pubVeh1 = rospy.Publisher("CooperateOutputMinor0", Cooperate_planList, queue_size=5)
    hm_pubVeh2 = rospy.Publisher("CooperateOutputMinor1", Cooperate_planList, queue_size=5)
    
    hm_subEgo = rospy.Subscriber("CooperateInputFromPlusGo", Cooperate_refLine, callback, queue_size=3)
    hm_subVeh1 = rospy.Subscriber("CooperateInputFromMinor0", Cooperate_refLine, callback1, queue_size=3)
    hm_subVeh2 = rospy.Subscriber("CooperateInputFromMinor1", Cooperate_refLine, callback2, queue_size=3)
    
    rate = rospy.Rate(100)
    rospy.spin()
