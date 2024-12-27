#!/usr/bin/env python3
# encoding: utf-8

"""
关于输入：
DataFromVeh是Cooperate_refLine结构体，返回的action是Cooperate_planList结构体。
两者内容几乎没有差别，datafromveh多了车辆当前的信息。
两者都包括一个由Cooperate_refPoint结构体构成的列表,这段规划路径大约长50米。
关于地图：
由于车辆运动采用航位推算，车辆当前位置不一定在预先定义的crossroad.txt地图中,
房世玉采用了最近邻匹配的方法将当前位置映射到地图上
另一种方法是采用车辆在当前时刻给出的未来规划路径DataFromVeh.refpoints来判断冲突点
关于输出：
车端只接收并执行action.refpoints第一个点的速度（m/s）值
"""

import rospy
from plusgo_msgs.msg import Cooperate_refPoint
from plusgo_msgs.msg import Cooperate_planList
from plusgo_msgs.msg import Cooperate_refLine

from copy import deepcopy
import numpy as np
# from pyproj import Transformer

global DataFromVeh1
global DataFromVeh2
    
    
def callback1(data):
    global DataFromVeh1, DataFromVeh2
    rospy.loginfo("接收到來自Veh1车的信息")
    DataFromVeh1 = deepcopy(data)
    action1, action2 = cal_action(DataFromVeh1, DataFromVeh2)
    hm_pubVeh1.publish(action1)
    hm_pubVeh2.publish(action2)
    
     
def callback2(data):
    global DataFromVeh2
    rospy.loginfo("接收到來自Veh2车的信息")
    DataFromVeh2 = deepcopy(data)


def cal_action(data1, data2):
    action1, action2 = Cooperate_planList(), Cooperate_planList()
    # 参考路径，车辆编号,时间戳不变化
    action1.refpoints = deepcopy(data1.refpoints)
    action2.refpoints = deepcopy(data2.refpoints)
    action1.Id = "2"
    action2.Id = "3"
    action1.timestamp='0'
    action2.timestamp='0'
    #计算并填入车辆速度
    v1, v2 = cal_speed()
    action1.refpoints[0].speed=str(v1)
    action2.refpoints[0].speed=str(v2)
    return action1, action2
    

def cal_speed():
    # 需要自行计算
    v1, v2 = 0, 0
    return v1, v2 

if __name__ == "__main__":
    rospy.init_node("intersection_2_zxc", anonymous=True)    
    hm_pubVeh1 = rospy.Publisher("CooperateOutputMinor0", Cooperate_planList, queue_size=5)
    hm_pubVeh2 = rospy.Publisher("CooperateOutputMinor1", Cooperate_planList, queue_size=5)
    
    hm_subVeh1 = rospy.Subscriber("CooperateInputFromMinor0", Cooperate_refLine, callback1, queue_size=3)
    hm_subVeh2 = rospy.Subscriber("CooperateInputFromMinor1", Cooperate_refLine, callback2, queue_size=3)
    rate = rospy.Rate(100)
    rospy.spin()


