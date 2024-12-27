#!/usr/bin/env python3
# encoding: utf-8


import rospy
from copy import deepcopy
import numpy as np

from plusgo_msgs.msg import SCANeR_plan_info  # 来自SCANeR的输入(主车参考路径+障碍物当前状态列表)
from plusgo_msgs.msg import SCANeR_ego_plan  # 车辆规划路径

from scaner_planner import dummy_planning, dummy_acc_steer_planning

    
    
def callback(data):
    # data是SCANeR_plan_info类型,数据结构：
    # string timestamp
    # SCANeR_point[] refpoints 主车参考路径，数据结构为
    # SCANeR_point[] obstacle_state_list 障碍物当前状态列表，数据结构：
    rospy.loginfo("接收到來自SCANeR的信息")
    
    ego_plan = dummy_planning(data)
    # 如果采用动作序列规划，则发布ego_plan
    # ego_plan = dummy_acc_steer_planning(data)

    ego_plan_pub.publish(ego_plan)
    


if __name__ == "__main__":
    rospy.init_node("SCANeR_Planning", anonymous=True)    
    ego_plan_pub = rospy.Publisher("PlanForSCANeR", SCANeR_ego_plan, queue_size=5)

    plan_info_sub = rospy.Subscriber("SCANeRInput", SCANeR_plan_info, callback, queue_size=3)

    rate = rospy.Rate(100)
    
    rospy.spin()


