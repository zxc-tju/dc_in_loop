import numpy as np
from plusgo_msgs.msg import SCANeR_ego_plan, SCANeR_action

# define a dummy planner
def dummy_planning(plan_info):
    """
    简单的运动规划器
    输入:
        ego_state: SCANeR_plan_info类型,包含:
            - timestamp: 时间戳
            - refpoints[]: SCANeR_point类型的列表,包含:
                - pointCount: 点序号
                - longitude: 经度
                - latitude: 纬度 
                - courseAngle: 航向角
                - speed: 速度
                - maxSpeed: 最大速度
            - obstacle_state_list[]: 障碍物状态列表,与refpoints结构相同
    输出:
        plan: SCANeR_ego_plan类型,包含:
            - timestamp: 时间戳
            - refpoints[]: SCANeR_point类型的列表,结构同上
            - actions[]: SCANeR_action类型的列表,包含:
                - actionCount: 动作序号
                - acc: 加速度
                - steer: 转向角
    """
    # 初始化SCANeR_ego_plan
    plan = SCANeR_ego_plan()
    plan.timestamp = plan_info.timestamp
    plan.refpoints = plan_info.refpoints.copy()
    plan.actions = []  # 初始化动作序列

    # 获取当前速度
    current_speed = plan_info.refpoints[0].speed
    
    # 遍历参考路径点,保持当前速度
    for point in plan.refpoints:
        point.speed = current_speed
        
    return plan


def dummy_acc_steer_planning(plan_info):
    """
    输出加速度和转向序列的规划器
    输入:
        ego_state: SCANeR_plan_info类型,包含:
            - timestamp: 时间戳
            - refpoints[]: SCANeR_point类型的列表
            - obstacle_state_list[]: 障碍物状态列表
    输出:
        plan: SCANeR_ego_plan类型,包含:
            - timestamp: 时间戳
            - refpoints[]: SCANeR_point类型的列表
            - actions[]: SCANeR_action类型的列表
    """
    # 初始化SCANeR_ego_plan
    plan = SCANeR_ego_plan()
    plan.timestamp = plan_info.timestamp
    plan.refpoints = plan_info.refpoints.copy()
    
    # 初始化动作序列
    plan.actions = []
    
    # 获取参考路径点
    ref_points = plan_info.refpoints
    
    # 计算每两点之间的加速度和转向
    for i in range(len(ref_points)-1):
        # 获取相邻两点信息
        p1 = ref_points[i]
        p2 = ref_points[i+1]
        
        # 计算速度差得到加速度
        v1 = float(p1.speed)
        v2 = float(p2.speed) 
        dt = 0.1  # 假设时间间隔0.1s
        acc = (v2 - v1)/dt
        
        # 计算航向角差得到转向角
        theta1 = float(p1.courseAngle)
        theta2 = float(p2.courseAngle)
        steer = theta2 - theta1
        # 归一化到[-pi, pi]
        steer = np.arctan2(np.sin(steer), np.cos(steer))
        
        # 创建SCANeR_action消息
        action = SCANeR_action()
        action.actionCount = str(i)
        action.acc = str(acc)
        action.steer = str(steer)
        plan.actions.append(action)
        
    return plan
