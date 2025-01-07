import numpy as np
from scipy.optimize import minimize

class MotionPlanner:
    def __init__(self, state_dict, reference_lines):
        """
        初始化运动规划器
        
        参数:
            state_dict: 车辆状态字典
            reference_lines: 参考线字典
        """
        self.state_dict = state_dict
        self.reference_lines = reference_lines
        
        # 规划参数
        self.dt = 0.1  # 时间步长
        self.prediction_horizon = 30  # 预测时域
        self.comfort_acc = 1.0  # 舒适加速度阈值  
        self.max_acc = 0.4  # 最大加速度
        self.min_acc = -0.5  # 最大减速度
        self.max_speed = 20.0  # 最大速度 km/h
        self.conflict_point = self._find_conflict_point()  # 计算冲突点
        self.target_speed = 20.0  # 目标速度 km/h
        self.max_dec = -0.5      # 最大减速度 m/s²

    def _find_conflict_point(self):
        """
        找到两条参考线的交叉点
        返回: (x, y) 冲突点坐标
        """
        # 假设第一条参考线是ego车的，第二条是other车的
        ego_line = list(self.reference_lines.values())[0]
        other_line = list(self.reference_lines.values())[1]
        
        min_dist = float('inf')
        conflict_point = None
        
        # 遍历两条线寻找最近点
        for i in range(len(ego_line['x'])):
            for j in range(len(other_line['x'])):
                dist = np.sqrt((ego_line['x'][i] - other_line['x'][j])**2 + 
                             (ego_line['y'][i] - other_line['y'][j])**2)
                if dist < min_dist:
                    min_dist = dist
                    conflict_point = (ego_line['x'][i], ego_line['y'][i])
        
        return conflict_point

    def _get_distance_to_conflict(self, vehicle_state, vehicle_id):
        """
        计算车辆沿参考线到冲突点的距离
        """
        if self.conflict_point is None:
            return float('inf')
        
        # 获取对应的参考线
        reference_line = self.reference_lines[f'Trajectory_{100 + int(vehicle_id)}']
        
        # 找到车辆在参考线上的投影点索引
        min_dist = float('inf')
        vehicle_idx = 0
        for i in range(len(reference_line['x'])):
            dist = np.sqrt(
                (reference_line['x'][i] - vehicle_state['x'])**2 +
                (reference_line['y'][i] - vehicle_state['y'])**2
            )
            if dist < min_dist:
                min_dist = dist
                vehicle_idx = i
        print(f'{vehicle_id}号车辆到参考线最近点的距离', min_dist)
        # print('参考线最近点', reference_line['x'][vehicle_idx], reference_line['y'][vehicle_idx])
        # print('车辆最近点', vehicle_state['x'], vehicle_state['y'])
        
        # 找到冲突点在参考线上的投影点索引
        min_dist = float('inf')
        conflict_idx = 0
        for i in range(len(reference_line['x'])):
            dist = np.sqrt(
                (reference_line['x'][i] - self.conflict_point[0])**2 +
                (reference_line['y'][i] - self.conflict_point[1])**2
            )
            if dist < min_dist:
                min_dist = dist
                conflict_idx = i
        # print('冲突点到参考线最近点的距离', min_dist)
        
        # 计算沿参考线的累积距离
        distance = 0
        
        if vehicle_id == '0':  # ego车辆
            start_idx = vehicle_idx
            end_idx = conflict_idx
        else:  # 对向车
            start_idx = vehicle_idx
            end_idx = conflict_idx
            if start_idx > end_idx:
                start_idx, end_idx = end_idx, start_idx
        
        for i in range(start_idx, end_idx):
            distance += np.sqrt(
                (reference_line['x'][i+1] - reference_line['x'][i])**2 +
                (reference_line['y'][i+1] - reference_line['y'][i])**2
            )
        
        return distance

    def _get_time_to_conflict(self, vehicle_state, vehicle_id):
        """
        计算车辆到达冲突点的预计时间
        """
        if self.conflict_point is None:
            return float('inf')
        
        distance = self._get_distance_to_conflict(vehicle_state, vehicle_id)
        print(f'{vehicle_id}到冲突点距离', distance)
        current_speed = vehicle_state['v'] / 3.6
        
        if current_speed < 0.1:
            return float('inf')
        
        return distance / current_speed

    def plan_ego_vehicle(self):
        """为自车规划轨迹并返回下一时刻的加速度"""
        
        ego_state = self.state_dict['0']
        other_state = self.state_dict['1']
        
        ego_time = self._get_time_to_conflict(ego_state, '0')
        other_time = self._get_time_to_conflict(other_state, '1')
        print('主车距离冲突点时间', ego_time)
        print('对向车距离冲突点时间', other_time)
        
        v_ego = ego_state['v']
        safe_time_gap = 3.0
        time_diff = ego_time - other_time
        
        if abs(time_diff) < safe_time_gap:
            if time_diff > 0:
                target_acc = self.min_acc
            else:
                if v_ego < self.max_speed:
                    target_acc = self.max_acc
                else:
                    target_acc = 0.0
        else:
            if v_ego < self.max_speed:
                target_acc = min(self.comfort_acc, 
                               (self.max_speed - v_ego) / 0.1)
            else:
                target_acc = 0.0
        
        target_acc = np.clip(target_acc, self.min_acc, self.max_acc)
        
        return target_acc

    def _get_closest_point(self, vehicle_state, reference_line):
        """获取参考线上距离车辆最近的点"""
        x = reference_line['x']
        y = reference_line['y']
        distances = np.sqrt((x - vehicle_state['x'])**2 + (y - vehicle_state['y'])**2)
        closest_idx = np.argmin(distances)
        return closest_idx

    def _calculate_cost(self, acc, ego_state, other_state):
        """计算给定加速度的代价函数"""
        # 计算预测轨迹
        future_v = ego_state['v'] + acc * self.dt
        future_x = ego_state['x'] + ego_state['v'] * self.dt + 0.5 * acc * self.dt**2
        future_y = ego_state['y']  # 简化模型，假设y方向不变
        
        # 安全性代价
        distance_cost = max(0, self.safe_distance - 
                          np.sqrt((future_x - other_state['x'])**2 + 
                                 (future_y - other_state['y'])**2))
        
        # 舒适性代价
        comfort_cost = abs(acc) / self.comfort_acc
        
        # 速度跟随代价
        speed_cost = abs(future_v - other_state['v'])
        
        total_cost = (10.0 * distance_cost + 
                     1.0 * comfort_cost + 
                     0.5 * speed_cost)
        
        return total_cost 

    def plan_other_vehicle(self):
        """规划其他车辆的运动
        
        Returns:
            float: 加速度 (m/s²)
        """
        # 获取当前速度 (转换为 m/s)
        current_speed = self.state_dict['1']['v'] / 3.6
        target_speed = self.target_speed / 3.6
        
        # 计算到目标速度的速度差
        speed_diff = target_speed - current_speed
        
        # 根据速度差决定加速度
        if abs(speed_diff) < 0.1:  # 速度接近目标速度
            return 0.0  # 保持匀速
        elif speed_diff > 0:  # 需要加速
            return min(self.max_acc, speed_diff / 1.0)  # 1秒内达到目标速度
        else:  # 需要减速
            return max(self.max_dec, speed_diff / 1.0) 
