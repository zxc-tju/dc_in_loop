import numpy as np
import matplotlib.pyplot as plt
from planner import MotionPlanner
import pandas as pd
import matplotlib
from data_transformer import ReferenceLineParser

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False     # 用来正常显示负号

def load_reference_lines(csv_path):
    """从CSV文件加载参考线数据"""
    df = pd.read_csv(csv_path)
    
    # 按轨迹名称分组
    trajectories = {}
    for name in df['TrajectoryName'].unique():
        trajectory_data = df[df['TrajectoryName'] == name]
        trajectories[name] = {
            'x': trajectory_data['X'].values,
            'y': trajectory_data['Y'].values,
            'time': trajectory_data['Time'].values
        }
    
    return trajectories

def simulate_scenario():
    """模拟两车交叉路口场景"""
    # 加载并重采样参考线
    parser = ReferenceLineParser()
    reference_lines = parser.read_reference_lines('pre_map.csv')
    resampled_lines = parser.get_reference_line_segments(reference_lines, min_distance=1.0)
    
    print("\n=== 参考线信息 ===")
    for name, line in resampled_lines.items():
        print(f"\n{name}:")
        print(f"点数量: {len(line['x'])}")
        print(f"起点: ({line['x'][0]:.2f}, {line['y'][0]:.2f})")
        print(f"终点: ({line['x'][-1]:.2f}, {line['y'][-1]:.2f})")
        distances = [np.sqrt((line['x'][i+1] - line['x'][i])**2 + (line['y'][i+1] - line['y'][i])**2) 
                    for i in range(len(line['x'])-1)]
        print(f"点间距: 最小={min(distances):.2f}m, 最大={max(distances):.2f}m, 平均={np.mean(distances):.2f}m")
    
    # 初始化车辆状态和规划器
    state_dict = {
        '0': {'x': resampled_lines['Trajectory_100']['x'][0],
              'y': resampled_lines['Trajectory_100']['y'][0],
              'v': 15.0},
        '1': {'x': resampled_lines['Trajectory_101']['x'][0],
              'y': resampled_lines['Trajectory_101']['y'][0],
              'v': 1.0}
    }
    
    planner = MotionPlanner(state_dict, resampled_lines)
    print(f"\n冲突点: {planner.conflict_point}")
    
    # 存储轨迹和时间差
    ego_trajectory = {'x': [state_dict['0']['x']], 'y': [state_dict['0']['y']], 'v': [state_dict['0']['v']]}
    other_trajectory = {'x': [state_dict['1']['x']], 'y': [state_dict['1']['y']], 'v': [state_dict['1']['v']]}
    time_gaps = []  # 存储时间差
    
    # 记录车辆在参考线上的位置索引
    ego_idx = 0
    other_idx = 0
    
    # 仿真参数
    dt = 0.1  # 时间步长
    max_time = 20.0  # 最大仿真时间
    
    # 记录最小距离和状态
    ego_min_dist = float('inf')
    other_min_dist = float('inf')
    ego_approaching = False
    other_approaching = False
    last_ego_dist = float('inf')
    last_other_dist = float('inf')
    
    # 开始仿真
    for t in np.arange(0, max_time, dt):
        # 计算到冲突点的实际距离
        ego_dist = np.sqrt(
            (state_dict['0']['x'] - planner.conflict_point[0])**2 +
            (state_dict['0']['y'] - planner.conflict_point[1])**2
        )
        other_dist = np.sqrt(
            (state_dict['1']['x'] - planner.conflict_point[0])**2 +
            (state_dict['1']['y'] - planner.conflict_point[1])**2
        )
        
        if t % 1.0 < dt:
            print(f"\nEgo距离变化: {last_ego_dist:.2f} -> {ego_dist:.2f}")
            print(f"Other距离变化: {last_other_dist:.2f} -> {other_dist:.2f}")
        
        # 检查是否通过冲突点
        if ego_dist > last_ego_dist and last_ego_dist < 5.0:
            print(f"\n=== 仿真结束：Ego车已通过冲突点 ===")
            print(f"时间: {t:.1f}s")
            print(f"最后距离: {ego_dist:.2f}m")
            break
            
        if other_dist > last_other_dist and last_other_dist < 5.0:
            print(f"\n=== 仿真结束：对向车已通过冲突点 ===")
            print(f"时间: {t:.1f}s")
            print(f"最后距离: {other_dist:.2f}m")
            break
        
        # 更新上一次的距离
        last_ego_dist = ego_dist
        last_other_dist = other_dist
        
        if t % 1.0 < dt:  # 每秒只打印一次
            print(f"\n=== 时间 {t:.1f}s ===")
            print(f"Ego车: 位置({state_dict['0']['x']:.2f}, {state_dict['0']['y']:.2f}), "
                  f"速度{state_dict['0']['v']:.1f}km/h, 加速度{planner.plan_ego_vehicle():.2f}m/s²")
            print(f"对向车: 位置({state_dict['1']['x']:.2f}, {state_dict['1']['y']:.2f}), "
                  f"速度{state_dict['1']['v']:.1f}km/h, 加速度{planner.plan_other_vehicle():.2f}m/s²")
        
        # 获取ego车辆的控制量
        acc_ego = planner.plan_ego_vehicle()
        # 获取对向车的控制量
        acc_other = planner.plan_other_vehicle()
        
        # 计算并存储时间差
        ego_time = planner._get_time_to_conflict(state_dict['0'], '0')
        other_time = planner._get_time_to_conflict(state_dict['1'], '1')
        time_gaps.append(ego_time - other_time)
        
        # 更新ego车辆状态
        state_dict['0']['v'] = max(0, state_dict['0']['v'] + acc_ego * dt * 3.6)  # 转换为km/h
        dx = state_dict['0']['v'] * dt / 3.6  # 转换回m/s进行位移计算
        
        # 更新对向车状态
        state_dict['1']['v'] = max(0, state_dict['1']['v'] + acc_other * dt * 3.6)  # 转换为km/h
        dx_other = state_dict['1']['v'] * dt / 3.6  # 转换为m/s进行位移计算
        
        # 记录更新前的索引
        old_ego_idx = ego_idx
        old_other_idx = other_idx
        
        # 更新ego车辆位置
        ego_line = resampled_lines['Trajectory_100']
        if ego_idx < len(ego_line['x']) - 1:
            # 计算当前位置到下一点的距离
            current_dist = np.sqrt(
                (ego_line['x'][ego_idx+1] - state_dict['0']['x'])**2 +
                (ego_line['y'][ego_idx+1] - state_dict['0']['y'])**2
            )
            
            # 如果移动距离大于到下一个点的距离，更新索引
            if dx >= current_dist:
                ego_idx += 1
                state_dict['0']['x'] = ego_line['x'][ego_idx]
                state_dict['0']['y'] = ego_line['y'][ego_idx]
            else:
                # 计算移动比例
                ratio = dx / current_dist
                state_dict['0']['x'] = state_dict['0']['x'] + \
                    ratio * (ego_line['x'][ego_idx+1] - state_dict['0']['x'])
                state_dict['0']['y'] = state_dict['0']['y'] + \
                    ratio * (ego_line['y'][ego_idx+1] - state_dict['0']['y'])
        
        # 更新other车辆位置
        other_line = resampled_lines['Trajectory_101']
        if other_idx < len(other_line['x']) - 1:
            # 计算当前位置到下一个点的距离
            current_dist = np.sqrt(
                (other_line['x'][other_idx+1] - state_dict['1']['x'])**2 +
                (other_line['y'][other_idx+1] - state_dict['1']['y'])**2
            )
            
            # 如果移动距离大于到下一个点的距离，更新索引
            if dx_other >= current_dist:
                other_idx += 1
                state_dict['1']['x'] = other_line['x'][other_idx]
                state_dict['1']['y'] = other_line['y'][other_idx]
            else:
                # 计算移动比例
                ratio = dx_other / current_dist
                state_dict['1']['x'] = state_dict['1']['x'] + \
                    ratio * (other_line['x'][other_idx+1] - state_dict['1']['x'])
                state_dict['1']['y'] = state_dict['1']['y'] + \
                    ratio * (other_line['y'][other_idx+1] - state_dict['1']['y'])
        
        # 获取并打印车辆到冲突点的距离和时间
        ego_dist = planner._get_distance_to_conflict(state_dict['0'], '0')
        other_dist = planner._get_distance_to_conflict(state_dict['1'], '1')
        ego_time = ego_dist / (state_dict['0']['v'] / 3.6) if state_dict['0']['v'] > 0 else float('inf')
        other_time = other_dist / (state_dict['1']['v'] / 3.6) if state_dict['1']['v'] > 0 else float('inf')
        
        # 记录轨迹
        ego_trajectory['x'].append(state_dict['0']['x'])
        ego_trajectory['y'].append(state_dict['0']['y'])
        ego_trajectory['v'].append(state_dict['0']['v'])
        
        other_trajectory['x'].append(state_dict['1']['x'])
        other_trajectory['y'].append(state_dict['1']['y'])
        other_trajectory['v'].append(state_dict['1']['v'])
        
        # 更新最小距离
        ego_min_dist = min(ego_min_dist, ego_dist)
        other_min_dist = min(other_min_dist, other_dist)
    
    return ego_trajectory, other_trajectory, resampled_lines, planner.conflict_point, time_gaps

def visualize_results(ego_traj, other_traj, reference_lines, conflict_point, time_gaps):
    """可视化仿真结果"""
    # 创建2x2的子图布局
    fig = plt.figure(figsize=(16, 12))
    
    # 轨迹图 (左上)
    ax1 = fig.add_subplot(221)
    # 绘制参考线
    for name, traj in reference_lines.items():
        ax1.plot(traj['x'], traj['y'], '--', label=f'参考线 {name}')
    # 绘制实际轨迹
    ax1.plot(ego_traj['x'], ego_traj['y'], 'r-', label='ego轨迹')
    ax1.plot(other_traj['x'], other_traj['y'], 'b-', label='other轨迹')
    # 绘制冲突点
    if conflict_point:
        ax1.plot(conflict_point[0], conflict_point[1], 'ko', markersize=10, label='冲突点')
    ax1.set_title('交叉路口场景仿真')
    ax1.set_xlabel('X (米)')
    ax1.set_ylabel('Y (米)')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    
    # 时间差曲线 (右上)
    ax4 = fig.add_subplot(222)
    t_gaps = np.arange(len(time_gaps)) * 0.1
    ax4.plot(t_gaps, time_gaps, 'g-', label='到达时间差')
    ax4.axhline(y=3.0, color='r', linestyle='--', label='安全时间间隔')
    ax4.axhline(y=-3.0, color='r', linestyle='--')
    ax4.fill_between(t_gaps, -3.0, 3.0, color='r', alpha=0.1, label='危险区域')
    ax4.set_title('到达冲突点的时间差 (ego - other)')
    ax4.set_xlabel('时间 (秒)')
    ax4.set_ylabel('时间差 (秒)')
    ax4.legend()
    ax4.grid(True)
    
    # 速度曲线 (左下)
    ax2 = fig.add_subplot(223)
    t = np.arange(len(ego_traj['v'])) * 0.1
    ax2.plot(t, ego_traj['v'], 'r-', label='ego速度')
    ax2.plot(t, other_traj['v'], 'b-', label='other速度')
    ax2.set_title('车辆速度变化')
    ax2.set_xlabel('时间 (秒)')
    ax2.set_ylabel('速度 (km/h)')
    ax2.legend()
    ax2.grid(True)
    
    # 冲突点的距离曲线 (右下)
    ax3 = fig.add_subplot(224)
    ego_distances = []
    other_distances = []
    for i in range(len(ego_traj['x'])):
        ego_dist = np.sqrt(
            (ego_traj['x'][i] - conflict_point[0])**2 +
            (ego_traj['y'][i] - conflict_point[1])**2
        )
        other_dist = np.sqrt(
            (other_traj['x'][i] - conflict_point[0])**2 +
            (other_traj['y'][i] - conflict_point[1])**2
        )
        ego_distances.append(ego_dist)
        other_distances.append(other_dist)
    
    t_dist = np.arange(len(ego_distances)) * 0.1
    ax3.plot(t_dist, ego_distances, 'r-', label='ego到冲突点距离')
    ax3.plot(t_dist, other_distances, 'b-', label='other到冲突点距离')
    ax3.set_title('车辆到冲突点距离变化')
    ax3.set_xlabel('时间 (秒)')
    ax3.set_ylabel('距离 (米)')
    ax3.legend()
    ax3.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    ego_traj, other_traj, reference_lines, conflict_point, time_gaps = simulate_scenario()
    visualize_results(ego_traj, other_traj, reference_lines, conflict_point, time_gaps) 