import xml.etree.ElementTree as ET
import pandas as pd
import matplotlib.pyplot as plt
import os
import json

'''last update 2024/6/24 fangshiyu'''

def parse_xosc(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    # 提取轨迹数据的列表
    trajectories = []

    # 查找所有 Trajectory 节点
    for trajectory in root.findall('.//Trajectory'):
        name = trajectory.get('name')

        # 获取轨迹的每个点的数据
        for vertex in trajectory.findall('.//Vertex'):
            time = vertex.get('time')
            position = vertex.find('.//WorldPosition')
            x = position.get('x')
            y = position.get('y')
            z = position.get('z')

            trajectories.append({
                'TrajectoryName': name,
                'Time': float(time),
                'X': float(x),
                'Y': float(y),
                'Z': float(z)
            })

    return trajectories


def save_to_csv(trajectories, output_file):
    df = pd.DataFrame(trajectories)
    df.to_csv(output_file, index=False)
    print(f'Trajectories saved to {output_file}')


def plot_trajectories(trajectories, ax):
    df = pd.DataFrame(trajectories)
    print(df.head())  # 检查 DataFrame 的内容
    # 绘制每条轨迹
    for trajectory_name, group in df.groupby('TrajectoryName'):
        label = trajectory_name
        plt.plot(group['X'], group['Y'], linestyle='-', label=label)
        # for p in range(10, 2000, 100):
        #     plt.scatter(group['X'][p], group['Y'][p], marker='o')
        # plt.plot([group['X'][850], group['X'][1200]], [group['Y'][850], group['Y'][1200]], linestyle='-', label=label)
        # plt.scatter(group['X'][850], group['Y'][850], marker='o')
        # plt.scatter(group['X'][1100], group['Y'][1100], marker='o')

    plt.title('Vehicle Trajectories')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend()
    plt.grid(True)
    plt.show()


def tongji_map(ax):
    with open('YYT_TJST_0507_unlimited.json', encoding='utf-8') as file:
        data = json.load(file)
    for road in range(len(data['road'])):
        for lane in data['road'][road]['lanes']:
            # print(road, data['road'][road]['points_tess'])
            x = [point[0] for point in lane['center_points_tess']]
            y = [point[1] for point in lane['center_points_tess']]
            plt.plot(x, y, color='b')
    for connector in range(len(data['connector'])):
        for link in data['connector'][connector]['links']:
            x = [point[0] for point in link['center_points_tess']]
            y = [point[1] for point in link['center_points_tess']]
            plt.plot(x, y, color='r')


# 使用示例
# xosc_file = './traj\主车.xosc'
# output_csv = 'pre_ego_traj.csv'
xosc_file = r'map\SC20241226210230LEBU904.xosc'
output_csv = r'map\processed\SC20241226210230LEBU904.csv'

fig, ax = plt.subplots(figsize=(10, 6))
# tongji_map(ax)
trajectories = parse_xosc(xosc_file)
plot_trajectories(trajectories, ax)
save_to_csv(trajectories, output_csv)
