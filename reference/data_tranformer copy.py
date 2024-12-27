import numpy as np
import Environment
from pyproj import Transformer

transformer = Transformer.from_crs("epsg:4326", "epsg:3857")
# 接受到data输入信息 将他转化为协同算法需要的格式
class lon_lat_to_xy:
    def __init__(self, data1, data2, data3):
        self.data_list = [data1, data2, data3]
        self.veh_num = len(self.data_list)

    def get_pos(self):  # 协同算法需要的x y v 等信息
        x_list, y_list, v_list, heading_list = [], [], [], []
        for veh in range(self.veh_num):
            data = self.data_list[veh]
            ref_point = data.refpoints[0]
            x, y = transformer.transform(ref_point.latitude, ref_point.longitude)
            x, y = x - Environment.min_x_list, y - Environment.min_y_list  # x y坐标转化数值特别大 放在0-x 0-y的范围内
            x_list.append(x)
            y_list.append(y)
            v_list.append(ref_point.speed)
            heading_list.append(ref_point.courseAngle)
        return x_list, y_list, v_list, heading_list

    def get_direction(self):  # 协同算法需要的行驶方向信息 会对应到不同的参考轨迹(但都已经设为循迹线上点)
        direction_list = []
        x_list, y_list, v_list, heading_list = self.get_pos()
        for veh in range(self.veh_num):
            data = self.data_list[veh]
            ref_point = data.refpoints[0]  # 第一个点
            x, y = x_list[veh], y_list[veh]
            dis = np.sqrt((np.array(Environment.x_list) - x)**2 + (np.array(Environment.y_list) - y)**2)
            index_idx = np.argmin(dis)
            #
            # min_distance = 10000
            # index_idx2 = 0
            # print(Environment.x_list[0], x, Environment.y_list[0], y)
            # for point in range(len(Environment.x_list)):
            #     distance = np.sqrt((Environment.x_list[point] - x)**2 + (Environment.y_list[point] - y)**2)
            #     if distance < min_distance:
            #         min_distance = distance
            #         index_idx2 = point
            #
            # index_idx = 0
            # for point in range(len(Environment.lon_list)):
            #     index_idx += 1
            #     # dis = np.sqrt((Environment.x_list[index_idx] - x)**2 + (Environment.y_list[index_idx] - y)**2)
            #     if float(Environment.lon_list[point]) == float(ref_point.longitude) and float(Environment.lat_list[point]) == float(ref_point.latitude):
            #         break
            # print('index', index_idx, index_idx1, index_idx2)
            index = index_idx
            # print('index', index)
            if 8285 <= index <= 8702:  # 8702
                direction = 'w-gs'
            elif 6312 <= index <= 6845:
                direction = 'w-lt'
            elif 3164 <= index <= 3718:
                direction = 's-gs'
            elif 5345 <= index <= 5811:
                direction = 's-lt'
            elif 198 <= index <= 618:
                direction = 'e-gs'
            elif 2228 <= index <= 2698:
                direction = 'e-lt'
            elif 4503 <= index <= 4913:
                direction = 'n-gs'
            elif 7693 <= index <= 8097:
                direction = 'n-lt'
            else:
                direction =None
                raise ValueError('initial position must in certain index range! check data_transformer.py.lon_lat_to_xy.get_direction')
            # print('data1', index, direction, data.refpoints[1].latitude, data.refpoints[1].longitude)
            # print('data2', index, direction, data.refpoints[1].latitude, data.refpoints[1].longitude)
            # print('data3', index, direction, data.refpoints[1].latitude, data.refpoints[1].longitude)
            direction_list.append(direction)
        return direction_list

    def transform(self):
        x_list, y_list, v_list, heading_list = self.get_pos()
        direction_list = self.get_direction()
        state_list, aggressiveness_list = [], []
        for veh in range(self.veh_num):
            aggressiveness_list.append('cav')
            state = [x_list[veh], y_list[veh], v_list[veh], heading_list[veh], Environment.exact_dis_to_destination([x_list[veh], y_list[veh]], central_vertices=Environment.get_central_vertices(direction_list[veh]), destination=Environment.get_destination(direction_list[veh]))]
            state_list.append(state)
        return state_list, direction_list, aggressiveness_list

class planList_form:
    def __init__(self, header, id, timestamp, refpoints):
        self.header = header
        self.Id = id
        self.timestamp = timestamp
        self.refpoints = refpoints


class xy_to_lon_lat:
    def __init__(self, state_list, direction_list, aggressiveness_list, data_list):
        self.state_list = state_list
        self.direction_list = direction_list
        self.aggressiveness_list = aggressiveness_list
        self.data_list = data_list
        self.veh_num = len(self.state_list)

    def transform(self):
        data_from_simulator = []
        for veh in range(self.veh_num):
            data = self.data_list[veh]
            # print('now its veh:', veh, '/original speed', data.refpoints[0].speed, '/PG planning speed', self.state_list[veh][2])
            data.refpoints[0].speed = str(self.state_list[veh][2])  # 只改第一个参考点的速度信息 改为规划得到的速度
            # print('after', self.state_list[veh][2], self.state_list[veh])
            data.Id = str(veh)
            # if veh == 0:
            #     data.refpoints[0].speed = str(5)
                # action = planList_form(None, veh, data.timestamp, data.refpoints)
            # action = planList_form(veh, data.timestamp + 1, data.refpoints)
            data_from_simulator.append(data)
        return data_from_simulator




