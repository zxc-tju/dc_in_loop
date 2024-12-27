import numpy as np

speed = float(input("请输入速度值"))
acc = float(input("请输入加速度值"))

def calculate_acc_to_thr(speed, acc, id = 0):
    """
    :param id: id是0的话，对应的车辆是比亚迪；id是1的话，对应的车辆是红旗
    :return:比亚迪是油门量；红旗是油门和刹车
    """
    if id == 0:
        if acc < -2 or acc > 2:
            print("加速度不在查询列表范围内，可能会出问题")
        if speed < 0:
            raise ValueError("速度不能小于0，请重新输入。")
        elif 0 <= speed <= 5:
            b = 23.818
            c = -0.234
        elif 5 < speed <= 10:
            b = 20.691
            c = 1.470
        elif 10 < speed <= 15:
            b = 22.603
            c = 0.808
        elif 15 < speed <= 20:
            b = 21.855
            c = 01.295
        elif 20 < speed <= 25:
            b = 20.890
            c = 1.108
        elif 25 < speed <= 30:
            b = 20.998
            c = 0.649
        else:
            b = 20.998
            c = 0.649
        thr = b * acc + c
        return thr

    elif id == 1:
        if acc < -2 or acc > 2:
            print("加速度不在查询列表范围内，可能会出问题")
        if speed < 0:
            raise ValueError("速度不能小于0，请重新输入。")
        elif 0 <= speed <= 5:
            if acc >= 0:
                b = 14.66
                c = -1.17
            else:
                b = 7.67
                c = 0.09
        elif 5 < speed <= 10:
            if acc >= -0.064:
                b, c = (13.88231351, 1.26666863)
            else:
                b, c = (10.49473419, 1.7106263)
        elif 10 < speed <= 15:
            if acc >= -0.11:
                b, c = (16.37800552, 1.60090816)
            else:
                b, c = (10.62592526, 1.82545212)
        elif 15 < speed <= 20:
            if acc >= -0.3:
                b, c = (16.45038338, 3.78120515)
            else:
                b, c = (12.8746911, 4.26309403)
        elif 20 < speed <= 25:
            if acc >= -0.276:
                b, c = (17.17195437, 4.50331698)
            else:
                b, c = (11.53475351, 3.32977061)
        else :
            if acc >= -0.29:
                b, c = (18.44566754, 5.35752729)
            else:
                b, c = (14.96020176, 4.78524255)
        thr = b * acc + c
        return thr

    else:
        print("完蛋了")
