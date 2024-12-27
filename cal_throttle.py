from typing import Tuple, Union, Literal
import numpy as np

VehicleType = Literal[0, 1]  # 0: BYD, 1: Hongqi

class ThrottleCalculator:
    """计算车辆油门/制动量的类
    
    支持两种车型：
    - BYD (id=0): 仅返回油门量
    - Hongqi (id=1): 返回油门和制动量
    """
    
    def __init__(self):
        # BYD车型的速度区间参数 (speed_range: (b, c))
        self.byd_params = {
            (0, 5): (23.818, -0.234),
            (5, 10): (20.691, 1.470),
            (10, 15): (22.603, 0.808),
            (15, 20): (21.855, 1.295),
            (20, 25): (20.890, 1.108),
            (25, 30): (20.998, 0.649),
            (30, float('inf')): (20.998, 0.649)  # 超过30的速度使用相同参数
        }
        
        # Hongqi车型的速度区间和加速度阈值参数
        self.hongqi_params = {
            (0, 5): {
                'threshold': 0,
                'positive': (14.66, -1.17),
                'negative': (7.67, 0.09)
            },
            (5, 10): {
                'threshold': -0.064,
                'positive': (13.88231351, 1.26666863),
                'negative': (10.49473419, 1.7106263)
            },
            (10, 15): {
                'threshold': -0.11,
                'positive': (16.37800552, 1.60090816),
                'negative': (10.62592526, 1.82545212)
            },
            (15, 20): {
                'threshold': -0.3,
                'positive': (16.45038338, 3.78120515),
                'negative': (12.8746911, 4.26309403)
            },
            (20, 25): {
                'threshold': -0.276,
                'positive': (17.17195437, 4.50331698),
                'negative': (11.53475351, 3.32977061)
            },
            (25, float('inf')): {  # 超过25的速度使用相同参数
                'threshold': -0.29,
                'positive': (18.44566754, 5.35752729),
                'negative': (14.96020176, 4.78524255)
            }
        }

    def _validate_inputs(self, speed: float, acc: float) -> None:
        """验证输入参数的有效性"""
        if speed < 0:
            raise ValueError("速度不能为负值")
        if not -2 <= acc <= 2:
            print("警告：加速度超出正常范围 [-2, 2]，结果可能不准确")

    def _get_speed_range(self, speed: float, ranges: dict) -> Tuple[float, float]:
        """根据速度获取对应的参数区间"""
        for (min_speed, max_speed), params in ranges.items():
            if min_speed <= speed < max_speed:
                return params
        return ranges[max(ranges.keys())]  # 返回最大区间的参数

    def calculate(self, speed: float, acc: float, vehicle_type: VehicleType = 0) -> float:
        """计算油门/制动量
        
        Args:
            speed: 当前速度 (km/h)
            acc: 目标加速度 (m/s²)
            vehicle_type: 车型 (0: BYD, 1: Hongqi)
            
        Returns:
            float: 油门/制动量
        
        Examples:
            >>> calculator = ThrottleCalculator()
            >>> calculator.calculate(10, 1.5, 0)  # BYD车型
            >>> calculator.calculate(15, -0.5, 1)  # 红旗车型
        """
        self._validate_inputs(speed, acc)
        
        if vehicle_type == 0:  # BYD
            b, c = self._get_speed_range(speed, self.byd_params)
            return b * acc + c
            
        elif vehicle_type == 1:  # Hongqi
            params = self._get_speed_range(speed, self.hongqi_params)
            threshold = params['threshold']
            b, c = params['positive'] if acc >= threshold else params['negative']
            return b * acc + c
            
        else:
            raise ValueError(f"不支持的车型: {vehicle_type}")

def main():
    """主函数用于测试"""
    try:
        speed = float(input("请输入速度值 (km/h): "))
        acc = float(input("请输入加速度值 (m/s²): "))
        
        calculator = ThrottleCalculator()
        
        # 计算两种车型的结果
        byd_result = calculator.calculate(speed, acc, 0)
        hongqi_result = calculator.calculate(speed, acc, 1)
        
        print(f"\nBYD油门量: {byd_result:.2f}")
        print(f"红旗油门/制动量: {hongqi_result:.2f}")
        
    except ValueError as e:
        print(f"错误: {e}")

if __name__ == "__main__":
    main()
