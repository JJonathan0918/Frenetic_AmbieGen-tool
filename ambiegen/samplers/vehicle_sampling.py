
import logging as log
import numpy as np
from pymoo.core.sampling import Sampling
import config as cf
#from ambiegen.utils.vehicle import Car
from ambiegen.solutions import VehicleSolution
from ambiegen.utils.frenet import frenet_to_cartesian_road_points_with_reframability_check
from ambiegen.utils.map import FrenetMap
from ambiegen.utils.vehicle_evaluate import evaluate_scenario
from ambiegen.utils.vehicle_evaluate import interpolate_road
from ambiegen.utils.road_validity_check import is_valid_road


def generate_random_road():
    """
    Generates a random road topology using Frenet coordinate system and FrenetMap class.
    Expands the road until it's invalid, and returns only the valid part of the road.
    """

    map_size = cf.vehicle_env["map_size"]
    lane_width = cf.vehicle_env["lane_width"]
    curvature_bound = 0.1

    valid_road = False
    scenario = []


    while not valid_road:
        ds = 5
        kappas_indices = []  # 初始化曲率列表
        test_map = FrenetMap(map_size)  # 创建FrenetMap对象
        valid_scenario = []  # 记录有效的场景部分

        # 初始道路有效，继续扩展
        done = False
        while not done:
            # 随机生成一个曲率值
            kappa = np.random.randint(-10, 10)

            kappas_indices.append(kappa)

            # 使用Frenet坐标系生成路面点
            road_points = test_map.generate_road(
                kappas=np.array(kappas_indices) * curvature_bound / 10,  # 曲率缩放
                ds=ds,
                lane_width=lane_width
            )

            # 检查道路是否有效
            if test_map.is_valid_road(road_points):
                # 如果道路有效，记录有效部分
                if kappa == 0:
                    valid_scenario.append((0, ds * 10, 0))  # 直行
                elif kappa > 0:
                    valid_scenario.append((2, 0, kappa * 10))  # 左转
                else:
                    valid_scenario.append((1, 0, -kappa * 10))  # 右转
            else:
                # 道路无效时，停止添加曲率
                done = True

        # 一旦扩展完，检查有效的场景部分
        if valid_scenario:
            # 使用FrenetMap类去生成实际的道路坐标
            road_points_from_scenario = test_map.get_points_from_frenet_scenario(valid_scenario)
            # 确保去掉无效部分
            if test_map.is_valid_road(road_points_from_scenario):
                scenario = valid_scenario  # 设置有效的场景
                valid_road = True  # 道路有效时，跳出循环

    return scenario



class VehicleSampling(Sampling):

    """
    Module to generate the initial population

    returns: a tensor of candidate solutions
    """

    def _do(self, problem, n_samples, **kwargs):

        X = np.full((n_samples, 1), None, dtype=object)

        for i in range(n_samples):
            states = generate_random_road()
            s = VehicleSolution()
            s.states = states
            #s.fitness = fitness
            X[i, 0] = s

        log.debug("Initial population of %d solutions generated", n_samples)
        return X
