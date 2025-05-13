
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
        ds = 3
        kappas_indices = []  # 记录曲率变化
        test_map = FrenetMap(map_size)  # 创建 FrenetMap
        valid_scenario = []  # 记录有效轨迹部分
        last_valid_scenario = []  # 记录最后的有效轨迹

        done = False
        while not done:
            # 生成新的随机曲率
            kappa = np.random.randint(-10, 10)
            kappas_indices.append(kappa)

            # 生成道路点
            road_points = test_map.generate_road(
                kappas=np.array(kappas_indices) * curvature_bound/10,  # 曲率缩放
                ds=ds,
                lane_width=lane_width,
            )

            filtered_road_points = [[point[0], point[1]] for point in road_points]

            # **前两个点不检查**
            if len(filtered_road_points) >= 3:
                if not is_valid_road(filtered_road_points):
                    # print("Invalid road detected! Keeping last valid scenario.")
                    valid_scenario = last_valid_scenario  # 回退到最后一个有效路段
                    # print(filtered_road_points)
                    break  # 退出 while 循环

            # **如果通过了检查，就更新最后的有效轨迹**
            last_valid_scenario = valid_scenario.copy()

            # 记录有效路径
            if kappa == 0:
                valid_scenario.append((0, ds * 10, 0))  # 直行
            elif kappa > 0:
                valid_scenario.append((2, ds * 10, kappa * 10))  # 左转
            else:
                valid_scenario.append((1, ds * 10, -kappa * 10))  # 右转

            # print(filtered_road_points)

            # print(valid_scenario)
        # **不再进行最终轨迹检查，直接返回**
        scenario = valid_scenario
        # print(scenario)
        # print(scenario)
        # print(scenario)
        valid_road = True  # 退出外部 while 循环

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
