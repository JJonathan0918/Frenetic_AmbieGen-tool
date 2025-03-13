import numpy as np
import config as cf
from ambiegen.utils.frenet import frenet_to_cartesian_road_points_with_reframability_check
from ambiegen.utils.road_validity_check import is_valid_road
from ambiegen.utils.vehicle_evaluate import interpolate_road


class FrenetMap:
    """
    A new Map class to handle road generation in Frenet coordinates.
    """

    def __init__(self, map_size):
        self.map_size = map_size
        self.road_points = []
        self.scenario = []

    def generate_road(self, kappas, ds, lane_width):
        """
        Generates a road using Frenet coordinates.
        :param kappas: Curvature values
        :param ds: Step size along the road
        :param lane_width: Lane width for the road
        :return: List of road points in Cartesian coordinates
        """
        x, y, theta = self.map_size / 2, self.map_size / 2, 0.0  # Initial position
        road_points = []

        for kappa in kappas:
            # 计算每个曲率点
            delta_x = ds * np.cos(theta)
            delta_y = ds * np.sin(theta)
            x += delta_x
            y += delta_y
            theta += kappa * ds  # 更新方向

            # 保存当前点
            road_points.append((x, y, theta))

        self.road_points = road_points
        return road_points

    def get_points_from_frenet_scenario(self, scenario):
        """
        Converts a given Frenet scenario into a list of Cartesian points.
        :param scenario: A list of (action, length, curvature)
        :return: A list of road points
        """
        road_points = []
        for action, length, curvature in scenario:
            if action == 0:  # Go straight
                road_points.append((length, 0, 0))
            elif action == 1:  # Turn right
                road_points.append((0, -length, -curvature))
            elif action == 2:  # Turn left
                road_points.append((0, length, curvature))

        return road_points

    def is_valid_road(self, road_points):
        """
        Check if the generated road points form a valid road.
        :param road_points: List of road points in Cartesian coordinates
        :return: Boolean indicating if the road is valid
        """

        for x, y, _ in road_points:
            if not (0 <= x <= self.map_size and 0 <= y <= self.map_size):
                return False
        return True


