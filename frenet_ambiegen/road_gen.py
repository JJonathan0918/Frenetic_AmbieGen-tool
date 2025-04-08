import math

import numpy as np

from frenet_ambiegen.road_validity_check import interpolation_distance, rounding_precision, smoothness, is_inside_map, \
    interpolate_test
from code_pipeline.tests_generation import min_num_nodes
from frenet_ambiegen.frenet_map import FrenetMap
from scipy.interpolate import splprep, splev
from numpy.ma import arange

from shapely.geometry import LineString
import frenet_ambiegen.config as cf
from frenet_ambiegen.vehicle import is_too_sharp


class RoadGen:
    def __init__(self, config):
        self.config = config

    def test_case_generate(self):
        """
        Generates a random road topology using Frenet coordinate system and FrenetMap class.
        Expands the road until it's invalid, and returns only the valid part of the road.
        """
        map_size = cf.model["map_size"]
        lane_width = cf.model["lane_width"]
        curvature_bound = 0.05

        ds = 3
        kappas_indices = []
        test_map = FrenetMap(map_size)
        valid_scenario = []
        last_valid_scenario = []

        while True:
            kappa = np.random.randint(-10, 10)
            kappas_indices.append(kappa)

            road_points = test_map.generate_road(
                kappas=np.array(kappas_indices) * curvature_bound / 10,
                ds=ds,
                lane_width=lane_width,
            )

            filtered_road_points = [[point[0], point[1]] for point in road_points]

            if len(filtered_road_points) >= 3 and not self.is_valid_road(filtered_road_points):
                valid_scenario = last_valid_scenario
                break

            last_valid_scenario = valid_scenario.copy()

            curvature = kappa * (curvature_bound / 10)
            segment_length = ds * 5
            angle_rad = curvature * segment_length
            angle_deg = abs(angle_rad) * (180 / math.pi)

            if kappa == 0:
                valid_scenario.append((0, ds * 10, 0))  # 直行
            elif kappa > 0:
                valid_scenario.append((2, ds * 10 , angle_deg))  # 左转
            else:
                valid_scenario.append((1, ds * 10 , angle_deg))  # 右转

        return self.states_to_dict(valid_scenario)

    def states_to_dict(self, scenario):
        """
        Convert generated road scenario into a dictionary format that matches the previous structure.
        """
        state_dict = {}
        action_map = {0: "straight", 1: "right", 2: "left"}

        for i, state in enumerate(scenario):
            # 判断是否为直行
            if state[0] == 0:  # straight
                value = state[1]
            else:
                value = state[2]

            state_dict[f"st{i}"] = {
                "state": action_map[state[0]],
                "value": value
            }
            # print(state_dict)

        return state_dict

    def interpolate_test(self,the_test):
        """
            Interpolate the road points using cubic splines and ensure we handle 4F tuples for compatibility
        """
        old_x_vals = [t[0] for t in the_test]
        old_y_vals = [t[1] for t in the_test]

        # This is an approximation based on whatever input is given
        test_road_lenght = LineString([(t[0], t[1]) for t in the_test]).length
        num_nodes = int(test_road_lenght / interpolation_distance)
        if num_nodes < min_num_nodes:
            num_nodes = min_num_nodes

        assert len(old_x_vals) >= 2, "You need at leas two road points to define a road"
        assert len(old_y_vals) >= 2, "You need at leas two road points to define a road"

        if len(old_x_vals) == 2:
            # With two points the only option is a straight segment
            k = 1
        elif len(old_x_vals) == 3:
            # With three points we use an arc, using linear interpolation will result in invalid road tests
            k = 2
        else:
            # Otheriwse, use cubic splines
            k = 3

        pos_tck, pos_u = splprep([old_x_vals, old_y_vals], s=smoothness, k=k)

        step_size = 1 / num_nodes
        unew = arange(0, 1 + step_size, step_size)

        new_x_vals, new_y_vals = splev(unew, pos_tck)

        # Return the 4-tuple with default z and defatul road width
        return list(zip([round(v, rounding_precision) for v in new_x_vals],
                        [round(v, rounding_precision) for v in new_y_vals],
                        [-28.0 for v in new_x_vals],
                        [8.0 for v in new_x_vals]))

    def calculate_curvature(self,x, y):
        """
        基于导数的曲率计算（适用于密集点）。
        :param x: x坐标列表
        :param y: y坐标列表
        :return: 曲率列表
        """
        dx = np.gradient(x)
        dy = np.gradient(y)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        curvature = (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** 1.5
        # print(curvature)
        return curvature

    def is_too_sharp(self,the_test, max_curvature=0.2):
        """
        基于导数的急弯检测。
        :param the_test: 道路点列表 [(x1, y1), (x2, y2), ...]
        :param max_curvature: 最大允许曲率
        :return: True 如果存在急弯，否则 False
        """
        x = [p[0] for p in the_test]
        y = [p[1] for p in the_test]
        curvature = self.calculate_curvature(x, y)
        return np.max(np.abs(curvature)) < max_curvature

    def is_valid_road(self,points):
        """
        If the road is not simple, or if the road is too sharp, or if the road has less than 3 points, or if
        the last point is not in range, then the road is invalid

        Args:
          points: a list of points that make up the road

        Returns:
          A boolean value.
        """

        in_range = is_inside_map(points, map_size=cf.model["map_size"])
        the_test = interpolate_test(points)

        road = LineString([(t[0], t[1]) for t in points])
        invalid = (
                (road.is_simple is False)
                # or (is_too_sharp(points) is True)
                or (in_range is False)
                or (is_too_sharp(the_test) is True)
                or (len(points) < 3)
        )
        return not (invalid)

    def find_circle(self,p1, p2, p3):
        """
        The function takes three points and returns the radius of the circle that passes through them

        Args:
          p1: the first point
          p2: the point that is the center of the circle
          p3: the point that is the furthest away from the line

        Returns:
          The radius of the circle.
        """
        temp = p2[0] * p2[0] + p2[1] * p2[1]
        bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
        cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
        det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

        if abs(det) < 1.0e-6:
            return np.inf

        # Center of circle
        cx = (bc * (p2[1] - p3[1]) - cd * (p1[1] - p2[1])) / det
        cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

        radius = np.sqrt((cx - p1[0]) ** 2 + (cy - p1[1]) ** 2)
        # print(radius)
        return radius

    def min_radius(self,x, w=5):
        """
        It takes a list of points (x) and a window size (w) and returns the minimum radius of curvature of
        the line segment defined by the points in the window

        Args:
          x: the x,y coordinates of the points
          w: window size. Defaults to 5

        Returns:
          The minimum radius of curvature of the road.
        """
        mr = np.inf
        nodes = x
        for i in range(len(nodes) - w):
            p1 = nodes[i]
            p2 = nodes[i + int((w - 1) / 2)]
            p3 = nodes[i + (w - 1)]
            radius = self.find_circle(p1, p2, p3)
            if radius < mr:
                mr = radius
        if mr == np.inf:
            mr = 0

        return mr * 3.280839895  # , mincurv

    def is_inside_map(self,points, map_size):
        """
        Check if all points are within the map boundaries.
        :param points: List of road points [(x1, y1), (x2, y2), ...]
        :param map_size: Tuple (width, height) of the map
        :return: True if all points are inside the map, False otherwise
        """
        for point in points:
            x, y = point[0], point[1]
            if not (0 <= x <= 200 and 0 <= y <= 200):
                return False
        return True