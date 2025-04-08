import numpy as np
import config as cf


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
        :return: A list of road points in Cartesian coordinates
        """
        x, y, theta = self.map_size / 2, self.map_size / 2, 0.0  # Initial position
        road_points = []

        for action, length, curvature in scenario:
            if action == 0:  # Go straight
                kappa = 0
            elif action == 1:  # Turn right
                kappa = -curvature
            elif action == 2:  # Turn left
                kappa = curvature

            # Generate points for this segment
            ds = 1.0  # Step size
            num_steps = int(length / ds)
            for _ in range(num_steps):
                delta_x = ds * np.cos(theta)
                delta_y = ds * np.sin(theta)
                x += delta_x
                y += delta_y
                theta += kappa * ds  # Update direction
                road_points.append((x, y, theta))
        # print(road_points)
        return road_points

    def is_valid_road(self, road_points):
        """
        Check if the generated road points form a valid road.
        :param road_points: List of road points in Cartesian coordinates
        :return: Boolean indicating if the road is valid
        """
        # Check if all points are within the map boundaries
        for x, y, _ in road_points:
            if not (0 <= x <= self.map_size and 0 <= y <= self.map_size):
                return False

        # Check for self-intersections
        def ccw(A, B, C):
            return (B[0] - A[0]) * (C[1] - A[1]) - (B[1] - A[1]) * (C[0] - A[0])

        def intersect(A, B, C, D):
            return (ccw(A, B, C) * ccw(A, B, D) < 0) and (ccw(C, D, A) * ccw(C, D, B) < 0)

        n = len(road_points)
        for i in range(n - 1):
            for j in range(i + 1, n - 1):
                A = (road_points[i][0], road_points[i][1])
                B = (road_points[i + 1][0], road_points[i + 1][1])
                C = (road_points[j][0], road_points[j][1])
                D = (road_points[j + 1][0], road_points[j + 1][1])
                if intersect(A, B, C, D):
                    return False  # Self-intersection detected

        return True


