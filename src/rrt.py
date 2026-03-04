"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

"""

import math
import random
import time
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from distance_field import ESDF
from obtacle_generater import obstacleList
show_animation = True


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:

        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])


    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
                 play_area=None,
                 robot_radius=0.0,
                 obstacle_map=None,
                 distance_field=None,
                 resolution=None,
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        play_area:stay inside this area [xmin,xmax,ymin,ymax]
        robot_radius: robot body modeled as circle with given radius

        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius

        # 距离场
        self.obstacle_map = obstacle_map
        self.distance_field = distance_field
        self.resolution = resolution
        # 碰撞检测时间统计
        self.checking_total_time = 0

    def planning(self, animation=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)
            # 记录程序开始时间
            start_time = time.time()

            if self.check_if_outside_play_area(new_node, self.play_area) and \
               self.check_collision_df(
                   new_node, self.robot_radius):
                self.node_list.append(new_node)

            # 记录程序结束时间
            end_time = time.time()
            # 计算程序运行时间
            runtime = end_time - start_time
            self.checking_total_time += runtime

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision_df(
                        final_node, self.robot_radius):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        # for (ox, oy, size) in self.obstacle_list:
        #     self.plot_circle(ox, oy, size)

        # 绘制每个障碍物
        for obstacle in self.obstacle_list:
            self.plot_poly(obstacle)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def plot_poly(poly, color="-b"):
        # 将顶点坐标分别提取到 x 和 y 列表中，并闭合多边形
        x = [point[0] for point in poly] + [poly[0][0]]
        y = [point[1] for point in poly] + [poly[0][1]]

        # 绘制多边形
        plt.plot(x, y, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok

    @staticmethod
    def check_collision(node, obstacleList, robot_radius):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size+robot_radius)**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def check_collision_poly(node, obstacleList, robot_radius):
        if node is None:
            return False

        for obstacle in obstacleList:
            polygon = Polygon(obstacle)

            for i in range(len(node.path_x)):
                point = Point(node.path_x[i], node.path_y[i])

                distance = polygon.exterior.distance(point)
                if distance <= robot_radius:
                    return False  # 发生碰撞

        return True  # 没有碰撞

    # # @staticmethod
    def check_collision_df(self, node, robot_radius):
        path_x = node.path_x
        path_y = node.path_y

        for i in range(len(path_x)):
            x = int((path_x[i]+2) / self.resolution)
            y = int((path_y[i]+2) / self.resolution)

            # Check if the current node is within the obstacle map
            if x < 0 or x >= self.obstacle_map.shape[1] or y < 0 or y >= self.obstacle_map.shape[0]:
                return False  # Out of bounds, collision

            # Check if the current node is within the obstacle
            if self.obstacle_map[y, x] == 1:
                return False  # Collision with obstacle

            # Check if the current node is too close to obstacles based on the distance field
            distance = self.distance_field[y, x] * self.resolution
            if distance <= robot_radius:
                return False  # Collision within robot radius

        return True  # Safe, no collision

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    def get_check_time(self):
        print("Collision Checking Total Time:", self.checking_total_time)

def main(gx=5, gy=7):
    print("start " + __file__)

    # ====Search Path with RRT====
    # obstacleList = [(5, 5, 1),
    #                 (3, 6, 2),
    #                 (3, 8, 2),
    #                 (3, 10, 2),
    #                 (7, 5, 2),
    #                 (9, 5, 2),
    #                 (8, 10, 1),
    #                 (1, 1, 0.5),
    #                 (3, 0, 1),
    #                 (0, 3, 1),
    #                 (-1, 5, 1),
    #                 (5, 0, 1),
    #                 ]  # [x, y, radius]

    # 定义地图大小和障碍物
    map_size = [-2, 15, -2, 15]

    # obstacleList = [[(0, 1), (0, 4), (12, 4), (12, 1)]]

    resolution = 0.1
    esdf = ESDF()

    # 创建障碍物地图
    obstacle_map = esdf.create_obstacle_map(map_size, obstacleList, resolution)

    # 生成欧式距离场
    distance_field = esdf.generate_distance_field(obstacle_map)

    # 可视化障碍物地图和距离场
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

    # 显示障碍物地图
    ax1.imshow(obstacle_map, cmap='gray', origin='lower', extent=map_size)
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_title('Obstacle Map')
    ax1.grid(True)

    # 显示距离场
    ax2.imshow(distance_field, cmap='hot', origin='lower', extent=map_size)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_title('Distance Field')
    ax2.grid(True)

    plt.tight_layout()
    plt.show()


    # Set Initial parameters
    rrt = RRT(
        start=[0, 0],
        goal=[gx, gy],
        rand_area=[-2, 15],
        obstacle_list=obstacleList,
        # play_area=[0, 10, 0, 14]
        robot_radius=0.8,
        obstacle_map=obstacle_map,
        distance_field=distance_field,
        resolution=resolution)
    path = rrt.planning(animation=show_animation)
    rrt.get_check_time()

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()
