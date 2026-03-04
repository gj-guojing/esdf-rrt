# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
# from matplotlib.patches import Polygon
#
# # 障碍物顶点坐标
# obstacles = [
#     [(1, 1), (2, 3), (4, 2), (3, 1)],
#     [(3, 4), (4, 6), (5, 5)],
#     [(5, 2), (7, 2), (6, 4), (5, 3)]
# ]
#
# # 创建一个图形对象和一个坐标轴对象
# fig, ax = plt.subplots()
#
# # 绘制每个障碍物
# for obstacle in obstacles:
#     polygon = Polygon(obstacle, edgecolor='red', facecolor='blue')
#     ax.add_patch(polygon)
#
# # 设置坐标轴的范围
# ax.set_xlim(0, 8)
# ax.set_ylim(0, 8)
#
# # 添加标题和坐标轴标签
# ax.set_title('Obstacles')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
#
# # 显示图形
# plt.show()
#
#

# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.patches import Polygon
#
# def create_obstacle_map(map_size, obstacles):
#     min_x, max_x, min_y, max_y = map_size
#     resolution = 0.5  # Resolution of the obstacle map, adjust as needed
#     width = int((max_x - min_x) / resolution)
#     height = int((max_y - min_y) / resolution)
#
#     obstacle_map = np.zeros((height, width), dtype=np.uint8)
#
#     # Draw obstacles on the obstacle map
#     for obstacle in obstacles:
#         polygon = Polygon(obstacle)
#         for i in range(height):
#             for j in range(width):
#                 x = j * resolution + min_x
#                 y = i * resolution + min_y
#                 if polygon.contains_point([x, y]):
#                     obstacle_map[i, j] = 1
#
#     return obstacle_map
#
# # Define map size and obstacles
# map_size = [-2, 15, -2, 15]
# obstacles = [
#     [(1, 1), (2, 3), (4, 2), (3, 1)],
#     [(3, 4), (4, 6), (5, 5)],
#     [(5, 2), (7, 2), (6, 4), (5, 3)]
# ]
#
# # Create obstacle map
# obstacle_map = create_obstacle_map(map_size, obstacles)
#
# # Visualize the obstacle map
# plt.imshow(obstacle_map, cmap='gray', origin='lower', extent=map_size)
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Obstacle Map')
# plt.grid(True)
# plt.show()
#
#
# import numpy as np
# import queue
#
# def generate_distance_field(obstacle_map):
#     height, width = obstacle_map.shape
#     distance_field = np.ones((height, width)) * np.inf
#     queue = []
#
#     # Initialize distance field with obstacle cells set to 0
#     for i in range(height):
#         for j in range(width):
#             if obstacle_map[i, j] == 1:
#                 distance_field[i, j] = 0
#                 queue.append((i, j))
#
#     # Breadth-first search to compute distance field
#     while queue:
#         cell = queue.pop(0)
#         x, y = cell
#
#         # Get neighbors of current cell
#         neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
#
#         for neighbor in neighbors:
#             nx, ny = neighbor
#
#             # Check if neighbor is within bounds and unvisited
#             if nx >= 0 and nx < height and ny >= 0 and ny < width and distance_field[nx, ny] == np.inf:
#                 # Compute distance to neighbor
#                 distance = distance_field[x, y] + 1
#
#                 # Update distance field and enqueue neighbor
#                 distance_field[nx, ny] = distance
#                 queue.append((nx, ny))
#
#     return distance_field
#
#


# import matplotlib.pyplot as plt
# from matplotlib.patches import Polygon
# from matplotlib.widgets import PolygonSelector
#
# class ObstacleSelector:
#     def __init__(self):
#         self.obstacleList = []  # 存储障碍物
#         self.fig, self.ax = plt.subplots()
#         self.ax.set_xlim(-2, 15)
#         self.ax.set_ylim(-2, 15)
#         self.ax.set_title('Draw Obstacles')
#         self.ax.set_xlabel('X')
#         self.ax.set_ylabel('Y')
#
#         self.polygon_selector = PolygonSelector(self.ax, self.on_polygon_select)
#
#     def on_polygon_select(self, vertices):
#         polygon = Polygon(vertices, closed=True, edgecolor='red', facecolor='blue')
#         self.ax.add_patch(polygon)
#         self.obstacleList.append(vertices)
#
#         self.fig.canvas.draw()
#
#     def show_plot(self):
#         # 设置 x 轴的刻度间距为 1
#         plt.xticks(range(-2, 15, 1))
#         # 设置 y 轴的刻度间距为 1
#         plt.yticks(range(-2, 15, 1))
#
#         plt.grid(True)
#         plt.show()
#
# # 创建障碍物选择器
# selector = ObstacleSelector()
#
# # 显示绘图界面
# selector.show_plot()
#
# # 打印障碍物列表
# print(selector.obstacleList)

import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import convolve

def compute_gradient_field(distance_field):
    sobel_x = np.array([[-1, 0, 1],
                       [-2, 0, 2],
                       [-1, 0, 1]])

    sobel_y = np.array([[-1, -2, -1],
                       [0, 0, 0],
                       [1, 2, 1]])

    gradient_x = convolve(distance_field, sobel_x)
    gradient_y = convolve(distance_field, sobel_y)

    gradient_magnitude = np.sqrt(gradient_x**2 + gradient_y**2)
    gradient_angle = np.arctan2(gradient_y, gradient_x)

    return gradient_magnitude, gradient_angle

# 示例距离场
distance_field = np.array([[3, 4, 5],
                           [2, 1, 2],
                           [3, 2, 1]])

# 计算梯度场
gradient_magnitude, gradient_angle = compute_gradient_field(distance_field)

# 显示梯度场
plt.figure()
plt.subplot(1, 2, 1)
plt.imshow(gradient_magnitude, cmap='hot')
plt.title('Gradient Magnitude')
plt.colorbar()

plt.subplot(1, 2, 2)
plt.imshow(gradient_angle, cmap='hsv')
plt.title('Gradient Angle')
plt.colorbar()

plt.show()
