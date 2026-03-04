import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.ndimage import convolve

class ESDF:
    """
    Class for Distance field
    """

    # def __int__(self, resolution):
    #     self.resolution
    def create_obstacle_map(self, map_size, obstacles, resolution=0.5):
        min_x, max_x, min_y, max_y = map_size
        # resolution = 0.5  # Resolution of the obstacle map, adjust as needed
        width = int((max_x - min_x) / resolution)
        height = int((max_y - min_y) / resolution)

        obstacle_map = np.zeros((height, width), dtype=np.uint8)

        # Draw obstacles on the obstacle map
        for obstacle in obstacles:
            polygon = Polygon(obstacle)
            for i in range(height):
                for j in range(width):
                    x = (j+1) * resolution + min_x
                    y = (i+1) * resolution + min_y
                    if polygon.contains_point([x, y]):
                        obstacle_map[i, j] = 1

        return obstacle_map

    def generate_distance_field(self, obstacle_map):
        height, width = obstacle_map.shape
        distance_field = np.ones((height, width)) * np.inf
        queue = []

        # Initialize distance field with obstacle cells set to 0
        for i in range(height):
            for j in range(width):
                if obstacle_map[i, j] == 1:
                    distance_field[i, j] = 0
                    queue.append((i, j))

        # Breadth-first search to compute distance field
        while queue:
            cell = queue.pop(0)
            x, y = cell

            # Get neighbors of current cell
            neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]

            for neighbor in neighbors:
                nx, ny = neighbor

                # Check if neighbor is within bounds and unvisited
                if nx >= 0 and nx < height and ny >= 0 and ny < width and distance_field[nx, ny] == np.inf:
                    # Compute distance to neighbor
                    distance = distance_field[x, y] + 1

                    # Update distance field and enqueue neighbor
                    distance_field[nx, ny] = distance
                    queue.append((nx, ny))

        return distance_field

    def compute_gradient_field(self, distance_field):
        sobel_x = np.array([[-1, 0, 1],
                            [-2, 0, 2],
                            [-1, 0, 1]])

        sobel_y = np.array([[-1, -2, -1],
                            [0, 0, 0],
                            [1, 2, 1]])
        # sobel_x = np.array([[-1, -2, 0, 2, 1],
        #                     [-2, -4, 0, 4, 2],
        #                     [-3, -6, 0, 6, 3],
        #                     [-2, -4, 0, 4, 2],
        #                     [-1, -2, 0, 2, 1]])
        #
        # sobel_y = np.array([[-1, -2, -3, -2, -1],
        #                     [-2, -4, -6, -4, -2],
        #                     [0, 0, 0, 0, 0],
        #                     [2, 4, 6, 4, 2],
        #                     [1, 2, 3, 2, 1]])

        gradient_x = convolve(distance_field, sobel_x)
        gradient_y = convolve(distance_field, sobel_y)

        gradient_magnitude = np.sqrt(gradient_x ** 2 + gradient_y ** 2)
        gradient_angle = np.arctan2(gradient_y, gradient_x)

        return gradient_magnitude, gradient_angle


def main():
    # 定义地图大小和障碍物
    map_size = [-2, 15, -2, 15]
    obstacles = [
        # [(1, 1), (2, 3), (4, 2), (3, 1)],
        # [(3, 4), (4, 6), (5, 5)],
        # [(5, 2), (7, 2), (6, 4), (5, 3)],
        [(1, 1), (1, 4), (12, 4), (12, 1)]

    ]

    esdf = ESDF()
    # 创建障碍物地图
    obstacle_map = esdf.create_obstacle_map(map_size, obstacles, resolution=0.1)

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
    # plt.show()

    # 计算梯度场
    gradient_magnitude, gradient_angle = esdf.compute_gradient_field(distance_field)

    # 显示梯度场
    plt.figure()
    plt.subplot(1, 2, 1)
    plt.imshow(gradient_magnitude, cmap='hot', origin='lower', extent=map_size)
    plt.title('Gradient Magnitude')
    plt.colorbar()

    plt.subplot(1, 2, 2)
    plt.imshow(gradient_angle, cmap='hsv', origin='lower', extent=map_size)
    plt.title('Gradient Angle')
    plt.colorbar()

    plt.show()


if __name__ == '__main__':
    main()

