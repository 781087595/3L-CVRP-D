import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def draw_packing_result(carriage_length, carriage_width, carriage_height, goods_dimensions, goods_positions, k):
    # 车厢尺寸
    # carriage_length = 12
    # carriage_width = 5
    # carriage_height = 6


    # # 货物尺寸列表，每个元素表示一个货物的长、宽、高
    # goods_dimensions = [
    #     (3, 2, 1),
    #     (5, 2, 2)
    # ]

    # # 货物位置列表，每个元素表示一个货物的左后下顶点坐标
    # goods_positions = [
    #     (0, -1, 0),
    #     (10,0, 0)
    # ]

    def create_box(position, dimensions, color):
        x, y, z = position
        dx, dy, dz = dimensions
        vertices = [
            (x, y, z),
            (x + dx, y, z),
            (x + dx, y + dy, z),
            (x, y + dy, z),
            (x, y, z + dz),
            (x + dx, y, z + dz),
            (x + dx, y + dy, z + dz),
            (x, y + dy, z + dz)
        ]
        faces = [
            (0, 1, 2, 3),
            (4, 5, 6, 7),
            (0, 1, 5, 4),
            (2, 3, 7, 6),
            (0, 3, 7, 4),
            (1, 2, 6, 5)
        ]
        return Poly3DCollection([[vertices[vertex] for vertex in face] for face in faces], facecolors=color, edgecolors='k', linewidths=1)


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制车厢
    carriage_box = create_box((0, 0, 0), (carriage_length, carriage_width, carriage_height), (0.678, 0.847, 0.902, 0.3))
    ax.add_collection3d(carriage_box)

    # 绘制货物
    for position, dimensions in zip(goods_positions, goods_dimensions):
        goods_box = create_box(position, dimensions, (1, 0.647, 0, 0.1))
        ax.add_collection3d(goods_box)

    # 设置坐标轴范围
    ax.set_xlim(0, carriage_length)
    ax.set_ylim(0, carriage_width)
    ax.set_zlim(0, carriage_height)

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()
    plt.savefig(f'卡车{k}.png', dpi=1000)