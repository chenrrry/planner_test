from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

# # 设置中文和负号正常显示
# plt.rcParams['font.sans-serif'] = ['SimHei']
# plt.rcParams['axes.unicode_minus'] = False


def quaternion_to_euler(x, y, z,w):
    """
    将四元数(w, x, y, z)转换为欧拉角(roll, pitch, yaw)。
    """
    rotation = R.from_quat([x, y, z, w])
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)
    return roll, pitch, yaw

vehicleNode3D = []

# 打开图片
index = 31
whether_fit = True
whtether_use_success_file = True
image = Image.open(f"./src/test_goalpoint_publisher/data/TPCAP/TPCAP_{index}.png")
width, height = image.size

if whtether_use_success_file:
    with open(f'./src/test_goalpoint_publisher/output/TPCAP_{index}_resultViz_0_success.txt', 'r') as file:
        content = file.read()
else:
    with open(f'./src/test_goalpoint_publisher/output/TPCAP_{index}_resultViz_0.txt', 'r') as file:
        content = file.read()

save_path = f"./src/test_goalpoint_publisher/trajectory/TPCAP_{index}_answer_trajectory.png"
if image.mode != 'RGBA':
    image = image.convert('RGBA')

image = image.resize((width*10, height*10))
width, height = image.size

# 创建一个可以在上面绘图的对象
tmp = Image.new('RGBA', image.size, (0, 0, 0, 0))
draw = ImageDraw.Draw(tmp)




# 使用split()按空白字符分割文件内容
words = content.split()


length=len(words)
flag=False

def calculate_position(x, y, scale=10):
    return float(x) * scale, height - float(y) * scale

def draw_point(draw, x, y, radius=8):
    left_up_point = (x - radius, y - radius)
    right_down_point = (x + radius, y + radius)
    draw.ellipse([left_up_point, right_down_point], fill='blue')

def process_type_1(draw, words, i):
    vehicle_x, vehicle_y = calculate_position(words[i+1], words[i+2])
    yaw = float(words[i+3])
    vehicleNode3D.append([vehicle_x, vehicle_y, yaw])
    return i + 11  

def process_type_2(words, i, vehicleNode3D):
    vehicle_x, vehicle_y = calculate_position(words[i+1], words[i+2])
    quaternion = [float(words[j]) for j in range(i+4, i+8)]
    # roll, pitch, yaw = quaternion_to_euler(*quaternion)
    # vehicleNode3D.append([vehicle_x, vehicle_y, yaw])
    return i + 15  

def process_type_3(draw, words, i):
    points = []
    i+=1
    for _ in range(4):
        point_x, point_y = calculate_position(words[i], words[i+1])
        points.append((point_x, point_y))
        i += 3
    draw.line(points + [points[0]], fill='blue', width=4)
    return i

i = 0
while i < length:
    if words[i] == "1":
        i = process_type_1(draw, words, i)
    elif words[i] == "2":
        i = process_type_2(words, i, vehicleNode3D)
    elif words[i] == "3":
        i = process_type_3(draw, words, i)

image = Image.alpha_composite(image, tmp)
# 保存修改后的图片
image.save(save_path)


if whether_fit:
    # 提取 x, y 和 t
    # imageBase = Image.open(save_path)
    imageBase = image
    # fig, ax = plt.subplots()
    # ax.imshow(imageBase)  # 在坐标轴上显示图片
    # plt.show()
    draw = ImageDraw.Draw(imageBase)
    x = [node[0] for node in vehicleNode3D]
    y = [node[1] for node in vehicleNode3D]
    t = [node[2] for node in vehicleNode3D]
    param = np.linspace(0, 1, len(x))
    cs_x = CubicSpline(param, x)
    cs_y = CubicSpline(param, y)

    # 生成平滑的路径
    param_smooth = np.linspace(0, 1, 300)
    x_smooth = cs_x(param_smooth)
    y_smooth = cs_y(param_smooth)

    # # 可视化结果
    # # ax.plot(x, y, 'o', label='original points')
    # ax.plot(x_smooth, y_smooth, label='path')
    # # ax.quiver(x, y, np.cos(t), np.sin(t), color='r', scale=10, label='direction')
    # ax.legend()
    # plt.savefig(f'./src/test_goalpoint_publisher/linePicture/TPCAP_{index}_forPaper.png')
    # plt.show()
    points = list(zip(x_smooth, y_smooth))
    draw.line(points, fill='red', width=10)  # 您可以选择线条的颜色和宽度
    imageBase.save(f'./src/test_goalpoint_publisher/linePicture/TPCAP_{index}_forPaper.png')
    imageBase.show()
