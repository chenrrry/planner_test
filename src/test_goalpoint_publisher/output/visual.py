
#  map_thin_1m_to_6pixel.png
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import numpy as np

whether_fit = True
def quaternion_to_euler(x, y, z,w):
    """
    将四元数(w, x, y, z)转换为欧拉角(roll, pitch, yaw)。
    """
    rotation = R.from_quat([x, y, z, w])
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)
    return roll, pitch, yaw

vehicleNode3D = []

# 打开图片
index = 2
image = Image.open(f"./src/test_goalpoint_publisher/data/TPCAP/TPCAP_{index}.png")
width, height = image.size

with open(f'./src/test_goalpoint_publisher/output/TPCAP_{index}_resultViz_0.txt', 'r') as file:
    # 读取整个文件内容
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
def convert_to_int(value):
    return round(float(value))

def calculate_position(x, y, scale=10):
    return convert_to_int(x) * scale, height - convert_to_int(y) * scale

def draw_point(draw, x, y, radius=8):
    left_up_point = (x - radius, y - radius)
    right_down_point = (x + radius, y + radius)
    draw.ellipse([left_up_point, right_down_point], fill='blue')

def process_type_1(draw, words, i):
    int_x, int_y = calculate_position(words[i+1], words[i+2])
    draw_point(draw, int_x, int_y)
    return i + 11  # 7 + 4

def process_type_2(words, i, vehicleNode3D):
    vehicle_x, vehicle_y = float(words[i+2]), float(words[i+3])
    quaternion = [float(words[j]) for j in range(i+5, i+9)]
    roll, pitch, yaw = quaternion_to_euler(*quaternion)
    vehicleNode3D.append([vehicle_x, vehicle_y, yaw])
    return i + 18  # 15 + 3

def process_type_3(draw, words, i):
    points = []
    for _ in range(4):
        point_x, point_y = calculate_position(words[i+1], words[i+2])
        points.append((point_x, point_y))
        i += 3
    draw.line(points + [points[0]], fill='black', width=2)
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

    # 可视化结果
    plt.figure()
    plt.plot(x, y, 'o', label='原始点')
    plt.plot(x_smooth, y_smooth, label='平滑路径')
    plt.quiver(x, y, np.cos(t), np.sin(t), color='r', scale=10, label='方向')
    plt.legend()
    plt.show()
