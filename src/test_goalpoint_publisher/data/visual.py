
#  map_thin_1m_to_6pixel.png
from PIL import Image, ImageDraw

# 打开图片
index = 10
image = Image.open(f"./data/TPCAP/TPCAP_{index}.png")
width, height = image.size

with open(f'./data/TPACP{index}_result_0.txt', 'r') as file:
    # 读取整个文件内容
    content = file.read()

if image.mode != 'RGBA':
    image = image.convert('RGBA')

image = image.resize((width*10, height*10))
width, height = image.size

# 创建一个可以在上面绘图的对象
tmp = Image.new('RGBA', image.size, (0, 0, 0, 0))
draw = ImageDraw.Draw(tmp)




# 使用split()按空白字符分割文件内容
words = content.split()

# 遍历并打印每个分割后的部分
# for word in words:
#     print(word)

length=len(words)
# draw.point((5,5), fill='blue')
i=0
flag=False
while i<length:
    if words[i] == "1":
        int_x=round(float(words[i+1]))*10
        int_y=height-round(float(words[i+2]))*10
        i+=7
        # color_r=round(float(words[i])*256)
        # color_g=round(float(words[i+1])*256)
        # color_b=round(float(words[i+2])*256)
        # color_a=round(float(words[i+3])*256)
        draw.point((int_x,int_y), fill='blue')
        i+=4
        continue
    if words[i] == "2":
        int_x=round(float(words[i+1]))
        int_y=round(float(words[i+2]))
        int_z=round(float(words[i+3]))
        if int_x==0 & int_y==0 & int_z==0:
            i+=13
        i+=15
        flag=True
        continue
    if words[i] == "3":
        i+=1
        point1_x=round(float(words[i]))*10
        point1_y=height-round(float(words[i+1]))*10
        i+=3
        point2_x=round(float(words[i]))*10
        point2_y=height-round(float(words[i+1]))*10
        i+=3
        point3_x=round(float(words[i]))*10
        point3_y=height-round(float(words[i+1]))*10
        i+=3
        point4_x=round(float(words[i]))*10
        point4_y=height-round(float(words[i+1]))*10
        i+=3
        points = [(point1_x, point1_y), (point2_x, point2_y), (point3_x, point3_y), (point4_x, point4_y)]
        draw.line(points + [points[0]], fill='black', width=2)
        continue

image = Image.alpha_composite(image, tmp)
# 保存修改后的图片
image.save(f"./trajectory/TPCAP{index}_answer_trajectory.png")
