#main.py -- put your code here!
import cpufreq
import pyb
import sensor,image, time,math
from pyb import LED,Timer,UART
import  os, ml, uos, gc
from ulab import numpy as np

import sensor, image, time, math, struct
import json
from pyb import LED,Timer
import find_a,find_start_point,find_pole,utils,find_pole
import find_code
import Message
import find_line
import video
import mjpeg, pyb



sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     #延时跳过一些帧，等待感光元件变稳定
sensor.set_auto_gain(True)          #黑线不易识别时，将此处写False
sensor.set_auto_whitebal(False)
clock = time.clock()                # Create a clock object to track the FPS.
#sensor.set_auto_exposure(True, exposure_us=5000) # 设置自动曝光sensor.get_exposure_us()

startPoint_threshold =(2, 26, -33, 16, -22, 31)
CROSS_MIN=10
CROSS_MAX=90
uart=UART(3,256000)
THRESHOLD = (0,100) # Grayscale threshold for dark things... (5, 70, -23, 15, -57, 0)(18, 100, 31, -24, -21, 70)
IMAGE_WIDTH=sensor.snapshot().width()
IMAGE_HEIGHT=sensor.snapshot().height()
IMAGE_DIS_MAX=(int)(math.sqrt(IMAGE_WIDTH*IMAGE_WIDTH+IMAGE_HEIGHT*IMAGE_HEIGHT)/2)
# ==================== 模型加载 ====================
net = None
labels = None
min_confidence = 0.5                    # 置信度阈值

try:
    # 动态判断是否加载到帧缓冲（内存不足时自动回退）
    net = ml.Model("trained.tflite",
                  load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    raise Exception('模型加载失败: ' + str(e))

try:
    with open("labels.txt", "r") as f:
        labels = [line.rstrip('\n') for line in f]  # 读取标签文件
except Exception as e:
    raise Exception('标签加载失败: ' + str(e))

# ==================== 可视化配置 ====================a
colors = [                              # 10种预定义颜色（按类别分配）
    (255, 0, 0),    # 红
    (0, 255, 0),    # 绿
    (255, 255, 0),  # 黄
    (0, 0, 255),    # 蓝
    (255, 0, 255),  # 紫
    (0, 255, 255),  # 青
    (255, 128, 0),  # 橙
    (128, 0, 255),  # 紫红
    (0, 128, 255),  # 天蓝
    (255, 255, 255) # 白
]
# ==================== FOMO后处理函数 ====================
def fomo_post_process(model, inputs, outputs):
    """将模型输出转换为检测框坐标"""
    ob, oh, ow, oc = model.output_shape[0]          # 获取输出特征图的形状（批量、行、列、通道数）
    x_scale = inputs[0].roi[2] / ow                 # X轴缩放比例
    y_scale = inputs[0].roi[3] / oh                 # Y轴缩放比例
    scale = min(x_scale, y_scale)                   # 等比例缩放因子
    x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]  # X偏移
    y_offset = ((inputs[0].roi[3] - (oh * scale)) / 2) + inputs[0].roi[1]   # Y偏移

    detection_lists = [[] for _ in range(oc)]       # 初始化检测结果列表

    for channel in range(oc):
        # 将特征图转为灰度图像
        channel_img = image.Image(outputs[0][0, :, :, channel] * 255)

        # 查找高置信度区域
        blobs = channel_img.find_blobs(
            [(math.ceil(min_confidence * 255), 255)],
            x_stride=1, y_stride=1,
            area_threshold=1,
            pixels_threshold=1
        )

        for blob in blobs:
            x, y, w, h = blob.rect()
            score = channel_img.get_statistics(
                thresholds=[(math.ceil(min_confidence * 255), 255)],
                roi=blob.rect()
            ).l_mean() / 255.0  # 计算置信度

            # 坐标映射回原图
            detection_lists[channel].append((
                int((x * scale) + x_offset),
                int((y * scale) + y_offset),
                int(w * scale),
                int(h * scale),
                score
            ))

    return detection_lists





class target_check(object):
    x=0          #int16_t
    y=0          #int16_t
    pixel=0      #uint16_t
    flag=0       #uint8_t
    state=0      #uint8_t
    angle=0      #int16_t
    distance=0   #uint16_t
    apriltag_id=0#uint16_t
    img_width=0  #uint16_t
    img_height=0 #uint16_t
    reserved1=0  #uint8_t
    reserved2=0  #uint8_t
    reserved3=0  #uint8_t
    reserved4=0  #uint8_t
    fps=0        #uint8_t
    range_sensor1=0
    range_sensor2=0
    range_sensor3=0
    range_sensor4=0
    camera_id=0
    reserved1_int32=0
    reserved2_int32=0
    reserved3_int32=0
    reserved4_int32=0

class rgb(object):
    def __init__(self):
        self.red=LED(1)
        self.green=LED(2)
        self.blue=LED(3)



class uart_buf_prase(object):
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0

class mode_ctrl(object):
    work_mode = 0x01 #工作模式.默认是点检测，可以通过串口设置成其他模式
    check_show = 1   #开显示，在线调试时可以打开，离线使用请关闭，可提高计算速度

ctr=mode_ctrl()


rgb=rgb()
R=uart_buf_prase()
target=target_check();
target.camera_id=0x01
target.reserved1_int32=65536
target.reserved2_int32=105536
target.reserved3_int32=65537
target.reserved4_int32=105537

HEADER=[0xFF,0xFC]
MODE=[0xF1,0xF2,0xF3]
#__________________________________________________________________
def package_blobs_data(mode):
    #数据打包封装
    data=bytearray([HEADER[0],HEADER[1],0xA0+mode,0x00,
                   target.x>>8,target.x,        #将整形数据拆分成两个8位
                   target.y>>8,target.y,        #将整形数据拆分成两个8位
                   target.pixel>>8,target.pixel,#将整形数据拆分成两个8位
                   target.flag,                 #数据有效标志位
                   target.state,                #数据有效标志位
                   target.angle>>8,target.angle,#将整形数据拆分成两个8位
                   target.distance>>8,target.distance,#将整形数据拆分成两个8位
                   target.apriltag_id>>8,target.apriltag_id,#将整形数据拆分成两个8位
                   target.img_width>>8,target.img_width,    #将整形数据拆分成两个8位
                   target.img_height>>8,target.img_height,  #将整形数据拆分成两个8位
                   target.fps,      #数据有效标志位
                   target.reserved1,#数据有效标志位
                   target.reserved2,#数据有效标志位
                   target.reserved3,#数据有效标志位
                   target.reserved4,#数据有效标志位
                   target.range_sensor1>>8,target.range_sensor1,
                   target.range_sensor2>>8,target.range_sensor2,
                   target.range_sensor3>>8,target.range_sensor3,
                   target.range_sensor4>>8,target.range_sensor4,
                   target.camera_id,
                   target.reserved1_int32>>24&0xff,target.reserved1_int32>>16&0xff,
                   target.reserved1_int32>>8&0xff,target.reserved1_int32&0xff,
                   target.reserved2_int32>>24&0xff,target.reserved2_int32>>16&0xff,
                   target.reserved2_int32>>8&0xff,target.reserved2_int32&0xff,
                   target.reserved3_int32>>24&0xff,target.reserved3_int32>>16&0xff,
                   target.reserved3_int32>>8&0xff,target.reserved3_int32&0xff,
                   target.reserved4_int32>>24&0xff,target.reserved4_int32>>16&0xff,
                   target.reserved4_int32>>8&0xff,target.reserved4_int32&0xff,
                   0x00])
    #数据包的长度
    data_len=len(data)
    data[3]=data_len-5#有效数据的长度
    #和校验
    sum=0
    for i in range(0,data_len-1):
        sum=sum+data[i]
    data[data_len-1]=sum
    #返回打包好的数据
    return data
#__________________________________________________________________



#串口数据解析
def Receive_Anl(data_buf,num):
    #和校验
    sum = 0
    i = 0
    while i<(num-1):
        sum = sum + data_buf[i]
        i = i + 1
    sum = sum%256 #求余
    if sum != data_buf[num-1]:
        return
    #和校验通过
    if data_buf[2]==0xA0:
        #设置模块工作模式
        ctr.work_mode = data_buf[4]
        print(ctr.work_mode)
        print("Set work mode success!")

#__________________________________________________________________
def uart_data_prase(buf):
    if R.state==0 and buf==0xFF:#帧头1
        R.state=1
        R.uart_buf.append(buf)
    elif R.state==1 and buf==0xFE:#帧头2
        R.state=2
        R.uart_buf.append(buf)
    elif R.state==2 and buf<0xFF:#功能字
        R.state=3
        R.uart_buf.append(buf)
    elif R.state==3 and buf<50:#数据长度小于50
        R.state=4
        R._data_len=buf  #有效数据长度
        R._data_cnt=buf+5#总数据长度
        R.uart_buf.append(buf)
    elif R.state==4 and R._data_len>0:#存储对应长度数据
        R._data_len=R._data_len-1
        R.uart_buf.append(buf)
        if R._data_len==0:
            R.state=5
    elif R.state==5:
        R.uart_buf.append(buf)
        R.state=0
        Receive_Anl(R.uart_buf,R.uart_buf[3]+5)
#        print(R.uart_buf)
        R.uart_buf=[]#清空缓冲区，准备下次接收数据
    else:
        R.state=0
        R.uart_buf=[]#清空缓冲区，准备下次接收数据

#__________________________________________________________________



def uart_data_read():
    buf_len=uart.any()
    for i in range(0,buf_len):
        uart_data_prase(uart.readchar())



def time_callback(info):
    rgb.red.toggle()

timer=Timer(2,freq=4)
timer.callback(time_callback)


# 绘制水平线
def draw_hori_line(img, x0, x1, y, color):
    for x in range(x0, x1):
        img.set_pixel(x, y, color)
# 绘制竖直线
def draw_vec_line(img, x, y0, y1, color):
    for y in range(y0, y1):
        img.set_pixel(x, y, color)
# 绘制矩形
def draw_rect(img, x, y, w, h, color):
    draw_hori_line(img, x, x+w, y, color)
    draw_hori_line(img, x, x+w, y+h, color)
    draw_vec_line(img, x, y, y+h, color)
    draw_vec_line(img, x+w, y, y+h, color)


blob_threshold_rgb=[40, 100,30,127,0,127]#(L Min, L Max, A Min, A Max, B Min, B Max)
# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds_rgb = [(30, 100, 15, 127, 15, 127), # generic_red_thresholds
                  (30, 100, -64, -8, -32, 32), # generic_green_thresholds
                  (0, 30, 0, 64, -128, 0)]     # generic_blue_thresholds




#寻色块
def opv_find_color_blob():
    target.flag=0
    if (ctr.work_mode&0x01)!=0:
        img=sensor.snapshot()
        target.img_width=IMAGE_WIDTH
        target.img_height=IMAGE_HEIGHT
        pixels_max=0
        for b in img.find_blobs([blob_threshold_rgb],pixels_threshold=30,merge=True,margin=50):
            img.draw_rectangle(b[0:4])#圈出搜索到的目标
            if pixels_max<b.pixels():
                pixels_max=b.pixels()
                target.x = b.cx()
                target.y = b.cy()
                target.pixel=pixels_max
                target.reserved1=b.w()>>8
                target.reserved2=b.w()
                target.flag = 1
        if target.flag==1:
            img.draw_cross(target.x,target.y, color=127, size = 15)
            img.draw_circle(target.x,target.y, 15, color = 127)
#        print(target.x,target.y,target.pixel,target.reserved1,target.reserved2)


b=0
#寻Apriltag
def opv_find_april_tag():
    img=sensor.snapshot()
    target.img_width=IMAGE_WIDTH
    target.img_height=IMAGE_HEIGHT
    apriltag_area=0
    apriltag_dis=IMAGE_DIS_MAX
    target.flag = 0
    for tag in img.find_apriltags():  # defaults to TAG36H11 without "families".
        img.draw_rectangle(tag.rect, color=(255, 0, 0))

        b = tag.rect[2] / (abs(math.sin(tag.rotation)) + abs(math.cos(tag.rotation)))

        # 保存最大像素面积的apriltag信息
        apriltag_dis_tmp = math.sqrt((tag.cx - 80) ** 2 + (tag.cy - 60) ** 2)
        apriltag_area_tmp = tag.w * tag.h

        if apriltag_dis > apriltag_dis_tmp:
            apriltag_area = tag.w * tag.h
            target.x = tag.cx
            target.y = tag.cy
            target.apriltag_id = tag.id
            target.pixel = int(b * b)  # 使用int包裹即可
            apriltag_dis = apriltag_dis_tmp
            target.flag = 1

    if target.flag==1:
        img.draw_cross(target.x,target.y, color=127, size = 15)
#        img.draw_circle(target.x,target.y, 15, color = 127)
#    print(target.x,target.y,target.pixel,target.apriltag_id,apriltag_dis)



class singleline_check():
    rho_err = 0
    theta_err = 0
    state = 0


singleline = singleline_check()
THRESHOLD = (0,100) # Grayscale threshold for dark things
thresholds =(0, 30, -30, 30, -30, 30)
#找线
def found_line():
    target.img_width=IMAGE_WIDTH
    target.img_height=IMAGE_HEIGHT
    target.flag = 0
    #sensor.set_pixformat(sensor.GRAYSCALE)
    #img=sensor.snapshot().binary([THRESHOLD])
    img=sensor.snapshot()
    target.img_width =IMAGE_WIDTH
    target.img_height=IMAGE_HEIGHT
    pixels_max=0
    singleline.state = img.get_regression([thresholds],x_stride=2,y_stride=2,pixels_threshold=10,robust = True)
    if(singleline.state):
        singleline.rho_err = abs(singleline.state.rho())
        singleline.theta_err = singleline.state.theta()
        target.x=singleline.rho_err;
        target.angle=singleline.theta_err;
        target.flag = 1
    if target.flag==1:
        img.draw_line(singleline.state.line(), color = 127)



#crops_threshold=(40, 70, -30, 10, -15, 30)
crops_threshold=(40, 70, -40, 0, 0, 40)
def find_crops():
    target.flag=0
    if (ctr.work_mode&0x01)!=0:
        img=sensor.snapshot()
        target.img_width=IMAGE_WIDTH
        target.img_height=IMAGE_HEIGHT
        pixels_max=0
        crops_roi=(int(target.img_width/2)-5,int(target.img_height/2)-5,10,10)#(x,y,w,h)
        for b in img.find_blobs([crops_threshold],roi=crops_roi,pixels_threshold=70,merge=True,margin=10):
            img.draw_rectangle(b[0:4])#圈出搜索到的目标
            if pixels_max<b.pixels():
                pixels_max=b.pixels()
                target.x = b.cx()
                target.y = b.cy()
                target.pixel=pixels_max
                target.reserved1=b.w()>>8
                target.reserved2=b.w()
                target.flag = 1
        img.draw_rectangle(crops_roi)
        if target.flag==1:
            img.draw_cross(target.x,target.y, color=127, size = 15)
            img.draw_circle(target.x,target.y, 15, color = 127)
        #print(target.x,target.y,target.pixel,target.flag,target.reserved1,target.reserved2)



def found_num():
    target.flag = 0
    target.img_width = IMAGE_WIDTH
    target.img_height = IMAGE_HEIGHT
    best_distance = IMAGE_DIS_MAX  # 初始化为最大距离

    img = sensor.snapshot()

    try:
        predictions = net.predict([img], callback=fomo_post_process)
    except Exception as e:
        print("预测错误:", e)
        return

    for class_id, detection_list in enumerate(predictions):
        if class_id == 0 or class_id >= len(labels):
            continue
        if not detection_list:
            continue

        for x, y, w, h, score in detection_list:
            center_x = x + w // 2
            center_y = y + h // 2
            distance = math.sqrt((center_x - IMAGE_WIDTH // 2) ** 2 + (center_y - IMAGE_HEIGHT // 2) ** 2)

            if distance < best_distance:
                best_distance = distance
                target.x = center_x
                target.y = center_y
                target.pixel = int(w * h)
                target.state = class_id-1  # 存数字类别（0~9）
                target.flag = 1

    if target.flag == 1:
        img.draw_circle((target.x, target.y, 15), color=(255, 255, 255), thickness=3)
        img.draw_string(target.x, target.y, "Num:%d" % target.state, color=(255, 255, 255), scale=2)



def find_crossShape(img, ROI):
    result = False
    result_point = (-1, -1)
    if ROI is None:
        return result, result_point
    lines = img.find_lines(roi=ROI, theta_margin=25, rho_margin=25)
    line_num = len(lines)
    for i in range(line_num - 1):
        for j in range(i, line_num):
            angle = utils.calculate_angle(lines[i], lines[j])
            if not (83 <= angle <= 90):
                continue
            intersect_pt = utils.CalculateIntersection(lines[i], lines[j])
            if intersect_pt is None:
                continue
            x, y = intersect_pt
            if not (0 <= x < utils.IMG_WIDTH and 0 <= y < utils.IMG_HEIGHT):
                continue
            result_point = (x, y)
            return True, result_point
    return False, result_point




def opv_find_cross_blob():
    img = sensor.snapshot()
    target.img_width = IMAGE_WIDTH
    target.img_height = IMAGE_HEIGHT
    target.flag = 0
    best_blob = None
    last_sub = 2.0

    blobs = img.find_blobs([startPoint_threshold], pixels_threshold=3, area_threshold=3, merge=True, margin=5)
    for blob in blobs:
        width = blob.w()
        height = blob.h()
        rate = width / height
        size_limit = width > CROSS_MIN and width < CROSS_MAX and height > CROSS_MIN and height < CROSS_MAX
        sub = abs(1.0 - rate)

        if last_sub > sub and size_limit:
            last_sub = sub
            best_blob = blob

    if best_blob is not None:
        cross_test_result, point = find_crossShape(img, best_blob.rect())
        if cross_test_result:
            img.draw_rectangle(best_blob.rect())
            img.draw_cross(point[0], point[1], 5, color=[0, 255, 0])
            target.flag = 1
            target.x = point[0]
            target.y = point[1]
            target.pixel = best_blob.pixels()


def find_AShape(img,blob):
    result=False
    if(blob==None):
        return result
    ROI=(blob.rect())
    lines=img.find_lines(roi=ROI,x_stride=1,y_stride=1, theta_margin = 25, rho_margin = 25)
    line_num = len(lines)
    for i in range(line_num -1):
            for j in range(i, line_num):
                # 判断两个直线之间的夹角是否为直角
                angle = utils.calculate_angle(lines[i], lines[j])
                # 判断角度是否在阈值范围内
                if not(angle >= 20 and angle <=  50):
                    continue#不在区间内
                intersect_pt = utils.CalculateIntersection(lines[i], lines[j])
                if intersect_pt is None:
                    continue
                #有交点
                x, y = intersect_pt
                #不在图像范围内
                if not(x >= 0 and x < utils.IMG_WIDTH and y >= 0 and y < utils.IMG_HEIGHT):
                    # 交点如果没有在画面中
                    continue
                result_point=(x,y)
                return True
    return result


Green_threshold=(36, 75, -79, -36, -12, 55)
A_threshold=(0, 30, -47, 0, 0, 39)
def opv_find_A_blob():
    img = sensor.snapshot()
    target.img_width = IMAGE_WIDTH
    target.img_height = IMAGE_HEIGHT
    target.flag = 0
    max_blob = None
    max_area = 0

    blobs = img.find_blobs([A_threshold], merge=True)
    for blob in blobs:
        width = blob.w()
        height = blob.h()
        short_side = min(width, height)
        long_side = max(width, height)
        rate = short_side / long_side
        area = short_side * long_side

        side_limit = (8 < short_side < 68) and (13 < long_side < 68)
        if side_limit and area > max_area and find_AShape(img, blob):
            max_area = area
            max_blob = blob

    if max_blob is not None:
        img.draw_rectangle(max_blob.rect())
        target.flag = 1
        target.x = max_blob.cx()
        target.y = max_blob.cy()
        target.pixel = max_blob.pixels()
        target.reserved1 = max_blob.w() >> 8
        target.reserved2 = max_blob.w()
        target.reserved3 = 1  # 可以用1代表A字颜色或类别
        target.reserved4 = 4  # 自定义4代表A字形状
        # 你可以在这里加绘制十字交点或其他
    else:
        target.flag = 0



thresholds_red =[(10, 100, 30, 127, -30, 127)]
thresholds_blue=[(15, 50, 40, 80, -127, 0)]
thresholds_r_b=[(10, 100, 30, 127, -30, 127),
                 (10, 50, 15, 40, -70, -40)]

def opv_find_color_blobs_max_only():
    img = sensor.snapshot()
    target.flag = 0
    target.img_width = IMAGE_WIDTH
    target.img_height = IMAGE_HEIGHT
    max_area = 0
    max_blob = None
    max_type_id = 0
    max_color_id = 0

    blobs = img.find_blobs(thresholds_r_b, pixels_threshold=50, merge=True, margin=10)

    for blob in blobs:
        type_id = 0
        color_id = 0

        # 颜色判断
        if blob.code() == 1:
            color_id = 1
        elif blob.code() == 2:
            color_id = 2
        else:
            continue

        # 圆形检测（优先）
        roi = (blob.x(), blob.y(), blob.w(), blob.h())
        circles = img.find_circles(
            roi=roi,
            threshold=1800,
            x_margin=20, y_margin=20, r_margin=20,
            r_min=2, r_max=100, r_step=2
        )

        found_circle = False
        for c in circles:
            if (blob.x() <= c.x() <= blob.x() + blob.w() and
                blob.y() <= c.y() <= blob.y() + blob.h()):
                type_id = 1
                found_circle = True
                break

        # 不是圆形时，判断矩形和三角形
        if not found_circle:
            dens = blob.density()
            if dens > 0.8:
                type_id = 2
            elif dens > 0.45:
                type_id = 3
            else:
                continue  # 忽略噪声

        # 找到最大面积目标
        area = blob.pixels()
        if area > max_area:
            max_area = area
            max_blob = blob
            max_type_id = type_id
            max_color_id = color_id

    # 如果检测到有效目标，则打印并更新target
    if max_blob:
        # 画图示
        if max_type_id == 1:
            img.draw_circle(max_blob.cx(), max_blob.cy(), int((max_blob.w() + max_blob.h()) / 4), color=(0, 255, 0))
            shape_str = "圆形"
        elif max_type_id == 2:
            img.draw_rectangle(max_blob.rect(), color=(255, 0, 0))
            shape_str = "矩形"
        elif max_type_id == 3:
            img.draw_cross(max_blob.cx(), max_blob.cy(), color=(0, 0, 255))
            shape_str = "三角形"
        else:
            shape_str = "未知"

        color_str = "红色" if max_color_id == 1 else "蓝色"

        print(f"颜色: {color_str}, 形状: {shape_str}, 中心:({max_blob.cx()},{max_blob.cy()}), 面积: {max_area:.0f}")

        target.x = max_blob.cx()
        target.y = max_blob.cy()
        target.pixel = max_area
        target.reserved3 = max_color_id
        target.reserved4 = max_type_id
        target.flag = 1
    else:
        print("未识别目标")
        target.flag = 0












ctr.work_mode=0x10
last_ticks=0
ticks=0
ticks_delta=0;
while True:
    clock.tick()

    if ctr.work_mode==0x00:#空闲模式
        opv_find_cross_blob()
        rgb.blue.toggle()

    elif ctr.work_mode==0x01:#色块模式
        opv_find_color_blob()
        rgb.blue.off()

    elif ctr.work_mode==0x02:#AprilTag模式
        opv_find_april_tag()
        rgb.blue.off()

    elif ctr.work_mode==0x03:#巡线模式
        found_line()
        rgb.blue.off()

    elif ctr.work_mode==0x04:#AprilTag模式
        opv_find_april_tag()
        rgb.blue.off()

    elif ctr.work_mode==0x05:#预留模式1,数字识别
        found_num()
        rgb.blue.toggle()

    elif ctr.work_mode==0x06:#预留模式2
        opv_find_A_blob()
        rgb.blue.toggle()
    elif ctr.work_mode==0x07:#识别底部颜色，用于2021年国赛植保飞行器
        find_crops()
        rgb.blue.toggle()
    elif ctr.work_mode==0x0B:#识别底部颜色，用于2021年国赛植保飞行器
        find_crops()
        rgb.blue.toggle()
    elif ctr.work_mode==0x10:
        opv_find_color_blobs_max_only()
        rgb.blue.toggle()
    else:
        rgb.blue.toggle()


    #uart.write(package_blobs_data(ctr.work_mode))
    data = package_blobs_data(ctr.work_mode)
    uart.write(data)
    print(' '.join('{:02X}'.format(b) for b in data))
    uart_data_read()
#__________________________________________________________________
    #计算fps
    last_ticks=ticks
    ticks=time.ticks_ms()#ticks=time.ticks_ms()
                      #新版本OPENMV固件使用time.ticks_ms()
                      #旧版本OPENMV固件使用time.ticks()
    ticks_delta=ticks-last_ticks
    if ticks_delta<1:
        ticks_delta=1
    target.fps=(int)(1000/ticks_delta)
    #target.fps = (int)(clock.fps())
#__________________________________________________________________
    #print(target.fps,ticks-last_ticks,ctr.work_mode)
