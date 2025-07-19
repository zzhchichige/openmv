#main.py -- put your code here!
import cpufreq
import pyb
import sensor,image, time,math
from pyb import LED,Timer,UART

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(20)     #延时跳过一些帧，等待感光元件变稳定
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()                # Create a clock object to track the FPS.
#sensor.set_auto_exposure(True, exposure_us=5000) # 设置自动曝光

uart=UART(3,256000)
THRESHOLD = (0,100) # Grayscale threshold for dark things... (5, 70, -23, 15, -57, 0)(18, 100, 31, -24, -21, 70)
IMAGE_WIDTH=sensor.snapshot().width()
IMAGE_HEIGHT=sensor.snapshot().height()
IMAGE_DIS_MAX=(int)(math.sqrt(IMAGE_WIDTH*IMAGE_WIDTH+IMAGE_HEIGHT*IMAGE_HEIGHT)/2)

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



thresholds_red =[(10, 100, 30, 127, -30, 127)]
thresholds_blue=[(15, 50, 40, 80, -127, 0)]
thresholds_r_b=[(10, 100, 30, 127, -30, 127),
                (0, 50, 0, 80, -127, 0)]

def opv_find_color_blobs():
    img=sensor.snapshot()
    target.flag=0
    target.img_width=IMAGE_WIDTH
    target.img_height=IMAGE_HEIGHT
    pixels_max=0
    type_id=0
    for b in img.find_blobs(thresholds_r_b,pixels_threshold=30,merge=True,margin=50):
        if b.code()==1:
            print('颜色为红色')
        else:
            print('颜色为蓝色')
         #density色块面积和外接矩形的比值
        if b.density()>0.8:#当检测的对象为矩形时,此值接近1
            type_id=2
            img.draw_rectangle(b.rect())
            print('矩形',b.cx(),b.cy(),b.density(),type_id)
        elif b.density()>0.65:#当检测的对象为圆形时,此值大于0.65
            type_id=1
            img.draw_circle((b.cx(), b.cy(),int((b.w()+b.h())/4)))
            print('圆形',b.cx(),b.cy(),b.density(),type_id)
        elif b.density()>0.40:#当检测的对象为三角形时,此值大于0.4
            type_id=3
            img.draw_cross(b.cx(), b.cy())
            print('三角形',b.cx(),b.cy(),b.density(),type_id)
        else: #忽略掉其它图形或者噪声干扰
            print("未识别")

        if pixels_max<b.pixels():
            pixels_max=b.pixels()
            target.x = b.cx()
            target.y = b.cy()
            target.pixel=pixels_max
            target.reserved1=b.w()>>8
            target.reserved2=b.w()
            target.reserved3=b.code()#颜色
            target.reserved4=type_id #形状
            target.flag = 1



thresholds_circle =[(0, 100, 30, 127,-30, 127)]
def opv_find_color_circle():
    img=sensor.snapshot().binary(thresholds_circle,invert=False)
    target.flag=0
    target.img_width=IMAGE_WIDTH
    target.img_height=IMAGE_HEIGHT
    pixels_max=0
    type_id=0
    # 圆形类有 4 个参数值： 圆心(x, y), r (半径)和 magnitude（量级）；
    # 量级越大说明识别到的圆可信度越高。
    # `threshold` 参数控制找到圆的数量，数值的提升会降低识别圆形的总数。
    # `x_margin`, `y_margin`, and `r_margin`控制检测到接近圆的合并调节.
    # r_min, r_max, and r_step 用于指定测试圆的半径范围。
    for b in img.find_circles(threshold = 5000, x_margin = 50, y_margin= 50, r_margin = 50,r_min = 10, r_max = 3000, r_step = 2):
        #画红色圆做指示
        img.draw_circle(b.x(), b.y(), b.r(), color = (255, 0, 0))
        print(b) #打印圆形的信息


ctr.work_mode=0x10
last_ticks=0
ticks=0
ticks_delta=0;
while True:
    clock.tick()
    opv_find_color_blobs()
    #opv_find_color_circle()
    #rgb.blue.toggle()
    uart.write(package_blobs_data(ctr.work_mode))
#    uart_data_read()
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
    #print(target.fps,ticks-last_ticks)







