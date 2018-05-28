import RPi.GPIO as GPIO
import time
import string
import serial
import threading
from threading import Thread

#按键值定义
run_car  = '1'  #按键前
back_car = '2'  #按键后
left_car = '3'  #按键左
right_car = '4' #按键右
stop_car = '0'  #按键停
#状态值定义
enSTOP = 0
enRUN =1
enBACK = 2
enLEFT = 3
enRIGHT = 4
enTLEFT =5
enTRIGHT = 6

class Vehicle():
    def __init__(self, speed=30.):
        #Vehicle status variables
        self.on = True
        self.carspeed = speed
        self.carstatus = enSTOP
        self.infrared_avoid_value = ''
        self.distance = 0

        # Threading lock
        self.lock = threading.Lock()

        #设置GPIO口为BCM编码方式
        GPIO.setmode(GPIO.BCM)
        #忽略警告信息
        GPIO.setwarnings(False)
        #小车电机引脚定义
        self.IN1 = 20
        self.IN2 = 21
        self.IN3 = 19
        self.IN4 = 26
        self.ENA = 16
        self.ENB = 13
        #小车按键定义
        self.key = 8
        #超声波引脚定义
        self.EchoPin = 0
        self.TrigPin = 1
        #RGB三色灯引脚定义
        self.LED_R = 22
        self.LED_G = 27
        self.LED_B = 24
        #舵机引脚定义
        self.ServoPin = 23
        #红外避障引脚定义
        self.AvoidSensorLeft = 12
        self.AvoidSensorRight = 17
        #蜂鸣器引脚定义
        self.buzzer = 8
        #灭火电机引脚设置
        self.OutfirePin = 2
        # Camera servo pin configuration
        self.CamHorPin = 11
        self.CamVerPin = 9
        #循迹红外引脚定义
        #TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
        #      3                 5                  4                   18
        self.TrackSensorLeftPin1  =  3   #定义左边第一个循迹红外传感器引脚为3口
        self.TrackSensorLeftPin2  =  5   #定义左边第二个循迹红外传感器引脚为5口
        self.TrackSensorRightPin1 =  4   #定义右边第一个循迹红外传感器引脚为4口
        self.TrackSensorRightPin2 =  18  #定义右边第二个循迹红外传感器引脚为18口
        #光敏电阻引脚定义
        self.LdrSensorLeft = 7
        self.LdrSensorRight = 6
        # vehicle PWM pins
        GPIO.setup(self.ENA,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(self.IN1,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.IN2,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.ENB,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(self.IN3,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.IN4,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(self.key,GPIO.IN)
        GPIO.setup(self.buzzer,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(self.OutfirePin,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(self.EchoPin,GPIO.IN)
        GPIO.setup(self.TrigPin,GPIO.OUT)
        GPIO.setup(self.LED_R, GPIO.OUT)
        GPIO.setup(self.LED_G, GPIO.OUT)
        GPIO.setup(self.LED_B, GPIO.OUT)
        GPIO.setup(self.ServoPin, GPIO.OUT)
        GPIO.setup(self.CamHorPin, GPIO.OUT)
        GPIO.setup(self.CamVerPin, GPIO.OUT)
        GPIO.setup(self.AvoidSensorLeft,GPIO.IN)
        GPIO.setup(self.AvoidSensorRight,GPIO.IN)
        GPIO.setup(self.LdrSensorLeft,GPIO.IN)
        GPIO.setup(self.LdrSensorRight,GPIO.IN)
        GPIO.setup(self.TrackSensorLeftPin1,GPIO.IN)
        GPIO.setup(self.TrackSensorLeftPin2,GPIO.IN)
        GPIO.setup(self.TrackSensorRightPin1,GPIO.IN)
        GPIO.setup(self.TrackSensorRightPin2,GPIO.IN)
        #设置pwm引脚和频率为2000hz
        self.pwm_ENA = GPIO.PWM(self.ENA, 2000)
        self.pwm_ENB = GPIO.PWM(self.ENB, 2000)
        self.pwm_ENA.start(0)
        self.pwm_ENB.start(0)
        #设置舵机的频率和起始占空比
        self.pwm_servo = GPIO.PWM(self.ServoPin, 50)
        self.pwm_servo.start(0)
        self.cam_hor_servo = GPIO.PWM(self.CamHorPin, 50)
        self.cam_hor_servo.start(0)
        self.cam_ver_servo = GPIO.PWM(self.CamVerPin, 50)
        self.cam_ver_servo.start(0)
        self.pwm_rled = GPIO.PWM(self.LED_R, 1000)
        self.pwm_gled = GPIO.PWM(self.LED_G, 1000)
        self.pwm_bled = GPIO.PWM(self.LED_B, 1000)
        self.pwm_rled.start(0)
        self.pwm_gled.start(0)
        self.pwm_bled.start(0)
        # define the serial port parameters
        self.InputString = ''
        self.InputStringcache = ''.encode()
        self.NewLineReceived = 0
        self.StartBit = 0
        self.ser = serial.Serial("/dev/ttyAMA0", 9600, timeout = 0)
        print('serial.isOpen() =', self.ser.isOpen())

    def run(self, CarSpeedControl=None):
        if CarSpeedControl:
            self.carspeed = CarSpeedControl
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(self.carspeed)
        self.pwm_ENB.ChangeDutyCycle(self.carspeed)

    #小车后退
    def back(self,CarSpeedControl=None):
        if CarSpeedControl:
            self.carspeed = CarSpeedControl
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_ENA.ChangeDutyCycle(self.carspeed)
        self.pwm_ENB.ChangeDutyCycle(self.carspeed)

    #小车左转	
    def left(self,CarSpeedControl=None):
        if CarSpeedControl:
            self.carspeed = CarSpeedControl
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(self.carspeed)
        self.pwm_ENB.ChangeDutyCycle(self.carspeed)

    #小车右转
    def right(self,CarSpeedControl=None):
        if CarSpeedControl:
            self.carspeed = CarSpeedControl
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(self.carspeed)
        self.pwm_ENB.ChangeDutyCycle(self.carspeed)

    #小车原地左转
    def spin_left(self,CarSpeedControl=None):
        if CarSpeedControl:
            self.carspeed = CarSpeedControl
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(self.carspeed)
        self.pwm_ENB.ChangeDutyCycle(self.carspeed)

    #小车原地右转
    def spin_right(self,CarSpeedControl=None):
        if CarSpeedControl:
            self.carspeed = CarSpeedControl
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_ENA.ChangeDutyCycle(self.carspeed)
        self.pwm_ENB.ChangeDutyCycle(self.carspeed)

    #小车停止	
    def brake(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)

    #按键检测
    def key_scan(self):
        while GPIO.input(self.key):
            pass
        while not GPIO.input(self.key):
            time.sleep(0.01)
            if not GPIO.input(self.key):
                time.sleep(0.01)
                while not GPIO.input(self.key):
                    pass

    # ultra sonic distance measurement
    def Distance_test(self):
        # Flag to indicate whether distance updated or not
        distance_updated = 0
        while self.on:
            self.lock.acquire()
            print("get lock")
            GPIO.output(self.TrigPin,GPIO.HIGH)
            time.sleep(0.000015)
            GPIO.output(self.TrigPin,GPIO.LOW)
            t0 = time.time()
            while not GPIO.input(self.EchoPin):
                t_buff = time.time()
                distance_updated = 1
                if (t_buff-t0)>1:
                    distance_updated = 0
                    break
                pass
            t1 = time.time()
            while GPIO.input(self.EchoPin):
                t_buff = time.time()
                distance_updated = 1
                if (t_buff-t1)>1:
                    distance_updated = 0
                    break
                pass
            t2 = time.time()
            if distance_updated:
                self.distance = ((t2 - t1)* 340 / 2) * 100
            self.lock.release()
            time.sleep(1)
            if distance_updated:
                print("distance is %d " % self.distance)
            else:
                print("distance is not updated!")
#        return int(distance)

    #舵机旋转到指定角度
    def servo_appointed_detection(self, pos):
        for i in range(18):
            self.pwm_servo.ChangeDutyCycle(2.5 + 10 * pos/180)

#    #巡线测试
#    def tracking_test():
#        global infrared_track_value
#        #检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW
#        #未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH
#        TrackSensorLeftValue1  = GPIO.input(self.TrackSensorLeftPin1)
#        TrackSensorLeftValue2  = GPIO.input(self.TrackSensorLeftPin2)
#        TrackSensorRightValue1 = GPIO.input(self.TrackSensorRightPin1)
#        TrackSensorRightValue2 = GPIO.input(self.TrackSensorRightPin2)
#        infrared_track_value_list = ['0','0','0','0']
#        infrared_track_value_list[0] = str(1^ TrackSensorLeftValue1)
#        infrared_track_value_list[1] =str(1^ TrackSensorLeftValue2)
#        infrared_track_value_list[2] = str(1^ TrackSensorRightValue1)
#        infrared_track_value_list[3] = str(1^ TrackSensorRightValue2)
#        infrared_track_value = ''.join(infrared_track_value_list)
#        
#    
    #避障红外引脚测试
    def infrared_avoid(self):
        #遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
        #未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
        LeftSensorValue  = GPIO.input(self.AvoidSensorLeft)
        RightSensorValue = GPIO.input(self.AvoidSensorRight)
        if (LeftSensorValue == 0) and (RightSensorValue == 0):
            self.servo_appointed_detection(90)
        elif (LeftSensorValue == 1) and (RightSensorValue == 1):
            self.servo_appointed_detection(90)
        elif LeftSensorValue == 0:
            self.servo_appointed_detection(180)
        elif RightSensorValue == 0:
            self.servo_appointed_detection(0)
        infrared_avoid_value_list = ['0','0']
        infrared_avoid_value_list[0] = str(1 ^ LeftSensorValue)
        infrared_avoid_value_list[1] = str(1 ^ RightSensorValue)
        infrared_avoid_value = ''.join(infrared_avoid_value_list)
#        	
#    #寻光引脚测试
#    def follow_light_test():
#        global LDR_value
#        #遇到光线,寻光模块的指示灯灭,端口电平为HIGH
#        #未遇光线,寻光模块的指示灯亮,端口电平为LOW
#        LdrSersorLeftValue  = GPIO.input(LdrSensorLeft)
#        LdrSersorRightValue = GPIO.input(LdrSensorRight)  
#        LDR_value_list = ['0','0']
#        LDR_value_list[0] = str(LdrSersorLeftValue)
#        LDR_value_list[1] = str(LdrSersorRightValue)	
#        LDR_value = ''.join(LDR_value_list)
#    	
    #小车鸣笛
    def whistle(self):
        GPIO.output(self.buzzer, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(self.buzzer, GPIO.HIGH)
        time.sleep(0.001)

    #七彩灯亮指定颜色
    def color_led_pwm(self, iRed, iGreen, iBlue):
        v_red = (100*iRed)/255
        v_green = (100*iGreen)/255
        v_blue = (100*iBlue)/255
        self.pwm_rled.ChangeDutyCycle(v_red)
        self.pwm_gled.ChangeDutyCycle(v_green)
        self.pwm_bled.ChangeDutyCycle(v_blue)
        time.sleep(0.02)

    def serialEvent(self):
        def serial_data_parse():
#            ReturnTemp = ''
            #解析上位机发来的通用协议指令,并执行相应的动作
            #如:$1,0,0,0,0,0,0,0,0,0#    小车前进
            if len(self.InputString) > 17:
                if self.InputString[3] == '1':
                    self.carstatus = enTLEFT
                    print("Car is spinning left!")
                elif self.InputString[3] == '2':
                    self.carstatus = enTRIGHT
                    print("Car is spinning right!")
                else :
                    self.carstatus = enSTOP

                if self.InputString[5] == '1':
                   self.whistle()
                if self.InputString[7] == '1':
                    self.carspeed += 20
                if self.carspeed > 100:
                    self.carspeed = 100
                if self.InputString[7] == '2':
                   self.carspeed -= 20
                if self.carspeed < 20:
                    self.carspeed = 20
                if self.InputString[9] == '1':
                    self.servo_appointed_detection(180)
                if self.InputString[9] == '2':
                    self.servo_appointed_detection(0)
                if self.InputString[17] == '1':
                    self.servo_appointed_detection(90)
                if self.InputString[13] == '1':
                    self.color_led_pwm(255, 255, 255)
                if self.InputString[13] == '2':
                    self.color_led_pwm(255, 0, 0)
                if self.InputString[13] == '3':
                    self.color_led_pwm(0, 255, 0)
                if self.InputString[13] == '4':
                    self.color_led_pwm(0, 0, 255)
                if self.InputString[13] == '5':
                    self.color_led_pwm(0, 255, 255)
                if self.InputString[13] == '6':
                    self.color_led_pwm(255, 0, 255)
                if self.InputString[13] == '7':
                    self.color_led_pwm(255, 255, 0)
                if self.InputString[13] == '8':
                    self.color_led_pwm(0,0,0)

                if self.InputString[15] == '1':
                    GPIO.output(self.OutfirePin,not GPIO.input(self.OutfirePin) )
                    time.sleep(1)

            if self.carstatus != enTLEFT and self.carstatus != enTRIGHT:
                if self.InputString[1] == run_car:
                    self.carstatus = enRUN
                    print("Car is moving forward!")
                elif self.InputString[1] == back_car:
                    self.carstatus = enBACK
                    print("Car is moving backward!")
                elif self.InputString[1] == left_car:
                    self.carstatus = enLEFT
                    print("Car is turning left!")
                elif self.InputString[1] == right_car:
                    self.carstatus = enRIGHT
                    print("Car is turning right!")
                elif self.InputString[1] == stop_car:
                    self.carstatus = enSTOP
                    print("Car stopped!")
                else:
                    self.carstatus = enSTOP
                    print("Car stopped!")
            #采集的传感器数据串口回发给上位机显示				
#            distance = Distance_test()
#            ReturnTemp += "$0,0,0,0,0,0,0,0,0,0,0,"
#            ReturnTemp += str(int(distance))
#            ReturnTemp += "cm,8.4v#"		
#            NewLineReceived = 0
#            ser.write(ReturnTemp)
#            InputString.zfill(len(InputString))	
#
        while True:
            size = self.ser.inWaiting()
            if size == 0:
                break
            else:
                while size != 0:
                    serialdatabit = self.ser.read(1)
                    size -= 1
                    if serialdatabit == '$'.encode():
                        self.StartBit = 1
                    if self.StartBit == 1:
                        self.InputStringcache += serialdatabit
                    if self.StartBit == 1 and serialdatabit == '#'.encode():
                        self.NewLineReceived = 1
                        self.InputString = self.InputStringcache.decode()
                        self.InputStringcache = ''.encode()
                        self.StartBit = 0
                        size = 0
                        serial_data_parse()
                        print (self.InputString)

    def print_status(self, delay):
        while self.on:
            time.sleep(delay)
            print ("%s: %s" % ("Now it's", time.ctime(time.time())))
            print(self.distance)

    def stop(self):
        self.ser.close()
        self.pwm_ENA.stop()
        self.pwm_ENB.stop()
        self.pwm_rled.stop()
        self.pwm_gled.stop()
        self.pwm_bled.stop()
        self.pwm_servo.stop()
        GPIO.cleanup()

    def start(self):
        """
        start the vehicle main drive loop
        """
        t_ser = Thread(target=self.serialEvent)
        t_ser.setDaemon(True)

        t_actor = Thread(target=self.update)
        t_actor.setDaemon(True)

#        t = Thread(target=self.print_status, args=(2,))
        t = Thread(target=self.Distance_test)
        t.setDaemon(True)

        try:
            self.on = True
#            t_ser.start()
#            t_actor.start()
            t.start()
            while self.on:
#                self.distance = self.Distance_test()
                self.serialEvent()
                self.update()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

# Car status update to excecution
    def update(self):
        if self.NewLineReceived == 1:
            if self.carstatus == enSTOP:
                self.brake()
            elif self.carstatus == enRUN:
                self.run()
            elif self.carstatus == enLEFT:
                self.left()
            elif self.carstatus == enRIGHT:
                self.right()
            elif self.carstatus == enBACK:
                self.back()
            elif self.carstatus == enTLEFT:
                self.spin_left()
            elif self.carstatus == enTRIGHT:
                self.spin_right()
            else:
                self.brake()
            self.NewLineReceived = 0

#        self.infrared_avoid()
#        self.Distance_test()
