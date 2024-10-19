import ADCPlatform
import threading
import time

import math
#from objdetect import  ObjDetector
from PIL import Image
import numpy as np
# import Avoid
import cv2
from accfunction import Accfunction
from laneLineDrive import LaneLineDrive
from tl_detection import TrafficLightDetection

class Master(object):
    def __init__(self):

        sensors = ADCPlatform.get_sensors()
        for sensor in sensors:
            print("传感器名称：" + sensor.Name + "  ID:" + str(sensor.ID))

            if "camera" in sensor.Name:

                if "车道线" in sensor.Name:
                    self.landLineId = sensor.ID
                elif "camera-1" in sensor.Name:
                    self.cameraId = sensor.ID

            elif "Radar" in sensor.Name:
                self.radarId = sensor.ID
            elif "Lidar" in sensor.Name:
                self.lidarId = sensor.ID
            elif "GPS" in sensor.Name:
                self.gpsId = sensor.ID
        # 车道保持
        self.lld = LaneLineDrive()
        self.tim_lld=None
        self.loop_lld = 0.1

        # self.obj=ObjDetector()
        self.tim_obj = None
        self.loop_obj = 0.1

        # 自动巡航
        self.acc = Accfunction()
        self.tim_acc = None
        self.loop_acc = 0.1

        # 交通灯检测
        self.tld = TrafficLightDetection()
        self.tld.cameraId=self.cameraId
        self.tim_tld = None
        self.loop_tld = 0.1

        self.tim_avoid=None
        self.loop_avoid=0.1

        self.tim_odometer = None
        self.loop_odometer = 0.100

        self.out_throttle=0
        self.out_steer=0
        self.out_brake=0
        self.gear=1
        self._lightType=None

        self.imageId = 0


    def run(self):
        self.tim_ldd=threading.Timer(interval=self.loop_lld,function=self.thread_ldd,args=())
        self.tim_ldd.start()

        # self.tim_obj = threading.Timer(interval=self.loop_obj, function=self.thread_obj, args=())
        # self.tim_obj.start()

        self.tim_acc = threading.Timer(interval=self.loop_acc, function=self.thread_acc, args=())
        self.tim_acc.start()

        self.tim_avoid = threading.Timer(interval=self.loop_avoid, function=self.thread_avoid, args=())
        self.tim_avoid.start()

        self.tim_tld = threading.Timer(interval=self.loop_tld, function=self.thread_tld, args=())
        self.tim_tld.start()


        while True:
            try:
                # 无用
                control_data_package = ADCPlatform.get_control_data()
                fs = control_data_package.json['FS']
                # self._lightType = control_data_package.json['TLC']

            except Exception as e:
                print('main:',e)
                pass
            time.sleep(0.1)

    def thread_ldd(self):
        self.tim_ldd = threading.Timer(interval=self.loop_lld, function=self.thread_ldd, args=())
        self.tim_ldd.start()
        try:

            self.out_throttle11,self.out_steer=self.lld.run(max_speed=55)
        except Exception as e:
            print('ldd:', e, e.__traceback__.tb_lineno)

    def thread_acc(self):
        self.tim_acc = threading.Timer(interval=self.loop_acc, function=self.thread_acc, args=())
        self.tim_acc.start()
        try:
            self.acc_throttle, self.acc_brake=self.acc.run()
        except Exception as e:
            print('acc:', e, e.__traceback__.tb_lineno)

    def thread_tld(self):
        self.tim_tld = threading.Timer(interval=self.loop_tld, function=self.thread_tld, args=())
        self.tim_tld.start()
        try:

            # self.acc_throttle, self.acc_brake=self.acc.run()
            self._lightType = self.tld.run()
        except Exception as e:
            print('tld:', e, e.__traceback__.tb_lineno)

    def thread_obj(self):
        self.tim_obj = threading.Timer(interval=self.loop_obj, function=self.thread_obj, args=())
        self.tim_obj.start()
        image_package = ADCPlatform.get_image(self.cameraId)
        if image_package is not None:
            pil_img = Image.open(image_package.byte)
            image_path = 'image1/'+str(self.imageId)+'.jpg'
            pil_img.save(image_path)
            self.imageId += 1
            print("image ID is: ", self.imageId)
            # cvimg = np.array(pil_img)
            # cvimg = cv2.cvtColor(cvimg, cv2.COLOR_RGB2BGR)
            # try:
            #     self.obj.run(cvimg)
            # except Exception as e:
            #     print('obj',e)
            #     pass
        else:
            print("Img is None !!!!!")


    def thread_avoid(self):
        self.tim_avoid = threading.Timer(interval=self.loop_avoid, function=self.thread_avoid, args=())
        self.tim_avoid.start()
        try:
            min_r = None
            y = 0
            type = 0
            x = 0
            #self.acc.obsDistance = 2000
            data_package = ADCPlatform.get_data(self.radarId)
            if data_package and len(data_package.json) > 0:
                print('hmb',data_package.json)
                for obj_data in data_package.json:
                    r=obj_data["distance"]/100
                    if min_r==None:
                        min_r=r
                        angle = obj_data["angle_Ver"] * math.pi / 180.0
                        y = min_r * math.sin(angle)
                        type = obj_data["type"]
                        x = min_r * math.cos(angle)
                    if min_r>r:
                        min_r=r
                        angle = obj_data["angle_Ver"] * math.pi / 180.0
                        y = min_r * math.sin(angle)
                        type=obj_data["type"]
                        x = min_r * math.cos(angle)
                    else:
                        continue
                print('X:',x,'Y:',y,'R:',min_r,'T:',type)

                if type==11:
                    print('llllllllllllllllllllddddddddddddddddddddddd')
                    self.lld.chang_flag = 1
                    #self.acc.acc_flag = 0
                    self.acc.obsDistance = 2000
                    self.acc.obsSpeed = data_package.json[0]["speed_sub"] * 0.01 * 3.6
                    self.lld.obs = [x, y, min_r]
                elif type==9 or type==7 :
                    print('aaaaaaaaaaaaaaaaaaaccccccccccccccccccccccc')
                    self.lld.laneIdx = self.lld.first_num
                    self.lld.chang_flag = 0
                    self.acc.acc_flag = 1
                    self.lld.obs = []

                    if y>-2.5 and y<3.5:
                        self.acc.obsDistance = min_r
                        self.acc.obsSpeed = data_package.json[0]["speed_sub"] * 0.01 * 3.6
                    else:
                        self.acc.obsDistance = 2000

                else:
                    self.acc.obsDistance = 2000
                    self.acc.obsSpeed = data_package.json[0]["speed_sub"] * 0.01 * 3.6
                    self.lld.obs = []
            else:
                self.acc.obsDistance = 2000
                self.lld.obs = []
            self.out_throttle = self.acc_throttle
            self.out_brake = self.acc_brake
            lightx=[]
            if data_package and len(data_package.json) > 0:
                tank_score = 0
                car_score = 0
                tank_y = 0
                car_y = 0
                for obj_data in data_package.json:
                    if obj_data["type"]==10:
                        lightangle = obj_data["angle_Ver"] * math.pi / 180.0
                        lightx.append((obj_data["distance"]/100)* (math.cos(lightangle)))
                        print('lightyyyy',lightx,self._lightType)

                    if obj_data["type"]==11 and obj_data["distance"]<=10000:
                        tank_score = 1
                        tank_y = obj_data["y"]
                    if obj_data["type"]==9 and obj_data["distance"]<=10000:
                        car_score = 1
                        car_y = obj_data["y"]
                if tank_score+car_score>=2:
                    if abs(tank_y - car_y)  > 500:
                        self.out_brake = 1
                        self.out_throttle = 0

                if lightx!=[]:
                    if (self._lightType == 1 or self._lightType == 2) and max(lightx) < 45:
                        print('redddddddddddddddddddddddd')
                        self.out_brake=1
                        self.out_throttle=0
            print('油门', self.out_throttle,
                  '方向', self.out_steer,
                  '刹车', self.out_brake)
            # 主要控制了油门和刹车
            ADCPlatform.control(self.out_throttle,
                                self.out_steer,
                                self.out_brake,
                                self.gear
                                )
        except Exception as e:
            print('avo',e,e.__traceback__.tb_lineno)

    def __del__(self):

        self.tim_ldd.cancel()
        self.tim_avoid.cancel()
        self.tim_obj.cancel()
        
