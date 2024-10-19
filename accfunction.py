# from os import W_OK
import ADCPlatform
from configparser import ConfigParser
import time
import csv
# import datetime
# import threading
# import json
# from PIL import Image
# from cv2 import cv2
# import numpy as np
import math
# import copy

# import usr
# from odom import Odometer
# from control import Controller
# from perception import Perception
# from planning import Planner

# import qingxia ganzhi


class Accfunction(object):
    def __init__(self):
        self.radarId = 10021
        self.target_throttle = 0.0
        self.target_steering = 0.0
        self.target_brake = 0.0
        self.target_gear = 0.0

        self.target_Speed = 30.0
        self.obcType = 3

        self.obsDistance = 200.0
        self.obsSpeed = 0.0
       
        self._acc = 0.0
        self._minDecelerate = 0.0
        self._vehspeed = 0.0
        self._lightType = 0.0

        self._lastU = 0.0
        self._lastEv = 0.0
        self._lastVt = 0.0
        self._Iv = 0.0
        self._lastTorque=0.0
        self.acc_flag=1
        self.tlDistance=1000000


    def run(self):
        # main loop
        #while True:

            # show object detection results
            # data_package = ADCPlatform.get_data(self.radarId)
            # if data_package and len(data_package.json) > 0:
            #     print(data_package.json)
            print("obsDistance: ", self.obsDistance)
            if self.obsDistance < 10.0:
                self._minDecelerate = -2.0
            #print("obsDistance = ", self.obsDistance, ", obsSpeed = ", self.obsSpeed, "======")
            cardata = None
            cardata = ADCPlatform.get_control_data()
            self.vehspeed = cardata.json['FS']
            #self._lightType = cardata.json['TLC']
            #print("_vehspeed = ", self.vehspeed, "-----")
            # if self.obcType == 0 or self.obcType == 5:
            controllspeed=0.0
            controllbrake=0.0
            if self.obsDistance > 30:
                if self.target_Speed-self.vehspeed >10:
                    controllspeed = 0.8
                    controllbrake = 0.0
                elif abs(self.target_Speed-self.vehspeed)< 2:
                    controllbrake = 0.0
                    controllspeed = max(self._lastTorque-0.1,0.0)
                else:
                    if self.target_Speed - self.vehspeed < -3:
                        controllspeed = 0.0
                        controllbrake = 0.2
                    else:
                        controllspeed = 0.3
                        controllbrake = 0.0
            elif self.obsDistance > 15:
                if abs(self.target_Speed-self.vehspeed)< 1:
                    controllbrake = 0.0
                    controllspeed = max(self._lastTorque-0.1,0.01)
                else:
                    controllbrake = 0.0
                    controllspeed = 0.1
            elif self.obsDistance>5:
                if abs(self.target_Speed-self.vehspeed)< 5:
                    controllbrake = 0.0
                    controllspeed = max(self._lastTorque-0.1,0.0)
                else:
                    controllbrake=0.05
                    controllspeed=0
            else:
                controllspeed=0.0
                controllbrake=1


            # if (self._lightType==1 or self._lightType==2 ) and self.tlDistance<10:
            #     controllspeed=0.0
            #     controllbrake=1
            # else:
            #     controllspeed = controllspeed
            #     controllbrake = controllbrake
            self._lastTorque = controllspeed

            self.target_throttle = controllspeed

            self.target_brake = controllbrake
            #print("throttle = ", self.target_throttle, ", brake = ", self.target_brake, "-----------------")
            # else:
            #     self._acc = self._acc
            #print("acc = ", self._acc,  "----")
            #print("throttle = ", self.target_throttle, ", brake = ", self.target_brake, "*******")
            #ADCPlatform.control(self.target_throttle, 0, self.target_brake, 1)
            #writer.writerow([self.obsDistance, self.obsSpeed, self.vehspeed, 0, self.target_throttle, self.target_brake])
            return self.target_throttle, self.target_brake
            #time.sleep(0.02)




if __name__ == '__main__':
    # 开启平台SDK
    # 读取配置文件
    config = ConfigParser()
    config.read("config.ini")
    print(config['ADCPlatform']['ServerUrl'])
    # 设置服务器访问地址
    serverUrl = config['ADCPlatform']['ServerUrl']
    # 设置登录用户名
    username = config['ADCPlatform']['UserName']
    # 设置登录密码
    password = config['ADCPlatform']['PassWord']
    result = ADCPlatform.start(serverUrl, username, password)
    #
    # csvfile = open('./acc_debug11.csv', 'w', newline='\n')
    #
    # headers = ['obsdis', 'obsspeed', 'vehspeed', 'acc', 'throttle', 'brake']
    # writer = csv.writer(csvfile)
    # writer.writerow(headers)
    if result:
        ADCPlatform.start_task()
        acc = Accfunction()
        acc.target_Speed = 60
        # acc.obcType = 2
        sensors = ADCPlatform.get_sensors()
        for sensor in sensors:
            print("传感器名称：" + sensor.Name + "  ID:" + str(sensor.ID))


            if "Radar" in sensor.Name:
                radarId = sensor.ID

        while 1:
            min_r = None
            data_package = ADCPlatform.get_data(radarId)
            if data_package and len(data_package.json) > 0:
                print('hmb', data_package.json)
                for obj_data in data_package.json:
                    r = obj_data["distance"] / 100
                    if min_r == None:
                        min_r = r
                        angle = obj_data["angle_Ver"] * math.pi / 180.0
                        y = min_r * math.sin(angle)
                        type = obj_data["type"]
                        x = min_r * math.cos(angle)
                    if min_r > r:
                        min_r = r
                        angle = obj_data["angle_Ver"] * math.pi / 180.0
                        y = min_r * math.sin(angle)
                        type = obj_data["type"]
                        x = min_r * math.cos(angle)
                    else:
                        continue
                print('X:', x, 'Y:', y, 'R:', min_r, 'T:', type)
            acc.obsDistance = min_r
            acc.obsSpeed = data_package.json[0]["speed_sub"] * 0.01 * 3.6
            target_throttle, target_brake=acc.run()
            ADCPlatform.control(target_throttle,
                                0,
                                target_brake,
                                1
                                )
        ADCPlatform.stop()

    else:
        # 停止平台
        ADCPlatform.stop()