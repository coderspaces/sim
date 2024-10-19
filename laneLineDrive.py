from os import W_OK
import ADCPlatform
import time
import datetime
import threading
import json
from PIL import Image
import numpy as np
import math
import copy

import matplotlib.pyplot as plt


class LaneLineDrive(object):
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
        self.time = 0

        self.leftLine = []
        self.rightLine = []
        self.trace = []

        self.laneIdx = -100
        self.allLane = []

        self.obs = []
        self.first_num = -100
        self.first_run = 1
        self.chang_flag = 1



    def run(self, max_speed):
        try:
            self.time = time.time()

            self.leftLine = []
            self.rightLine = []
            self.trace = []

            # 获取车辆控制数据包
            control_data_package = ADCPlatform.get_control_data()
            if not control_data_package:
                print("任务结束")

            fs = control_data_package.json['FS']
            #print("当前车车速：" + str(fs))
            # print(control_data_package.json)

            laneLine_package = ADCPlatform.get_data(self.landLineId)

            if laneLine_package is None:
                print("Error: no laneline detected!!!")
            else:
                print(laneLine_package.json)

            if self.first_run:
                self.compute_first_num(laneLine_package)
                self.getAllLane(laneLine_package)
                self.first_run = 0
                self.laneIdx = self.first_num

            #print("laneIdx:", self.laneIdx)

            obs_lane = self.find_curves(laneLine_package)
            #print("obs lane is: ", obs_lane)
            if self.chang_flag:
                if obs_lane != -100:
                    self.change_lane(obs_lane)


            self.getLaneline(laneLine_package)

            self.getTrace()

            target_angle = self.getPidAngle(fs)
            #print("target_angle:", target_angle)

            a = 0.8 if fs < max_speed else 0.2
            #ADCPlatform.control(a, target_angle, 0, 1)

            return a,target_angle
        except Exception as e:
            print('ldd self:', e, e.__traceback__.tb_lineno)


    def getLaneline(self, laneLine_package):
        # get left and right laneline
        if laneLine_package is not None and len(laneLine_package.json) > 1:
            #print(laneLine_package.json)
            for laneline in laneLine_package.json:
                if laneline['Num'] == self.laneIdx:
                    self.leftLine = list(laneline.values())
                if laneline['Num'] == self.laneIdx+1:
                    self.rightLine = list(laneline.values())

    def getAllLane(self, laneLine_package):
        allLane = []
        for laneline in laneLine_package.json:
            if laneline['Num'] >= 0:
                allLane.append(laneline['Num'])
                self.allLane = allLane[:-1]





    def CalcKappa(self, preIndex):
        if len(self.trace) == 0:
            return 0
        if preIndex > len(self.trace):
            return 0
        if preIndex == 0:
            return 0

        denominator = 2 * self.trace[preIndex][1] *(-1)
        numerator = pow(self.trace[preIndex][1],2) + pow(self.trace[preIndex][0],2)

        if math.fabs(denominator)>0:
            fRadius = numerator/denominator
        else:
            fRadius = 1e9

        if fRadius == 0: return 0

        return 1.0/fRadius

    def getPidAngle(self, speed):
        kv = 0.8
        estimateDistance = max(8,kv*speed)
        sumDistance = 0.0
        pointIdx = 0
        for i, p in enumerate(self.trace):
            if i == 0:
                continue
            sumDistance += self.getDistance(self.trace[i-1], self.trace[i])
            if sumDistance > estimateDistance:
                pointIdx = i
                break

        wheelBase = 2.91
        wheelRatio = 1
        kappa = self.CalcKappa(pointIdx)

        pp = (180.0 / math.pi) * kappa
        #print('pp:', pp, end=' ')
        wheelAngle =  -2.5* wheelBase * wheelRatio * pp
                      # * math.tan(min(abs(kappa*20),math.pi/2))

        print('wheelAngle:',wheelAngle)
        wheelAngle = wheelAngle * min(abs(wheelAngle/200), 0.8)
        return wheelAngle

    def getDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2+((p1[1]-p2[1])**2))

    def sample_midline(self, poly1_coeffs, poly2_coeffs, distance_interval, num_samples):
        # 计算两条曲线的中线
        midline_coeffs = [(p1 + p2) / 2 for p1, p2 in zip(poly1_coeffs, poly2_coeffs)]
        midline_poly = np.poly1d(midline_coeffs)

        # 根据距离间隔和采样点数量计算采样范围
        min_x = 0
        max_x = distance_interval * (num_samples - 1)

        # 生成等间隔采样点的 x 坐标
        x_samples = np.linspace(min_x, max_x, num_samples)

        # 计算采样点的 y 坐标
        y_samples = midline_poly(x_samples)

        for i in range(len(y_samples)): y_samples[i] = y_samples[i] + 0.25

        # 将采样点的 x 和 y 坐标组合成一个列表
        sampled_points = list(zip(x_samples, y_samples))

        return sampled_points

    def getTrace(self):

        poly1 = reversed(self.leftLine[2:])

        poly2 = reversed(self.rightLine[2:])
        distance_interval = 0.5
        num_samples = 400

        self.trace = self.sample_midline(poly1, poly2, distance_interval, num_samples)


    def plot_sampled_points(self, points, interval=0.1):

        fig, ax = plt.subplots()
        ax.set_xlabel("x")
        ax.set_ylabel("y")

        for point in points:
            plt.scatter(point[0], point[1], color='r', marker='o')
            plt.draw()
            plt.pause(interval)

        plt.show()

    def computeObsOnLane(self, laneLine_package):
        obs_lane = -100
        if self.obs == []:
            return obs_lane

        ob_x = []
        ob_x.append(self.obs[0])
        ob_y = self.obs[1]

        poly_yy = {}

        if laneLine_package is not None and len(laneLine_package.json) > 1:
            for idx, laneline in enumerate(laneLine_package.json):
                poly = list(laneline.values())
                poly = reversed(poly[2:])
                coeffs = [p for p in poly]
                midline_poly = np.poly1d(coeffs)
                poly_y = midline_poly(ob_x)
                poly_yy[laneline['Num']] = poly_y
        else:
            print("Error: no laneline detected!!!")

        for num, poly_y in poly_yy.items():
            if ob_y > poly_y:
                obs_lane = num

        return obs_lane


    def change_lane(self, obs_lane):
        if self.chang_flag:
            if self.laneIdx == obs_lane:
                if self.obs[2] < 30:
                    for lane in self.allLane:
                        if lane != obs_lane:
                            self.laneIdx = lane
                            print("laneIdx change:",self.laneIdx)
                            break

    def compute_first_num(self, laneLine_package):
        for laneline in laneLine_package.json:
            if laneline['A1'] < 0:
                self.first_num = laneline['Num']


    def find_curves(self, laneLine_package):
        obs_lane = -100
        if self.obs == []:
            return obs_lane

        # self.obs = [30, 0, 30]

        point = (self.obs[0], self.obs[1])
        curve_collection = {}
        for laneline in laneLine_package.json:
            l_value = list(laneline.values())
            l_num = l_value[0]
            l_A = l_value[2:]
            curve_collection[l_num] = l_A

        for curve_number, curve_coefficients in curve_collection.items():
            curve_x = curve_coefficients  # 获取曲线的 x 系数，排除最高次项
            # curve_y = curve_coefficients[-1]  # 获取曲线的 y 系数，即最高次项

            # 使用多项式系数计算曲线在给定 x 坐标处的 y 值
            curve_y_value = sum(coef * point[0] ** exp for exp, coef in enumerate(curve_x))

            # 比较点的 y 坐标与曲线的 y 值，判断点在曲线的上方还是下方
            if point[1] < curve_y_value:
                right_curve = curve_number
            else:
                obs_lane = curve_number

        return obs_lane

if __name__=='__main__':
    from configparser import ConfigParser

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
    lld = LaneLineDrive()
    if result:
        print("算法接入成功！")
        print("启动任务")
        ADCPlatform.start_task()
        sensors = ADCPlatform.get_sensors()
        for sensor in sensors:
            if "Radar" in sensor.Name:
                radarId = sensor.ID

        # 启动算法接入任务控制车辆
        # control.run()
        #LLD.run()
        while True:

            data_package = ADCPlatform.get_data(radarId)

            if data_package and len(data_package.json) > 0:

                print(data_package.json)
                r=data_package.json[0]["distance"]/1000
                angle=data_package.json[0]["angle_Hori"]*math.pi/180.0
                x = r*math.cos(angle)
                y = r * math.sin(angle)
                print('X:',x,'Y:',y,'R:',r)
                lld.obs=[x,y,r]
            else:
                lld.obs = []
            out_throttle,out_steer=lld.run(max_speed=40)
            ADCPlatform.control(out_throttle,
                                out_steer,
                                0,
                                1
                                )
            time.sleep(0.01)
        # 停止平台
        ADCPlatform.stop()

    else:
        # 停止平台
        ADCPlatform.stop()


