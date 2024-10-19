import ADCPlatform
from configparser import ConfigParser
import threading
import math
import time
import numpy as np


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

        self.lld = LaneLineDrive()
        self.tim_lld = None
        self.loop_lld = 0.1

        self.acc = Accfunction()
        self.tim_acc = None
        self.loop_acc = 0.1

        self.tim_avoid = None
        self.loop_avoid = 0.1

        self.out_throttle = 0
        self.out_steer = 0
        self.out_brake = 0

    def run(self):
        self.tim_ldd = threading.Timer(interval=self.loop_lld, function=self.thread_ldd, args=())
        self.tim_ldd.start()

        self.tim_acc = threading.Timer(interval=self.loop_acc, function=self.thread_acc, args=())
        self.tim_acc.start()

        self.tim_avoid = threading.Timer(interval=self.loop_avoid, function=self.thread_avoid, args=())
        self.tim_avoid.start()

    def thread_ldd(self):
        self.tim_ldd = threading.Timer(interval=self.loop_lld, function=self.thread_ldd, args=())
        self.tim_ldd.start()
        self.out_throttle, self.out_steer = self.lld.run(max_speed=55)

    def thread_acc(self):
        self.tim_acc = threading.Timer(interval=self.loop_acc, function=self.thread_acc, args=())
        self.tim_acc.start()
        self.acc_throttle, self.acc_brake = self.acc.run()

    def thread_avoid(self):
        self.tim_avoid = threading.Timer(interval=self.loop_avoid, function=self.thread_avoid, args=())
        self.tim_avoid.start()
        min_r = None
        y = 0
        type = 0
        x = 0
        data_package = ADCPlatform.get_data(self.radarId)
        if data_package and len(data_package.json) > 0:
            print('hmb', data_package.json)
            for obj_data in data_package.json:
                print("obj_data:",obj_data)
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
            if type == 11:
                self.lld.chang_flag = 1
                self.acc.obsDistance = 2000
                self.lld.obs = [x, y, min_r]
                print("self.lld.obs:",self.lld.obs)
            elif type == 9 or type == 7:
                self.lld.laneIdx = self.lld.first_num
                self.lld.chang_flag = 0
                self.acc.acc_flag = 1
                self.lld.obs = []
                if y > -2.5 and y < 3.5:
                    self.acc.obsDistance = min_r
                else:
                    self.acc.obsDistance = 2000
        self.out_throttle = self.acc_throttle
        self.out_brake = self.acc_brake
        print('油门', self.out_throttle,
              '方向', self.out_steer,
              '刹车', self.out_brake)
        ADCPlatform.control(self.out_throttle,
                            self.out_steer,
                            self.out_brake, )

class Accfunction(object):
    def __init__(self):
        self.radarId = 10021
        self.target_Speed = 30.0
        self.obsDistance = 200.0
        self._lastTorque = 0.0
        self.acc_flag = 1

    def run(self):
        print("obsDistance:", self.obsDistance)
        cardata = ADCPlatform.get_control_data()
        self.vehspeed = cardata.json['FS']
        if self.obsDistance > 30:
            if self.target_Speed - self.vehspeed > 10:
                controllspeed = 0.8
                controllbrake = 0.0
            elif abs(self.target_Speed - self.vehspeed) < 2:
                controllbrake = 0.0
                controllspeed = max(self._lastTorque - 0.1, 0.0)
            else:
                if self.target_Speed - self.vehspeed < -3:
                    controllspeed = 0.0
                    controllbrake = 0.2
                else:
                    controllspeed = 0.3
                    controllbrake = 0.0
        elif self.obsDistance > 15:
            if abs(self.target_Speed - self.vehspeed) < 1:
                controllbrake = 0.0
                controllspeed = max(self._lastTorque - 0.1, 0.01)
            else:
                controllbrake = 0.0
                controllspeed = 0.1
        elif self.obsDistance > 5:
            if abs(self.target_Speed - self.vehspeed) < 5:
                controllbrake = 0.0
                controllspeed = max(self._lastTorque - 0.1, 0.0)
            else:
                controllbrake = 0.05
                controllspeed = 0
        else:
            controllspeed = 0.0
            controllbrake = 1
        self._lastTorque = controllspeed
        self.target_throttle = controllspeed
        self.target_brake = controllbrake
        return self.target_throttle, self.target_brake

class LaneLineDrive(object):
    def __init__(self):
        sensors = ADCPlatform.get_sensors()
        for sensor in sensors:
            print("传感器名称：" + sensor.Name + "ID:" + str(sensor.ID))
            if "camera" in sensor.Name:
                if "车道线" in sensor.Name:
                    self.landLineId = sensor.ID
                elif "camera-1" in sensor.Name:
                    self.cameraId = sensor.ID
        self.first_run = 1
        self.first_num = -100
        self.laneIdx = -100
        self.obs = []
        self.chang_flag = 1

    def run(self, max_speed):
        self.time = time.time()
        self.leftLine = []
        self.rightLine = []
        self.trace = []
        control_data_package = ADCPlatform.get_control_data()
        if not control_data_package:
            print("任务结束")
        fs = control_data_package.json['FS']
        laneLine_package = ADCPlatform.get_data(self.landLineId)
        if laneLine_package is None:
            print("Error:no laneline detected!!!!")
        else:
            print(laneLine_package.json)

        if self.first_run:
            self.compute_first_num(laneLine_package)
            self.getAllLane(laneLine_package)
            self.first_run = 0
            self.laneIdx = self.first_num

        obs_lane = self.find_curves(laneLine_package)
        self.change_lane(obs_lane)
        self.getLaneline(laneLine_package)
        self.getTrace()
        target_angle = self.getPidAngle(fs)
        a = 0.8 if fs < max_speed else 0.2
        return a, target_angle

    def getLaneline(self, laneLine_package):
        if laneLine_package is not None and len(laneLine_package.json) > 1:
            for laneline in laneLine_package.json:
                if laneline['Num'] == self.laneIdx:
                    self.leftLine = list(laneline.values())
                if laneline['Num'] == self.laneIdx + 1:
                    self.rightLine = list(laneline.values())

    def compute_first_num(self, laneLine_package):
        for laneline in laneLine_package.json:
            if laneline['A1'] < 0:
                self.first_num = laneline['Num']

    def getAllLane(self, laneLine_package):
        allLane = []
        for laneline in laneLine_package.json:
            if laneline['Num'] >= 0:
                allLane.append(laneline['Num'])
                self.allLane = allLane[:-1]

    def getPidAngle(self, speed):

        k_v = 0.8    #用于计算预测距离
        PreviewDistance = max(8, k_v * speed)   #预测距离，最小为8或者k_v与速度的乘积
        sumdis = 0.0  #初始化累计距离
        pointldx = 0  #初始化索引
        for i, p in enumerate(self.trace):
            if i == 0:
                continue
            sumdis += self.getDistance(self.trace[i - 1], self.trace[i])  #计算当前点与前一点距离
            if sumdis > PreviewDistance:                                   #累计距离超过预测距离时，记录索引并退出循环
                pointldx = i
                break
        wheel_base = 2.91       #转向比
        wheelratio = 1          #轮胎比率
        kappa = self.CalcKappa(pointldx)  #计算曲率
        print('kappa:', kappa)
        pp = (180 / math.pi) * kappa      #将弧度转化为角度
        wheel_angle1 = -2.5 * wheel_base * wheelratio * pp
        print("wheel_angle1:",wheel_angle1)
        wheel_angle = wheel_angle1 * min(abs(wheel_angle1 / 200), 0.8)     #限制转向角度
        print("wheel_angle2:",wheel_angle)
        return wheel_angle

    def CalcKappa(self, preIndex):
        denominator = 2 * self.trace[preIndex][1] * (-1)
        numerator = pow(self.trace[preIndex][1], 2) + pow(self.trace[preIndex][0], 2)
        if math.fabs(denominator) > 0:
            fRadius = numerator / denominator
            return 1.0 / fRadius

    def getTrace(self):
        poly1 = reversed(self.leftLine[2:])
        poly2 = reversed(self.rightLine[2:])
        distance_interval = 0.5
        num_samples = 400
        self.trace = self.sample_midline(poly1, poly2, distance_interval, num_samples)

    def getDistance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + ((p1[1] - p2[1]) ** 2))

    def sample_midline(self, poly1_coeffs, poly2_coeffs, distance_interval, num_samples):
        #计算两条曲线的中线的系数
        midline_coeffs = [(p1 + p2) / 2 for p1, p2 in zip(poly1_coeffs, poly2_coeffs)]
        midline_poly = np.poly1d(midline_coeffs)
        #根据距离间隔和采样点数量的计算采样范围
        min_x = 0
        max_x = distance_interval * (num_samples - 1)

        #生成等间隔的x坐标
        x_samples = np.linspace(min_x, max_x, num_samples)
        #生成等间隔的y坐标
        y_samples = midline_poly(x_samples)
        for i in range(len(y_samples)):
            y_samples[i] = y_samples[i] + 0.25    #调整y坐标的偏移量
        sampled_points = list(zip(x_samples, y_samples))
        return sampled_points

    def find_curves(self, laneLine_package):
        obs_lane = -100
        if self.obs == []:
            return obs_lane
        point = (self.obs[0], self.obs[1])
        print("point:",point)
        curce_collection = {}
        for laneline in laneLine_package.json:
            l_value = list(laneline.values())
            l_num = l_value[0]
            l_A = l_value[2:]
            curce_collection[l_num] = l_A

        for curve_number, curve_coefficients in curce_collection.items():
            curve_x = curve_coefficients
            curve_y_value = sum(coef * point[0] ** exp for exp, coef in enumerate(curve_x))
            # 比较点的 y 坐标与曲线的 y 值，判断点在曲线的上方还是下方
            if point[1] >= curve_y_value:
                obs_lane = curve_number
        return obs_lane

    def change_lane(self, obs_lane):
        if self.chang_flag:          #检查是否允许车道切换
            if self.laneIdx == obs_lane:   #如果当前车道和障碍物所在车道一致
                if self.obs[2] < 30:       #如果障碍物距离小于30
                    for lane in self.allLane:  #遍历所有可选车道
                        if lane != obs_lane:   #如果车道不是障碍物所在车道
                            self.laneIdx = lane  #切换车道
                            print("laneIdx change:", self.laneIdx)
                            break

if __name__ == '__main__':
    config = ConfigParser()
    config.read("config.ini")
    print(config['ADCPlatform']['ServerUrl'])
    serverUrl = config['ADCPlatform']['ServerUrl']
    username = config['ADCPlatform']['UserName']
    password = config['ADCPlatform']['PassWord']
    result = ADCPlatform.start(serverUrl, username, password)
    mst = Master()
    if result:
        print("算法接入成功！")
        print("启动任务")
        ADCPlatform.start_task()
        print('Master启动')
        mst.run()
        ADCPlatform.stop()
    else:
        ADCPlatform.stop()