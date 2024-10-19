from os import W_OK
import ADCPlatform
import time
import datetime
import threading
import json
import math
import copy
import torch
# import torchvision.transforms as T
from PIL import Image

import cv2
import numpy as np
from io import BytesIO

# import matplotlib.pyplot as plt


class TrafficLightDetection(object):
    def __init__(self):
        self.cameraId = 10011

        self.model_path = './model/best.pt'
        self.model = torch.hub.load('./ultralytics_yolov5_master', 'custom', path=self.model_path, source='local')




    def run(self):
        color = None
        try:
            image_package = ADCPlatform.get_image(self.cameraId)
            if image_package is not None:
                # image_stream = BytesIO(image_package.byte)
                pil_img = Image.open(image_package.byte)
                cvimg = np.array(pil_img)
                cvimg = cv2.cvtColor(cvimg, cv2.COLOR_RGB2BGR)


                bounding_boxes = self.detect_traffic_light(pil_img)
                if bounding_boxes == []:
                    color = 0
                    print("No Traffic light color:", color)
                else:
                    bounding_box = bounding_boxes[0].values()  # 替换为实际的边界框坐标
                    # 提取红绿灯颜色
                    color = self.extract_traffic_light_color(pil_img, bounding_box)
                    print("Traffic light color:", color)

                # self.show_detection_results(pil_img, bounding_boxes)

            return color
        except Exception as e:
            print('tld self:', e, e.__traceback__.tb_lineno)


    def extract_traffic_light_color(self, image, bounding_box):
        image = np.array(image)
        # 提取红绿灯区域
        x, y, w, h = bounding_box
        traffic_light_region = image[y:y+h, x:x+w]

        #traffic_light_region = cv2.cvtColor(traffic_light_region, cv2.COLOR_RGB2BGR)
        # 将图像转换为HSV颜色空间
        # cv2.imshow('111', traffic_light_region)
        # cv2.waitKey(10)
        hsv_image = cv2.cvtColor(traffic_light_region, cv2.COLOR_BGR2HSV)

        # 定义红色和绿色的HSV颜色范围
        lower_red1 = np.array([0, 100, 150])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([100, 100, 100])
        upper_red2 = np.array([130, 255, 255])
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])

        # 根据颜色阈值分割图像
        mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

        # print("iiiiiiiiiiimage",hsv_image)
        # 计算红色和绿色像素的数量
        red_pixels = cv2.countNonZero(mask_red1) + cv2.countNonZero(mask_red2)
        green_pixels = cv2.countNonZero(mask_green)

        # 判断红绿灯颜色
        print("red_pixels: ", red_pixels)
        print("green_pixels: ", green_pixels)
        if red_pixels == 0 and green_pixels == 0:
            return -1
        if red_pixels > green_pixels:
            return 1
        else:
            return 3




    def detect_traffic_light(self, image):
        # 加载Yolov5模型
        model = self.model

        # 运行检测
        results = model(image)

        # 解析检测结果
        detections = results.pandas().xyxy[0]  # 获取检测到的对象
        traffic_lights = detections[detections['name'] == 'traffic light']  # 获取红绿灯对象

        bounding_boxes = []  # 存储红绿灯检测区域的边界框

        for _, det in traffic_lights.iterrows():
            bbox = {
                'x': int(det['xmin']),
                'y': int(det['ymin']),
                'width': int(det['xmax'] - det['xmin']),
                'height': int(det['ymax'] - det['ymin'])
            }
            bounding_boxes.append(bbox)

        return bounding_boxes

    def show_detection_results(self, image, bounding_boxes):
        # 读取图像
        image = np.array(image)

        # 在图像上绘制边界框
        if bounding_boxes != []:
            for bbox in bounding_boxes:
                x, y, w, h = bbox['x'], bbox['y'], bbox['width'], bbox['height']
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # 显示图像
        cv2.namedWindow('Traffic Light Detection', cv2.WINDOW_NORMAL)
        cv2.imshow('Traffic Light Detection', image)
        cv2.waitKey(1)



if __name__=='__main__':
    from configparser import ConfigParser

    tld = TrafficLightDetection()
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
    lld = TrafficLightDetection()
    if result:
        print("算法接入成功！")
        print("启动任务")
        ADCPlatform.start_task()
        # 启动算法接入任务控制车辆
        # control.run()
        #LLD.run()
        sensors = ADCPlatform.get_sensors()
        for sensor in sensors:
            print("传感器名称：" + sensor.Name + "  ID:" + str(sensor.ID))

            if "camera" in sensor.Name:

                if "车道线" in sensor.Name:
                    a = sensor.ID
                elif "camera-1" in sensor.Name:
                    tld.cameraId = sensor.ID
        while True:
            tld.run()
            radarId = 10021
            data_package = ADCPlatform.get_data(radarId)

            if data_package and len(data_package.json) > 0:

                print(data_package.json)
                r=data_package.json[0]["Range"]/100
                angle=data_package.json[0]["Angle"]*math.pi/180.0
                x = r*math.cos(angle)
                y = r * math.sin(angle)
                print('X:',x,'Y:',y,'R:',r)
                lld.obs=[x,y,r]
            else:
                lld.obs = []
            # out_throttle,out_steer=lld.run(max_speed=40)
            # ADCPlatform.control(out_throttle,
            #                     out_steer,
            #                     0,
            #                     1
            #                     )
        # 停止平台
        ADCPlatform.stop()

    else:
        # 停止平台
        ADCPlatform.stop()


