import ADCPlatform
from configparser import ConfigParser


from master import Master

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

    # 登陆并拉取任务
    result = ADCPlatform.start(serverUrl, username, password)

    # 初始化传感器id，并初始化场景(车道保持｜acc定速巡航｜交通灯检测)的参数
    mst=Master()

    if result:
        print("算法接入成功！")
        print("启动任务")

        # 请求参数写死的1，是请求第1个吧？
        ADCPlatform.start_task()
        # 启动算法接入任务控制车辆
        #control.run()
        #LLD.run()
        mst.run()
        # 停止平台
        # 退出登陆
        ADCPlatform.stop()

    else:
        # 停止平台
        ADCPlatform.stop()
