import math
import time
import cv2
import mediapipe as mp
from djitellopy import Tello
from p_control import *


def run():    
    #定义并使用mediapipe的hands模块
    mpHands = mp.solutions.hands
    hands = mpHands.Hands()
    mpDraw = mp.solutions.drawing_utils

    #初始化用于FPS计算的变量 
    pTime = 0
    cTime = 0
    

    FingerXY = [] #定义手指坐标列表
    FingerStatus = [] #定义手指曲伸状态列表
    topid = [4,8,12,16,20] #手指顶部ID列表
    midid = [5,6,10,14,18] #手指中部ID列表
    Time = time.time() #初始化控制时间
    isStop = 0
    STime = time.time()

    #初始化手指坐标列表
    for i in range(0, 21):
        FingerXY.insert(i, [0,0])
    #初始化手指曲伸状态列表
    for i in range(0,5):
        FingerStatus.insert(i, 0)

    #是否第一次启动为真
    IsFirstBoot = 1

    

    while True:
        img = tello.get_frame_read().frame #获取一帧
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) #cv2图像初始化
        results = hands.process(imgRGB)


        if results.multi_hand_landmarks: 
            for handLms in results.multi_hand_landmarks: #如果检测到手，则输出21个节点ID以每个ID的坐标
                for id, lm in enumerate(handLms.landmark):
                    h, w, c = img.shape # h:高度 w:宽度
                    cx, cy = int(lm.x * w), int(lm.y * h) #计算坐标
                    FingerXY[id] = [cx,cy] #将每个ID的坐标更新到FingerXY的列表里

                for FingerNum in range(0,5): #循环设置五根手指的曲伸状态
                    if FingerNum == 0:  #大拇指结构与其他手指不一样，需额外设置
                        #如果 大拇指顶部节点 到 手掌根部节点 的L2范数  小于  食指根部 到 手掌根部节点 的L2范数 就为按下，否则为伸出
                        if math.dist(FingerXY[topid[0]], FingerXY[0])-30 < math.dist(FingerXY[midid[0]], FingerXY[0]):
                            FingerStatus[FingerNum] = 0
                        else:
                            FingerStatus[FingerNum] = 1
                    else:
                        #如果 每个手指顶部节点 到 手掌根部节点 的L2范数  小于  每个手指的中/下部节点 到 手掌根部节点 的L2范数 就为按下，否则为伸出
                        if math.dist(FingerXY[topid[FingerNum]], FingerXY[0]) < math.dist(FingerXY[midid[FingerNum]], FingerXY[0]):
                            FingerStatus[FingerNum] = 0
                        else:
                            FingerStatus[FingerNum] = 1

                '''    左右移动控制部分 '''
                if FingerStatus[1] == 1 and FingerStatus[2] == 1 and FingerStatus[3] == 0 and FingerStatus[4] == 0:
                #Notice: FingerStatus[0]由于判断不稳定已经去掉
                        #中指到食指的L2范数小于55
                        if math.floor(math.dist(FingerXY[12], FingerXY[8])) < 55 :

                            #如果食指的X坐标小于图像中间 即手指在左边
                            if FingerXY[8][0] < w/2:
                                if time.time() - Time > 0.05:
                                    #误差设置为食指的X坐标到图像中间的L2范数
                                    err = (math.floor(math.dist([FingerXY[8][0]], [w/2])))
                                    #调用PID控制
                                    Control = math.floor(P_Control(err, 0, 0.18))    
                                    tello.send_command_without_return("left "+str(Control))
                                    Time = time.time()
                                else:
                                    tello.send_command_without_return("stop")
                            #如果食指的X坐标大于图像中间 即手指在右边
                            elif FingerXY[8][0] > w/2:
                                    #误差设置为食指的X坐标到图像中间的L2范数
                                    err = math.floor(math.dist([FingerXY[8][0]], [w/2]))
                                    #调用PID控制
                                    Control = math.floor(P_Control(err, 0, 0.18))
                                    if time.time() - Time > 0.05:
                                        tello.send_command_without_return("right "+str(Control))
                                        Time = time.time()
                                    else:
                                        tello.send_command_without_return("stop")
                '''    左右移动控制部分 '''

                '''    左右翻转部分 '''
                if FingerStatus[1] == 1 and FingerStatus[2] == 1 and FingerStatus[3] == 1 and FingerStatus[4] == 0:
                #Notice: FingerStatus[0]由于判断不稳定已经去掉
                        #中指到食指的L2范数小于55和中指到无名指的L2范数小于55
                        if math.floor(math.dist(FingerXY[12], FingerXY[8])) < 55 and math.floor(math.dist(FingerXY[8], FingerXY[16])) < 55:

                            #如果食指的X坐标小于图像中间 即手指在左边
                            if FingerXY[8][0] < w/2:
                                if time.time() - Time > 2:
                                    tello.send_command_without_return("flip l")
                                    Time = time.time()
                            #如果食指的X坐标大于图像中间 即手指在右边
                            elif FingerXY[8][0] > w/2:
                                    #误差设置为食指的X坐标到图像中间的L2范数
                                    if time.time() - Time > 2:
                                        tello.send_command_without_return("flip r")
					                    
                                        Time = time.time()
                '''    左右翻转部分 '''

                '''     OK降落   '''
                if time.time() - STime > 5:
                     if FingerStatus[1] == 0 and FingerStatus[2] == 1 and FingerStatus[3] == 1 and FingerStatus[4] == 1:
                        if math.floor(math.dist(FingerXY[4], FingerXY[8])) < 55 :
                            if isStop == 0:
                                tello.send_command_without_return("streamoff")
                                tello.send_command_without_return("land")
                                isStop = 1
                    
                        
                #连接手指关节点
                mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

        print(tello.get_battery())

        #第一次启动图像成功传输后才起飞
        if IsFirstBoot == 1:
            tello.takeoff()
            time.sleep(1)
            tello.move_up(70)
            IsFirstBoot = 0

        '''
        视频FPS计算
        '''
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        
        #显示FPS数值
        cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3,
                    (255, 0, 255), 3) 

        #显示图像
        cv2.imshow("HandsImage", img) 
        #监听键盘
        key = cv2.waitKey(1)
        #如果ESC键按下
        if key == 27:
            #降落流并结束
            tello.land()
            break

    

if __name__ == '__main__':
    tello = Tello()
    tello.connect() #连接Tello 并根据SDK文档发送command进入SDK模式
    tello.streamon() #打开视频流
    run()


