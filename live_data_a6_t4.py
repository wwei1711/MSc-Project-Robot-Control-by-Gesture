
# from collections import deque
# from threading import Lock
import numpy as np


import sys
sys.path.append('./my_pack')
from examples.my_pack import sim
import time
import cv2
import threading

import torch

global data1,data1i
缓冲区长度=100
data1=np.zeros([缓冲区长度,14])+np.arange(14)-7
data1i=0

global conflag
conflag=[2]
test=0
# test=1
if test==0:
    import myo
    class Listener(myo.DeviceListener):
        def on_paired(self, event):
            print("Hello, {}!".format(event.device_name))
            event.device.vibrate(myo.VibrationType.short)
        def on_unpaired(self, event):
            return False  # Stop the hub
        def on_orientation(self, event):
            global data1,data1i
            quat = event.orientation
            acc = event.acceleration
            gyro = event.gyroscope
        
            data_temp1=[acc.x, acc.y, acc.z]
            data_temp2=[gyro.x, gyro.y, gyro.z]
            data1[data1i%缓冲区长度,0:3]=data_temp1
            data1[data1i%缓冲区长度,11:]=data_temp2
            data1i+=1
            # print("quat: ", quat.x, quat.y, quat.z, quat.w)
            # print("acc: ", acc.x, acc.y, acc.z)
            # print("gyro: ", gyro.x, gyro.y, gyro.z)
            # ... do something with that
      
        def on_connected(self, event):
            event.device.stream_emg(True)
      
        def on_emg(self, event):
            global data1,data1i
            emg = event.emg
            # print("EMGs: ", emg)
            data_temp=np.asarray(emg)/128
            data1[data1i%缓冲区长度,3:11]=data_temp


# data_dict = {}

def coppeliasim_con():
    robot_p_now=[0,0,0]
    robot_p_aim=robot_p_now.copy()
    constep=0.05
    con_speed=0.001*3
    movstepspeed=0.01
    xyzrange=[0.1,0.1,0.1]
    
    robot_p_int=[-1.1222e-01,+2.9873e-01,+3.0382e-01-0.05]
    def errok():
        err=(robot_p_now[0]-robot_p_aim[0])**2+\
            (robot_p_now[1]-robot_p_aim[1])**2+\
            (robot_p_now[2]-robot_p_aim[2])**2
        ret1=np.sqrt(err)<0.002
        return ret1
    
    
    #Start Program and just in case, close all opened connections
    print('Program started')
    sim.simxFinish(-1)
    
    #Connect to simulator running on localhost
    #V-REP runs on port 19997, a script opens the API on port 19999
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    thistime = time.time()
    
    #Connect to the simulation
    if clientID != -1:
        print('Connected to remote API server')
    
        res,effector = sim.simxGetObjectHandle(clientID, 'tar', sim.simx_opmode_oneshot_wait)
        if res!=0:
            effector=-1
        
        res,effector_Position = sim.simxGetObjectPosition(clientID, effector, -1, sim.simx_opmode_oneshot_wait)
        robot_p_now=effector_Position.copy()
        robot_p_aim=effector_Position.copy()
        # robot_p_int=effector_Position.copy()
        print('Starting control loop')
        global conflag
        global close
        state=2
        while (sim.simxGetConnectionId(clientID) != -1):
            #Get image from Camera
            lasttime = thistime
            thistime = time.time()
            con=np.zeros([500,500])
            
            # cv2.imshow('con', con)
    
            print("conflag",conflag)
            keypress = cv2.waitKey(100) & 0xFF #will read a value of 255 if no key is pressed
    
            if keypress == ord('q'):
                close=0
                break
    
            # 状态切换 
            # 0 初始状态
            # 1 自动抓取状态
            if keypress == ord('0'):
                state=0
            elif keypress == ord('1'):
                state=1
            
            state=conflag[0]
            
            

            
            if keypress == ord('w'):
                # robot_p_aim[2]+=constep
                state=0
            elif keypress == ord('s'):
                # robot_p_aim[2]-=constep
                state=2
            if keypress == ord('a'):
                # robot_p_aim[0]+=constep
                state=4
            elif keypress == ord('d'):
                # robot_p_aim[0]-=constep
                state=3
            elif keypress == ord('x'):
                state=1
            elif keypress == ord('c'):
                state=conflag[0]
            print("state",state)
            

            if state==3:
                robot_p_aim[0]-=con_speed
            if state==4:
                robot_p_aim[0]+=con_speed
            if state==1:
                robot_p_aim[2]-=con_speed
            if state==0:
                robot_p_aim[2]+=con_speed
            text="state:{0}".format(state)
            cv2.putText(con, text, (50,50 ), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
            cv2.imshow('con', con)
            for i in range(3):
                if robot_p_aim[i]>=robot_p_int[i]+xyzrange[i]:
                    robot_p_aim[i]=robot_p_int[i]+xyzrange[i]
                if robot_p_aim[i]<=robot_p_int[i]-xyzrange[i]:
                    robot_p_aim[i]=robot_p_int[i]-xyzrange[i]
                
            errsum=0
            for ii in range(3):
                err_t=robot_p_aim[ii]-robot_p_now[ii]
                errsum=errsum+err_t**2
            errsum=errsum**0.5
            for ii in range(3):
                if(errsum>movstepspeed):
                    robot_p_now[ii]=robot_p_now[ii]+movstepspeed*(robot_p_aim[ii]-robot_p_now[ii])/errsum
                else:
                    robot_p_now[ii]=robot_p_aim[ii]
            # 反解位置，获得关节角度
    
            # 控制vrep中关节角度
            if effector>0:
                sim.simxSetObjectPosition(clientID, effector, -1, robot_p_now, sim.simx_opmode_oneshot)
            # 控制vrep中爪子开合
            ret=sim.simxSetFloatSignal(clientID, "con", state, sim.simx_opmode_oneshot)
            
        #结束连接
        sim.simxFinish(clientID)
    
    else:
        print('Could not connect to remote API server')
    
    
    sim.simxFinish(clientID)
    cv2.destroyAllWindows()
    print('Simulation ended')
def show(arg):
    global close
    global data1,data1i
    global conflag
    close=1000
    net2 = torch.load('net.pkl')
    while 1:
        time.sleep(0.2)
        if data1i>100:
            data_mean=np.mean(data1,0).tolist()
            data_abs_mean=np.mean(np.abs(data1),0).tolist()
            print("多线程数据测试:",data_mean,data_abs_mean)
            datain=data_mean[0:3]+data_abs_mean[0:3]+\
                    data_mean[3:11]+data_abs_mean[3:11]+\
                    data_mean[11:]+data_abs_mean[11:]
            X_test=np.asarray(datain).reshape([1,28])
            print("X_test:",X_test)
            cnn_test_x = torch.from_numpy(X_test) .type(torch.FloatTensor)
            out_test = net2(cnn_test_x)
            prediction = torch.max(out_test, 1)[1]
            pred_y = prediction.data.numpy()
            # np.sort()
            print("多线程预测结果:",pred_y,out_test.data.numpy())
            conflag=pred_y
        close-=1
        if close<0:
            break
if __name__ == '__main__':

    t1 = threading.Thread(target=show, args=(1,))
    t2 = threading.Thread(target=coppeliasim_con, args=())
    
    
    # test=0
    if test==1:
        data1i=1000
    
    t1.start()
    t2.start()
    # 

    if test==0:
        myo.init(sdk_path=r'C:\Users\WeiWei\Desktop\myo-python-master\myo-sdk-win-0.9.0')
        hub = myo.Hub()
        listener = Listener()


        while hub.run(listener.on_event, 500):
    
            pass
    