#------ADRC Control ZMP NAO----------------
#------WORKING!!!!-------------------------
#------09/03/2017------

import os
import sys
import time
from naoqi import ALProxy
import numpy as np
import scipy
import matplotlib.pyplot as plt
import msvcrt

IP = "192.168.0.100"
PORT = 9559

tts = ALProxy("ALTextToSpeech",IP,PORT)
postureProxy = ALProxy("ALRobotPosture",IP,PORT)
motionProxy = ALProxy("ALMotion", IP, PORT)
memoryProxy = ALProxy("ALMemory", IP, PORT)
postureProxy.goToPosture("Stand",0.5)
T = 0.001
Tf = 100
#Parameters
g = 9.81;
zc = 0.333
dx = 0
x = 0
#States
zmp = np.zeros(Tf)
zmpd = 0.0*np.ones(Tf)
x = np.zeros(Tf)
u = np.zeros(Tf)
dx = np.zeros(Tf)
#Observed states
v = np.zeros(Tf)
zmpg = np.zeros(Tf)
xg = np.zeros(Tf)
dxg = np.zeros(Tf)
e = np.zeros(Tf)
k = 0
x0 = 0
#Control variables
K1 = 6.2917*(g/zc)*T
K2 = 37.7014*T
K3 = 32.5359*T
Ks = 1
fractionMaxSpeed = 0.1

plt.ion()
fig = plt.figure(1)
for k in range(Tf):
    # Get The Accelerometer X value
    ddx = memoryProxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value")
    # Get The Left Foot Force Sensor Values
    LFsrFL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    # Total weight at the left foot
    WLF = LFsrFL+LFsrFR+LFsrBL+LFsrBR
    # ZMP at the left foot
    ZMPxL = (LFsrFL*0.07025 + LFsrFR*0.07025 - LFsrBL*0.03025 - LFsrBR*0.02965)/WLF   
    # Get The Right Foot Force Sensor Values
    RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    # Total weight at the right foot
    WRF = RFsrFL+RFsrFR+RFsrBL+RFsrBR
    # ZMP at the right foot
    ZMPxR = (RFsrFL*0.07025 + RFsrFR*0.07025 - RFsrBR*0.03025 - RFsrBL*0.02965)/WRF
    #state variables
    zmp[k] = (ZMPxL + ZMPxR)/2
    x[k] = (ZMPxL + (zc/g)*ddx)+0.008
    dx[k] = (x[k]-x[k-1])/T
    x0 = x
    #Control input
    if(k>0):
        #Controller
        e[k] = zmpd[k]-zmp[k]
        if(e[k] > 0.001):
            ux =  K1*abs(e[k]) - K2*(xg[k]) - K3*dxg[k] - Ks*v[k]
        elif(e[k] < -0.001):
            ux = -K1*abs(e[k]) + K2*(xg[k]) + K3*dxg[k] + Ks*v[k]
        else:
            ux = 0
        motionProxy.changeAngles("LHipPitch", -ux, fractionMaxSpeed)
        motionProxy.changeAngles("RHipPitch", -ux, fractionMaxSpeed)
        #Observer
        zmpg[k] = zmpg[k-1]*(1-T) + T*xg[k-1] + (T*T/2)*dxg[k-1] + np.tanh(zmp[k-1]-zmpg[k-1]) + 0.04097*ux
        xg[k] = xg[k-1] + T*dxg[k-1] + np.tanh(x[k-1]-xg[k-1]) + T*T/2*ux
        dxg[k] = dxg[k-1] + np.tanh(dx[k-1]-dxg[k-1]) + T*ux
        v[k] = v[k-1] + np.tanh((x[k]-xg[k-1])-v[k-1])
    else:
        e[k] = 0
        ux = 0
        zmpg[k] = 0
        zmp[k] = 0
        xg[k] = 0
        x[k] = 0
        dxg[k] = 0
        dx[k] = 0
        v[k] = 0
    # Plots
    plt.subplot(4,1,1)
    plt.title('Control de balance ADRC con ZMP',fontsize = 20)
    plt.plot(k,zmp[k],'ko')
    plt.plot(k,zmpd[k],'bo')
    plt.ylabel('zmp [m]',fontsize = 18)
    plt.legend(['ZMP_fsr','ZMP_ref'])

    plt.subplot(4,1,2)
    plt.plot(k,x[k],'ko')
    plt.plot(k,xg[k],'ro')
    plt.ylabel('x [m]',fontsize = 18)
    plt.legend(['x_fsr/acc','x_obs'])
    
    plt.subplot(4,1,3)
    plt.plot(k,dx[k],'ko')
    plt.plot(k,dxg[k],'ro')
    plt.ylabel('dx [m/s]',fontsize = 18)
    plt.legend(['dx_acc','dx_obs'])

    plt.subplot(4,1,4)
    plt.plot(k,10*v[k],'ko')
    plt.ylabel('w [N]',fontsize = 18)
    plt.legend(['Disturbance'])
    plt.pause(T)
plt.savefig("measuresADRC.jpg")
