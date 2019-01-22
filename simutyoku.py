import tkinter as Tkinter                   #GUI
import math                                 #数学関数
import sys

import numpy as np                          #データ処理
import matplotlib.pyplot as plt             #プロット

from mpl_toolkits.mplot3d import Axes3D     #3次元
from matplotlib.ticker import ScalarFormatter

#root = Tkinter.Tk()
#root.title(u"2-beam_Fizeau")
#root.geometry("1280x720")
#root.geometry("1920x1080")


def calc(t,theta,theta_s,arr1,arr2):
    #可変パラメータ
    #theta = 45
    #theta_s = 45
    #t = 0

    #定数
    d = 10
    ramda1 = 637
    ramda2 = 639

    P1 = 3
    P2 = 3

    W1 = 3
    W2 = 3

    #光路
    L1 = 0.05
    L2 = 0.1
    L3 = 1
    L4 = 1
    sigma = 0.005

    #屈折率の計算 *空気の屈折率=1
    ITOref1 = (-1.7 * 10 ** 6 * ramda1 * 10 ** -9) + 2.6924
    ITOref2 = (-1.7 * 10 ** 6 * ramda2 * 10 ** -9) + 2.6924
    Quaref1 = -2 * (10 ** 5) * (ramda1 * 10 ** -9) + 1.5798
    Quaref2 = -2 * (10 ** 5) * (ramda2 * 10 ** -9) + 1.5798

    #屈折角
    Qtheta_t1 = math.degrees(math.asin(math.sin(math.radians(theta)) / Quaref1))
    Qtheta_t2 = math.degrees(math.asin(math.sin(math.radians(theta)) / Quaref2))

    Itheta_t1 = math.degrees(math.asin(math.sin(math.radians(theta)) / ITOref1))
    Itheta_t2 = math.degrees(math.asin(math.sin(math.radians(theta)) / ITOref2))

    #材料厚さ
    d1 = 10
    d2 = 1
    d3 = 1

    Y1 = 0.2
    Y2 = d2 - Y1 - d3 * math.tan(math.radians(Itheta_t1))


    #レーザ間隔X
    X1 = 2 * d1 * (10 ** -6) * math.cos(math.radians(theta)) * math.tan(math.radians(Itheta_t1))
    X2 = 2 * d1 * (10 ** -6) * math.cos(math.radians(theta_s)) * math.tan(math.radians(Itheta_t2))

    #光路計算
    #後退光路
    L10 = L1 - X1 * math.tan(math.radians(theta_s))
    L20 = L2 - X2 * math.tan(math.radians(theta_s))
    L30 = L3 + X1 * math.tan(math.radians(theta_s)) - X1 * math.tan(math.radians(theta))
    L30d = L3 + X2 - X2 * math.tan(math.radians(theta))

    #材料光路
    L5 = (2 * d * 10 ** -6) / math.cos(math.radians(ITOref1))
    L5d = (2 * d * 10 ** -6) / math.cos(math.radians(ITOref2))
    L50 = d3 * 10 ** -6 / math.cos(math.radians(Itheta_t1))
    L50d = d3 * 10 ** -6 / math.cos(math.radians(Itheta_t2))
    #BS内光路
    L0 = sigma / math.cos(math.asin((math.sin(math.radians(theta))) / Quaref1))

    #前進光路
    L100 = L1 + X1 * math.tan(math.radians(theta_s))
    L200 = L2 + X2 * math.tan(math.radians(theta_s))
    L300 = L3 - X1 * math.tan(math.radians(theta_s)) + X1 * math.tan(math.radians(theta))
    L300d = L3 + X2 - X2 * math.tan(math.radians(theta))
    L400 = L4 - X1 * math.tan(math.radians(theta))
    L400d = L4 - X2 * math.tan(math.radians(theta))

    #後進光路
    L1000 = L1 - X1 * 2 * math.tan(theta_s)
    L2000 = L2 - X2 * 2 * math.tan(theta_s)
    L3000 = L30 + 2 * X1 * math.tan(math.radians(theta)) - X1 * 2 * math.tan(math.radians(theta))
    L3000d = L30d + 2 * X2 - X2 * 2 * math.tan(math.radians(theta))
    L4000 = L4 + X1 * math.tan(math.radians(theta))
    L4000d = L4 + X2 * math.tan(math.radians(theta))

    ipsy = 8.854 * (10 ** -12)
    C = 3 * (10 ** 8)

    #反射率,透過率
    R1 = (math.sin(math.radians(theta - Itheta_t1)) ** 2) / (math.sin(math.radians(theta + Itheta_t1)) ** 2)
    R2 = (math.sin(math.radians(theta - Itheta_t2)) ** 2) / (math.sin(math.radians(theta + Itheta_t2)) ** 2)
    T1 = (math.sin(math.radians(2 * theta)) * math.sin(math.radians(2 * Itheta_t1))) / (math.sin(math.radians(theta + Itheta_t1))) ** 2
    T2 = (math.sin(math.radians(2 * theta)) * math.sin(math.radians(2 * Itheta_t2))) / (math.sin(math.radians(theta + Itheta_t2))) ** 2

    M1 = (-2 * math.pi / (ramda1 * 10 ** -9)) * ((C * t) - (L1 + L0 * Quaref1 + L3 + L4))
    M11d = (-2 * math.pi / (ramda1 * 10 ** -9)) * ((C * t) - (L10 + L0 * Quaref1 + L30 + L4 + L5 * ITOref1))

    M2 = (-2 * math.pi / (ramda2 * 10 ** -9)) * ((C * t) - (L2 + L3 + L4))
    M22d = (-2 * math.pi / (ramda2 * 10 ** -9)) * ((C * t) - (L20 + L30d + L4 + L5d * ITOref2))

    #電界項配列の定義
    MA = np.zeros((41,41))
    MB = np.zeros((41,41))
    MC = np.zeros((41,41))
    MD = np.zeros((41,41))

    IS = np.zeros((41,41))

    Sramda1 = math.sin(math.radians(90 - 34.3 - Itheta_t1)) * L50 * 1000
    Sramda2 = math.sin(math.radians(90 - 34.3 - Itheta_t2)) * L50d * 1000

    S = np.abs(Sramda1 - Sramda2) * 1000

    #電界項の計算

    MA = R1 * T1 * math.sqrt((2 * P1 * 10 ** -3) / (C * ipsy * math.pi * W1 ** 2)) * np.exp(-((arr1 + 0) ** 2 + arr2 ** 2) / (2 * (W1 ** 2)))

    MB = T1 ** 2 * math.sqrt((2 * P1 * 10 ** -3) / (C * ipsy * math.pi * W1 ** 2)) * np.exp(-((arr1 + 0) ** 2 + arr2 ** 2) / (2 * (W1 ** 2)))

    MC = R2 * T2 * math.sqrt((2 * P2 * 10 ** -3) / (C * ipsy * math.pi * W2 ** 2)) * np.exp(-((arr1 + S) ** 2 + arr2 ** 2) / (2 * (W2 ** 2)))

    MD = T2 ** 2 * math.sqrt((2 * P2 * 10 ** -3) / (C * ipsy * math.pi * W2 ** 2)) * np.exp(-((arr1 + S) ** 2 + arr2 ** 2) / (2 * (W2 ** 2)))

    print (MD)

    #光強度の計算
    IS = (((MA ** 2) + (MB ** 2) + (MC ** 2) + (MD ** 2)) + 2 
                * (MA * MB * math.cos(M1 - M11d) + MA * MC * math.cos(M1 - M2) 
                + MA * MD * math.cos(M1 - M22d) + MB * MC * math.cos(M11d - M2)
                + MB * MD * math.cos(M11d - M22d) + MC * MD * math.cos(M2 - M22d))) * C * ipsy / 2


    return IS

def input_val():
    print ("tmin[ps]")
    tmin = input('>>')
    print ("tmax[ps]")
    tmax = input('>>')
    print ("step")
    step = input('>>')

    tmax = float(tmax)
    tmin = float(tmin)
    step = float(step)

    abs = tmax - tmin
    frame = abs / step

    frame = int(frame)

    main(tmin,tmax,step,abs,frame)

def main(tmin,tmax,step,abs,frame):
    #fig = plt.figure()
    fig = plt.figure(figsize=(5, 5))
    ax = Axes3D(fig)
    x = y = np.arange(-0.04,0.042,0.002)
    #x = y = np.array([-0.04,-0.038,-0.036,-0.034,-0.032,-0.03,-0.028,-0.026,-0.024,-0.022,-0.02,-0.018,-0.16,-0.014,-0.012,-0.01,-0.008,-0.006,-0.004,-0.002,0,0.002,0.004,0.006,0.008,0.01,0.12,0.014,0.016,0.018,0.02,0.022,0.024,0.026,0.028,0.03,0.032,0.034,0.036,0.038,0.04])
    X, Y = np.meshgrid(x, y)
   
    arr1 = X 

    arr2 = X.transpose()

    for cnt in range(frame+1):
        plt.cla()
        #ax.set_zlim([0,12])
        #ax.set_xlim([-0.04,0.04])
        #ax.set_ylim([-0.04,0.04])
        IS = calc((tmin + (step * cnt)) * 10 ** -12,45,45,arr1,arr2)

        #print (np.ndarray.max(IS))
        Z = np.matrix(IS)
        surf = ax.plot_surface(X, Y, Z,color='white',edgecolor='g',shade=False)

        plt.title("t="+str(round(tmin+(step*cnt),2))+" [ps]",fontsize=24)
        ax.set_xlabel("[mm]",fontsize=18)
        ax.set_ylabel("[mm]",fontsize=18)
        ax.set_zlabel("[a.u.]",fontsize=18)
        ax.view_init(25, 45)
        plt.tick_params(labelsize = 20)
        ax.zaxis.set_major_formatter(ScalarFormatter(useMathText=True))
        plt.gca().ticklabel_format(style="sci", scilimits=(0,0), axis="z")
        ax.zaxis.offsetText.set_fontsize(20)
        plt.savefig("testSci"+str(cnt)+".png",bbox_inches="tight")

        plt.pause(0.0001)

if __name__ == '__main__':
    input_val()