import math                                 #数学関数
import sys

import numpy as np                          #データ処理
import plotly.offline as offline
import plotly.graph_objs as go


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
    ramda1 = 780
    ramda2 = 779

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


    #レーザ間隔X
    X1 = 2 * d * (10 ** -6) * math.tan(math.radians(Itheta_t1)) * math.cos(math.radians(theta))
    X2 = 2 * d * (10 ** -6) * math.tan(math.radians(Itheta_t2)) * math.cos(math.radians(theta))

    #光路計算
    #後退光路
    L10 = L1 - X1 * math.tan(math.radians(theta_s))
    L20 = L2 - X2 * math.tan(math.radians(theta_s))
    L30 = L3 + X1 * math.tan(math.radians(theta_s)) - X1 * math.tan(math.radians(theta))
    L30d = L3 + X2 - X2 * math.tan(math.radians(theta))

    #材料光路
    L5 = (2 * d * 10 ** -6) / math.cos(math.radians(ITOref1))
    L5d = (2 * d * 10 ** -6) / math.cos(math.radians(ITOref2))
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

    S = -10

    #電界項の計算
    MA = R1 * math.sqrt((2 * P1 * 10 ** -3) / (C * ipsy * math.pi * W1 ** 2)) * np.exp(-(arr1 + arr2 ) / (2 * W1 ** 2))

    MB = T1 * math.sqrt((2 * P1 * 10 ** -3) / (C * ipsy * math.pi * W1 ** 2)) * np.exp(-(arr1 + arr2) / (2 * W1 ** 2))

    MC = R2 * math.sqrt((2 * P1 * 10 ** -3) / (C * ipsy * math.pi * W1 ** 2)) * np.exp(-(arr1 + arr2) / (2 * W1 ** 2))

    MD = T2 * math.sqrt((2 * P1 * 10 ** -3) / (C * ipsy * math.pi * W1 ** 2)) * np.exp(-(arr1 + arr2) / (2 * W1 ** 2))


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
    x = y = np.arange(-10,10.5,0.5)
    X, Y = np.meshgrid(x, y)
    arr1 = X ** 2
    arr2 = X.transpose() ** 2

    frames = []

    for cnt in range(frame):
        IS = calc((tmin + (step * cnt)) * 10 ** -12,45,45,arr1,arr2)
        Z = IS
        scatter = go.Surface(x=X,y=Y,z=Z)
        data = [scatter,scatter]

        frames.append({"data": data})
        
    layout = go.Layout(
        title='title',
        autosize=True,
        scene = dict(
        zaxis = dict(
            range=[0,0.00045],),),
        width=1280,height=920,
    )

    fig = go.Figure(data=data,layout=layout,frames=frames)
    offline.plot(fig,filename='test.html')


if __name__ == '__main__':
    input_val()