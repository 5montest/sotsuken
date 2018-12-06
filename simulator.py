import tkinter as Tkinter
import subprocess
import sys
import math

print("a")
root = Tkinter.Tk()
root.title(u"2-beam_Fizeau")
#root.geometry("1280x720")
root.geometry("1920x1080")

def calculation():

    #可変パラメータ
    theta = 45
    theta_s = 45
    t = 0

    #定数
    d = 10
    ramda1 = 780
    ramda2 = 779
    #光路
    L1 = 0.05
    L2 = 0.1
    L3 = 1
    L4 = 1
    sigma = 0.005

    #屈折率の計算 *空気の屈折率=1
    ITOref1 = -1.7 * 10 ** 6 * ramda1 + 2.6924
    ITOref2 = -1.7 * 10 ** 6 * ramda2 + 2.6924
    Quaref1 = -2 * 10 ** 5 * ramda1 + 1.5798
    Quaref2 = -2 * 10 ** 5 * ramda2 + 1.5798

    #屈折角
    Qtheta_t1 = math.asin(math.sin(theta) / Quaref1)
    Qtheta_t2 = math.asin(math.sin(theta) / Quaref2)
    Itheta_t1 = math.asin(math.sin(theta) / ITOref1)
    Itheta_t2 = math.asin(math.sin(theta) / ITOref2)


    #レーザ間隔X
    X1 = 2 * d * (10 ** -6) * math.tan(Itheta_t1) * math.cos(theta)
    X2 = 2 * d * (10 ** -6) * math.tan(Itheta_t2) * math.cos(theta)

    #光路計算
    L10 = L1 - X1 * math.tan(theta_s)
    L20 = L2 - X2 * math.tan(theta_s)
    L30 = L3 + X1 * math.tan(theta_s) - X1 * math.tan(theta)
    L30, = L3 + X2 - X2 * math.tan(theta)

    L5 = (2 * d * (10 ** -6)) / math.cos(Itheta_t1)
    L5, = (2 * d * (10 ** -6)) / math.cos(Itheta_t2)
    #BS内光路
    L0 = sigma / math.cos(math.asin((math.sin(theta)) / Qtheta_t1))



root.mainloop()