
import math
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
import matplotlib.pyplot as plt

    #可変パラメータ
theta = 45
theta_s = 45
t = 8 * 10 **-13

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

MA = np.zeros((41,41))
MB = np.zeros((41,41))
MC = np.zeros((41,41))
MD = np.zeros((41,41))

IS = np.zeros((41,41))

S = -10

for i in range(41):
    V = -10
    for j in range(41):
        MA[i,j] = R1 * math.sqrt((2 * P1 * 10 ** -3) / (C * ipsy * math.pi * W1 ** 2)) * math.exp(-(V ** 2 + S ** 2) / (2 * W1 ** 2))
        V += 0.5
    S += 0.5

S = -10

for i in range(41):
    V = -10
    for j in range(41):
        MB[i,j] = T1 * math.sqrt((2 * P1 * 10 ** -3) / (C * ipsy * math.pi * W1 ** 2)) * math.exp(-(V ** 2 + S ** 2) / (2 * W1 ** 2))
        V += 0.5
    S += 0.5

S = -10

for i in range(41):
    V = -10
    for j in range(41):
        MC[i,j] = R2 * math.sqrt((2 * P1 * 10 ** -3) / (C * ipsy * math.pi * W1 ** 2)) * math.exp(-(V ** 2 + S ** 2) / (2 * W1 ** 2))
        V += 0.5
    S += 0.5

S = -10

for i in range(41):
    V = -10
    for j in range(41):
        MD[i,j] = T2 * math.sqrt((2 * P1 * 10 ** -3) / (C * ipsy * math.pi * W1 ** 2)) * math.exp(-(V ** 2 + S ** 2) / (2 * W1 ** 2))
        V += 0.5
    S += 0.5

S = -10


for i in range(41):
    for j in range(41):
        IS[i,j] = (((MA[i,j] ** 2) + (MB[i,j] ** 2) + (MC[i,j] ** 2) + (MD[i,j] ** 2)) + 2 
                    * (MA[i,j] * MB[i,j] * math.cos(M1 - M11d) + MA[i,j] * MC[i,j] * math.cos(M1 - M2) 
                    + MA[i,j] * MD[i,j] * math.cos(M1 - M22d) + MB[i,j] * MC[i,j] * math.cos(M11d - M2)
                    + MB[i,j] * MD[i,j] * math.cos(M11d - M22d) + MC[i,j] * MD[i,j] * math.cos(M2 - M22d))) * C * ipsy / 2


fig = plt.figure()

ax = Axes3D(fig)
ax.set_zlim([0,0.00045])

x = y = np.arange(-10,10.5,0.5)
X, Y = np.meshgrid(x, y)
Z = np.matrix(IS)

surf = ax.plot_surface(X, Y, Z)

plt.show()

"""
print (MA)
print (MB)
print (MC)
print (MD)

print (IS)

print (theta)
print (theta_s)
print (Qtheta_t1)
print (Qtheta_t2)
print (Itheta_t1)
print (Itheta_t2)
print (1)
print (Quaref1)
print (Quaref2)
print (ITOref1)
print (ITOref2)
print (sigma)
print (d)
print (P1)
print (P2)
print (ramda1)
print (ramda2)
print (W1)
print (W2)
print (L1)
print (L2)
print (L3)
print (L4)
print (L5)
print (L5d)
print (L0)
print (L10)
print (L20)
print (L30)
print (L30d)
print (L100)
print (L200)
print (L300)
print (L300d)
print (L400)
print (L400d)
print (L1000)
print (L2000)
print (L3000)
print (L3000d)
print (L4000)
print (L4000d)
print (ipsy)
print (C)
print (X1)
print (X2)
print (R1)
print (R2)
print (T1)
print (T2)
print (t)
print (M1)
print (M11d)
print (M2)
print (M22d)

"""