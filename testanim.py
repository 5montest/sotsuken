
import numpy as np                          #データ処理
import matplotlib.pyplot as plt             #プロット
import matplotlib.animation as animation    #アニメーション

from mpl_toolkits.mplot3d import Axes3D     #3次元

fig = plt.figure()

def plot(data):
    plt.cla()                      # 現在描写されているグラフを消去
    rand = np.random.randn(100)    # 100個の乱数を生成
    im = plt.plot(rand)            # グラフを生成

ani = animation.FuncAnimation(fig, plot, interval=100)
plt.show()