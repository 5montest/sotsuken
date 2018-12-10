import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

box_size = 4.0
R = 6400.0e+3

fig = plt.figure()
#if version > 1.0.0
#ax = plt.add_subplot(111, projection='3d')

start = 0
stop = 1
step = 1

Nnode = 4
for step in range(start, stop, step):
    print (step)
    tag = []
    x = []
    y = []
    z = []
    for n in range(Nnode):
        data = np.loadtxt("./result/%05d_%05d_%05d.dat" % (step, Nnode, n), skiprows=2, delimiter="\t");
        tag.extend(data[:,1])
        x.extend(data[:,3])
        y.extend(data[:,4])
        z.extend(data[:,5])

    x_tag = [[], [], [], []]
    y_tag = [[], [], [], []]
    z_tag = [[], [], [], []]
    size_tag = [[], [], [], []]

    for i in range(len(x)):
        x[i] = x[i]/R
        y[i] = y[i]/R
        z[i] = z[i]/R
        x_tag[int(tag[i])].append(x[i])
        y_tag[int(tag[i])].append(y[i])
        z_tag[int(tag[i])].append(z[i])
        size_tag[int(tag[i])].append(1.0)


    clr = ["orange", "gray", "red", "black"]

    ax = Axes3D(fig)
    for tag in range(4):
        ax.scatter(x_tag[tag], y_tag[tag], z_tag[tag], s=size_tag[tag], c=clr[tag], edgecolor=clr[tag], alpha=0.1)

    ax.set_aspect('equal')
    ax.set_xlim3d(-box_size, box_size)
    ax.set_ylim3d(-box_size, box_size)
    ax.set_zlim3d(-box_size, box_size)
    ax.view_init(9.0, 45.0)
    plt.show()