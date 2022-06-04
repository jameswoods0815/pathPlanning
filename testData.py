import math
import numpy as np
import matplotlib.pyplot as plt
from dijkstra_search import DijkstraSearch
from scipy.spatial import cKDTree, Voronoi

a=np.load('map.npy')

ox = []
oy = []

plt.figure(1)
plt.grid(True)
plt.axis("equal")
a1=np.load('360map.npy')
print("数据类型",type(a1))           #打印数组数据类型
print("数组元素数据类型：",a1.dtype) #打印数组元素数据类型
print("数组元素总数：",a1.size)      #打印数组尺寸，即数组元素总数
print("数组形状：",a1.shape)         #打印数组形状
print("数组的维度数目",a1.ndim)      #打印数组的维度数目

#map1=[];
#for i in range(4):
 #   for j in range (18):
  #      map1.append(a[j])
#np.save('360map.npy',map1)




for i in range(1) :
    for y in range(150):
        for x in range(400):
          if a1[18][y][x]:
                ox.append(x)
                oy.append(-y)

    plt.figure(1)
    plt.grid(True)
    plt.axis("equal")
    plt.plot(ox, oy, 'ro')
    plt.pause(1)
    plt.show()
