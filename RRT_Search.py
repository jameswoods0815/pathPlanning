import random
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

class Node:
    def __init__(self, x,y,z,cost,parent):
        self.x=x
        self.y=y
        self.z=z
        self.cost=cost
        self.parent=parent


class RRT_Search_ALgo:

    def __init__(self,map1,startx,starty,goalx,goaly,expend_dis=60,path_res=2,goal_sample_rate=50,max_iter=50000):
        self.map=np.load(map1)
        self.start=Node(startx,starty,0,0,None)
        self.end=Node(goalx,goaly,0,0,None)
        self.expend_dis=expend_dis
        self.path_res=path_res
        self.goal_sample_rate=goal_sample_rate
        self.max_iter=max_iter
        self.node_list=[]
        self.mapImage=np.load('mapImage.npy')

    # if false, the path is OK: just go
    def check_collision(self,sx,sy,sz,gx,gy,gz):
        if sx<0 or sx>=800 or sy<0 or sy>=300 or sz<0 or sz>360 :
            return True
        if gx<0 or gx>=800 or gy<0 or gy>=300 or gz<0 or gz>360 :
            return True
        # check angle first
        dx=gx-sx
        dy=gy-sy
        angle=math.atan2(dy,dx)
        if angle < 0:
            angle = angle + 3.1415 * 2

        Degree=angle*180/3.14

        step1=int(Degree-sz)/5+1
        for i in range(int(abs(step1))) :
            temp=sz+i*5
            if temp>=360:
                temp=temp-360
            if temp<0:
                temp=temp+360
            indexz=int(temp/5)
            if self.map[indexz][int(sy/2)][int(sx/2)] :
                return True

        step2=int (gz-Degree)/5+1
        for i in range(int(abs(step2))):
            temp=gz+i*5
            if temp>=360:
                temp=temp-360
            if temp<0:
                temp=temp+360
            indexz=int(temp/5)
            if self.map[indexz][int(gy/2)][int(gx/2)]:
                return True
        # check the path:
        dis=math.hypot(dx,dy)
        step3=int(dis/2)+1

        tempx=sx
        tempy=sy
        for i in range(int(step3)):
            tempx+=2*math.cos(angle)
            tempy+=2*math.sin(angle)
            indexz=int(Degree/5)
            indexx=int (tempx/2)
            indexy=int(tempy/2)
            if indexz<0:
                indexz=0
            if indexz>71:
                indexz=71
            if indexx>399:
                indexx=399
            if indexy>299:
                indexy=299
            if indexy<0:
                indexy=0

            if self.map[indexz][indexy][indexx]:
                return True
        return False
    def RRT_Planning(self):
        rx,ry,rz=[],[],[]
        self.node_list.append(self.start)
        for i in range (self.max_iter):
            print (i)
            rnd_node=self.getRandomNode()
            nearest_index=self.get_nearest_node_index(rnd_node)
            nearestNode=self.node_list[nearest_index]
            newNode= self.explore(nearestNode,rnd_node)
            if not self.check_collision(nearestNode.x,nearestNode.y,nearestNode.z,newNode.x,newNode.y,newNode.z):
                self.node_list.append(newNode)
            if self.cal_dist_to_goal(self.node_list[-1])<=self.expend_dis :
                final_node=self.explore(self.node_list[-1],self.end)

                if not self.check_collision(final_node.x,final_node.y,final_node.z,self.end.x,self.end.y,self.end.z):
                     rx,ry,rz=self.generate_final_path(len(self.node_list)-1)
                     print('path is found')
                     show=1
                     if show:
                         for k in range(len(self.node_list)):
                             if self.node_list[k].parent :
                                plt.plot([self.node_list[k].x,self.node_list[k].parent.x],[self.node_list[k].y,self.node_list[k].parent.y],'g-')

                     return rx,ry,rz


        return rx,ry,rz


    def getRandomNode(self):
        if random.randint(0,100) >self.goal_sample_rate:
            x=random.uniform(0,799.99)
            y=random.uniform(0,299.99)
            z=random.uniform(0,359.99)
            node=Node(x,y,z,0,None)
        else :
            node=self.end
        return node

    def get_nearest_node_index(self, rndNode):
        dis_list=[(node.x-rndNode.x)**2+(node.y-rndNode.y)**2 for node in self.node_list]
        index=dis_list.index(min(dis_list))
        return index


    def explore(self,nearestNode,rnd_node):
        dx=rnd_node.x-nearestNode.x
        dy=rnd_node.y-nearestNode.y
        theta=math.atan2(dy,dx)
        dis=math.hypot(dx,dy)

        if self.expend_dis>dis :
            extend_len=dis
        else :
            extend_len=self.expend_dis

        x=nearestNode.x+extend_len*math.cos(theta)
        y=nearestNode.y+extend_len*math.sin(theta)
        if theta<0:
            Degree=theta+2*3.14
        else:
            Degree=theta
        z=Degree*180/3.14
        newNode=Node(x,y,z,0,nearestNode)
        return newNode

    def cal_dist_to_goal(self,node):
        dx=node.x-self.end.x
        dy=node.y-self.end.y
        return math.hypot(dx,dy)

    def generate_final_path(self,goal_index):
        rx,ry,rz=[],[],[]
        rx.append(self.end.x)
        ry.append(self.end.y)
        rz.append(self.end.z)
        node=self.node_list[goal_index]
        while node.parent is not None:
            rx.append(node.x)
            ry.append(node.y)
            rz.append(node.z)
            node=node.parent
        # append start
        rx.append(node.x)
        ry.append(node.y)
        rz.append(node.z)
        return rx,ry,rz

    def showMap(self):
        ox = []
        oy = []
        #np.save('mapImage',self.map)
        for y in range(150):
            for x in range(400):
                if self.mapImage[y][x]:
                    ox.append(x*2)
                    oy.append(y*2)
        plt.grid(True)
        plt.axis("equal")
        plt.plot(ox, oy, 'ro')
        #plt.show()

def main():
    x=RRT_Search_ALgo('360map.npy',50,50,750,50)
    rx,ry,rz=x.RRT_Planning()
    print(rx,ry,rz)
    for i in range(len(rx) - 1):
        dx = 25 * math.cos(rz[i])
        dy = 25 * math.sin(rz[i])
        plt.plot([rx[i], rx[i] + dx], [ry[i], ry[i] + dy], 'b-')

    tmp = 0
    for i in range(len(rx) - 1):
        dx = rx[i] - rx[i + 1]
        dy = ry[i] - ry[i + 1]
        tmp = tmp + math.hypot(dx, dy)
    print('the total length is :', tmp)

    x.showMap()
    plt.plot(rx,ry)
    plt.show()

if __name__=='__main__':
    main()
