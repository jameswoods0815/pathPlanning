import random
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

class Node:
    def __init__(self, x,y,z,cost,parent_index):
        self.x=x
        self.y=y
        self.z=z
        self.cost=cost
        self.parent_index=parent_index


class PRM_PLAN():
    def __init__(self,map1,sample_num=500,k_nn=50,max_edge_len=8000):
        self.map=np.load(map1)
        dimx = len(self.map[0][0])
        dimy = len(self.map[0])
        dimz = len(self.map)
        self.N_sample=sample_num
        self.N_KNN=k_nn
        self.MAX_EDGE_LEN=max_edge_len
        self.mapImage=np.load('mapImage.npy')

        print(dimx, dimy, dimz)


    def prm_planning(self,sx,sy,gx,gy):

        sample_x, sample_y,sample_z=self.sample_points(sx,sy,gx,gy)
        road_map=self.generate_road_map(sample_x,sample_y,sample_z)
        rx,ry,rz=self.dijkstra_planning(sx,sy,gx,gy,road_map,sample_x,sample_y,sample_z)
        return rx,ry,rz

    def sample_points(self,sx,sy,gx,gy):
        sample_x=[]
        sample_y=[]
        sample_z=[]
        while (len(sample_x)<self.N_sample+1):
            tx=random.uniform(0,1)*799.99
            ty=random.uniform(0,1)*299.99
            tz=random.uniform(0,1)*359.99
            if not self.map[int(tz/5)][int(ty/2)][int(tx/2)] :
                sample_x.append(tx)
                sample_y.append(ty)
                sample_z.append(tz)

        sample_x.append(sx)
        sample_y.append(sy)
        sample_z.append(0)

        sample_x.append(gx)
        sample_y.append(gy)
        sample_z.append(0)

        return sample_x,sample_y,sample_z


    def generate_road_map(self,sample_x,sample_y,sample_z):
        road_map=[]
        n_sample=len(sample_x)
        sample_kd_tree=cKDTree(np.vstack((sample_x,sample_y)).T)

        for (i,ix,iy,iz) in zip(range(n_sample),sample_x,sample_y,sample_z):
            dist,indexes=sample_kd_tree.query([ix,iy],k=n_sample)
            edge_id=[]
            for j in range(1,len(indexes)) :
                nx=sample_x[indexes[j]]
                ny=sample_y[indexes[j]]
                nz=sample_z[indexes[j]]
                if not self.check_collision(ix,iy,iz,nx,ny,nz):
                    edge_id.append(indexes[j])
                if len(edge_id)>=self.N_KNN :
                    break
            road_map.append(edge_id)
        show=1
        if show:
             for i, _ in enumerate(road_map):
                 for ii in range(len(road_map[i])):
                      ind = road_map[i][ii]
                      plt.plot(sample_x[i],sample_y[i],'*')
                      plt.plot([sample_x[i], sample_x[ind]],
                      [sample_y[i], sample_y[ind]], "-k")

            # plt.show()
        return road_map


    def dijkstra_planning(self,sx,sy,gx,gy,roadmap, sample_x,sample_y,sample_z):
        rx=[]
        ry=[]
        rz=[]
        start_node=Node(sx,sy,0,0.0,-1)
        goal_node=Node(gx,gy,0,0.0,-1)
        open_set, closed_set=dict(),dict()
        open_set[len(roadmap)-2]=start_node

        path_found=True
        while True:
            if not open_set:
                path_found=False
                print('can not find a path')
                break
            ID=min(open_set,key=lambda o:open_set[o].cost)
            current=open_set[ID]
            if ID==(len(roadmap)-1):
                print('goal find now')
                goal_node.parent_index=current.parent_index
                goal_node.cost=current.cost
                break
            del open_set[ID]
            closed_set[ID]=current

            for i in range(len(roadmap[ID])):
                now_id=roadmap[ID][i]
                dx=sample_x[now_id]-current.x
                dy=sample_y[now_id]-current.y
                d=math.hypot(dx,dy)
                node=Node(sample_x[now_id],sample_y[now_id],sample_z[now_id],current.cost+d,ID)
                if now_id in closed_set:
                    continue
                if now_id in open_set:
                    if open_set[now_id].cost>node.cost:
                        open_set[now_id].cost=node.cost
                        open_set[now_id].parent_index=ID
                else:
                    open_set[now_id]=node
        if path_found is False:
            return [],[],[]
        rx.append(goal_node.x)
        ry.append(goal_node.y)
        rz.append(goal_node.z)
        parent_index=goal_node.parent_index
        while parent_index!=-1:
            node=closed_set[parent_index]
            rx.append(node.x)
            ry.append(node.y)
            rz.append(node.z)
            parent_index=node.parent_index
        show = 1
        if show:
            for i in range(len(rx)-1):

              plt.plot([rx[i], rx[i+1]],[ry[i], ry[i+1]], "-r")

            plt.show()
        return rx,ry,rz

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

    def showMap(self):
        ox = []
        oy = []
        # np.save('mapImage',self.map)
        for y in range(150):
            for x in range(400):
                if self.mapImage[y][x]:
                    ox.append(x * 2)
                    oy.append(y * 2)
        plt.grid(True)
        plt.axis("equal")
        plt.plot(ox, oy, 'ro')


def main():
    x=PRM_PLAN('360map.npy')
    rx,ry,rz=x.prm_planning(50,50,750,50)

    for i in range (len(rx)-1):
        dx=25*math.cos(rz[i])
        dy=25*math.sin(rz[i])
        plt.plot([rx[i],rx[i]+dx],[ry[i],ry[i]+dy],'b-')

    tmp=0
    for i in range (len(rx)-1):
        dx=rx[i]-rx[i+1]
        dy=ry[i]-ry[i+1]
        tmp=tmp+math.hypot(dx,dy)
    print ('the total length is :', tmp )



    x.showMap()
    plt.plot(rx,ry)
    plt.show()

if __name__ =='__main__':
    main()