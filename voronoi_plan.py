import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree, Voronoi

class Node:
    def __init__(self, x,y,z,cost,parent_index):
        self.x=x
        self.y=y
        self.z=z;
        self.cost=cost
        self.parent_index=parent_index



class VoronoiPlan:
    def __init__(self,map1):
        self.N_KNN=10
        self.map=np.load(map1)
        self.mapImage = np.load('mapImage.npy')


    def getObstaclePoint(self):
        obx,oby,obz,=[],[],[]
        for j in range(150):
            for k in range(400):
                for i in range(1):
                    if self.map[i][j][k]:
                        obx.append(k)
                        oby.append(j)
                        obz.append(i)
                        break

        print('obstacle size',len(obx))

        return obx,oby,obz


    def planning (self,sx,sy,gx,gy):
        ox,oy,oz=self.getObstaclePoint()
        sample_x,sample_y,sample_z=self.Voronoi_sampling(sx,sy,gx,gy,ox,oy,oz)
        print('OK')

        roadmap=self.generate_road_map(sample_x,sample_y,sample_z)
        print('roadmap done')
        rx, ry, rz = self.dijkstra_planning(sx, sy, gx, gy, roadmap, sample_x, sample_y, sample_z)
        return rx,ry,rz

    # all the postion is mody by 2 if we want to check the avilable , we need to mutipliy 2;


    def Voronoi_sampling(self,sx,sy,gx,gy,ox,oy,oz):
        oxyz=np.vstack((ox,oy)).T
        print('start now')
        sample_x,sample_y,sample_z=[],[],[]
        vor=Voronoi(oxyz)

        for [ix,iy] in vor.vertices:
            x=int(ix)
            y=int(iy)
            tmp=True
            for i in range(72):
                if self.map[i][y][x] or x<0 or x>399 or y<0 or y>149:
                    tmp=False
                    break
            if tmp:
                sample_x.append(ix*2)
                sample_y.append(iy*2)
                sample_z.append(0)


        plt.plot(sample_x,sample_y,'.')
        plt.show()
        sample_x.append(sx)
        sample_y.append(sy)
        sample_z.append(0)
        sample_x.append(gx)
        sample_y.append(gy)
        sample_z.append(0)
        return sample_x,sample_y,sample_z



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

        for i in range(len(rx)-1):
            dx=rx[i]-rx[i+1]
            dy=ry[i]-ry[i+1]
            theta=math.atan2(dy,dx)
            if theta<0:
                temp=theta+3.14*2
            else:
                temp=theta
            degree=temp*180/3.14
            rz[i]=degree
        rz[-1]=0


        if show:
            for i in range(len(rx)-1):

              plt.plot([rx[i], rx[i+1]],[-ry[i], -ry[i+1]], "-r")

            plt.show()
        return rx,ry,rz

    def generate_road_map(self,sample_x,sample_y,sample_z):
        road_map=[]
        n_sample=len(sample_x)
        sample_kd_tree=cKDTree(np.vstack((sample_x,sample_y)).T)

        for (i,ix,iy,iz) in zip(range(n_sample),sample_x,sample_y,sample_z):
            print(i)
            dist,indexes=sample_kd_tree.query([ix,iy],k=n_sample)
            edge_id=[]
            for j in range(1,len(indexes)) :
                nx=sample_x[indexes[j]]
                ny=sample_y[indexes[j]]
                nz=sample_z[indexes[j]]
                if not self.check_collision(ix,iy,iz,nx,ny,nz):
                    edge_id.append(indexes[j])
                    print('add edge')
                if len(edge_id)>=self.N_KNN :
                    break
            road_map.append(edge_id)
        print ('get road map success')
        show=1
        if show:
             for i, _ in enumerate(road_map):
                 for ii in range(len(road_map[i])):
                      ind = road_map[i][ii]
                      plt.plot(sample_x[i],-sample_y[i],'*')
                      plt.plot([sample_x[i], sample_x[ind]],
                      [-sample_y[i], -sample_y[ind]], "-k")

             plt.show()
        return road_map
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
    x=VoronoiPlan('360map.npy')
    rx,ry,rz=x.planning(50,50,750,50)
    plt.plot(rx,ry)
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
    plt.show()

    #print(rx)
    #print(ry)
    #print(rz)

if __name__=='__main__':
    main()