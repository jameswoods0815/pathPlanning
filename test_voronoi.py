import math
import numpy as np
import matplotlib.pyplot as plt
from dijkstra_search import DijkstraSearch
from scipy.spatial import cKDTree,Voronoi
show_animation =True

class VoronoiRoadMapPlanner:
    def __init__(self):
        self.N_KNN=10.0
        self.MAX_EDGE_LEN=30.0

    def planning (self, sx,sy,gx,gy,ox,oy, robot_radius):
        obstacle_tree=cKDTree(np.vstack((ox,oy)).T)
        sample_x, sample_y= self.voronoi_sampling(sx,sy,gx,gy,ox,oy)
        if show_animation:
            plt.plot(sample_x, sample_y,'.b')
        road_map_info=self.generate_road_map_info(
            sample_x,sample_y,robot_radius,obstacle_tree
        )

        rx,ry=DijkstraSearch(show_animation).search(sx,sy,gx,gy,sample_x,sample_y,road_map_info)
        return rx,ry
    @staticmethod
    def voronoi_sampleling(sx,sy,gx,gy,ox,oy):
        oxy=np.vstack((ox,oy)).T;
        vor=Voronoi(oxy)
        sample_x=[ix for[ix, _] in vor.vertices]
        sample_y=[iy for[_,iy] in vor.vertices]

        sample_x.append(sx)
        sample_y.append(sy)
        sample_x.append(gx)
        sample_y.append(gy)

        return sample_x, sample_y

    def is_collision(self, sx, sy,gx,gy,rr,obstacle_kd_tree):
       x=sx
       y=sy
       dx=gx-sx
       dy=gy-sy
       yaw=math.atan2(dy,dx)
       d=math.hypot(dx,dy)

       if d>= self.MAX_EDGE_LEN :
           return True
       D=rr
       n_step=round(d/D)

       for i in range(n_step):
           dist, _=obstacle_kd_tree.query([x,y])
           if dist <= rr :
               return True
           x+= D*math.cos(yaw)
           y+=D*math.sin(yaw)

       dist, _=obstacle_kd_tree.query([gx,gy])
       if dist<= rr:
           return True
       return False

    def generate_road_map_info (self, node_x,node_y, rr,obstacle_tree):
        road_map=[]
        n_sample=len(node_x)
        node_tree=cKDTree(np.vstack((node_x,node_y)).T)
        for (i,ix,iy) in zip(range(n_sample), node_x, node_y):
            dist, indexes= node_tree.query([ix,iy], k=n_sample)
            edge_id=[]
            for ii in range(1,len(indexes)):
                nx=node_x(indexes[ii])
                ny=node_y(indexes[ii])
                if not self.is_collision(ix,iy,nx,ny,rr,obstacle_tree):
                    edge_id.append(indexes[ii])
                if len(edge_id)>=self.M_KNN:
                    break
                road_map.append(edge_id)
        return road_map





