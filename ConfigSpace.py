import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree


class ConfigSpace:
    angleRes = 5
    spaceRes = 2
    carX = [-25, 25, 25, -25]
    carY = [25, 25, -25, -25]
    map = np.zeros((150, 400))
    car = np.zeros((39, 39));

    def constructMap(self):
        # init map: obstable=1
        for i in range(400):
            self.map[0][i] = 1
            self.map[149][i] = 1
        for i in range(100):
            self.map[99][i + 50] = 1
            self.map[99][i + 250] = 1
        for i in range(150):
            self.map[i][0] = 1
            self.map[i][399] = 1
        for i in range(100):
            self.map[i][99] = 1
            self.map[i][299] = 1
            self.map[i + 50][199] = 1

    def __int__(self):
        return

    def rotateCar(self, angle):
        num = len(self.carX)
        carNowx = []
        carNowy = []
        for i in range(num):
            tmpx = self.carX[i] * math.cos(angle * 3.1415 / 180) - self.carY[i] * math.sin(angle * 3.1415 / 180)
            tmpy = self.carX[i] * math.sin(angle * 3.1415 / 180) + self.carY[i] * math.cos(angle * 3.1415 / 180)
            carNowx.append(tmpx)
            carNowy.append(tmpy)

        return carNowx, carNowy

    def Area(self, x1, y1, x2, y2, x3, y3):
        return abs(0.5 * (x1 * y2 - x2 * y1 + x2 * y3 - x3 * y2 + x3 * y1 - x1 * y3))

    def isInCar(self, x, y, carX, carY):
        a1 = self.Area(x, y, carX[0], carY[0], carX[1], carY[1])
        a2 = self.Area(x, y, carX[1], carY[1], carX[2], carY[2])
        a3 = self.Area(x, y, carX[2], carY[2], carX[3], carY[3])
        a4 = self.Area(x, y, carX[3], carY[3], carX[0], carY[0])

        if abs(a1 + a2 + a3 + a4) > 2501:
            return False
        else:
            return True

    def GetCarGrid(self, carX, carY):
        car=np.zeros((39,39))
        for i in range(39):
            for j in range(39):
                x = (i - 19) * 2
                y = (j - 19) * 2
                if self.isInCar(x, y, carX, carY):
                    car[i][j] = 1

        return car

    def getAngleSpace(self, angle):
        tmpMap = np.zeros((150 + 2 * 39, 400 + 2 * 39))
        finalMap = np.zeros((150 + 2 * 39, 400 + 2 * 39))
        NowMap = np.zeros((150, 400))
        carX, carY = self.rotateCar(angle)
        car = self.GetCarGrid(carX, carY)
        if 0:
            plt.figure(1)
            self.showGridCar(car)

        self.constructMap();
        for y in range(150):
            for x in range(400):
                tmpMap[y + 39][x + 39] = self.map[y][x]

        for y in range(39,150+39,1):
            for x in range(39,400+39,1):
                obs=False
                for j in range (39):
                    if obs :
                        break
                    for k in range (39):
                        if tmpMap[y-19+j][x-19+k] and car[j][k]:
                            obs=True
                            break
                finalMap[y][x]=obs

        for y in range(150):
            for x in range(400):
                NowMap[y][x] = finalMap[y+39][x+39]

        show = 0
        if show:
            ox = []
            oy = []
            for y in range(150):
                for x in range(400):
                    if NowMap[y][x]:
                        ox.append(x)
                        oy.append(-y)
            plt.figure(2)
            plt.grid(True)
            plt.axis("equal")
            plt.plot(ox, oy, 'ro')
            plt.show()
        return NowMap

    def getConfigSpace(self):
        map=[];
        for i in range(18):
            tmp=self.getAngleSpace(i*5)
            map.append(tmp)
            print(i)
        np.save('map.npy',map)
        return map




    def showCar(self, carX, carY):
        plt.figure(2)
        plt.plot([carX[0], carX[1]], [carY[0], carY[1]])
        plt.plot([carX[1], carX[2]], [carY[1], carY[2]])
        plt.plot([carX[2], carX[3]], [carY[2], carY[3]])
        plt.plot([carX[3], carX[0]], [carY[3], carY[0]])
        plt.grid(True)
        plt.axis("equal")
        plt.show()

    def showMap(self):
        ox = []
        oy = []
        #np.save('mapImage',self.map)
        for y in range(150):
            for x in range(400):
                if self.map[y][x]:
                    ox.append(x)
                    oy.append(-y)
        plt.grid(True)
        plt.axis("equal")
        plt.plot(ox, oy, 'ro')
        plt.show()

    def showGridCar(self, car):
        ox = []
        oy = []
        for y in range(39):
            for x in range(39):
                if car[y][x]:
                    ox.append(x)
                    oy.append(y)
        plt.grid(True)
        plt.axis("equal")
        plt.plot(ox, oy, 'ro')
        plt.show()


def main():
    x = ConfigSpace()
    x.constructMap()
    x.showMap()
    #carX, carY = x.rotateCar(20)
    #car = x.GetCarGrid(carX, carY)
    # x.showGridCar(car);
    #x.getConfigSpace()
    #x1 = x.getAngleSpace(130)

    # carX,carY = x.rotateCar(45)
    # x.showCar(carX,carY)


if __name__ == '__main__':
    main()
