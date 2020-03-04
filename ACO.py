import math
import random
import numpy as np
import matplotlib.pyplot as plt

def calcDist(city1, city2):
    return math.sqrt((city1.x - city2.x)**2 + (city1.y - city2.y)**2)

class City(object):
    def __init__(self, x, y, no):
        self.x = x
        self.y = y
        self.no = no

class Ant(object):
    def __init__(self, stop1):
        self.path = [stop1]

    def goNext(self, dist, pheromone, α, β):
        curPoint = self.path[-1]
        choices = list(set(range(len(dist))) - set(self.path))
        prob = []
        for nextPoint in choices:
            pathPheromone = pheromone[curPoint][nextPoint]
            prob.append((pathPheromone**α) * ((1 / dist[curPoint][nextPoint])**β))
        probSum = sum(prob)
        prob = np.array(prob) / probSum
        choicePoint = np.random.choice(choices, size=1, replace=False, p=prob)
        self.path.append(int(choicePoint))

    def goToFirstStop(self):
        self.path.append(self.path[0])

    def refreshState(self):
        #第一种方法：把结束位置作为初始位置
        finalPoint = self.path[-1]
        self.path.clear()
        self.path.append(finalPoint)

class ACO(object):
    def __init__(self, cities, maxIter=100, ρ=0.9, α=1, β=1, Q=1, antsNum=100):
        self.cities = cities
        self.maxIter = maxIter
        self.α = α
        self.β = β
        self.ρ = ρ
        self.Q = Q
        self.antsNum = antsNum
        self._citiesNum_ = len(cities)
        self._ants_ = []
        self._dist_ = None
        self._pheromone_ = None
        self.initParam()

    def initParam(self):
        self._dist_ = np.zeros((self._citiesNum_, self._citiesNum_), dtype=float)
        self._pheromone_ = np.zeros((self._citiesNum_, self._citiesNum_), dtype=float)
        #初始化距离矩阵、信息素矩阵为1.0
        for i in range(self._citiesNum_):
            for j in range(self._citiesNum_):
                if j > i:
                    self._dist_[i][j] = self._dist_[j][i] = calcDist(self.cities[i], self.cities[j])
                    self._pheromone_[i][j] = self._pheromone_[j][i] = 1.0
            self._dist_[i][i] = 0
            self._pheromone_[i][i] = 10000000000
        #初始化蚁群
        for i in range(self.antsNum):
            first_stop = random.choice(range(self._citiesNum_))
            self._ants_.append(Ant(first_stop))

    def calcAntPathDist(self, ant):
        dist = 0
        for i in range(len(ant.path) - 1):
            dist += calcDist(self.cities[ant.path[i]], self.cities[ant.path[i + 1]])
        return dist

    def run(self):
        bestPath = None
        shortestdist = 1000000000000
        plt.ion()
        for iter in range(self.maxIter):
            #蚁群寻路
            deltaPheromone = np.zeros((self._citiesNum_, self._citiesNum_), dtype=float)
            for ant in self._ants_:
                for _ in range(self._citiesNum_ - 1):
                    ant.goNext(self._dist_, self._pheromone_, self.α, self.β)
                ant.goToFirstStop()
                dist = self.calcAntPathDist(ant)
                
                if dist < shortestdist:
                    bestPath = ant.path[:]
                    shortestdist = dist
                for i in range(len(ant.path) - 1):
                    prePoint = ant.path[i]
                    aftPoint = ant.path[i + 1]
                    deltaPheromone[prePoint][aftPoint] += self.Q / dist
                    deltaPheromone[aftPoint][prePoint] = deltaPheromone[prePoint][aftPoint]
                ant.refreshState()

            #更新信息素
            self._pheromone_ = self._pheromone_ * self.ρ + deltaPheromone   
            print('iter' + str(iter))     
            print('best Path:' + str(bestPath))
            print('dist:' + str(shortestdist))
            print('-' * 100)

            if iter % 10 == 0:
                x = [self.cities[index].x for index in bestPath]
                y = [self.cities[index].y for index in bestPath]
                plt.cla()
                plt.plot(x,y)
                plt.pause(0.5)
        plt.ioff()
        plt.show()

        print('best Path:' + str(bestPath))
        print('dist:' + str(shortestdist))

# def clacmin(coors):
#     dist = 0
#     for i in range(len(coors) - 1):
#         dist += math.sqrt((coors[i][0] - coors[i+1][0])**2 + (coors[i][1] - coors[i+1][1])**2)
#     dist += math.sqrt((coors[0][0] - coors[-1][0])**2 + (coors[0][1] - coors[-1][1])**2)
#     print('***min:' + str(dist))

if __name__ == '__main__':
    coors = [[749, 32],[694, 313], [384, 322], [628, 494], [538, 777], [794, 613], [1049, 772], [959, 494], [1206, 322], [893, 313]]
    cities = []
    i = 0
    for coor in coors:
        c = City(coor[0], coor[1], i)
        i += 1
        cities.append(c)
    ACOInstance = ACO(cities, 200, 0.9, 1, 1, 1, 50)
    # ACOInstance = ACO(cities, 200, 0.9, 1, 1, 1, 100)
    ACOInstance.run()
