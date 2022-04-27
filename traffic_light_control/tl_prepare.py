import numpy
import cvxopt
from cvxopt import matrix, solvers
import traci
import tripInfo
import math
import random
import pandas as pd
import numpy as np


class Prepare_Stage():

    # 应急救援等级
    def getERL(self):
        """
        I级:特别重大
        II级：重大
        III： 较大
        IIII： 一般
        """
        return 1

    # 路段拥堵等级
    def getCLoRS(self, tl):
        # TODO 当车流量大的时候，平均速度更可靠，
        # if traci.inductionloop.getLastStepOccupancy(tl.tl_e1detId) == 0:
        #     return 4
        # elif
        # 当前道路默认不拥堵
        v = traci.lane.getLastStepMeanSpeed(tl.tl_laneId)
        if v < 25 * 1000/3600:
            return 1
        elif v < 35 * 1000/3600 and v >= 25 * 1000/3600:
            return 2
        elif v < 40 * 1000/3600 and v >= 35 * 1000/3600:
            return 3
        elif v >= 40 * 1000/3600:
            return 4
        else:
            print('未知路段拥堵等级')
            return 4

    # 时间紧迫等级
    def getTUL(self, tl):
        eta = tl.tl_eta.ETA
        if eta < 300:
            return 1
        elif eta < 600 and eta >= 300:
            return 2
        elif eta < 1800 and eta >= 600:
            return 3
        else:
            print('未知时间紧迫等级')
            return 3

    # 更新延长时间
    def update_extend_time(self, tl):
        new_extend_time = self.prepare_main(tl)
        if new_extend_time != tl.prepare_extend_time:

            return True
        else:
            return False
        pass

    # main
    def prepare_main(self, tl):
        ERL = self.getERL()
        CLoRS = self.getCLoRS(tl)
        TUL = self.getTUL(tl)
        RD = math.pow(ERL, 0.1031) * math.pow(CLoRS, 0.6053) * math.pow(TUL, 0.2915)
        # 用于计算中心值与聚类，为了实验效率，直接将结果保存在数组中
        # k_means = K_Means()
        # dataset = k_means.createDataSet()
        # centroids, cluster = k_means.kmeans(dataset, 5)
        # # print(centroids)

        centroids = [[1.367772091296443], [2.0591145857156907], [2.5319600114819347], [3.05867047203621], [3.557355219832053]]
        cluster = [[[1.0], [1.223912143262], [1.377465866113178], [1.521295064767786], [1.0740789185173687], [1.3145782311951237], [1.4795070477694328], [1.633990957911594], [1.1199308327890887], [1.37069694586409], [1.5426664945746749], [1.7037452488033173], [1.1536455232034402], [1.4119607648685337], [1.589107329807017]], [[1.8619315032538442], [2.095532024004062], [1.9444711121725657], [1.9998613753683074], [2.250766770060795], [2.0885154292505743], [2.0852344990353173], [2.177673151989747], [1.755035240940844], [2.148008943240249], [2.2432303935562947]], [[2.3798618064101698], [2.678442584660838], [2.314338674086822], [2.5561593952498263], [2.485782380146095], [2.3468509247790736], [2.6652806147758863], [2.59189923862605], [2.4175011382217297], [2.7455169188079442], [2.669926450536848]], [[2.832547206835738], [3.1879225261802286], [2.876858714643378], [3.0423792405675227], [2.9996704344169713], [3.172256952265955], [3.0899732969514266], [3.2677554044284576]], [[3.424080379236818], [3.5702527296121187], [3.6777325506472227]]]

        centroid = -1
        for index in range(len(cluster)):
            if [RD] in cluster[index]:
                centroid = index

        if centroid == 0:
            return 40
        elif centroid == 1:
            return 30
        elif centroid == 2:
            return 20
        elif centroid == 3:
            return 10
        elif centroid == 4:
            return 0
        else:
            print('Oops in prepare_main')
            return 0

        pass

class K_Means():
    # 计算欧拉距离
    def calcDis(self, dataSet, centroids, k):
        clalist = []
        for data in dataSet:
            diff = np.tile(data, (k,
                                  1)) - centroids  # 相减   (np.tile(a,(2,1))就是把a先沿x轴复制1倍，即没有复制，仍然是 [0,1,2]。 再把结果沿y方向复制2倍得到array([[0,1,2],[0,1,2]]))
            squaredDiff = diff ** 2  # 平方
            squaredDist = np.sum(squaredDiff, axis=1)  # 和  (axis=1表示行)
            distance = squaredDist ** 0.5  # 开根号
            clalist.append(distance)
        clalist = np.array(clalist)  # 返回一个每个点到质点的距离len(dateSet)*k的数组
        return clalist

    # 计算质心
    def classify(self, dataSet, centroids, k):
        # 计算样本到质心的距离
        clalist = self.calcDis(dataSet, centroids, k)
        # 分组并计算新的质心
        minDistIndices = np.argmin(clalist, axis=1)  # axis=1 表示求出每行的最小值的下标
        newCentroids = pd.DataFrame(dataSet).groupby(
            minDistIndices).mean()  # DataFramte(dataSet)对DataSet分组，groupby(min)按照min进行统计分类，mean()对分类结果求均值
        newCentroids = newCentroids.values

        # 计算变化量
        changed = newCentroids - centroids

        return changed, newCentroids

    # 使用k-means分类
    def kmeans(self, dataSet, k):
        # 随机取质心
        centroids = random.sample(dataSet, k)

        # 更新质心 直到变化量全为0
        changed, newCentroids = self.classify(dataSet, centroids, k)
        while np.any(changed != 0):
            changed, newCentroids = self.classify(dataSet, newCentroids, k)

        centroids = sorted(newCentroids.tolist())  # tolist()将矩阵转换成列表 sorted()排序

        # 根据质心计算每个集群
        cluster = []
        clalist = self.calcDis(dataSet, centroids, k)  # 调用欧拉距离
        minDistIndices = np.argmin(clalist, axis=1)
        for i in range(k):
            cluster.append([])
        for i, j in enumerate(minDistIndices):  # enymerate()可同时遍历索引和遍历元素
            cluster[j].append(dataSet[i])

        return centroids, cluster

    # 创建数据集
    def createDataSet(self, ):
        dataSet = []
        for i in range(4):
            for j in range(4):
                for k in range(3):
                    ii = i + 1
                    jj = j + 1
                    kk = k + 1
                    RD = math.pow(ii, 0.1031) * math.pow(jj, 0.6053) * math.pow(kk, 0.2915)
                    dataSet.append([RD])

        return dataSet