import traci
import json
import numpy as np

"""
get Saturate Rate and Arrival Rate
文档使用方法： 首先readlaneId()，然后调用generate_arrival_rate_main_()，
函数上面的内容为获取一段时间内的到达率，下半部分为各时间段到达率取均值+标准差
先执行上半部分函数，再执行下半部分函数，各部分函数执行时，另一半注释掉
"""

# class
class  Listener(traci.StepListener):
    """ 计算包和流率 """
    N = 0   # Initial Queue Number
    NN = 0  # Queue Number at the end of the green phase
    gt = 45 # greentime
    GBvIds = [] # 绿灯开始时道路上的车辆
    GEvIds = [] # 绿灯开始到结束期间道路上经过的车辆

    def getSaturateFlowRate(self, t):
        """ Return the saturation flow rate, for every lane, the S is the same. """
        laneId = '-gneE74_2'
        tl = 'gneJ47'
        # laneId = '-gneE10_2'
        # tl = 'gneJ10'
        linkId = 14
        Links = traci.trafficlight.getControlledLinks(tl)
        # linkId = ''
        # get linkId
        # for index in range(len(Links)):
        #     if laneId == Links[index][0][0]:
        #         linkId = index
        #         break

        redYellowGreenState = traci.trafficlight.getRedYellowGreenState(tl)
        if t >= 151 and t<=196:
            if (redYellowGreenState[linkId] == 'G'):
                self.gt -= 1
                self.GEvIds += traci.lane.getLastStepVehicleIDs(laneId)
                if self.gt == 44:
                    self.N = traci.lane.getLastStepVehicleNumber(laneId)
                    self.GBvIds += traci.lane.getLastStepVehicleIDs(laneId)

                if self.gt == 0:
                    self.NN = traci.lane.getLastStepVehicleNumber(laneId)
                    arrivalvIds = list(set(self.GEvIds).difference(set(self.GBvIds)))
                    arrivalvIdsNumber = len(arrivalvIds)
                    S = (self.N - self.NN + arrivalvIdsNumber) / 45
                    print(S)

        return True

class TripInfoListener(traci.StepListener):
    """ 生成到达率文件，对于不同的laneId，对应不同的到达率 """

    laneIds = []

    BeginVIdsDict = {}       # key:laneId value:[vhId,] Initial
    EndVIdsDict = {}         # key:laneId value:[vhId,]   In the interval time end
    ArrivalRatesDict = {}         # key:laneId value:arrival rate
    AverageArrivalRatesDict = {}  # 平均到达率

    N = 0           # Initial Queue Number
    NN = 0          # Queue Number at the end of the interval time
    it = 45         # interval time 间隔时间

    IntervalFlag = True

    _saturate_rate_ = 0.7111111111111111  # SaturateFlowRate

    def readlaneId(self):
        # 读取XML文件
        net = open("redefine.net.xml", "r")
        flag = 1
        while flag:
            line = net.readline()

            if line[5:12] == "tlLogic":
                flag = 0

            if line[9:13] == "lane" and line[18] != ":":
                laneId = ''
                for i in line[18:27]:
                    if i == '"':
                        break
                    else:
                        laneId += i
                self.laneIds.append(laneId)
            else:
                continue

        net.close()
        return True

    def initialDict(self):
        """ 生成初始字典
        BeginVIdsDict = {}       # key:laneId value:[vhId,] Initial
        EndVIdsDict = {}         # key:laneId value:[vhId,]   In the interval time end
        """

        # 生成laneId => 初始排队车辆Id 字典
        for laneId in self.laneIds:
            # Initial
            newdict = {laneId: []}
            self.BeginVIdsDict.update(newdict)
            newdict2 = {laneId: []}
            self.EndVIdsDict.update(newdict2)

            vehIds = traci.lane.getLastStepVehicleIDs(laneId)
            for vehid in vehIds:
                self.BeginVIdsDict.get(laneId).append(vehid)
                self.EndVIdsDict.get(laneId).append(vehid)

    def get_arrival_vehicleId(self):
        for laneId in self.laneIds:
            # 道路上的汽车ID
            vehIds = traci.lane.getLastStepVehicleIDs(laneId)
            for vehid in vehIds:
                if vehid in self.EndVIdsDict.get(laneId):
                    continue
                else:
                    self.EndVIdsDict.get(laneId).append(vehid)

    def generate_arrival_rate(self, interval):
        for laneId in self.laneIds:
            N = len(self.BeginVIdsDict.get(laneId))
            NN = len(self.EndVIdsDict.get(laneId))
            arrival_veh_numbber = NN - N
            ar = {laneId:(NN-N)/float(interval)}
            self.ArrivalRatesDict.update(ar)

    def generate_ave_arrival_rate(self):
        # Scale Traffic = 5
        # f_0_200 = open('./A/_5_1_200_arrival_rate.json', 'r')
        # f_201_400 = open('./A/_5_201_400_arrival_rate.json', 'r')
        # f_401_600 = open('./A/_5_401_600_arrival_rate.json', 'r')
        # f_601_800 = open('./A/_5_601_800_arrival_rate.json', 'r')
        # f_801_1000 = open('./A/_5_801_1000_arrival_rate.json', 'r')

        # Scale Traffic = 4
        # f_0_200 = open('./A/_4_1_200_arrival_rate.json', 'r')
        # f_201_400 = open('./A/_4_201_400_arrival_rate.json', 'r')
        # f_401_600 = open('./A/_4_401_600_arrival_rate.json', 'r')
        # f_601_800 = open('./A/_4_601_800_arrival_rate.json', 'r')
        # f_801_1000 = open('./A/_4_801_1000_arrival_rate.json', 'r')

        # Scale Traffic = 3
        # f_0_200 = open('./A/_3_1_200_arrival_rate.json', 'r')
        # f_201_400 = open('./A/_3_201_400_arrival_rate.json', 'r')
        # f_401_600 = open('./A/_3_401_600_arrival_rate.json', 'r')
        # f_601_800 = open('./A/_3_601_800_arrival_rate.json', 'r')
        # f_801_1000 = open('./A/_3_801_1000_arrival_rate.json', 'r')

        # Scale Traffic = 2
        # f_0_200 = open('./A/_2_1_200_arrival_rate.json', 'r')
        # f_201_400 = open('./A/_2_201_400_arrival_rate.json', 'r')
        # f_401_600 = open('./A/_2_401_600_arrival_rate.json', 'r')
        # f_601_800 = open('./A/_2_601_800_arrival_rate.json', 'r')
        # f_801_1000 = open('./A/_2_801_1000_arrival_rate.json', 'r')

        # Scale Traffic = 1
        f_0_200 = open('./A/_1_1_200_arrival_rate.json', 'r')
        f_201_400 = open('./A/_1_201_400_arrival_rate.json', 'r')
        f_401_600 = open('./A/_1_401_600_arrival_rate.json', 'r')
        f_601_800 = open('./A/_1_601_800_arrival_rate.json', 'r')
        f_801_1000 = open('./A/_1_801_1000_arrival_rate.json', 'r')
        f_1001_1200 = open('./A/_1_1001_1200_arrival_rate.json', 'r')

        ar_0_200 = json.loads(f_0_200.read())
        ar_201_400 = json.loads(f_201_400.read())
        ar_401_600 = json.loads(f_401_600.read())
        ar_601_800 = json.loads(f_601_800.read())
        ar_801_1000 = json.loads(f_801_1000.read())
        ar_1001_1200 = json.loads(f_1001_1200.read())

        arlist = []
        for laneId in self.laneIds:
            arlist.append(ar_0_200[laneId])
            arlist.append(ar_201_400[laneId])
            arlist.append(ar_401_600[laneId])
            arlist.append(ar_601_800[laneId])
            arlist.append(ar_801_1000[laneId])
            arlist.append(ar_1001_1200[laneId])
            ave = np.mean(arlist)
            std = np.std(arlist)
            ave_ar_dict = {laneId: ave+std}
            self.AverageArrivalRatesDict.update(ave_ar_dict)

        f_0_200.close()
        f_201_400.close()
        f_401_600.close()
        f_601_800.close()
        f_801_1000.close()
        f_1001_1200.close()

        # f = open('./A/_5_ave_plus_std_arrival_rate.json', 'w')
        # f = open('./A/_4_ave_plus_std_arrival_rate.json', 'w')
        # f = open('./A/_3_ave_plus_std_arrival_rate.json', 'w')
        # f = open('./A/_2_ave_plus_std_arrival_rate.json', 'w')
        f = open('./A/_1_ave_plus_std_arrival_rate.json', 'w')

        f.write(json.dumps(self.AverageArrivalRatesDict))
        f.close()

        return True

    def get_arrival_rate(self, laneId):
        # f = open('./A/_1_ave_plus_std_arrival_rate.json', 'r')
        # f = open('./A/_2_ave_plus_std_arrival_rate.json', 'r')
        # f = open('./A/_3_ave_plus_std_arrival_rate.json', 'r')
        # f = open('./A/_4_ave_plus_std_arrival_rate.json', 'r')
        # f = open('./A/_5_ave_plus_std_arrival_rate.json', 'r')
        # TODO
        f = open('./B/_2_1_1000_arrival_rate.json', 'r')

        data = json.loads(f.read())
        f.close()
        return data[laneId]

    def get_saturate_rate(self):
        return self._saturate_rate_

    def generate_arrival_rate_main_(self, t):
        """ 获取各时间间隔内的到达率，并保存到相应文件中 """
        start = 1
        end = 1000
        interval = end - start

        if t >= start and t <= end:
            if self.IntervalFlag:
                self.initialDict()
                self.IntervalFlag = False
            else:
                self.get_arrival_vehicleId()

        if t == end:
            self.generate_arrival_rate(interval)
            f = open('./B/_4_1_1000_arrival_rate.json', 'w')
            f.write(json.dumps(self.ArrivalRatesDict))
            f.close()
            return True
        # """ 生成到达率文件：A = mean + std """
        # self.generate_ave_arrival_rate()
