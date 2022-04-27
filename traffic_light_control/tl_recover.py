from cvxopt import matrix, solvers
import traci
import tripInfo
import math


class Recover_Stage():
    # 获取sum（g'-g）
    # TODO 根据相位得出对应路口的laneId
    # N = max{getNumber}
    # 对于不同的目标相位 需要获取不同的到达率
    def recover_4(self, tl):
        """ 四相位，十字路口 """
        k = tl.tl_target_phase.tp_phaseIndex
        tripInfoListener = tripInfo.TripInfoListener()
        q = tripInfoListener.get_saturate_rate()    # 饱和流率
        N_AVE = 5  # 路口平均停留车辆数N_AVE
        yr = tl.tl_programInfo.YR
        t_max = 200
        t_min = 10

        # 相位1
        laneIds = self.get_k_laneId(tl, 0)
        N1 = 0
        a1 = tripInfoListener.get_arrival_rate(laneIds[0])
        for laneId in laneIds:
            N = traci.lane.getLastStepHaltingNumber(laneId)
            if N > N1:
                N1 = N
                a1 = tripInfoListener.get_arrival_rate(laneId)
                pass
            pass

        # 相位2
        laneIds = self.get_k_laneId(tl, 3)
        N2 = 0
        a2 = tripInfoListener.get_arrival_rate(laneIds[0])
        for laneId in laneIds:
            N = traci.lane.getLastStepHaltingNumber(laneId)
            if N > N2:
                N2 = N
                a2 = tripInfoListener.get_arrival_rate(laneId)
                pass
            pass


        # 相位3
        laneIds = self.get_k_laneId(tl, 6)
        N3 = 0
        a3 = tripInfoListener.get_arrival_rate(laneIds[0])
        for laneId in laneIds:
            N = traci.lane.getLastStepHaltingNumber(laneId)
            if N > N3:
                N3 = N
                a3 = tripInfoListener.get_arrival_rate(laneId)
                pass
            pass

        # 相位4
        laneIds = self.get_k_laneId(tl, 9)
        N4 = 0
        a4 = tripInfoListener.get_arrival_rate(laneIds[0])
        for laneId in laneIds:
            N = traci.lane.getLastStepHaltingNumber(laneId)
            if N > N4:
                N4 = N
                a4 = tripInfoListener.get_arrival_rate(laneId)
                pass
            pass

        c = matrix([1.0, 1.0, 1.0, 1.0])

        b = matrix([N_AVE-N1-4*a1*yr, N_AVE-N2-4*a2*yr, N_AVE-N3-4*a3*yr, N_AVE-N4-4*a4*yr, t_max, -t_min, t_max, -t_min, t_max, -t_min, t_max, -t_min])

        A = matrix([[a1-q, a2, a3, a4, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [a1, a2-q, a3, a4, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0],
                    [a1, a2, a3-q, a4, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0],
                    [a1, a2, a3, a4-q, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0]])

        try:
            sol = solvers.lp(c, A, b)

            if sol['status'] == 'optimal':
                optimal_result = []

                for x in sol['x']:
                    optimal_result.append(x)

                tl.recover_phases = optimal_result

                return True
            else:
                return False
        except:
            print("An error occured")

        return False

    def recover_2(self, tl):
        """ 二相位，十字路口 """
        # 暂不开放
        return False
        k = tl.tl_target_phase.tp_phaseIndex
        tripInfoListener = tripInfo.TripInfoListener()
        q = tripInfoListener.get_saturate_rate()  # 饱和流率
        a = tripInfoListener.get_arrival_rate(tl.tl_laneId)  # 到达率
        N_AVE = 100  # 路口平均停留车辆数N_AVE
        N = traci.lane.getLastStepHaltingNumber(tl.tl_laneId)
        yr = tl.tl_programInfo.YR
        t_max = 200
        t_min = 7

        c = matrix([1.0, 1.0])

        b = matrix([N_AVE - N - 4 * a * yr, t_max, -t_min, t_max, -t_min])

        if k == 0:
            """ 目标相位k=1 """
            A = matrix([[a - q, 1.0, -1.0, 0.0, 0.0],
                        [a, 0.0, 0.0, 1.0, -1.0]])
            pass
        elif k == 3:
            """ 目标相位k=2 """
            A = matrix([[a, 1.0, -1.0, 0.0, 0.0],
                        [a - q, 0.0, 0.0, 1.0, -1.0]])
            pass
        else:
            print('error56')

        try:
            sol = solvers.lp(c, A, b)

            if sol['status'] == 'optimal':
                optimal_result = []

                for x in sol['x']:
                    optimal_result.append(x)

                tl.recover_phases = optimal_result

                return True
            else:
                return False
        except:
            print("An error occured")

        return False

    def recover_main(self, tl):
        """ 非侵入式信号抢占，可行时返回True,否则返回false """
        if tl.tl_programInfo.PN == 4:
            tl.recoverFlag = self.recover_4(tl)
        elif tl.tl_programInfo.PN == 2:
            tl.recoverFlag = False
            # tl.recoverFlag = self.recover_2(tl)

    def if_recover_completed(self, tl):

        # if tl.recover_time_found:
        #     if traci.simulation.getTime() == tl.recover_time:
        #         tl.set_logic(tl.tl_phases)
        #         tl.updateTL()
        #         return True
        #     pass
        # else:
        #     currentPhase = traci.trafficlight.getPhase(tl.tlId)
        #     lastPhase = len(tl.tl_phases) -1
        #
        #     if currentPhase == lastPhase:
        #         tl.recover_time = traci.trafficlight.getNextSwitch(tl.tlId)
        #         tl.recover_time_found = True
        return False

    def get_k_laneId(self, tl, k):
        links = traci.trafficlight.getControlledLinks(tl.tlId)
        realLinks = []
        for link in links:
            if len(link) == 0:
                continue
                pass
            else:
                realLinks.append(link)

        laneIds = []
        if tl.tl_programInfo.PN == 4:
            if k == 0:
                laneIds.append(realLinks[4][0][0])
                laneIds.append(realLinks[10][0][0])
                pass
            elif k == 3:
                laneIds.append(realLinks[5][0][0])
                laneIds.append(realLinks[11][0][0])
                pass
            elif k == 6:
                laneIds.append(realLinks[1][0][0])
                laneIds.append(realLinks[7][0][0])
                pass
            elif k == 9:
                laneIds.append(realLinks[2][0][0])
                laneIds.append(realLinks[8][0][0])
                pass
            else:
                print('wrong in 174')
            pass
        else:
            # PN = 2 暂不开放
            # if k == 0:
            #     laneIds.append(link[4][0][0])
            #     laneIds.append(link[10][0][0])
            #     pass
            # elif k == 2:
            #     laneIds.append(link[5][0][0])
            #     laneIds.append(link[11][0][0])
            #     pass
            pass

        return laneIds

