import json

import numpy
import cvxopt
from cvxopt import matrix, solvers
import traci
import tripInfo
import math
import time


class Preemption_Stage():
    # noninvasive_optimal_result = []
    # 获取sum（g'-g）
    def get_pow_sum(self, optimal_result, tl, g):
        sum = 0
        for i in range(tl.tl_programInfo.PN):
            sum += pow(g[i] - optimal_result[i+1], 2)
        return sum

    # 从所有最优解中选出最优解
    def compute_optimal_result(self, optimal_results, tl, g):
        if len(optimal_results) == 1:
            tl.noninvasive_optimal_result = optimal_results[0]

            return True

        optimal_index = 0
        effect = 10000
        for i in range(len(optimal_results)):
            optimal_result = optimal_results[i]
            effect_i = pow(tl.tl_programInfo.Cycle - optimal_result[0], 2) + 0.5 * self.get_pow_sum(optimal_result, tl, g)
            if effect_i < effect:
                effect = effect_i
                optimal_index = i

        tl.noninvasive_optimal_result = optimal_results[optimal_index]
        return True

    def min_noninvasive_main(self, tl):
        """ 非侵入式信号抢占，可行时返回True,否则返回false """
        if tl.tl_programInfo.PN == 4:
            tl.noninvasiveFlag = self.min_noninvasive_4(tl)
        elif tl.tl_programInfo.PN == 2:
            tl.noninvasiveFlag = self.min_noninvasive_2(tl)

    def min_noninvasive_4(self, tl):
        """ 四相位，十字路口 """
        beta = 0.5
        g = []
        k = tl.tl_target_phase.tp_phaseIndex

        g.append(tl.tl_programInfo.phasesDuration[0])
        g.append(tl.tl_programInfo.phasesDuration[3])
        g.append(tl.tl_programInfo.phasesDuration[6])
        g.append(tl.tl_programInfo.phasesDuration[9])

        t = traci.simulation.getTime()

        t_i = t + tl.tl_eta.ETA
        L_i = tl.getL_i()

        P = matrix([[2.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 1.0]])
        q = matrix([-2 * tl.tl_programInfo.Cycle, -2 * beta * g[0], -2 * beta * g[1],
                    -2 * beta * g[2], -2 * beta * g[3]]);

        A = matrix([[1.0], [-1.0], [-1.0], [-1.0], [-1.0]])
        b = matrix([0])

        optimal_results = []

        for index in range(6):
            n = index + 1
            w_i = tl.min_getW_i(n)

            if k == 0:
                """ 目标相位k=1 """
                G = matrix([[n, -n, n, -n, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, n],
                            [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -n],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0]])
                h = matrix([t_i - t - L_i + tl.tl_eta.deta,
                            -(t_i - t - L_i) - tl.tl_eta.deta,
                            t_i - t - L_i - tl.tl_eta.deta,
                            -(t_i - t - L_i) + tl.tl_eta.deta,
                            tl.tl_programInfo.T_max,
                            -tl.tl_programInfo.T_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            t_i - t - L_i - w_i - n*g[0]])
                pass
            elif k == 3:
                """ 目标相位k=2 """
                G = matrix([[n, -n, n, -n, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, n],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                            [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, -n],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0]])
                h = matrix([t_i - t - L_i - tl.tl_programInfo.YR + tl.tl_eta.deta,
                            -(t_i - t - L_i) + tl.tl_programInfo.YR - tl.tl_eta.deta,
                            t_i - t - L_i - tl.tl_programInfo.YR - tl.tl_eta.deta,
                            -(t_i - t - L_i) + tl.tl_programInfo.YR + tl.tl_eta.deta,
                            tl.tl_programInfo.T_max,
                            -tl.tl_programInfo.T_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            t_i - t - L_i - w_i - tl.tl_programInfo.YR - n*g[1]])
                pass
            elif k == 6:
                """ 目标相位k=3 """
                G = matrix([[n, -n, n, -n, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, n],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                            [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -n],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0]])
                h = matrix([t_i - t - L_i - 2 * tl.tl_programInfo.YR + tl.tl_eta.deta,
                            -(t_i - t - L_i) + 2 * tl.tl_programInfo.YR - tl.tl_eta.deta,
                            t_i - t - L_i - 2 * tl.tl_programInfo.YR - tl.tl_eta.deta,
                            -(t_i - t - L_i) + 2 * tl.tl_programInfo.YR + tl.tl_eta.deta,
                            tl.tl_programInfo.T_max,
                            -tl.tl_programInfo.T_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            t_i - t - L_i - w_i - 2 * tl.tl_programInfo.YR - n*g[2]])
                pass
            elif k == 9:
                """ 目标相位k=4 """
                G = matrix([[n, -n, n, -n, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, n],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 1.0],
                            [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, -n]])
                h = matrix([t_i - t - L_i - 3 * tl.tl_programInfo.YR + tl.tl_eta.deta,
                            -(t_i - t - L_i) + 3 * tl.tl_programInfo.YR - tl.tl_eta.deta,
                            t_i - t - L_i - 3 * tl.tl_programInfo.YR - tl.tl_eta.deta,
                            -(t_i - t - L_i) + 3 * tl.tl_programInfo.YR + tl.tl_eta.deta,
                            tl.tl_programInfo.T_max,
                            -tl.tl_programInfo.T_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            t_i - t - L_i - w_i - 3 * tl.tl_programInfo.YR - n*g[3]])
                pass
            else:
                print('error409')

            try:
                sol = solvers.qp(P, q, G, h, A, b)

                if sol['status'] == 'optimal':
                    optimal_result = []

                    for x in sol['x']:
                        optimal_result.append(x)

                    optimal_results.append(optimal_result)

            except:
                print("An error occured")


        if optimal_results == []:
            return False
        else:
            self.compute_optimal_result(optimal_results, tl, g)
            return True

        return False

    def min_noninvasive_2(self, tl):
        """ 二相位，十字路口 """
        beta = 0.5
        g = []
        k = tl.tl_target_phase.tp_phaseIndex

        g.append(tl.tl_programInfo.phasesDuration[0])
        g.append(tl.tl_programInfo.phasesDuration[2])

        t_i = traci.simulation.getTime() + tl.tl_eta.ETA
        t = traci.simulation.getTime()
        L_i = tl.getL_i()

        P = matrix([[2.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]])
        q = matrix([-2 * tl.tl_programInfo.Cycle, -2 * beta * g[0], -2 * beta * g[1]]);

        A = matrix([[1.0], [-1.0], [-1.0]])
        b = matrix([0])

        optimal_results = []

        for index in range(6):
            n = index + 1
            w_i = tl.min_getW_i(n)

            if k == 0:
                """ 目标相位k=1 """
                G = matrix([[n, -n, n, -n, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, n],
                            [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -n],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0]])
                h = matrix([t_i - t - L_i + tl.tl_eta.deta,
                            -(t_i - t - L_i) - tl.tl_eta.deta,
                            t_i - t - L_i - tl.tl_eta.deta,
                            -(t_i - t - L_i) + tl.tl_eta.deta,
                            tl.tl_programInfo.T_max,
                            -tl.tl_programInfo.T_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            t_i - t - L_i - w_i - n*g[0]])
                pass
            elif k == 2:
                """ 目标相位k=2 """
                G = matrix([[n, -n, n, -n, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, n],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 1.0],
                            [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, -n]])
                h = matrix([t_i - t - L_i - tl.tl_programInfo.YR + tl.tl_eta.deta,
                            -(t_i - t - L_i) + tl.tl_programInfo.YR - tl.tl_eta.deta,
                            t_i - t - L_i - tl.tl_programInfo.YR - tl.tl_eta.deta,
                            -(t_i - t - L_i) + tl.tl_programInfo.YR + tl.tl_eta.deta,
                            tl.tl_programInfo.T_max,
                            -tl.tl_programInfo.T_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            t_i - t - L_i - w_i - tl.tl_programInfo.YR - n*g[1]])
                pass
            else:
                print('error257')

            try:
                sol = solvers.qp(P, q, G, h, A, b)
                if sol['status'] == 'optimal':
                    optimal_result = []
                    for x in sol['x']:
                        optimal_result.append(x)
                    optimal_results.append(optimal_result)
            except:
                print("An error occured")


        if optimal_results == []:
            return False
        else:
            self.compute_optimal_result(optimal_results, tl, g)
            return True
        return False

    def noninvasive_main(self, tl):
        """ 非侵入式信号抢占，可行时返回True,否则返回false """
        pathFile = '../output/qp.json'
        f = open(pathFile, 'w+')

        if tl.tl_programInfo.PN == 4:
            s_time = time.time()
            tl.noninvasiveFlag = self.noninvasive_4(tl)
            e_time = time.time()
            data = {'method': 'QP', 'PN': 4, 'duration': "use {:.5}s".format(e_time - s_time)}
            f.write(json.dumps(data))
            f.write('\n')
            f.close()
            # print("use {:.5}s".format(e_time - s_time))
        elif tl.tl_programInfo.PN == 2:
            tl.noninvasiveFlag = self.noninvasive_2(tl)

    def noninvasive_4(self, tl):
        """ 四相位，十字路口 """
        beta = 0.5
        # beta = 0.25
        g = []
        k = tl.tl_target_phase.tp_phaseIndex

        g.append(tl.tl_programInfo.phasesDuration[0])
        g.append(tl.tl_programInfo.phasesDuration[3])
        g.append(tl.tl_programInfo.phasesDuration[6])
        g.append(tl.tl_programInfo.phasesDuration[9])

        t = traci.simulation.getTime()

        t_i = t + tl.tl_eta.ETA
        L_i = tl.getL_i()

        P = matrix([[2.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 1.0]])
        q = matrix([-2 * tl.tl_programInfo.Cycle, -2 * beta * g[0], -2 * beta * g[1],
                    -2 * beta * g[2], -2 * beta * g[3]]);

        A = matrix([[1.0], [-1.0], [-1.0], [-1.0], [-1.0]])
        b = matrix([4 * tl.tl_programInfo.YR])

        optimal_results = []

        for index in range(6):
            n = index + 1
            w_i = tl.getW_i(n)

            if k == 0:
                """ 目标相位k=1 """
                G = matrix([[n, -n, n, -n, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, n],
                            [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -n],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0]])
                h = matrix([t_i - t - L_i + tl.tl_eta.deta,
                            -(t_i - t - L_i) - tl.tl_eta.deta,
                            t_i - t - L_i - tl.tl_eta.deta,
                            -(t_i - t - L_i) + tl.tl_eta.deta,
                            tl.tl_programInfo.T_max,
                            -tl.tl_programInfo.T_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            t_i - t - L_i - w_i])
                pass
            elif k == 3:
                """ 目标相位k=2 """
                G = matrix([[n, -n, n, -n, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, n],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                            [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, -n],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0]])
                h = matrix([t_i - t - L_i - tl.tl_programInfo.YR + tl.tl_eta.deta,
                            -(t_i - t - L_i) + tl.tl_programInfo.YR - tl.tl_eta.deta,
                            t_i - t - L_i - tl.tl_programInfo.YR - tl.tl_eta.deta,
                            -(t_i - t - L_i) + tl.tl_programInfo.YR + tl.tl_eta.deta,
                            tl.tl_programInfo.T_max,
                            -tl.tl_programInfo.T_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            t_i - t - L_i - w_i - tl.tl_programInfo.YR])
                pass
            elif k == 6:
                """ 目标相位k=3 """
                G = matrix([[n, -n, n, -n, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, n],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                            [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -n],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0]])
                h = matrix([t_i - t - L_i - 2 * tl.tl_programInfo.YR + tl.tl_eta.deta,
                            -(t_i - t - L_i) + 2 * tl.tl_programInfo.YR - tl.tl_eta.deta,
                            t_i - t - L_i - 2 * tl.tl_programInfo.YR - tl.tl_eta.deta,
                            -(t_i - t - L_i) + 2 * tl.tl_programInfo.YR + tl.tl_eta.deta,
                            tl.tl_programInfo.T_max,
                            -tl.tl_programInfo.T_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            t_i - t - L_i - w_i - 2 * tl.tl_programInfo.YR])
                pass
            elif k == 9:
                """ 目标相位k=4 """
                G = matrix([[n, -n, n, -n, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, n],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 1.0],
                            [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, -n]])
                h = matrix([t_i - t - L_i - 3 * tl.tl_programInfo.YR + tl.tl_eta.deta,
                            -(t_i - t - L_i) + 3 * tl.tl_programInfo.YR - tl.tl_eta.deta,
                            t_i - t - L_i - 3 * tl.tl_programInfo.YR - tl.tl_eta.deta,
                            -(t_i - t - L_i) + 3 * tl.tl_programInfo.YR + tl.tl_eta.deta,
                            tl.tl_programInfo.T_max,
                            -tl.tl_programInfo.T_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            t_i - t - L_i - w_i - 3 * tl.tl_programInfo.YR])
                pass
            else:
                print('error409')

            try:
                sol = solvers.qp(P, q, G, h, A, b)

                if sol['status'] == 'optimal':
                    optimal_result = []

                    for x in sol['x']:
                        optimal_result.append(x)

                    optimal_results.append(optimal_result)

            except:
                print("An error occured")


        if optimal_results == []:
            return False
        else:
            self.compute_optimal_result(optimal_results, tl, g)
            return True

        return False

    def noninvasive_2(self, tl):
        """ 二相位，十字路口 """
        # beta = 0.5
        beta = 0.25
        g = []
        k = tl.tl_target_phase.tp_phaseIndex

        g.append(tl.tl_programInfo.phasesDuration[0])
        g.append(tl.tl_programInfo.phasesDuration[2])

        t_i = traci.simulation.getTime() + tl.tl_eta.ETA
        t = traci.simulation.getTime()
        L_i = tl.getL_i()

        # # if
        # tl.tl_programInfo.T_max = 2000
        # tl.tl_programInfo.T_min = 0
        # tl.tl_programInfo.tou_max = 500
        # tl.tl_programInfo.tou_min = 0

        P = matrix([[2.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]])
        q = matrix([-2 * tl.tl_programInfo.Cycle, -2 * beta * g[0], -2 * beta * g[1]]);

        A = matrix([[1.0], [-1.0], [-1.0]])
        b = matrix([2 * tl.tl_programInfo.YR])

        optimal_results = []

        for index in range(6):
            n = index + 1
            w_i = tl.getW_i(n)

            if k == 0:
                """ 目标相位k=1 """
                G = matrix([[n, -n, n, -n, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, n],
                            [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -n],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0]])
                h = matrix([t_i - t - L_i + tl.tl_eta.deta,
                            -(t_i - t - L_i) - tl.tl_eta.deta,
                            t_i - t - L_i - tl.tl_eta.deta,
                            -(t_i - t - L_i) + tl.tl_eta.deta,
                            tl.tl_programInfo.T_max,
                            -tl.tl_programInfo.T_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            t_i - t - L_i - w_i])
                pass
            elif k == 2:
                """ 目标相位k=2 """
                G = matrix([[n, -n, n, -n, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, n],
                            [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 1.0],
                            [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, -n]])
                h = matrix([t_i - t - L_i - tl.tl_programInfo.YR + tl.tl_eta.deta,
                            -(t_i - t - L_i) + tl.tl_programInfo.YR - tl.tl_eta.deta,
                            t_i - t - L_i - tl.tl_programInfo.YR - tl.tl_eta.deta,
                            -(t_i - t - L_i) + tl.tl_programInfo.YR + tl.tl_eta.deta,
                            tl.tl_programInfo.T_max,
                            -tl.tl_programInfo.T_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            tl.tl_programInfo.tou_max,
                            -tl.tl_programInfo.tou_min,
                            t_i - t - L_i - w_i - tl.tl_programInfo.YR])
                pass
            else:
                print('error233')

            try:
                sol = solvers.qp(P, q, G, h, A, b)
                if sol['status'] == 'optimal':
                    optimal_result = []
                    for x in sol['x']:
                        optimal_result.append(x)
                    optimal_results.append(optimal_result)
            except:
                print("An error occured")


        if optimal_results == []:
            return False
        else:
            self.compute_optimal_result(optimal_results, tl, g)
            return True
        return False


    # 获取非侵入式信号抢占的最优解
    # def get_noinvasive_result(self, tl):
    #     return tl.noninvasive_optimal_result

    # 侵入式信号抢占
    def invasive_main(self, tl):
        # 清空排队车辆所需时间
        CQT = math.ceil(tl.get_CQT())
        YT = 5  # 避让时间
        LJT = traci.simulation.getTime() + tl.tl_eta.ETA - tl.tl_eta.deta - tl.tl_programInfo.YR - CQT - YT

        D = tl.getD(LJT)

        if D > 7:
            tl.invasive_jump_time = LJT
        else:
            tl.invasive_jump_time = LJT - D
        return tl.invasive_jump_time

    # Qin
    def qin_invasive_main(self, tl):
        # 清空排队车辆所需时间
        t = int(traci.simulation.getTime())
        CQT = math.ceil(tl.get_CQT())
        YT = 0  # 避让时间
        LJT = t + tl.tl_eta.ETA - tl.tl_eta.deta - tl.tl_programInfo.YR - CQT - YT
        tl.invasive_jump_time = LJT
        return tl.invasive_jump_time

    # 侵入式是否跳转
    def invasive_if_jump(self, tl):
        if traci.simulation.getTime() >= tl.invasive_jump_time:
            tl.invasiveFlag = True
            return True
        else:
            return False

    # 信号跳转
    def invasive_jump(self, tl):
        currentPhaseIndex = traci.trafficlight.getPhase(tl.tlId)
        targetPhaseIndex = tl.tl_target_phase.tp_phaseIndex

        programeId = int(traci.trafficlight.getProgram(tl.tlId))
        if (programeId == 2):
            programeId = 1;
        a = traci.trafficlight.getAllProgramLogics(tl.tlId)
        a[programeId].phases[targetPhaseIndex].duration = 90
        traci.trafficlight.setProgramLogic(tl.tlId, a[programeId])

        if currentPhaseIndex != targetPhaseIndex:
            traci.trafficlight.setPhase(tl.tlId, targetPhaseIndex)
