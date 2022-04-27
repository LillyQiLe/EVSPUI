import json

import traci
import math
import tripInfo


""" signal　traffic light　information"""
class SingleTLInfo:
    tlId = ''
    evId = ''
    tl_linkIndex = ''
    tl_program = -1
    tl_target_phase = {}
    tl_programInfo = {}
    tl_laneId = ''   # ev将要通过的路段
    tl_e1detId = ''  # 传感器Id
    tl_phases = []   # 存放原始信号相位时长

    distance = ''
    tl_gry_state = ''
    tl_eta = {}

    tl_stage = 0      # 未请求状态0， 准备1，非侵入式抢占2， 侵入式抢占3， 强制跳转4，信号恢复5，结束状态6
    L_i = 0           # 当前周期剩余时间

    prepare_extend_time = 0

    # 非侵入式
    noninvasiveFlag = False     # 非侵入式信号抢占是否有可行解
    noninvasive_start_time_found = False # 是否得出L_i结束的时刻
    noninvasive_start_time = -1     # L_i结束的时刻
    noninvasiveing = False  # 信号灯正处于非侵入式信号抢占过程中
    noninvasive_optimal_result = []  # 非侵入式信号抢占列表

    # 侵入式信号抢占
    invasiveFlag = False
    invasive_jump_time = -1

    # 车辆通过交叉口
    ev_passed_by = False    # 应急车辆是否通过

    # 信号恢复
    recoverFlag = False
    recover_phases = []
    recover_time_found = False
    recover_time = -1
    recover_gap_time = 3
    recovering = False
    recover_complete_time = 0


    # 更新信号灯信息
    def updateTL(self):
        junctions = traci.vehicle.getNextTLS(self.evId)
        for junction in junctions:
            if (junction[0] == self.tlId):
                self.distance = junction[2]
                self.tl_gry_state = junction[3]

        self.getETA()
        self.getCycleAndPhaseDuration()
        pass


    # 返回原相位（包含黄灯和全红）阶段的目标相位信息
    def getTargetPhase(self):
        # 有满足条件的相位时返回相应相位，否则返回-1
        programLogic = traci.trafficlight.getAllProgramLogics(self.tlId)
        programPhases = {}
        for i in range(len(programLogic)):
            if programLogic[i].programID == self.tl_program:
                # 获取当前程序逻辑
                programPhases = programLogic[i].phases
                break

        tp = targetPhase()

        for i in range(len(programPhases)):
            phaseInfo = programPhases[i]
            if phaseInfo.state[self.tl_linkIndex] == 'G':
                tp.tp_duration = phaseInfo.duration
                tp.tp_allState = phaseInfo.state
                tp.tp_minDur = phaseInfo.minDur
                tp.tp_maxDur = phaseInfo.maxDur
                tp.tp_phaseIndex = i
                break

        if len(programPhases) % 3 == 0:
            tp.tp_newDuration = tp.tp_duration + 5
            tp.tp_newPhaseIndex = int(tp.tp_phaseIndex / 3)
        elif len(programPhases) % 2 == 0:
            tp.tp_newDuration = tp.tp_duration + 3
            tp.tp_newPhaseIndex = int(tp.tp_phaseIndex / 2)

        self.tl_target_phase = tp

        return tp
        pass

    # 获取预计到达时间相关信息, 相对时间，时长
    def getETA(self):
        lane_speed = traci.lane.getMaxSpeed('-gneE30_0')
        speed_factor = 2.0
        speed_dev = 0.2

        speed = lane_speed*speed_factor
        detaSpd = lane_speed*speed_dev

        # deta = self.distance * detaSpd / (speed * speed - detaSpd * detaSpd)  # ETA偏差
        deta = self.distance/(speed-detaSpd) - self.distance/speed

        eta = ETA()
        eta.ETA = math.ceil(self.distance / speed)  # 预计到达时间
        eta.deta = math.ceil(deta)
        eta.begin = eta.ETA - eta.deta
        eta.end = eta.ETA + eta.deta
        self.tl_eta = eta

        return eta
        pass

    # 生成程序逻辑信息，programInfo():
    def getCycleAndPhaseDuration(self):
        # 有满足条件的相位时返回相应相位，否则返回-1
        programLogic = traci.trafficlight.getAllProgramLogics(self.tlId)
        programPhases = {}
        for i in range(len(programLogic)):
            if programLogic[i].programID == self.tl_program:
                # 获取当前程序逻辑
                programPhases = programLogic[i].phases
                break

        self.tl_programInfo = programInfo()
        if len(programPhases) == 12:
            self.tl_programInfo.PN = 4
            self.tl_programInfo.YR = 5.0
        elif len(programPhases) == 4:
            self.tl_programInfo.PN = 2
            self.tl_programInfo.YR = 3.0

        self.tl_programInfo.phasesDuration = []
        for phase in programPhases:
            self.tl_programInfo.Cycle += phase.duration
            self.tl_programInfo.phasesDuration.append(phase.duration)

        self.tl_programInfo.getTmax()
        self.tl_programInfo.getTmin()
        pass

    # 返回所有相位信息
    def getPhases(self):
        # 有满足条件的相位时返回相应相位，否则返回-1
        programLogic = traci.trafficlight.getAllProgramLogics(self.tlId)
        programPhases = {}
        for i in range(len(programLogic)):
            if programLogic[i].programID == self.tl_program:
                # 获取当前程序逻辑
                programPhases = programLogic[i].phases
                break
        return programPhases

    # 匹配字符串
    def laneInedge(self, laneId, edgeId):
        for i in range(len(edgeId)):
            if laneId[i] == edgeId[i]:
                continue
            else:
                return False
        return True

    # 返回ev将到达的路口所在路段
    def getlaneId(self):
        links = traci.trafficlight.getControlledLinks(self.tlId)
        edges = traci.vehicle.getRoute(self.evId)
        for i in range(len(links)):
            if len(links[i]) == 0:
                continue
            start = links[i][0][0]
            end = links[i][0][1]
            for j in range(len(edges)):
                edge = edges[j]
                if self.laneInedge(start, edge):
                    if self.laneInedge(end, edges[j+1]):
                        self.tl_laneId = start
                        self.tl_e1detId = 'e1det_' + start
                        return start
                else:
                    continue
            else:
                continue

    # 获取当前周期剩余时间
    def getL_i(self):
        t = traci.simulation.getTime()
        currentPhaseIndex = traci.trafficlight.getPhase(self.tlId)
        nextSwitch = traci.trafficlight.getNextSwitch(self.tlId)
        phases = self.getPhases()

        remian = nextSwitch - t

        for index in range(len(phases)):
            if index > currentPhaseIndex:
                remian += phases[index].duration

        self.L_i = remian
        return remian

    # 清空排队车辆所需时间
    def get_CQT(self):
        N = traci.lane.getLastStepHaltingNumber(self.tl_laneId)
        tripInfoListener = tripInfo.TripInfoListener()
        Q = tripInfoListener.get_saturate_rate()
        A = tripInfoListener.get_arrival_rate(self.tl_laneId)

        CQT = N/(Q-A)

        if CQT != 0:
            CQT += 5
        return CQT

    # 获取W_i
    def getW_i(self, n):
        CQT = self.get_CQT()
        Tlost = 5
        Tyt = 3  # 普通车辆为应急车辆避让时间
        if CQT == 0:
            return 0
        else:
            return CQT + (n+1) * Tlost + Tyt
            pass
        pass

    # min获取W_i
    def min_getW_i(self, n):
        CQT = self.get_CQT()
        return CQT

    # 返回eta所在相位开始时间
    def getD(self, LJT):
        if self.tl_programInfo.PN == 4:
            return LJT % 50
        elif self.tl_programInfo.PN == 2:
            return LJT % 45
        else:
            print("待补充")
            return -1

    # ev通过交叉路口
    def evPassBy(self):
        junctions = traci.vehicle.getNextTLS(self.evId)
        for junction in junctions:
            if junction[0] == self.tlId:
                self.ev_passed_by = False  # ev未通过
                return False

        self.ev_passed_by = True
        print(str(self.tlId) + ' 已通过')
        return True

    def ini_phases(self):
        self.tl_phases = self.tl_programInfo.phasesDuration

    # 延长第index相位的相位时间
    def add_extend_time(self):
        AllProgramLogics = traci.trafficlight.getAllProgramLogics(self.tlId)

        targetPhaseIndex = self.tl_target_phase.tp_phaseIndex
        ini_phase_time = self.tl_phases[targetPhaseIndex]

        programId = traci.trafficlight.getProgram(self.tlId)

        for i in range(len(AllProgramLogics)):
            if AllProgramLogics[i].programID == programId:
                programId = int(i)
                break


        # TODO 同方向所有信号灯都要延长信号时长
        if self.tl_programInfo.PN == 4:
            if targetPhaseIndex == 0 or targetPhaseIndex == 3:
                ini_phase_time = self.tl_phases[0]
                AllProgramLogics[programId].phases[0].duration = ini_phase_time + self.prepare_extend_time
                ini_phase_time = self.tl_phases[3]
                AllProgramLogics[programId].phases[3].duration = ini_phase_time + self.prepare_extend_time
            elif targetPhaseIndex == 6 or targetPhaseIndex == 9:
                ini_phase_time = self.tl_phases[6]
                AllProgramLogics[programId].phases[6].duration = ini_phase_time + self.prepare_extend_time
                ini_phase_time = self.tl_phases[9]
                AllProgramLogics[programId].phases[9].duration = ini_phase_time + self.prepare_extend_time
            else:
                print('ERROR: 降低道路饱和度阶段不能延长黄灯或红灯的时间')

            # AllProgramLogics[1].phases[targetPhaseIndex].duration = ini_phase_time + self.prepare_extend_time
            traci.trafficlight.setProgramLogic(self.tlId, AllProgramLogics[programId])
            pass
        elif self.tl_programInfo.PN == 2:
            AllProgramLogics[0].phases[targetPhaseIndex].duration = ini_phase_time + self.prepare_extend_time
            traci.trafficlight.setProgramLogic(self.tlId, AllProgramLogics[0])
            pass
        else:
            print('wrong2')

    # 设置恢复阶段各相位时长
    def setRecoverPhases(self):
        AllProgramLogics = traci.trafficlight.getAllProgramLogics(self.tlId)
        programId = traci.trafficlight.getProgram(self.tlId)
        for i in range(len(AllProgramLogics)):
            if AllProgramLogics[i].programID == programId:
                programId = int(i)
                break
        if self.tlId != 'gneJ1':
            a = 0

        if self.tl_programInfo.PN == 4:
            if self.recover_phases[0] - 0 < 0.1 and self.recover_phases[0] - 0 > -0.1 :
                AllProgramLogics[programId].phases[0].duration = 0
                AllProgramLogics[programId].phases[1].duration = 0
                AllProgramLogics[programId].phases[2].duration = 0
            else:
                AllProgramLogics[programId].phases[0].duration = int(self.recover_phases[0])
                AllProgramLogics[programId].phases[1].duration = 3
                AllProgramLogics[programId].phases[2].duration = 2

            if self.recover_phases[1] - 0 < 0.1 and self.recover_phases[1] - 0 > -0.1 :
                AllProgramLogics[programId].phases[3].duration = 0
                AllProgramLogics[programId].phases[4].duration = 0
                AllProgramLogics[programId].phases[5].duration = 0
            else:
                AllProgramLogics[programId].phases[3].duration = int(self.recover_phases[1])
                AllProgramLogics[programId].phases[4].duration = 3
                AllProgramLogics[programId].phases[5].duration = 2

            if self.recover_phases[2] - 0 < 0.1 and self.recover_phases[2] - 0 > -0.1 :
                AllProgramLogics[programId].phases[6].duration = 0
                AllProgramLogics[programId].phases[7].duration = 0
                AllProgramLogics[programId].phases[8].duration = 0
            else:
                AllProgramLogics[programId].phases[6].duration = int(self.recover_phases[2])
                AllProgramLogics[programId].phases[7].duration = 3
                AllProgramLogics[programId].phases[8].duration = 2

            if self.recover_phases[3] - 0 < 0.1 and self.recover_phases[3] - 0 > -0.1 :
                AllProgramLogics[programId].phases[9].duration = 0
                AllProgramLogics[programId].phases[10].duration = 0
                AllProgramLogics[programId].phases[11].duration = 0
            else:
                AllProgramLogics[programId].phases[9].duration = int(self.recover_phases[3])
                AllProgramLogics[programId].phases[10].duration = 3
                AllProgramLogics[programId].phases[11].duration = 2

            traci.trafficlight.setPhase(self.tlId, 0)
            traci.trafficlight.setProgramLogic(self.tlId, AllProgramLogics[programId])

            sum = 0
            for phase in AllProgramLogics[programId].phases:
                sum += phase.duration

            return sum
            pass
        elif self.tl_programInfo.PN == 2:
            self.tl_stage = 6
            self.recover_logic(self.tl_phases)
            return 0
            pass
        else:
            print('wrong2')

        pass

    # 设置各信号相位时间
    def set_logic(self, phases):
        AllProgramLogics = traci.trafficlight.getAllProgramLogics(self.tlId)
        programId = traci.trafficlight.getProgram(self.tlId)
        for i in range(len(AllProgramLogics)):
            if AllProgramLogics[i].programID == programId:
                programId = int(i)
                break

        if self.tl_programInfo.PN == 4:
            AllProgramLogics = traci.trafficlight.getAllProgramLogics(self.tlId)
            AllProgramLogics[programId].phases[0].duration = phases[0]
            AllProgramLogics[programId].phases[3].duration = phases[1]
            AllProgramLogics[programId].phases[6].duration = phases[2]
            AllProgramLogics[programId].phases[9].duration = phases[3]

            traci.trafficlight.setProgramLogic(self.tlId, AllProgramLogics[programId])
            pass
        elif self.tl_programInfo.PN == 2:
            AllProgramLogics = traci.trafficlight.getAllProgramLogics(self.tlId)
            AllProgramLogics[0].phases[0].duration = phases[0]
            AllProgramLogics[0].phases[2].duration = phases[1]

            traci.trafficlight.setProgramLogic(self.tlId, AllProgramLogics[0])
            pass
        else:
            print('wrong2')

    # 恢复各信号相位时间
    def recover_logic(self, phases):
        AllProgramLogics = traci.trafficlight.getAllProgramLogics(self.tlId)
        programId = traci.trafficlight.getProgram(self.tlId)
        for i in range(len(AllProgramLogics)):
            if AllProgramLogics[i].programID == programId:
                programId = int(i)
                break
        if self.tl_programInfo.PN == 4:
            AllProgramLogics = traci.trafficlight.getAllProgramLogics(self.tlId)
            for index in range(len(AllProgramLogics[programId].phases)):
                AllProgramLogics[programId].phases[index].duration = phases[index]

            traci.trafficlight.setProgramLogic(self.tlId, AllProgramLogics[programId])
            pass
        elif self.tl_programInfo.PN == 2:
            AllProgramLogics = traci.trafficlight.getAllProgramLogics(self.tlId)
            for index in range(len(AllProgramLogics[0].phases)):
                AllProgramLogics[0].phases[index].duration = phases[index]

            traci.trafficlight.setProgramLogic(self.tlId, AllProgramLogics[0])
            pass
        else:
            print("wrong1")

    # 当前周期结束 （用于非侵入式信号抢占过程中）
    def currentCycleEnd(self):
        if self.noninvasive_start_time_found:
            if traci.simulation.getTime() >= self.noninvasive_start_time - 1:
                return True
            else:
                return False
            pass
        else:
            currentPhaseIndex = traci.trafficlight.getPhase(self.tlId)
            lastPhaseIndex = len(self.tl_phases) - 1

            if currentPhaseIndex == lastPhaseIndex:
                self.noninvasive_start_time = traci.trafficlight.getNextSwitch(self.tlId)
                self.noninvasive_start_time_found = True
                pass
            pass
        return False

    def state_report(self):
        # tl_stage = 0  # 未请求状态0， 准备1，非侵入式抢占2， 侵入式抢占3， 强制跳转4，信号恢复5，结束状态6
        if self.tlId != 'gneJ1':
            return 0

        condition = []
        condition.append('\t当前时刻:' + str(traci.simulation.getTime()))
        condition.append('\t信号灯ID:' + str(self.tlId))
        condition.append('\t当前阶段:' + str(self.tl_stage))
        condition.append('\t预计到达时间:' + str(self.tl_eta.ETA))
        condition.append('\tdeta:' + str(self.tl_eta.deta))

        if self.tl_stage == 1:

            condition.append('\t延长时间:' + str(self.prepare_extend_time))
            pass
        elif self.tl_stage == 2:
            condition.append('\t非侵入开始时刻:' + str(self.noninvasive_start_time))
            condition.append('\t非侵入是否开始:' + str(self.noninvasiveing))
            condition.append('\t目标相位:' + str(self.tl_target_phase.tp_phaseIndex))
            condition.append('\t当前相位:' + str(traci.trafficlight.getPhase(self.tlId)))
            pass
        elif self.tl_stage == 3:
            condition.append('\t侵入式跳转时间:' + str(self.invasive_jump_time))
            condition.append('\t侵入式是否开始:' + str(self.invasiveFlag))
            condition.append('\t信号灯当前颜色:' + str(traci.vehicle.getNextTLS('ev')[0][3]))
            pass
        elif self.tl_stage == 4:
            condition.append('\t信号灯当前颜色:' + str(traci.vehicle.getNextTLS('ev')[0][3]))
            pass
        elif self.tl_stage == 5:
            condition.append('\t恢复是否开始:' + str(self.recovering))

            condition.append('\t恢复周期:' + str(self.recover_phases))
            pass
        elif self.tl_stage == 6:
            # condition.append(self.tl_programInfo.phasesDuration)
            # condition.append(self.tl_phases)
            pass

        condition.append('\t信号灯周期:' + str(self.tl_programInfo.phasesDuration))

        f = open('./tl_state_flow.txt', 'a', encoding="utf-8")
        # f.write(json.dumps(condition))
        for i in condition:
            f.write(i)
        f.write('\n')
        f.close()





# 目标相位
class targetPhase():
    """ 目标相位 """
    tp_phaseIndex = 0
    tp_duration = 0
    tp_newDuration = 0
    tp_allState = ''
    tp_minDur = 0
    tp_maxDur = 0
    tp_newPhaseIndex = 0

# 预计到达时间
class ETA():
    ETA = 0
    deta = 0
    begin = 0
    end = 0

# programLogic
class programInfo():
    Cycle = 0
    PN = 0  # 相位个数
    iniPhaseDuration = [] # 初始状态各相位时间
    phasesDuration = [] # 各相位时间
    YR = 0   # 相位间隔时间
    tou_max = 200
    tou_min = 7
    T_max = 0
    T_min = 0

    def getTmax(self):
        self.T_max = (self.tou_max + self.YR) * self.PN
        return self.T_max

    def getTmin(self):
        self.T_min = (self.tou_min + self.YR) * self.PN
        return self.T_min
