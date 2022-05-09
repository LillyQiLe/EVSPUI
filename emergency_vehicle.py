import traci
import traffic_light_control.tl_controller_info as tl_info
import traffic_light_control.tl_preempt
import traffic_light_control.tl_prepare
import traffic_light_control.tl_recover
import stage_control.stage_0
import stage_control.stage_1
import stage_control.stage_2
import stage_control.stage_3
import stage_control.stage_4
import stage_control.stage_5


""" Emergency Vehicle """
class EmergencyVehicleListener(traci.StepListener):
    pass

class EmergencyVehicle(traci.StepListener):
    evId = ''               # 应急车辆ID
    pass_by_edges = ''      # 经过边缘
    pass_by_junctions = ()   # 经过交叉口
    tls_info = []           # 路径上所有路口信号灯信息


    def addEV(self, EVFrom, EVTo):
        """ 添加应急车辆 """
        # TODO
        self.pass_by_edges = traci.simulation.findRoute(fromEdge=EVFrom, toEdge=EVTo).edges

        # 添加车辆
        self.evId = 'ev'
        traci.route.add("trip", self.pass_by_edges)
        traci.vehicle.add(routeID="trip", vehID=self.evId, typeID="ambulance", departPos='base', departSpeed='max')
        # traci.vehicle.setVehicleClass('ev', 'emergency')

    def get_edges(self):
        """ 返回路径 """
        return self.pass_by_edges

    def get_junctions(self):
        """ 返回经过的交叉口 """
        self.pass_by_junctions = traci.vehicle.getNextTLS(self.evId)
        return self.pass_by_junctions

    def ini_ev_to_tl(self):
        """ 所有信号灯的初始状态 """
        for tl in self.pass_by_junctions:
            newTL = tl_info.SingleTLInfo()
            newTL.tlId = tl[0]
            newTL.evId = self.evId
            newTL.tl_linkIndex = tl[1]
            newTL.distance = tl[2]
            newTL.tl_gry_state = tl[3]
            newTL.tl_program = traci.trafficlight.getProgram(tl[0])
            newTL.getlaneId()

            newTL.getTargetPhase()
            newTL.getETA()
            newTL.getCycleAndPhaseDuration()
            newTL.ini_phases()

            self.tls_info.append(newTL)
        pass

    # 信号灯分类
    def tl_classification(self):
        Flag = True     # 若后面的交叉口无法得出非侵入抢占的解,前面的交叉口更不可能
        for i in range(len(self.tls_info)-1, -1, -1):
            tl = self.tls_info[i]
            # 更新变量
            tl.updateTL()
            preempt = traffic_light_control.tl_preempt.Preemption_Stage()

            """ test区域 """
            #
            # if tl.tlId == 'gneJ13':
            #     a = 1

            """ test区域 """


            if tl.tl_stage == 0:
                if Flag:
                    preempt.noninvasive_main(tl)

                if tl.noninvasiveFlag:
                    pass
                else:
                    Flag = False

                stage_control.stage_0.stage_0(preempt, tl)
                # print('stage=0')
            elif tl.tl_stage == 1:
                if Flag:
                    if traci.simulation.getTime()%20 == 0:
                        preempt.noninvasive_main(tl)

                if tl.noninvasiveFlag:
                    pass
                else:
                    Flag = False

                stage_control.stage_1.stage_1(preempt, tl)
                # print('stage=1')
            elif tl.tl_stage == 2:
                # 非侵入式信号抢占
                stage_control.stage_2.stage_2(preempt, tl)
                # print('stage=2')
                pass
            elif tl.tl_stage == 3:
                # 侵入式信号抢占
                stage_control.stage_3.stage_3(preempt, tl)
                # print('stage=3')
                pass
            elif tl.tl_stage == 4:
                # 强制跳转
                stage_control.stage_4.stage_4(preempt, tl)
                # print('stage=4')
            elif tl.tl_stage == 5:
                # 信号恢复状态
                stage_control.stage_5.stage_5(preempt, tl)
                # print('stage=5')
            else:
                # print('tlID:' + str(tl.tlId) + 'stage:' + str(tl.tl_stage))
                # print('stage=6')
                pass
        pass

    # 只包含降低道路饱和度和侵入式信号抢占的分类
    def tl_classification_1(self):
        for tl in self.tls_info:
            # 更新变量
            tl.updateTL()
            preempt = traffic_light_control.tl_preempt.Preemption_Stage()

            if tl.tl_stage == 0:
                # 初始状态
                stage_control.stage_0.new_stage_0(preempt, tl)
            elif tl.tl_stage == 1:
                # 降低道路饱和度状态
                stage_control.stage_1.new_stage_1(preempt, tl)
            elif tl.tl_stage == 2:
                # 非侵入式信号抢占, 不存在这样的状态
                # stage_control.stage_2.stage_2(preempt, tl)
                # print("tl.tl_stage" + str(tl.tl_stage))
                pass
            elif tl.tl_stage == 3:
                # 侵入式信号抢占
                stage_control.stage_3.new_stage_3(tl)
                pass
            elif tl.tl_stage == 4:
                # 强制跳转
                stage_control.stage_3.new_stage_3(tl)
            elif tl.tl_stage == 5:
                # 信号恢复状态 没有该状态
                # stage_control.stage_5.stage_5(tl)
                print("tl.tl_stage" + str(tl.tl_stage))
            else:
                print(tl.tl_stage)
                print("已完成")
        pass

    # Min
    def tl_classification_Min(self):
        # 逆转数组，从后面的信号灯开始计算
        tls = []
        for tl in self.tls_info:
            tls.insert(0, tl)
        Flag = True  # 若后面的交叉口无法得出非侵入抢占的解,前面的交叉口更不可能

        for tl in tls:
            # 更新变量
            tl.updateTL()
            preempt = traffic_light_control.tl_preempt.Preemption_Stage()

            if tl.tl_stage == 0:
                # 初始状态
                if Flag:
                    if traci.simulation.getTime() % 20 == 0:
                        preempt.noninvasive_main(tl)

                if tl.noninvasiveFlag:
                    pass
                else:
                    Flag = False

                stage_control.stage_0.min_stage_0(preempt, tl)
            elif tl.tl_stage == 2:
                # 非侵入式信号抢占
                stage_control.stage_2.min_stage_2(preempt, tl)
                pass
            elif tl.tl_stage == 4:
                # 强制跳转
                stage_control.stage_4.min_stage_4(preempt, tl)
            else:
                pass
                # print("tl.tl_stage" + str(tl.tl_stage))
                # print("已完成")
        pass

    # Qin
    def tl_classification_Qin(self):
        for tl in self.tls_info:
            # 更新变量
            tl.updateTL()
            preempt = traffic_light_control.tl_preempt.Preemption_Stage()

            if tl.tl_stage == 0:
                # 初始状态
                stage_control.stage_0.qin_stage_0(preempt, tl)
            elif tl.tl_stage == 1:
                # 降低道路饱和度状态
                print('不应该存在降低饱和度状态')
                # stage_control.stage_1.new_stage_1(preempt, tl)
            elif tl.tl_stage == 2:
                # 非侵入式信号抢占, 不存在这样的状态
                # stage_control.stage_2.stage_2(preempt, tl)
                print('不应该存在非侵入式抢占状态')
                # print("tl.tl_stage" + str(tl.tl_stage))
                pass
            elif tl.tl_stage == 3:
                # 侵入式信号抢占
                stage_control.stage_3.new_stage_3(preempt, tl)
                pass
            elif tl.tl_stage == 4:
                # 强制跳转
                stage_control.stage_3.new_stage_3(preempt, tl)
            elif tl.tl_stage == 5:
                # 信号恢复状态 没有该状态
                # stage_control.stage_5.stage_5(tl)
                print('不应该存在恢复状态' + + str(tl.tl_stage))
                # print("tl.tl_stage" + str(tl.tl_stage))
            else:
                print(tl.tl_stage)
                print("已完成")

            # tl.state_report()
        pass