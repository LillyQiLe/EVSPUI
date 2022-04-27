import traci
import traffic_light_control.tl_controller_info as tl_info
import traffic_light_control.tl_preempt
import traffic_light_control.tl_prepare
import traffic_light_control.tl_recover

def stage_2(preempt ,tl):
    # 是否通过交叉口
    passed = tl.evPassBy()
    if passed:
        # TODO DELETE
        tl.tl_stage = 6
        tl.recover_logic(tl.tl_phases)
        pass
        # # 通过了， 进入恢复状态
        # tl.tl_stage = 5
        # # 3到5秒的安全时间间隔
        # traci.trafficlight.setPhase(tl.tlId, 2)
        #
        # recover = traffic_light_control.tl_recover.Recover_Stage()
        # recover.recover_main(tl)
    else:
        # 未通过
        # 侵入式抢占时间到
        preempt.invasive_main(tl)  # 计算invasive算法跳转时间
        if preempt.invasive_if_jump(tl):
            # 侵入式信号抢占时间到
            tl.tl_stage = 3
            preempt.invasive_jump(tl)
        else:
            end = tl.currentCycleEnd()
            if tl.currentCycleEnd():
                optimal = tl.noninvasive_optimal_result
                tl.set_logic(optimal[1:])
                tl.updateTL()
                tl.noninvasiveing = True
                pass
            pass


def min_stage_2(preempt, tl):
    # 是否通过交叉口
    passed = tl.evPassBy()
    if passed:
        tl.tl_stage = 6
        tl.recover_logic(tl.tl_phases)
    else:
        # 未通过
        if tl.distance < 5:
            # 强制跳转到目标相位
            tl.tl_stage = 4
            preempt.invasive_jump(tl)
        pass