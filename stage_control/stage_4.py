import traci
import traffic_light_control.tl_controller_info as tl_info
import traffic_light_control.tl_preempt
import traffic_light_control.tl_prepare
import traffic_light_control.tl_recover

def stage_4(preempt, tl):
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
        if traci.vehicle.getNextTLS('ev')[0][3] != 'G':
            preempt.invasive_jump(tl)
        pass


def min_stage_4(preempt, tl):
    # 是否通过交叉口
    passed = tl.evPassBy()
    if passed:
        tl.tl_stage = 6
        tl.recover_logic(tl.tl_phases)
    else:
        # 未通过
        if traci.vehicle.getNextTLS('ev')[0][3] != 'G':
            preempt.invasive_jump(tl)
        pass