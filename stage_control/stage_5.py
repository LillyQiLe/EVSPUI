import traci
import traffic_light_control.tl_controller_info as tl_info
import traffic_light_control.tl_preempt
import traffic_light_control.tl_prepare
import traffic_light_control.tl_recover

def stage_5(preempt, tl):
    if tl.tlId == 'gneJ1':
        a = 1
    if tl.recovering:
        # recover = traffic_light_control.tl_recover.Recover_Stage()
        # if recover.if_recover_completed(tl):
        #     tl.tl_stage = 6
        #     tl.recover_logic(tl.tl_phases)
        currentTime = traci.simulation.getTime()
        if currentTime > tl.recover_complete_time:
            tl.tl_stage = 6
            tl.recover_logic(tl.tl_phases)
        pass
    else:
        if tl.recover_gap_time:
            tl.recover_gap_time -= 1
            pass
        else:
            if tl.recoverFlag:
                duration = tl.setRecoverPhases()
                # tl.set_logic(tl.recover_phases)
                tl.recovering = True
                tl.recover_complete_time = traci.simulation.getTime() + duration
            else:
                tl.tl_stage = 6
                tl.recover_logic(tl.tl_phases)
                pass
            pass


    # 是否恢复完毕






