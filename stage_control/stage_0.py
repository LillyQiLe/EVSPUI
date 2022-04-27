import traci
import traffic_light_control.tl_controller_info as tl_info
import traffic_light_control.tl_preempt
import traffic_light_control.tl_prepare
import traffic_light_control.tl_recover

def stage_0(preempt, tl):
    if tl.noninvasiveFlag:
        # 非侵入式信号抢占有可行解
        tl.tl_stage = 2
        if tl.currentCycleEnd():
            optimal = tl.noninvasive_optimal_result
            tl.set_logic(optimal[1:])
            tl.updateTL()
            tl.noninvasiveing = True
            pass
    else:
        # 非侵入式信号抢占无可行解
        preempt.invasive_main(tl)  # 计算invasive算法跳转时间
        if preempt.invasive_if_jump(tl):
            # 侵入式信号抢占时间到
            tl.tl_stage = 3
            preempt.invasive_jump(tl)
        else:
            # 侵入式信号抢占时间到没有到
            if tl.distance < 5:
                # 强制跳转到目标相位
                tl.tl_stage = 4
                preempt.invasive_jump(tl)
                pass
            else:
                # 降低道路饱和度
                tl.tl_stage = 1
                prepare = traffic_light_control.tl_prepare.Prepare_Stage()
                tl.prepare_extend_time = prepare.prepare_main(tl)
                tl.add_extend_time()
                tl.updateTL()
                pass

# 只包含降低道路饱和度以及侵入式信号抢占
def new_stage_0(preempt, tl):
    preempt.invasive_main(tl)  # 计算invasive算法跳转时间
    if preempt.invasive_if_jump(tl):
        # 侵入式信号抢占时间到
        tl.tl_stage = 3
        preempt.invasive_jump(tl)
    else:
        # 侵入式信号抢占时间到没有到
        if tl.distance < 5:
            # 强制跳转到目标相位
            tl.tl_stage = 4
            preempt.invasive_jump(tl)
            pass
        else:
            # 降低道路饱和度
            tl.tl_stage = 1
            prepare = traffic_light_control.tl_prepare.Prepare_Stage()
            tl.prepare_extend_time = prepare.prepare_main(tl)
            tl.add_extend_time()
            tl.updateTL()
            pass

# Min
def min_stage_0(preempt, tl):
    if tl.noninvasiveFlag:
        # 非侵入式信号抢占有可行解
        tl.tl_stage = 2
        if tl.currentCycleEnd():
            optimal = tl.noninvasive_optimal_result
            tl.set_logic(optimal[1:])
            tl.updateTL()
            tl.noninvasiveing = True
            pass
    else:
        if tl.distance < 30:
            # 强制跳转到目标相位
            tl.tl_stage = 4
            preempt.invasive_jump(tl)
        pass

# Qin
def qin_stage_0(preempt, tl):
    preempt.qin_invasive_main(tl)  # 计算invasive算法跳转时间
    if preempt.invasive_if_jump(tl):
        # 侵入式信号抢占时间到
        tl.tl_stage = 3
        preempt.invasive_jump(tl)
    else:
        # 侵入式信号抢占时间到没有到
        if tl.distance < 5:
            # 强制跳转到目标相位
            tl.tl_stage = 4
            preempt.invasive_jump(tl)
            pass
