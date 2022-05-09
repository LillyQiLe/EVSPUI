import json

import numpy
import traci
import emergency_vehicle

def run(pathFile, EVFrom, EVTo):
    step = 0
    Emergency_Vehicle = emergency_vehicle.EmergencyVehicle()
    Emergency_Vehicle.addEV(EVFrom, EVTo)

    traci.simulationStep()
    step += 1
    """ 初始化通信信息 """
    Emergency_Vehicle.get_junctions()
    Emergency_Vehicle.ini_ev_to_tl()
    traci.vehicle.highlight('ev')

    evSpeed = []
    Flag = True

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1

        try:
            traci.vehicle.getNextTLS('ev')
            evSpeed.append(traci.vehicle.getSpeed('ev'))
            print('PK ')
            traci.addStepListener(Emergency_Vehicle.tl_classification())
        except:
            if Flag:
                EV_depart_time = step - 1
                data = {'travleTime': EV_depart_time, 'speedVar': numpy.var(evSpeed), 'evSpeed': evSpeed}
                f = open(pathFile, 'w+')
                f.write(json.dumps(data))
                f.write('\n')
                f.close()
                Flag = False

def traci_runner(_from_, _end_, _scale_, _path_):
    port = 7000
    label = 1

    # _from_ = '-gneE30'
    # _end_ = '-gneE26'

    tripInfo = "./output/myMethod_tripinfo.xml"
    json_myMethod = "./output/myMethod.json"

    traci.start(["sumo-gui", "-c", "simple.sumo.cfg", "--start", "true", "--lateral-resolution", "1",
                             "--tripinfo-output", tripInfo, "--tripinfo-output.write-unfinished", "true",
                             "--duration-log.statistics", "true", "--scale", str(_scale_)], port=port, label=str(label))
    traci.switch(str(label))
    run(json_myMethod, _from_, _end_)
    traci.close(False)