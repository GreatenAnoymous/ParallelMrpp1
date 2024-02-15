import numpy as np

import os
import json
from subprocess import STDOUT, check_output
import time
import maps.problem_generator as pg
import utils
import Grids
import MAPF

OUTPUT = "./tmp/tmp.json"
CBS_EXE = "./cbs"
ECBS_EXE = "./ecbs"
EECBS_EXE = "./eecbs"
ECBST_EXE = "./ecbst"
ECBSDE_EXE = "./ecbsde"
ECBSDER_EXE = "./ecbsderandom"
HCA_EXE = "./hca"
LACAM_EXE = "./lacam"
RTM_EXE = "./rtm_lba"
RTM_EXE2 = "./rtm_exe"
ECBST_EXE2="./ecbst2"
PUSH_EXE2="./push"
DDM_EXE="./ddm_exe"
ECBSW_EXE="./ecbsw"
input_folder = "./instances/warehouse-10-20-10-2-1/"
map_file = "./maps/warehouse-10-20-10-2-1.map"
num_agents = list(range(50, 701, 50))
num_instances = 20

def read_tmp_json_data():
    with open(OUTPUT, "r") as f:
        data_dict = json.load(f)
    makespan = data_dict["mkpn"]
    soc = data_dict["soc"]
    runtime = data_dict["runtime"]
    try:
        num_exp = data_dict["numExpansion"]
    except:
        num_exp = 0

    return makespan, soc, runtime, num_exp


def run_exe(map, scen, exeFile):

    cmd = [exeFile, map, scen, OUTPUT]
    try:
        output = check_output(cmd, stderr=STDOUT, timeout=125).decode('utf-8')
    except:
        print("out of time")


def dumpToCsv(header, data, csvFile):
    with open(csvFile, "w") as f:
        numItems = len(header)
        numAgents = len(data[0])
        for k in range(numItems):
            f.write(header[k])
            if k != numItems-1:
                f.write(",")
            else:
                f.write("\n")
        # print(numItems,numAgents,len(data),len(data[0]))
        for i in range(numAgents):
            for k in range(numItems):
                f.write(str(data[k][i]))
                if k != numItems-1:
                    f.write(",")
                else:
                    f.write("\n")


def evalAlgorithm(EXE_FILE, outputCSV):
    header = ["num_agents", "makespan", "soc",
              "runtime", "success_rate", "num_expansions"]
    makespanData = []
    socData = []
    expansionData = []
    runtimeData = []
    rateData = []

    agentsData = []
    total = num_instances
    for n in num_agents:
        makespan_sum = 0
        soc_sum = 0
        runtime_sum = 0
        succ_sum = 0
        expansion_sum = 0
        for k in range(num_instances):
            instance = input_folder + "agents"+str(n)+"_"+str(k)+".json"
            print(instance)
            t0 = time.time()
            run_exe(map_file, instance, EXE_FILE)
            t1 = time.time()

            try:
                makespan, cost, runtime, num_exp = read_tmp_json_data()
                if cost<1:
                    print(instance)
                    raise RuntimeError()
                makespan_sum += makespan
                soc_sum += cost
                runtime_sum += runtime
                succ_sum += 1
                expansion_sum += num_exp
            except:
                # unkonwn segmentation fault, bugs
                # if t1-t0 < 60:
                #     total -= 1
                # else:

                
                # runtime_sum += 120
                # failed
                pass

            try:
                # break
                os.remove(OUTPUT)
            except:
                pass
        if succ_sum == 0:
            succ_sum += 1e-6
        makespanData.append(makespan_sum/succ_sum)
        socData.append(soc_sum/succ_sum)
        runtimeData.append(runtime_sum/succ_sum)
        expansionData.append(expansion_sum/total)

        rateData.append(succ_sum/total)
        agentsData.append(n)
        dumpToCsv(header, [agentsData, makespanData,
                  socData, runtimeData, rateData, expansionData], outputCSV)


def evalAlgorithmGrids(EXE_FILE, outputCSV, ls=None,  input="./instances/one_third/",density=1/3):

    header = ["num_agents", "makespan", "soc",
              "runtime", "success_rate", "num_expansions"]
    makespanData = []
    socData = []
    expansionData = []
    runtimeData = []
    rateData = []

    agentsData = []
    total = 1
    for l in ls:
        makespan_sum = 0
        soc_sum = 0
        runtime_sum = 0
        succ_sum = 0
        expansion_sum = 0
        n = int(l*l*density)
        map_file = "./maps/"+str(l)+'x'+str(l)+".map"
        for k in range(total):
            instance = input + str(l) + "x"+str(l)+"_"+str(k)+".json"
            print(instance)
            t0 = time.time()
            run_exe(map_file, instance, EXE_FILE)
            t1 = time.time()

            try:
                makespan, cost, runtime, num_exp = read_tmp_json_data()
                makespan_sum += makespan
                soc_sum += cost
                runtime_sum += runtime
                succ_sum += 1
                expansion_sum += num_exp
            except:
                # unkonwn segmentation fault, bugs
                # if t1-t0 < 60:
                #     total -= 1
                # else:
                runtime_sum += 120
                # failed
                pass

            try:
                # break
                os.remove(OUTPUT)
            except:
                pass
        if succ_sum == 0:
            succ_sum += 1e-6
        makespanData.append(makespan_sum/succ_sum)
        socData.append(soc_sum/succ_sum)
        runtimeData.append(runtime_sum/total)
        expansionData.append(expansion_sum/total)

        rateData.append(succ_sum/total)
        agentsData.append(n)
        dumpToCsv(header, [agentsData, makespanData,
                  socData, runtimeData, rateData, expansionData], outputCSV)
        
def eveluate_sortation():
    outputCSV="./sortation_push_swap.csv"
    ls = [30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360]
    header = ["num_agents", "makespan", "soc",
              "runtime", "success_rate", "num_expansions"]
    makespanData = []
    socData = []
    expansionData = []
    runtimeData = []
    rateData = []
    total=20
    agentsData = []
    for m in ls:
        makespan_sum = 0
        soc_sum = 0
        runtime_sum = 0
        succ_sum = 0
        pg.generate_sortation_map(m)
        map_file="./tmp/tmp.map"
        graph=Grids.Grid(map_file)
        for k in range(total):
            starts, goals=utils.generate_random_instance(graph, int(m*m*2/9))
            instance="./tmp/tmp_instance.json"
            data_dict = dict()
            data_dict["starts"]=[(start.pos.x, start.pos.y) for start in starts]
            data_dict["goals"] = [(goal.pos.x, goal.pos.y) for goal in goals]
            os.makedirs(os.path.dirname(instance), exist_ok=True)
            with open(instance, "w") as f:
                json.dump(data_dict, f)
            t0 = time.time() 
            run_exe(map_file, instance, PUSH_EXE2)
            t1 = time.time()
            try:
                makespan, cost, runtime, num_exp = read_tmp_json_data()
                makespan_sum += makespan
                soc_sum += cost
                runtime_sum += runtime
                succ_sum += 1
            except:
                runtime_sum += 120
                pass
            if succ_sum == 0:
                succ_sum += 1e-6
        makespanData.append(makespan_sum/succ_sum)
        socData.append(soc_sum/succ_sum)
        runtimeData.append(runtime_sum/total)
        expansionData.append(-1)

        rateData.append(succ_sum/total)
        agentsData.append(m)
        dumpToCsv(header, [agentsData, makespanData,
                socData, runtimeData, rateData, expansionData], outputCSV)



if __name__ == "__main__":


    # evalAlgorithmGrids(
    #     RTM_EXE2, "./data/full/rtm2x3.csv", list(range(30, 331, 30)), input="./instances/full/",density=1)

    # evalAlgorithmGrids(LACAM_EXE, "./data/blocks/lacam.csv",
    #                 list(range(30, 361, 30)), input="./instances/block_json/",density=1/3)
    
    # evalAlgorithmGrids(LACAM_EXE, "./data/rings/lacam.csv",
    #                 list(range(30, 361, 30)), input="./instances/ring_json/",density=1/3)

    # evalAlgorithmGrids(
    #     LACAM_EXE, "./data/half/lacam.csv", [270,300],input="./instances/half/",density=0.5)

    # evalAlgorithmGrids(
    #     LACAM_EXE, "./data/full/lacam.csv", [30, 60, 90, 120, 150, 180], input="./instances/full/",density=1.0)

    #  evalAlgorithmGrids(
    # RTM_EXE, "./data/full/rtm2x3lba.csv", [30, 60, 90, 120, 150, 180, 210,240,270,300,330], input="./instances/full/",density=1.0)

    # map = "vary_dense"
    # input_folder = "./instances/"+map+"/"
    # map_file = "./maps/"+map+".map"
    # num_agents = list(range(20000, 40001, 2000))
    # # num_agents = [9,16,25,36,49,64]
    # evalAlgorithm(LACAM_EXE, "./data/"+map+"/lacam.csv")
    # map = "random-32-32-20"
    # input_folder = "./instances/"+map+"/"
    # map_file = "./maps/"+map+".map"
    # num_agents = list(range(370, 411, 5))
    # # num_agents = [9,16,25,36,49,64]
    # evalAlgorithm(LACAM_EXE, "./data/"+map+"/lacam.csv")

    # map = "orz201d"
    # input_folder = "./instances/"+map+"/"
    # map_file = "./maps/"+map+".map"
    # num_agents = list(range(200, 301, 10))
    # # num_agents = [9,16,25,36,49,64]
    # evalAlgorithm(LACAM_EXE, "./data/"+map+"/lacam.csv")


    # eveluate_sortation()

  

    # map = "corner_dense"
    # input_folder = "./instances/"+map+"/"
    # map_file = "./maps/"+map+".map"
    # # num_agents = list(range(200, 301, 10))
    # num_agents = [9,16,25,36,49,64]
    # evalAlgorithm(LACAM_EXE, "./data/"+map+"/lacam.csv")
    # evalAlgorithm(ECBSDE_EXE, "./data/"+map+"/ecbsdede.csv")
    # evalAlgorithm(ECBS_EXE, "./data/"+map+"/ecbs.csv")
    # evalAlgorithm(ECBSDER_EXE, "./data/"+map+"/ecbsderandom.csv")

    map = "Shanghai_0_256"
    input_folder = "./instances/"+map+"/"
    map_file = "./maps/"+map+".map"
    num_agents = list(range(1000, 4001, 200))
    evalAlgorithm(ECBSW_EXE, "./data/"+map+"/"+"ecbsw8.csv")


    # map= "w_woundedcoast"
    # input_folder = "./instances/"+map+"/"
    # map_file = "./maps/"+map+".map"
    # num_agents = list(range(500, 1001, 100))
    # evalAlgorithm(ECBST_EXE, "./data/"+map+"/ecbs1.csv")

    # map = "Shanghai_0_256"
    # input_folder = "./instances/"+map+"/"
    # map_file = "./maps/"+map+".map"
    # num_agents = list(range(1000, 2001, 100))
    # evalAlgorithm(ECBST_EXE, "./data/"+map+"/ecbs.csv")

    # evalAlgorithm(ECBST_EXE, "./data/"+map+"/ecbst.csv")
    # evalAlgorithm(CBST_EXE_SEQ, "./data/"+map+"/cbst_seq.csv")
    # evalAlgorithm(CBST2_EXE, "./data/"+map+"/cbst2.csv")
    # evalAlgorithm(CBST4_EXE, "./data/"+map+"/cbst4.csv")
    # evalAlgorithm(CBST8_EXE, "./data/"+map+"/cbst8.csv")

    # evalAlgorithm(CBST24_EXE, "./data/"+map+"/cbst24.csv")
    # evalAlgorithm(CBST32_EXE, "./data/"+map+"/cbst32.csv")

    # map = "lak503d"
    # input_folder = "./instances/"+map+"/"
    # map_file = "./maps/"+map+".map"
    # evalAlgorithm(CBST_EXE, "./data/"+map+"/cbst.csv")
    # # evalAlgorithm(CBST2_EXE, "./data/"+map+"/cbst2.csv")
    # # evalAlgorithm(CBST4_EXE, "./data/"+map+"/cbst4.csv")
    # # evalAlgorithm(CBST8_EXE, "./data/"+map+"/cbst8.csv")
    # # evalAlgorithm(CBST_EXE_SEQ, "./data/"+map+"/cbst_seq.csv")
    # # evalAlgorithm(CBST24_EXE, "./data/"+map+"/cbst24.csv")
    # # evalAlgorithm(CBST32_EXE, "./data/"+map+"/cbst32.csv")
    # # evalAlgorithm(CBSP_EXE, "./data/"+map+"/cbsp.csv")

    # map = "arena"
    # input_folder = "./instances/"+map+"/"
    # map_file = "./maps/"+map+".map"
    # evalAlgorithm(CBST_EXE, "./data/"+map+"/cbst.csv")
    # # evalAlgorithm(CBST2_EXE, "./data/"+map+"/cbst2.csv")
    # # evalAlgorithm(CBST4_EXE, "./data/"+map+"/cbst4.csv")
    # # evalAlgorithm(CBST8_EXE, "./data/"+map+"/cbst8.csv")
    # # evalAlgorithm(CBST_EXE_SEQ, "./data/"+map+"/cbst_seq.csv")
    # # evalAlgorithm(CBST24_EXE, "./data/"+map+"/cbst24.csv")
    # # evalAlgorithm(CBST32_EXE, "./data/"+map+"/cbst32.csv")

    # map = "warehouse-10-20-10-2-1"
    # num_agents = list(range(10, 81, 10))
    # input_folder = "./instances/"+map+"/"
    # map_file = "./maps/"+map+".map"
    # evalAlgorithm(CBST_EXE, "./data/"+map+"/cbst.csv")
    # # evalAlgorithm(CBST2_EXE, "./data/"+map+"/cbst2.csv")
    # # evalAlgorithm(CBST4_EXE, "./data/"+map+"/cbst4.csv")
    # # evalAlgorithm(CBST8_EXE, "./data/"+map+"/cbst8.csv")
    # # evalAlgorithm(CBST_EXE_SEQ, "./data/"+map+"/cbst_seq.csv")
    # # evalAlgorithm(CBST24_EXE, "./data/"+map+"/cbst24.csv")
    # # evalAlgorithm(CBST32_EXE, "./data/"+map+"/cbst32.csv")
