
import Grids
import os

from common import *
import scipy
import json
import math
import random
import numpy


def generate_random_instance(graph, num_agents, lcp=None):
    if lcp is None:
        lcp = graph.getLargestConnectedComponent()
    starts = np.copy(lcp)
    # print("largest cp", len(starts))
    # starts = graph.getV()
    goals = np.copy(starts)
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    return starts[:num_agents], goals[:num_agents]


def generate_sortation_warehouse(m, agents):
    obstacles=[]
    for x in range(1,m,3):
        for y in range(1,m,3):
            obstacles.append((x,y))
    graph=Grids.Grid(m,m, obstacles)



def generate_json(map_name):
    map_file = "./maps/" + map_name + ".map"
    graph = Grids.Grid(map_file)
    # print(len(graph.getV()))
    # lcp = graph.getV()
    lcp = graph.getLargestConnectedComponent()
    # print(len(graph.getLargestConnectedComponent()))
    # exit(0)
    num_agents = range(1000,4001,200)
    num_cases = 50

    for k in range(num_cases):
        for agent in num_agents:
            starts, goals = generate_random_instance(graph, agent, lcp)
            data_dict = dict()
            name = "./instances/" + map_name + "/agents" + \
                str(agent) + "_" + str(k) + ".json"

            data_dict["starts"] = [(start.pos.x, start.pos.y)
                                   for start in starts]
            data_dict["goals"] = [(goal.pos.x, goal.pos.y) for goal in goals]

            # Create the directory if it doesn't exist
            os.makedirs(os.path.dirname(name), exist_ok=True)

            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def generate_grids_one_third():
    ls = [30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360]
    for l in ls:
        graph = Grids.Grid(l, l, [])
        lcp = graph.getV()
        n = int((l*l)/3)
        for k in range(25):

            starts, goals = generate_random_instance(graph, n, lcp)
            data_dict = dict()
            name = "./instances/" + "one_third"+ "/" +str(l)+"x"+ \
                str(l) + "_" + str(k) + ".json"

            data_dict["starts"] = [(start.pos.x, start.pos.y)
                                   for start in starts]
            data_dict["goals"] = [(goal.pos.x, goal.pos.y) for goal in goals]

            # Create the directory if it doesn't exist
            os.makedirs(os.path.dirname(name), exist_ok=True)

            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def generate_grids_full():
    ls = [30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360]
    for l in ls:
        graph = Grids.Grid(l, l, [])
        lcp = graph.getV()
        n = l*l
        for k in range(25):

            starts, goals = generate_random_instance(graph, n, lcp)
            data_dict = dict()
            name = "./instances/" + "full"+ "/" +str(l)+"x"+ \
                str(l) + "_" + str(k) + ".json"

            data_dict["starts"] = [(start.pos.x, start.pos.y)
                                   for start in starts]
            data_dict["goals"] = [(goal.pos.x, goal.pos.y) for goal in goals]

            # Create the directory if it doesn't exist
            os.makedirs(os.path.dirname(name), exist_ok=True)

            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)




def generate_grids_half():
    ls = [30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360]
    for l in ls:
        graph = Grids.Grid(l, l, [])
        lcp = graph.getV()
        n = int(l*l/2)
        for k in range(25):

            starts, goals = generate_random_instance(graph, n, lcp)
            data_dict = dict()
            name = "./instances/" + "half"+ "/" +str(l)+"x"+ \
                str(l) + "_" + str(k) + ".json"

            data_dict["starts"] = [(start.pos.x, start.pos.y)
                                   for start in starts]
            data_dict["goals"] = [(goal.pos.x, goal.pos.y) for goal in goals]

            # Create the directory if it doesn't exist
            os.makedirs(os.path.dirname(name), exist_ok=True)

            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)


def generate_grids_different():
    size=300
    ls = list(range(80000, 90001, 2000))
    graph = Grids.Grid(size, size, [])
    lcp = graph.getV()
    for n in ls:
        # n = int(l*l/2)
        for k in range(25):

            starts, goals = generate_random_instance(graph, n, lcp)
            data_dict = dict()
            name = "./instances/" + "vary_dense"+ "/agents" +str(n)+ "_" + str(k) + ".json"

            data_dict["starts"] = [(start.pos.x, start.pos.y)
                                   for start in starts]
            data_dict["goals"] = [(goal.pos.x, goal.pos.y) for goal in goals]

            # Create the directory if it doesn't exist
            os.makedirs(os.path.dirname(name), exist_ok=True)

            with open(name, "w") as f:
                print(name)
                json.dump(data_dict, f)
if __name__ == "__main__":
    # generate_grids_half()
    # generate_grids_different()
    generate_json("Shanghai_0_256")
