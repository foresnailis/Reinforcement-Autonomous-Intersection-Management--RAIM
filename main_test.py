from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import time
import torch
import random
import platform
import traceback
import subprocess

import numpy as np

from collections import defaultdict

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.path.append("/usr/share/sumo/bin") # Para linux
    sys.path.append("/usr/share/sumo/tools") # Para linux
    #sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

# sys.path.append("/usr/share/sumo/bin") # Para linux
# sys.path.append("/usr/share/sumo/tools") # Para linux

#%
pltf = platform.system()
if pltf == "Windows":
    print("Your system is Windows")
    netgenBinary = checkBinary('netgenerate.exe')
    sumoBinary = checkBinary('sumo-gui.exe')

else:
    print("Your system is Linux")
    netgenBinary = checkBinary('netgenerate')
    sumoBinary = checkBinary('sumo-gui')
#%
with open('proj-root.txt', 'r') as file:
    root = file.read().strip()

os.chdir(root)

sys.path.append(f"{root}/algorithms")
sys.path.append(f"{root}/api_sumo")
sys.path.append(f"{root}/api_sumo/sumo_elems")
sys.path.append(f"{root}/graphs")
sys.path.append(f"{root}/scenarios")

from algorithms import *  # noqa
from api_sumo import *  # noqa
from graphs import *  # noqa
from scenarios import *  # noqa

# % 设置种子可以确保每次运行代码时得到相同的随机数序列，从而使实验可重现
SEED = 2024

torch.manual_seed(SEED)
np.random.seed(SEED)
random.seed(SEED)

import argparse

parser = argparse.ArgumentParser(description="Traffic Simulation Parameters")
parser.add_argument('--nrows', type=int, default=1, help='Number of rows in the grid')
parser.add_argument('--ncols', type=int, default=1, help='Number of columns in the grid')
parser.add_argument('--nlanes', type=int, default=2, help='Number of lanes per road')
parser.add_argument('--length', type=int, default=200, help='Length of each road segment (in meters)')
parser.add_argument('--seed', type=int, default=42, help='Random seed for reproducibility')
parser.add_argument('--agent', type=str, default='TD3', help='TD3 or DDPG')
parser.add_argument('--policy_noise', type=bool, default=True, help='POLICY NOISE')
parser.add_argument('--class_learn', type=bool, default=True, help='Class Learning')
parser.add_argument('--cf', type=bool, default=False, help='Car Following and lane change.')
parser.add_argument('--gui', type=bool, default=True, help='Whether to use SUMO GUI')
parser.add_argument('--flow', type=int, default=150, help='Vehicle flow rate (vehicles per hour)')
parser.add_argument('--model_name', type=str, default="Test", help='Name of the model to use')
parser.add_argument('--epochs', type=int, default=200, help='Number of training epochs')
args = parser.parse_args()

if args.gui:
    sumoBinary = checkBinary('sumo-gui')
else:
    sumoBinary = checkBinary('sumo')

red_manhattan = ManhattanGraph(3, 3, 300)
escenario = ScenarioThree(red_manhattan, 250, 500, 800, 900)
Fixed = FixedAlgorithm(greentime=(120-10)//2, lanes=args.nlanes)

simulation = SumoSimulation(red_manhattan, gui=args.gui, lanes=args.nlanes,
                            nrows=args.nrows, ncols=args.ncols, leng=args.length,
                            seed=args.seed, flow=args.flow,
                            policy_noise=args.policy_noise, cf= args.cf, model_name=args.model_name, agent=args.agent,
                            map = 'map')
simulation.seed = SEED
simulation.change_algorithm(Fixed) # 设置控制算法
simulation.change_scenario(escenario) # 设置交通场景

model_list = [
    # 'DDPG-CL',
    'TD3-CL',
    # 'TD3-PER',
    # 'Krauss'
]

# flow_list = [100, 125, 150, 175, 200, 250, 300, 350, 400]
flow_list = [150]
import seaborn as sns
import requests
import pandas as pd
import openpyxl
import matplotlib.pyplot as plt

# 准备数据存储结构
results = []

for model_name in model_list:
    for flow in flow_list:
        print(f"Model name : {model_name} Flow = {flow}")
        simulation.flow = flow
        if model_name == 'Krauss':
            c = simulation.run_test_simulation(is_agent=False)
            ti = simulation.getTripinfo()
        else:
            weight_path = os.path.join('ckpt', model_name, '150_best')
            if 'DDPG' in model_name:
                simulation.change_agent('DDPG')
            elif 'TD3' in model_name:
                if model_name == 'TD3-PER':
                    simulation.change_maxSpeed(13.39)
                    simulation.change_agent('TD3', cf=True)
                else:
                    simulation.change_agent('TD3')
            
            c = simulation.run_test_simulation(weight_path=weight_path, is_agent=True)
            ti = simulation.getTripinfo()
        
        # 存储结果
        results.append({
            'Model': model_name,
            'Flow': flow,
            'Mean Duration': ti[5],
            'Collisions': c,
            'CO': ti[9],
            'CO2': ti[10],
            'HC': ti[11],
            'PMx': ti[12],
            'NOx': ti[13],
            'Fuel Consumption': ti[14],
        })

# 转换为DataFrame
df = pd.DataFrame(results)
# df.to_excel('simulation_results.xlsx')
