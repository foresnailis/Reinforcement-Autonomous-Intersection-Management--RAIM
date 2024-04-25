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
# 此处需修改为本地仓库的路径
if pltf == "Windows":
    root = 'D:/TongjiCourse/Multi-Agent/Reinforcement-Autonomous-Intersection-Management--RAIM'
else:
    root = '/root/RAIM'

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
# Params
nrows = 1
# Number of columns:
ncols = 1
# Number of lanes:
nlanes = 2 # 车道数
# Lenght (m):
length = 200

red_manhattan = ManhattanGraph(3, 3, 300)
# escenario = ScenarioTwo(red_manhattan,prob=100)
# escenario = ScenarioThree(red_manhattan, 50, 1000, 1500, 3600)
# escenario = ScenarioFour(red_manhattan)
escenario = ScenarioFive(red_manhattan)

nlanes = 2
simulacion = SumoSimulation(red_manhattan, gui=True, lanes=nlanes,
                            nrows=nrows, ncols=ncols, leng=length,
                            seed=SEED, flow=250)

# Algoritmo para controlar los semáforos. Deprecated in v3
# 控制交通灯的算法。V3中折旧
Fixed = FixedAlgorithm(greentime=(120-10)//2, lanes=nlanes)

# simulacion.im.agent.load_param()
# simulacion.im.agent.load_checkpoint(checkpoint_path='ckpt/ep_collisions_70.pth.tar')
# simulacion.im.agent.load_checkpoint(step=15)
# simulacion.im.agent.load_imitationLearning(path='ckpt/m_4096_2048_1024_0.0209.pkl')
# simulacion.im.agent.load('ckpt/TD3/150')
# simulacion.im.agent.load('ckpt/TD3/300_best')

time_now = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
start_time = time.time()
# simulacion.create_route_files_v2()

flow = 150

simulacion.seed = SEED # 基于当前轮次的索引更新了随机种子，以改变随机性
simulacion.change_algorithm(Fixed) # 设置控制算法
simulacion.change_scenario(escenario) # 设置交通场景

elapsed_time = time.time() - start_time
print(time.strftime("Elapsed time: %H:%M:%S", time.gmtime(elapsed_time)))
simulacion.simulation_duration = 5*60
simulacion.flow = flow

c = simulacion.run_test_simulation(weight_path='ckpt/TD3')  # 执行一次仿真

ti = simulacion.getTripinfo() # 获取仿真车辆的行程信息

if ti[0] > 0: # No ha habido error en tripInfo por el # de veh 检查车辆行程信息是否有效
    print(f'Mean duration: {ti[5]:.2f}, Mean wtime: {ti[6]:.2f}, Mean timeloss: {ti[7]:.2f}, flow: {simulacion.flow}, collisions: {c}\n')

elapsed_time = time.time() - start_time
print(time.strftime("%H:%M:%S", time.gmtime(elapsed_time)))