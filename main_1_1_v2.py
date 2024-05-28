# -*- coding: utf-8 -*-
"""
Created on Wed Jun 10 16:49:54 2020

@author: anton
"""

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

from torch.utils.tensorboard import SummaryWriter
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

with open('proj-root.txt', 'r') as file:
    root = file.read().strip()

os.chdir(root)

sys.path.append(f"{root}/algorithms")
sys.path.append(f"{root}/api_sumo")
sys.path.append(f"{root}/api_sumo/sumo_elems")
sys.path.append(f"{root}/graphs")
sys.path.append(f"{root}/scenarios")

from algorithms import *
from api_sumo import *
from graphs import *
from scenarios import *

# % 设置种子可以确保每次运行代码时得到相同的随机数序列，从而使实验可重现
SEED = 42
torch.manual_seed(SEED)
np.random.seed(SEED)
random.seed(SEED)

# Writer will output to ./runs/ directory by default
writer = SummaryWriter()

import argparse

parser = argparse.ArgumentParser(description="Traffic Simulation Parameters")
parser.add_argument('--nrows', type=int, default=1, help='Number of rows in the grid')
parser.add_argument('--ncols', type=int, default=1, help='Number of columns in the grid')
parser.add_argument('--nlanes', type=int, default=2, help='Number of lanes per road')
parser.add_argument('--length', type=int, default=200, help='Length of each road segment (in meters)')
parser.add_argument('--seed', type=int, default=42, help='Random seed for reproducibility')
parser.add_argument('--policy_noise', type=bool, default=True, help='POLICY NOISE')
parser.add_argument('--class_learn', type=bool, default=True, help='Class Learning')
parser.add_argument('--cf', type=bool, default=False, help='Car Following and lane change.')
parser.add_argument('--gui', type=bool, default=False, help='Whether to use SUMO GUI')
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


model_weight_path=os.path.join('ckpt', args.model_name)

simulacion = SumoSimulation(red_manhattan, gui=args.gui, lanes=args.nlanes,
                            nrows=args.nrows, ncols=args.ncols, leng=args.length,
                            seed=args.seed, flow=args.flow, weight_path=model_weight_path, 
                            policy_noise=args.policy_noise, cf= args.cf, model_name=args.model_name)

time_now = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
start_time = time.time()

training_tripinfo = [] # 训练过程车辆行程信息
i = 0
change_seed_every = 5
best_timeloss = 9999 # 记录最佳时间损失
best_collisions = 9999 # 记录最佳碰撞次数

flow = args.flow

try:
    for epoch in np.arange(args.epochs):
        simulacion.i_ep = epoch # 将当前轮次的索引传递给仿真环境
        simulacion.seed = int(epoch/change_seed_every) # 基于当前轮次的索引更新了随机种子，以改变随机性
        simulacion.change_algorithm(Fixed) # 设置控制算法
        simulacion.change_scenario(escenario) # 设置交通场景
        # simulacion.flow = flow
        # 根据智能体的记忆是否已满来调整仿真环境中的流量参数。如果记忆已满，
        # 则使用预定义的流量值；否则，随机选择一个流量值。
        if simulacion.im.agent.memory.is_full():
            elapsed_time = time.time() - start_time
            print(time.strftime("Elapsed time: %H:%M:%S", time.gmtime(elapsed_time)))
            simulacion.simulation_duration = 5*60
            simulacion.flow = flow
        else:
            elapsed_time = time.time() - start_time
            print(time.strftime("Elapsed time: %H:%M:%S", time.gmtime(elapsed_time)))
            simulacion.simulation_duration = 5*60
            simulacion.flow = np.random.randint(25, 600)

        [r,t,s,a,c,q1loss,q2loss,aloss,q1,q2] = simulacion.run_simulation()  # 执行一次仿真
        ti = simulacion.getTripinfo() # 获取仿真车辆的行程信息

        if ti[0] > 0: # No ha habido error en tripInfo por el # de veh 检查车辆行程信息是否有效
            try:
                training_tripinfo.append(ti) 
                a = np.reshape(training_tripinfo, (-1, 9))
                # 下面将训练的指标写入
                writer.add_scalar('Global/Density', flow, i)
                writer.add_scalar('Global/# of Vehicles', ti[0], i)
                writer.add_scalar('Global/# Collisions', np.sum(c), i)
                writer.add_scalar('Global/Reward', t[1], i)
                writer.add_scalar('Global/Score', t[2], i)

                writer.add_scalar('Timeloss/Total', ti[1], i)
                writer.add_scalar('Duration/Total', ti[2], i)
                writer.add_scalar('wTime/Total', ti[3], i)
                writer.add_scalar('Timeloss/Relative', ti[4], i)
                writer.add_scalar('Duration/Average', ti[5], i)
                writer.add_scalar('wTime/Average', ti[6], i)
                writer.add_scalar('Timeloss/Average', ti[7], i)
                writer.add_scalar('Timeloss/Max', ti[8], i)
                writer.add_scalar('NetworkLoss/Q1', q1loss, i)
                writer.add_scalar('NetworkLoss/Q2', q2loss, i)
                writer.add_scalar('NetworkLoss/A', aloss, i)
                writer.add_scalar('Action Value/Q1', q1, i)
                writer.add_scalar('Action Value/Q2', q2, i)
                i += 1

                # 课程学习
                # 根据历史车辆行程信息的变化情况来动态调整交通流量参数，以优化模型的训练和性能。
                # 这一行代码检查了历史记录数据量是否大于250。这里的 a 是一个二维数组，存储了历史车辆行程信息。
                # 当历史记录数据量超过250时，表示已经收集了一定数量的数据，可以进行后续的判断。
                # 如果方差小于0.005倍的流量，则认为环境变化不大，可以增加流量。
                # 如果历史记录数据量超过1000条，即已经收集了一定量的数据，也可以增加流量。
                if args.class_learn:
                    if len(a) > 250:
                        # if np.mean(a[:,7][-1000:]) < 0.5:
                        if np.var(a[:,7][-250:]) < 0.005*flow or len(a) > 1000:
                            flow += 25
                            simulacion.flow = flow
                            print(f'提升flow至: {flow}；此时方差为: {np.var(a[:,7][-100:])}')
                            training_tripinfo = []
                            best_timeloss = 9999
                            best_collisions = 9999
                # 保留最好
                # 当前的碰撞次数和时间损失均优于历史最佳值
                if best_collisions >= np.sum(c) and best_timeloss >= ti[7]:
                    best_timeloss = ti[7]
                    best_collisions = np.sum(c)
                    # simulacion.im.agent.save_checkpoint(str(flow) + '_best')
                    save_dir = os.path.join(model_weight_path, str(flow) + '_best')
                    os.makedirs(save_dir, exist_ok=True)
                    simulacion.im.agent.save_weights(save_dir)

                print(f'Simulation: {epoch}; Mean duration: {ti[5]:.2f}, Mean wtime: {ti[6]:.2f}, Mean timeloss: {ti[7]:.2f}, flow: {simulacion.flow}, reward: {t[1]}, score: {t[2]}\n')
            except Exception as e:
                print("type error: " + str(e))
    simulacion.im.agent.memory.save_experience()
except Exception as e:
    print("type error: " + str(e))
    print(traceback.format_exc())
    simulacion.close_simulation()

elapsed_time = time.time() - start_time
print(time.strftime("%H:%M:%S", time.gmtime(elapsed_time)))

save_dir = os.path.join(model_weight_path, str(flow) + '_simple')
os.makedirs(save_dir, exist_ok=True)
simulacion.im.agent.save_weights(save_dir)