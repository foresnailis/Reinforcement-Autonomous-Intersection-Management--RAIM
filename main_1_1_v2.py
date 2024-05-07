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

# from sumolib import checkBinary  # noqa
import traci  # noqa

# sys.path.append("/usr/share/sumo/bin") # Para linux
# sys.path.append("/usr/share/sumo/tools") # Para linux

#%
pltf = platform.system()
# if pltf == "Windows":
#     print("Your system is Windows")
#     netgenBinary = checkBinary('netgenerate.exe')
#     sumoBinary = checkBinary('sumo-gui.exe') 

# else:
#     print("Your system is Linux")
#     netgenBinary = checkBinary('netgenerate')
#     sumoBinary = checkBinary('sumo-gui')
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
SEED = 42

torch.manual_seed(SEED)
np.random.seed(SEED)
random.seed(SEED)

# Writer will output to ./runs/ directory by default
writer = SummaryWriter()

model_name = "TD3"
# Params
nrows = 1
# Number of columns:
ncols = 1
# Number of lanes:
nlanes = 2 # 车道数
# Lenght (m):
length = 200

# Estas líneas son *heredadas* y van a estar deprecated en la versión v3
red_manhattan = ManhattanGraph(3, 3, 300)
escenario = ScenarioThree(red_manhattan, 250, 500, 800, 900)

# Crea la simulación
nlanes = 2
simulacion = SumoSimulation(red_manhattan, gui=False, lanes=nlanes,
                            nrows=nrows, ncols=ncols, leng=length,
                            seed=SEED, flow=25)

# Algoritmo para controlar los semáforos. Deprecated in v3
# 控制交通灯的算法。V3中折旧
Fixed = FixedAlgorithm(greentime=(120-10)//2, lanes=nlanes)

#%
# simulacion.im.agent.load_param()
# simulacion.im.agent.load_checkpoint(checkpoint_path='ckpt/ep_collisions_70.pth.tar')
# simulacion.im.agent.load_checkpoint(step=15)
# simulacion.im.agent.load_imitationLearning(path='ckpt/m_4096_2048_1024_0.0209.pkl')
# simulacion.im.agent.load('ckpt/TD3/150')
# simulacion.im.agent.load('ckpt/TD3/300_best')

# weight_path = 'ckpt'
# simulacion.im.agent.load_weights(weight_path)

time_now = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
start_time = time.time()
epochs = 200 # 训练轮次
rewards = [] # 训练奖励值
training_records = [] # 训练统计数据
training_tripinfo = [] # 训练过程车辆行程信息
aux = []
collisions = []
# simulacion.create_route_files_v2()

flow = 150
i = 0
change_seed_every = 5
best_timeloss = 9999 # 记录最佳时间损失
best_collisions = 9999 # 记录最佳碰撞次数

simulacion.im.agent.load_weights('ckpt/' + model_name)
try:
    for epoch in np.arange(epochs):
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
        rewards.append(r) # 奖励值
        training_records.append(t) # 训练记录
        ti = simulacion.getTripinfo() # 获取仿真车辆的行程信息
        collisions.append(np.sum(c)) # 碰撞次数

        if ti[0] > 0: # No ha habido error en tripInfo por el # de veh 检查车辆行程信息是否有效
            try:
                training_tripinfo.append(ti) 
                aux.append(ti)
                a = np.reshape(training_tripinfo, (-1, 9))

                b = np.reshape(aux, (-1, 9))
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
                # flow += 50

                '''
                # 根据历史车辆行程信息的变化情况来动态调整交通流量参数，以优化模型的训练和性能。
                # 这一行代码检查了历史记录数据量是否大于250。这里的 a 是一个二维数组，存储了历史车辆行程信息。
                # 当历史记录数据量超过250时，表示已经收集了一定数量的数据，可以进行后续的判断。
                # 如果方差小于0.005倍的流量，则认为环境变化不大，可以增加流量。
                # 如果历史记录数据量超过1000条，即已经收集了一定量的数据，也可以增加流量。
                if len(a) > 250:
                    # if np.mean(a[:,7][-1000:]) < 0.5:
                    if np.var(a[:,7][-250:]) < 0.005*flow or len(a) > 1000:
                        flow += 25
                        simulacion.flow = flow
                        print(f'Increasing flow to: {flow} due to, the var is: {np.var(a[:,7][-100:])}')
                        training_tripinfo = []
                        best_timeloss = 9999
                        best_collisions = 9999
                '''

                # 保留最好
                # 当前的碰撞次数和时间损失均优于历史最佳值
                if best_collisions >= np.sum(c) and best_timeloss >= ti[7]:
                    best_timeloss = ti[7]
                    best_collisions = np.sum(c)
                    # simulacion.im.agent.save_checkpoint(str(flow) + '_best')
                    save_dir = 'ckpt/'+ model_name + '/' + str(flow) + '_best'
                    os.makedirs(save_dir, exist_ok=True)
                    simulacion.im.agent.save_weights(save_dir)

                print(f'Simulation: {epoch}; Mean duration: {ti[5]:.2f}, Mean wtime: {ti[6]:.2f}, Mean timeloss: {ti[7]:.2f}, flow: {simulacion.flow}, reward: {t[1]}, score: {t[2]}\n')
                # print(f'Training records: {t}')
            except Exception as e:
                print("type error: " + str(e))
    simulacion.im.agent.memory.save_experience()
except Exception as e:
    print("type error: " + str(e))
    print(traceback.format_exc())
    simulacion.close_simulation()

elapsed_time = time.time() - start_time
print(time.strftime("%H:%M:%S", time.gmtime(elapsed_time)))

save_dir = 'ckpt/'+ model_name + '/' + str(flow) + '_simple'
os.makedirs(save_dir, exist_ok=True)
simulacion.im.agent.save_weights(save_dir)