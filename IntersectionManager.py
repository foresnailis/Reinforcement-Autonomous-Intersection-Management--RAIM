#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 16 12:31:55 2020

@author: antonio
"""
import os
import traci
import traceback
import torch
import math
import random
import threading
import logging

import numpy as np
import traci.constants as tc

from functools import partial
from collections import defaultdict, namedtuple
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
import concurrent.futures

from TD3PER.ddpg_agent import Agent


class IntersectionManager:
    """Intersection Manager for each intersection that regulates
    the vehicles in each intersection.
    """

    def __init__(self, inter_id, inductionloops, seed, model_name, train=False, policy_noise=True, cf=False):
        self._id = inter_id # 交叉路口id
        self._traci = traci # sumo路由

        self._num_queues = 4 # 队列数量

        self._in_lanes = set(['Ai0', 'Ai1', 'Ai2', 'Ai3', 'Ai4', 'Ai5', 'Ai6', 'Ai7']) # 入口车道
        self._out_lanes = set(['Ao0', 'Ao1', 'Ao2', 'Ao3', 'Ao4', 'Ao5', 'Ao6', 'Ao7']) # 出口车道

        # self._id_veh_inter = set() # The params of vehicles inside the intersection

        self._max_dist_detect = 50 # 最大检测距离（相对于路口中心）
        self._queue_width = 12 # 队列宽度
        self._scaler = 1 # 缩放系数
        self._input_variables = 16 # speed, acceleration, etc 输入变量
        self.max_vehicles = 32 # 最大从车辆数

        self.state = defaultdict(partial(np.ndarray, 0)) # S_i
        self.new_state = defaultdict(partial(np.ndarray, 0)) # S_{i+1}

        self.raw_data = defaultdict(list) # 原始数据
        self.new_raw_data = defaultdict(list) # 下一个原始数据

        self.actions = defaultdict(list) # 动作记录
        self.rewards = defaultdict(float) # 奖励记录
        self.reward = -999 # 总奖励
        self.score = 0 # 得分
        self.vehicles_removed = set() # 被移除的车辆
        self.vehicles_first_time_outside = set() # 首次离开交叉口的车辆

        self._activeRequest = False # 是否有活跃请求
        self._greenTimeSoFar = 0 # 绿灯时间
        self._greenTimer = 60 # 绿灯计时器初值
        self._tlspeds = False # 是否有交通灯速度信息
        self._WALKINGAREAS = [':A0_w0', ':A0_w1', ':A0_w2', ':A0_w3'] # N, E, S, W 行人区域
        self._CROSSINGS = [':A0_c0', ':A0_c1', ':A0_c2', ':A0_c3'] # N, E, S, W 交叉口区域
            # check both sides of the crossing

        # PPO variables
        self._SEED = seed # 随机种子
        self._GAMMA = 0.9 # 折扣因子
        self._LOG_EACH = 1 # 日志记录间隔

        self._TrainingRecord = namedtuple('TrainingRecord', ['ep', 'reward']) # 训练记录
        self._Transition = namedtuple('Transition', ['s', 'a', 'a_log_p', 'r', 's_']) # 状态转换信息
        self._training_records = [] # 训练记录
        self._running_reward = -1000 # 运行奖励

        torch.manual_seed(self._SEED)
        self._device = torch.device("cuda:0") # 设备

        # PPO Agent
        # self.agent = Agent(self._device)
        # Define and build DDPG agent
        hidden_size = tuple([2048, 256, 256, 128]) # 隐藏层大小

        gamma = 0.9 # 折扣因子
        tau = 0.01 # 更新目标网络的软更新参数
        self.extra_variables = 6 # Timer (1), quarter (4), and priority signal (1) 额外的状态变量数量
        self.observation_space = self._input_variables * self.max_vehicles + self.extra_variables # 观测空间大小
        action_space = 1 # 动作空间大小
        checkpoint_dir = 'ckpt/' # 测试点路径
        self.batch_size = 128 # 批处理大小

        self.test = train # 是否在测试
        self.workers = os.cpu_count() - 1 # 运行线程
        self.rw_data = defaultdict(list) # 原始数据
        self.already_update = False # 是否更新
        self.cycle = 60 # 交通灯周期长度

# =============================================================================
#         # DDPG
# =============================================================================
        # self.agent = DDPG(gamma,
        #               tau,
        #               hidden_size,
        #               observation_space,
        #               action_space,
        #               checkpoint_dir=checkpoint_dir
        #               )

        # # Initialize replay memory
        # replay_size = 4096*16
        # self.memory_crash = ReplayMemory(int(replay_size))
        # self.memory_normal = ReplayMemory(int(replay_size))
        # self.action_noise = 0.5
        # Initialize OU-Noise
        # nb_actions = 1
        # noise_stddev = 5
        # self.ou_noise = OrnsteinUhlenbeckActionNoise(mu=np.zeros(nb_actions),
        #                                         sigma=float(noise_stddev) * np.ones(nb_actions))

# =============================================================================
#         # TD3
# =============================================================================
        # state_dim = observation_space
        # action_dim = action_space
        # max_action = 1
        # min_action = -1
        # discount = 0.99
        # tau = 0.005

        # self.agent = TD3.TD3(state_dim, action_dim, max_action, min_action)
        # self.replay_buffer = utils_TD3.ReplayBuffer(state_dim, action_dim)

        # self._exit = False

# =============================================================================
#         # TD3-PER
# =============================================================================
        state_size = self.observation_space # 状态空间大小
        action_size = action_space # 动作空间大小
        self.agent = Agent(state_size, action_size, model_name, policy_noise) # 代理
        self.LEARN_EVERY = 60 # 学习频率
        self.epoch = 0 # 轮次

        self.cf = cf

        # self._score = 0

    def reset_values(self): # 重置状态
        self.rewards = defaultdict(float) # 奖励记录重置
        self.vehicles_removed = set() # 车辆列表重置
        self.actions = defaultdict(list) # 动作列表重置
        self.vehicles_first_time_outside = set() # 首次离开的车辆列表重置
        self._exit = False # 退出标志位
        self.already_update = False # 更新标志位
        
    '''
    初始化状态函数first_state()和更新状态函数update_state()中都有obtain_state()
    此函数会返回一个list,表示当前所有还未通行车辆的所有信息
    每一项的内容都是这样的一维数组[x, y, dist, speed, angle, inlane, way, queue, is_inside]
    x, y       归一化后的相对坐标，相对于路口中央，取值[-1, 1]
    dist       车辆距离路口中央的直线距离,也映射到了[-1, 1]
    speed      把从SUMOapi拿到的速度,做了speed=speed/15*2-1
    angle      把从SUMOapi拿到的角度,angle=angle/180-1
    inlane     代表车道，因为默认三车道，所以有三个值,比如一车道[0, 0, 1]
    way        表示[左转,直行,右转]，也是三个值
    queue      有四个值,代表[W,E,S,N]
    is_inside  取值0/1,表示车辆是否在路口中
    '''
    def first_state(self):
        # self.state = self.new_state.copy()
        # self.raw_data = self.new_raw_data
        self.raw_data = self.obtain_state() # 调用方法得到原始数据
        self.state = self._update_state(self.raw_data) # 用原始数据更新状态
        return self.raw_data # 返回原始数据

    def update_state(self): # 得到后继状态
        self.new_raw_data = self.obtain_state() # 调用方法得到下一个状态的原始数据
        self.new_state = self._update_state(self.new_raw_data) # 使用该状态的原始数据得到状态
        return self.new_raw_data # 返回后继原始数据

    def swap_states(self): # 状态指针移动
        if self.new_raw_data: # 如果有下一状态的数据，更新当前状态为下一状态
            self.raw_data = self.new_raw_data.copy() 
            self.state = self.new_state.copy()
        else: # 否则重置当前状态
            self.raw_data = defaultdict(list)
            self.state = defaultdict(partial(np.ndarray, 0))

    def _update_state_mp(self, data): # 多进程环境更新状态 实现函数
        state = defaultdict(partial(np.ndarray, 0)) 
        # print('Data: ', data)
        veh, val = data
        # print(f'veh: {veh}; \n val: {val}')
        try:
            state = self.set_vehicle_highlight(state, veh, val)
            state = self.set_other_vehicles(state, self.new_raw_data, veh)
            state = self.zero_padding(state, veh)
        except Exception as e:
            print(f"_update_state_mp throws {e}")
            print(traceback.format_exc())
        return state

    def _update_state(self, rw_data): # 更新状态实现函数
        if rw_data and not rw_data == [-1]: # 若存在新数据
            # print('There are something')
            # These might be a defaultdict
            state = defaultdict(partial(np.ndarray, 0)) 
            # with ThreadPoolExecutor(max_workers=self.workers) as executor:
            #     results = executor.map(self._update_state_mp, rw_data.items())
            #     for k in results:
            #         key = list(k.keys())[0]
            #         state[key] = k[key]

            for k, v in rw_data.items(): # 对于所有rw中的车辆及其状态值
                # Highlight the vehicle selected
                state = self.set_vehicle_highlight(state, k, v) # 设置主控车辆状态
                state = self.set_other_vehicles(state, rw_data, k) # 设置其他车辆状态
                state = self.zero_padding(state, k) # 零填充

            return state
        else: # 若不存在
            state = defaultdict(partial(np.ndarray, 0))

    '''
    在原先一辆车的参数[x,y,dist,....]前面加上时间和优先级信息
    即[timer, quarter, priority, x, y, dist, ...]
    '''
    def set_vehicle_highlight(self, state, k, v):
        # state[k] = list(np.array(v[:2])/100) + [queue/3] + list(np.array(v[2:6])/np.array([13.89, 3, 2, 1])) + list(np.array(v[9:])/np.array([5, 5, 360]))
        # queue = self.transform_queue(queue)
        # state[k] = list(np.array(v[:2])) + queue + list(np.array(v[2:]))
        ctime = self._traci.simulation.getTime()
        # timer表示当前循环的时间，是个相对时间
        timer = ctime - (ctime // self.cycle) * self.cycle
        state[k] = [timer/(self.cycle-1)]
        state[k] += self._get_quarter(timer)
        state[k] += [self._get_priority(v, timer)]
        state[k] += v[:]

        # state[k] += [self.is_inside(k, self._id)]
        return state

    def _get_quarter(self, timer): # 得到时间块位置？当前timer在周期的哪个1/4，独热编码
        quarter = self.cycle//4

        if timer < quarter:
            return [1.0, 0.0, 0.0, 0.0]
        elif quarter <= timer < quarter*2:
            return [0.0, 1.0, 0.0, 0.0]
        elif quarter*2 <= timer < quarter*3:
            return [0.0, 0.0, 1.0, 0.0]
        elif quarter*3 <= timer < quarter*4:
            return [0.0, 0.0, 0.0, 1.0]
        else: # 错误情况 全零填充
            return [0.0, 0.0, 0.0, 0.0]

    def _get_priority(self, v, timer): # 得到车辆优先级
        # 具体而言，一辆车在同一时刻只可能有两种优先级：“优先”“不优先”，如果在N队列且时间在第一片，就优先，以此类推
        """
        Return -1 or +1 depends on the queue and the timer 
        The first quarter is for N queue
        the second quarter is for S queue
        third quarter is for E queue
        fourth quarter is for W queue

        Parameters
        ----------
        v : custom variable
            the important is the [-5:-1] digits that includes the queue
            [N, S, E, W]

        timer : int
            counter that goes from 0 to _cycle_var.

        Returns
        -------
        veh : int
            +1 if the vehicle has priority.
            -1 if the vehicle hasn't priority
        """
        # 根据车辆所处queue和当前时间，获取优先级
        # 一个循环里，被分为了四段，每一段允许一个方位的queue通行,顺序是N,S,E,W
        queue = np.where(np.array(v[-5:-1]) == 1.0)[0][0]

        quarter = self.cycle//4

        if queue == 0: # N queue
            if timer < quarter:
                return 1.0
        elif queue == 1: # S queue
            if quarter <= timer < quarter*2:
                return 1.0
        elif queue == 2: # E queue
            if quarter*2 <= timer < quarter*3:
                return 1.0
        elif queue == 3: # W queue
            if quarter*3 <= timer < quarter*4:
                return 1.0
        else:
            return -1.0
        return -1.0

    '''
    把其他车辆的state也加进去
    '''
    def set_other_vehicles(self, state, rw_data, k): # 更新其他车辆状态
        i = self._input_variables + self.extra_variables # 参数计数（当前只有主控信息）

        for k2, v2 in rw_data.items():
            if not k == k2: 
                (x,y) = v2[:2] # 位置信息
                # rel_x = np.floor(x*self._scaler)
                # rel_y = np.floor(y*self._scaler)
                # queue = self.obtain_queue(x, y)
                # queue = self.transform_queue(queue)

                # state[k] += list(np.array(v2[:2])) + queue + list(np.array(v2[2:]))
                state[k] += v2[:-1] # 在主控末尾添加该车辆的v（除去最后一个元素）

                state[k] += [self.dist_to_veh(k, x, y, rw_data)] # 在主控状态末尾添加当前车辆与主控的距离
                i += self._input_variables # 添加当前这里状态计数

                if i >= self.observation_space - 1: # 如果状态空间长度超限，就跳出
                    break
        return state

    def zero_padding(self, state, k): # 零填充 如果主控状态短于观测空间，就零填充
        l = len(state[k]) 
        pad = self.observation_space - l
        # print(f'Padding with: {pad/self._input_variables}')
        state[k] += list(np.zeros(pad))
        return state

    def is_inside(self, veh, int_id): # 看车辆在不在交叉口内，方法是比较车辆道路id和交叉口id
        r = self._traci.vehicle.getRoadID(veh)
        if r[:1+len(int_id)] == f':{int_id}': # Dentro de la interseccion, para penalizar más que estén dentro
            return 1.0
        else:
            return 0.0

    def dist_to_veh(self, veh, x2, y2, rw_data): # 求主控和某座标的欧式距离，再由transform_dist进行映射
        [x1, y1] = rw_data[veh][:2]
        dist = math.hypot((x2 - x1), (y2 - y1))/(2*self._max_dist_detect)
        dist = self.transform_dist(dist)
        return dist
        # return dist/(2*self._max_dist_detect)*2-1

    def transform_actions(self, actions): # 将原[-1,1]动作空间转换至[0.5,13.5]，线性映射
        actions = (actions + 1) / 2 # [0,1]
        actions = actions*13 # [0,13]
        actions = actions + 0.5 # [0.5, 13.5]
        return actions

    def select_actions_mp(self, veh): # 多进程环境下，单个车辆的动作选择（未使用）
        # print(f'Executing {veh} Task on thread: {threading.current_thread().name}')
        s = torch.Tensor(self.state[veh]).to(self._device) # 状态张量

        try:
            # action = self.agent.calc_action(state=s, action_noise=self.ou_noise) #calc_action(s)
            if self.agent.memory.is_full() or self.test: # 如果经验池已满或正在测试，就用代理的动作网络选择动作，否则随机选择动作
                action = self.agent.select_action(state=s) #calc_action(s)
            # Filling the replay buffer with random actions
            else:
                print(f'Len buff: {len(self.agent.memory)}')
                action = torch.Tensor([[np.random.uniform(-1, 1)]])
                # action = self.agent.select_action(state=s) #calc_action(s)
        except Exception as e:
            print("Error on select action: " + str(e))
            print(traceback.format_exc())
            print('Selecting 0.0 action')
            action = 0
        self.actions[veh] = action#.cpu() 将动作保存再动作字典中，键值为车辆的id

    # 把一个车辆的状态向量，输入到强化学习框架里，得到对应的action
    # 循环所有车辆，得到所有车辆的action到一个数组中，返回这个数组
    def select_actions(self):
        if self.raw_data and not self.raw_data == [-1]:
            # print('There are something')
            # with ThreadPoolExecutor(max_workers=3) as executor:
                # future = executor.map(self.select_actions_mp, self.raw_data)
                # print(results.result())
            # keys = self.raw_data.keys()
            # with ThreadPoolExecutor(max_workers=self.workers) as executor:
                # results = executor.map(self.select_actions_mp, keys)

            for k, v in self.raw_data.items():
                s = torch.Tensor(self.state[k]).to(self._device)
                try:
                    # action = self.agent.calc_action(state=s, action_noise=self.ou_noise) #calc_action(s)
                    if self.agent.memory.is_full() or self.test: # 如果经验池满/在测试，用动作网络
                        action = self.agent.select_action(state=s) #calc_action(s)


                    # Filling the replay buffer with random actions
                    else: # 否则也使用动作网络？
                        # print(f'Len buff: {len(self.agent.memory)}')
                        # action = torch.Tensor([[np.random.uniform(-1, 1)]])
                        action = self.agent.select_action(state=s) #calc_action(s)

                    # action = traci.vehicle.getSpeedWithoutTraCI(k)
                    # action = self.transform_actions(action)
                    # action = torch.Tensor([(action/13.89-0.5)*2])
                except Exception as e:
                    print("Error on select action: " + str(e))
                    print(traceback.format_exc())
                    print('Selecting 0.0 action')
                    action = 0
                # I delete the actions_dict before next+1 perform_action() to avoid perform actions twice
                self.actions[k] = action#.cpu() # 更新动作列表
        return self.actions

    # 执行所有的action
    # 貌似只改变速度
    def perform_actions(self):
        if self.actions:
            for k, v in self.actions.items():
                try:
                    if v.item()!=1:
                        print(v.item())
                    v = (v + 1)/2*13.39 + 0.5 # ？？什么玩意调整 13.39
                    traci.vehicle.slowDown(k, v, traci.simulation.getDeltaT()) # 选择sumo路由中的对应车辆，将其加/减速到对应值
                except Exception as e:
                    print(f"Error while perform action")
                    print(e)
                # print(f'Vehicle: {k}; Action: {v[0]}')


    def obtain_exit(self): # 得到退出标志位
        return self._exit

    # SUMO直接就有检测碰撞的api
    # 这里只是对碰撞信息进行打印，并惩罚相应车辆reward
    def obtain_collisions(self):
        # Obtain the number of vehicles involved in collision:
        collision_veh = set(traci.simulation.getCollidingVehiclesIDList()) # 获取碰撞集合
        if collision_veh: # 如果碰撞，退出位设置为1，返回碰撞次数，没有就返回0
            # print('WARNING!!!! --- collided vehicles on the road')
            # print(collision_veh)
            self._exit = True 

            for veh in collision_veh: # 每次碰撞，奖励列表添加一项-10，更新碰撞车辆的末尾符号位为1，表明发生了碰撞
                self.rewards[veh] += -20
                # traci.vehicle.setSpeed(veh, 0.01) # 将车辆速度降低，以增加平均等待时间并干扰系统
                # self.new_state[veh][-1] = 1
                try:
                    self.new_state[veh][-1] = 1 # 这里没看懂为什么要改？state最后一个参数是什么？
                    self._exit = True

                except Exception as e:
                    print(f"Error while obtain collisions")
                    print(traceback.format_exc())
                    self._exit = True


                self.vehicles_removed.add(veh)
            return len(collision_veh)
        else:
            return 0

    def obtain_reward(self): # 得到每辆车辆的单步奖励
        if self.raw_data and not self.raw_data == [-1]: # 如果有数据
            # Obtain reward:
            # rel_speed = []
            w1 = -0.05
            w2 = -0.075
            w3 = -0.075 # 三权重
			
            stepLength = traci.simulation.getDeltaT() # 仿真时间步长（没用到）
            for k, v in self.actions.items(): # 遍历动作列表，更新奖励列表
                try:
                    # speed = traci.vehicle.getSpeed(k)
                    # max_speed = traci.vehicle.getAllowedSpeed(k)
                    # rel_speed.append(speed/max_speed)
                    # self.rewards[k] = speed/max_speed # ratio between current speed and max_speed
                    # low_speed = 1 + np.abs(max_speed - speed) # Penalizamos las velocidades bajas
                    # speed_dev = 1 #+ np.abs(speed - self.actions[k][0]) # Penalizo las acciones cuando intenta ir rápido, pero va lento por congestión

                    # r = traci.vehicle.getRoadID(k)
                    # if r[-2:] == self._id: #  Aproximandose a la interseccion:
                        # factor = 1
                    # elif r[:1+len(self._id)] == f':{self._id}': # Dentro de la interseccion, para penalizar más que estén dentro
                        # factor = 2
                    # else:
                        # factor = 1

                    # speed_notraci = traci.vehicle.getSpeedWithoutTraCI(k)
                    # v = (v + 1)/2*13.39 + 0.5
                    # rew = stepLength + np.abs(speed_notraci - v)

                    speed = traci.vehicle.getSpeed(k) # 拿到车辆速度
                    max_speed = traci.vehicle.getAllowedSpeed(k) # 拿到限速
                    delay = 1 - speed/max_speed # 计算一个delay值，似乎是逼迫速度提高的量
                    wt = self._traci.vehicle.getWaitingTime(k) # 得到停留时间
                    acc_wt = self._traci.vehicle.getAccumulatedWaitingTime(k) # 得到累计停留时间
                    '''
                    奖励函数公式
                    '''
                    rew = w1*delay + w2*wt + w3*acc_wt  # 根据以上数据计算该步奖励
                    # print_log = open("printlog.txt",'a')
                    # print(f"{k}: {speed}, {v}",file = print_log)
                    # print_log.close()

                    if k in self.rewards: # 在第k个量中增加该步奖励
                        self.rewards[k] += rew #-stepLength #-(rew * low_speed * speed_dev * factor) #-stepLength * low_speed * speed_dev * factor #-stepLength self.rewards[k]*1
                    else:
                        self.rewards[k] = rew #-stepLength #-(rew * low_speed * speed_dev * factor)#-stepLength * low_speed * speed_dev * factor #-stepLength

                    # print(f'{k}; A: {v[0][0]:.2f}; S:{speed:.2f}; Max_s: {max_speed:.2f}; S notraci: {speed_notraci:.2f}; Reward: {self.rewards[k]}')
                    # if self.rewards[k]>0:
                        # print('STOP')
                except Exception as e:
                    print(f"Error while obtain reward, the vehicle doesn't exist111")
                    print(e)
                    print(traceback.format_exc())


            # Obtain the state, action, reward, and new_state for each vehicle
            self.already_update = False # 设置更新状态位为0（网络）
            for k in self.raw_data.items(): # 遍历数据列表来更新网络
                try:
# =============================================================================
#                     s = torch.Tensor([self.state[k[0]]]).to(self._device)
#                     r = torch.Tensor([self.rewards[k[0]]]).to(self._device)
#                     a = torch.Tensor([self.actions[k[0]]]).view((1, -1)).to(self._device)
#                     # done = True if r.cpu()[0] > 0 else False
#                     done = k[0] in self.vehicles_first_time_outside
#
#                     mask = torch.Tensor([done]).to(self._device)
#
#                     new_s = torch.Tensor([self.new_state[k[0]]]).to(self._device)
# =============================================================================
                    s = self.state[k[0]]
                    r = self.rewards[k[0]]
                    a = self.actions[k[0]] # 得到当前车辆的s_i，r_i，a_i
                    done = k[0] in self.vehicles_first_time_outside  # 标志位，记录当前车辆是否离开

                    new_s = self.new_state[k[0]] # 当前车辆s_{i+1}
                    # desire_shape = self.observation_space

                    # TD3
# =============================================================================
#                     # if (len(s) == desire_shape) and (len(new_s) == desire_shape):
#                         # self.replay_buffer.add(s, a, new_s, r, done)
#
#                     # if len(self.replay_buffer) > self.batch_size:
#                         # self.agent.train(self.replay_buffer, self.batch_size)
# =============================================================================

                    if (len(s) == self.observation_space) and (len(new_s) == self.observation_space): # 如果两个状态都合法
                        self.agent.step(s, a, r, new_s, done) # 令代理更新
                        self.score += r # 分数增加

                        # The replay buffer is full
                        if self.agent.memory.is_full(): # 如果经验池已满
                            if self.epoch % self.LEARN_EVERY == 0 and self.already_update == False: # 如果到了需要学习的时候，且还未更新
                                print(f'Updating agent bcs: {self.epoch}')
                                self.agent.learn() # 执行更新
                                self.already_update = True # 更新标志位

                        # self.epoch += 1

                except Exception as e:
                    # print(f"Error while obtain reward, the vehicle doesn't exist22222")
                    print(e)
                    # print(traceback.format_exc())
                # else:
                    # print('Look at me!')
                # try:
                # epoch_value_loss = 0
                # epoch_policy_loss = 0
            self.epoch += 1 # 迭代步数加1

        self.actions.clear() # 清空动作列表
        # if self._exit:
            # self._traci.close()

        return self.score # 返回得分


    def get_score(self): # 得到得分
        return self.score

    def control_tls(self): # 信号灯更新函数 设置id节点的信号灯。从北口开始，每个车道为一个位置，顺时针编码
        # self._greenTimer = self._greenTimer - self._traci.simulation.getDeltaT()
        self._traci.trafficlight.setRedYellowGreenState( 
                 self._id, 'rrGGGGGGrrGGGGGGrrGGGGGGrrGGGGGGrrrrGG')

    def step(self): # 步进函数
        self.control_tls() # Control the tls associated in the intersection


    def obtain_position(self, queue, x, y): # 得到车辆相对于路口中心的位置
        d = self._max_dist_detect #* self._scaler 

        if queue == 0:  # N queue
            return [int(d - y), int(np.abs(x))]

        elif queue == 1:  # S queue
            return [int(d + y), int(x)]

        elif queue == 2:  # E queue
            return [int(d - x), int(y)]

        elif queue == 3:  # W queue
            return [int(d + x), int(np.abs(y))]

        else:
            return 0

    def obtain_state(self):

        vehicles = self._obtain_vehicles_in_intersection()
        if not vehicles == -1 and not vehicles == set():
            data = self._obtain_vehicles_params(vehicles)
            return data
        else:
            return defaultdict(list)

    def _obtain_vehicles_in_intersection(self): # 以上函数的细节实现
        try:
            # https://sumo.dlr.de/pydoc/traci.constants.html
            self._traci.junction.subscribeContext(self._id,
                                                  tc.CMD_GET_VEHICLE_VARIABLE,
                                                  self._max_dist_detect,
                                                  [tc.VAR_ROAD_ID]) # 向路由订阅一定范围内的车辆信息，注册了一个回调函数，在车辆数目变化时返回状态更新

            vehicles = traci.junction.getContextSubscriptionResults(self._id)
            # 返回的vehicles是包含车辆信息的字典，键是车辆的标识符，值是包含车辆信息的字典

            if vehicles: # 如果有车，调用rma
                if self.cf == False:
                    for veh in vehicles:
                        traci.vehicle.setLaneChangeMode(veh,0b000000000000)
                        traci.vehicle.setSpeedMode(veh, 000000)
                # print(f'\n Found {len(vehicles)} vehicles before filtering')
                # print(f'\t Keys: {vehicles.keys()}')
                return self._remove_moving_away(self._id, vehicles) # If vehicles are approaching the intersection 

            else:
                # print('No vehicles found')
                return -1

        except Exception as e:
            print("type error: " + str(e))
            print(traceback.format_exc())
            return 0

    def _remove_moving_away(self, j, v):
        # j代表交叉路口ID，v就是通过sumoapi获取的车辆字典集合
        """ Obtain the vehicles that are aproaching the intersection or are inside. """
        # stepLength = traci.simulation.getDeltaT()
        
        # 更新当前所有车辆的状态，这里的状态指车辆正在approach/inside/moving away交叉路口
        # 我这里之所以说更新，是因为以下的for循环把正在远离（成功通行）的车辆加入到了一个类成员变量self.vehicles_removed里
        # 并且把所有已通行车辆的奖励加上
        for key, val in v.items():
            # 如果车辆不在vehicles_removed（已通行集合）里
            if not key in self.vehicles_removed:
                # val[tc.VAR_ROAD_ID] === ':xx'; 
                # 其中xx表示交叉路口ID，前面的冒号是对车辆此时在交叉路口的状态标志，如果是冒号就代表在交叉路口里
                if not (val[tc.VAR_ROAD_ID][-2:] == j) and not (val[tc.VAR_ROAD_ID][:1+len(j)] == f':{j}'):
                    # speed = traci.vehicle.getSpeed(key)
                    # neg_speed = 1 + np.abs(speed - self.actions[key])
                    # traci.vehicle.setSpeedMode(key, 31)
                    self.rewards[key] += 10 #-stepLength # 奖励+10
                    self.vehicles_removed.add(key) # 添加到列表中
            else:
                if not (val[tc.VAR_ROAD_ID][-2:] == j) and not (val[tc.VAR_ROAD_ID][:1+len(j)] == f':{j}'): # 否则也看是不是在远离，如果是，奖励+10
                    # speed = traci.vehicle.getSpeed(key)
                    # neg_speed = 1 + np.abs(speed - self.actions[key])
                    self.rewards[key] += 10 #-stepLength

        # 这里体现了函数名称_remove_moving_away，剔除已通行车辆，因为这些车辆已经对交叉路口通行无影响
        # 简单粗暴，重新定义新的set，然后把val[tc.VAR_ROAD_ID]==':{j}'的车加进去，然后返回
        # 为了重复加入已通行车辆，还设置了self.vehicles_first_time_outside，表示车辆第一次被记录在已通行集合里
        vehicles = set()
        for veh, val in v.items():
            if (val[tc.VAR_ROAD_ID][-2:] == j): # If vehicles are approaching the intersection 车辆道路id以交叉口id开头，认为是
                vehicles.add(veh)
            elif (val[tc.VAR_ROAD_ID][:1+len(j)] == f':{j}'): # If vehicles are inside the intersection 如果车辆在交叉口中，认为是
                vehicles.add(veh)
            elif not veh in self.vehicles_first_time_outside: # If vehicles is just leaving the intersection 如果车辆第一次离开交叉口，认为是
                vehicles.add(veh)
                self.vehicles_first_time_outside.add(veh) # 并将这辆车加到首次离开之中

        return vehicles # 返回这个列表

    def _obtain_vehicles_params(self, vehicles): # 得到车辆参数
        """
        Parameters
        ----------
        vehicles : TYPE
            DESCRIPTION.

        Returns
        -------
        updated data.

        """
        data = defaultdict(list)

        for veh in vehicles:
            new_params = self._obtain_veh_params(veh)
            data[veh] = new_params

        return data


    def _obtain_veh_params(self, veh): # 上面函数的实际实现
        # 得到：交叉口位置、车辆绝对位置、车辆相当于交叉口中心的相对归一化位置、
        # 相对于交叉口中心的归一化转化距离、车辆的归一化速度、车辆车道索引、
        # 车辆目的位置、车辆所在独热队列、车辆归一化角度
        """
        Parameters
        ----------
        vehicle : str
            the identifier of the vehicle to obtain it's params.

        Returns
        -------
        params : list(params)
            multiple params.

        """
        self._position = self._traci.junction.getPosition(self._id)

        (x, y) = self._traci.vehicle.getPosition(veh)
        center_x = self._position[0]  # - self._max_dist_detect
        center_y = self._position[1]  # + self._max_dist_detect
        rel_x = (x - center_x)/self._max_dist_detect
        rel_y = (y - center_y)/self._max_dist_detect  #center_y - y

        rel_x = self.transform_position(rel_x)
        rel_y = self.transform_position(rel_y)

        dist = math.hypot((center_x - x), (center_y - y)) / self._max_dist_detect # 返回车辆与路口中央的欧几里得距离
        dist = self.transform_dist(dist)

        speed = self._traci.vehicle.getSpeed(veh)/15
        # acc = self._traci.vehicle.getAcceleration(veh)
        inlane = self._traci.vehicle.getLaneIndex(veh)
        if inlane == 1:
            inlane = [0.0, 0.0, 1.0]
        elif inlane == 2:
            inlane = [0.0, 1.0, 0.0]
        elif inlane == 3:
            inlane = [1.0, 0.0, 0.0]
        else:
            inlane = [0.0, 0.0, 0.0]
        way = self._get_way(veh) # Way that the vehicle follows (right:+2; straigh:+1; left:0)/2
        queue = self.obtain_queue(rel_x, rel_y)
        queue = self.transform_queue(queue)
        # 这里的queue=[W,E,S,N]。在哪个方位就置1，其他0

        # wt = self._traci.vehicle.getWaitingTime(veh)
        # acc_wt = self._traci.vehicle.getAccumulatedWaitingTime(veh)
        # vtype = self._traci.vehicle.getTypeID(veh)
        # width = self._traci.vehicle.getWidth(veh)
        # length = self._traci.vehicle.getLength(veh)
        angle = self._traci.vehicle.getAngle(veh)/180-1

        # params = [rel_x, rel_y, speed, acc, inlane, way, wt,
                  # acc_wt, vtype, width, length, angle]
        # params = [(rel_x+1)/2, (rel_y+1)/2, dist/self._max_dist_detect, speed] + inlane + way
        params = [rel_x, rel_y, dist, speed*2 - 1, angle] + inlane + way + queue + [self.is_inside(veh, self._id)]

        return params

    # Sigmoid函数，归一化到[-a/2, a/2]，约等于[-1, 1]
    def transform_position(self, pos):
        a = 2.025
        b = 5
        return a/(1 + np.exp(-b*pos)) - a/2

    # 映射到[-c, a-c], 这里约等于[-1, 1]
    def transform_dist(self, dist):
        a = 2.1
        b = 3
        c = 1.1
        return a * np.exp(-b*dist) - c

    def _get_way(self, veh): # 得到行为

        route = self._traci.vehicle.getRoute(veh) # 拿到当前route
        route_index = self._traci.vehicle.getRouteIndex(veh) # 拿到当前route的索引

        try: 
            if route_index == len(route) - 1: #It already/is finish 当前路线已经完成，则起点是
                o = route[route_index - 1].replace(self._id,'')[0] # 得到前一个路径点，去除其中的交叉路口id，取首字母
                d = route[route_index].replace(self._id,'')[0] # 得到当前路径点，相同操作
            elif not route_index == -1:
                o = route[route_index].replace(self._id,'')[0] # 得到当前路径点，相同操作
                d = route[route_index + 1].replace(self._id,'')[0] # 得到下一路径点，相同操作
        except:
            print('An unexpected error has happened, please fix it')
            print(f'route: {route}')
            print(f'route_index: {route_index}')
            o = 'l'
            d = 'r'
        od = o+d
        # l:left, r:right, t:top, b:bottom
        forward_way = set(['lr', 'rl', 'bt', 'tb']) # 所有直行操作 left->right, ...
        turn_right = set(['lb', 'rt', 'br', 'tl']) # 所有右转操作
        turn_left = set(['lt', 'rb', 'bl', 'tr']) # 所有左转操作

        if od in forward_way:  # Forward path
            return [0.0, 1.0, 0.0]

        elif od in turn_left:  # This is the left-turn path
            return [1.0, 0.0, 0.0]

        elif od in turn_right:  # This is the right-turn path
            return [0.0, 0.0, 1.0]

        else:
            print('There is an error with get_way')
            print(f'route: {route}')
            print(f'route_index: {route_index}')
            print(f'od: {od}')
            return [0,0,0]

    def obtain_queue(self, x, y):
        if x >= 0 and y >= 0:  # E wueue
            return 2

        elif x < 0 and y >= 0:  # N queue
            return 0

        elif x >= 0 and y < 0:  # S queue
            return 1

        elif x < 0 and y < 0:  # W queue
            return 3

        else:
            return 4
    
    def transform_queue(self, queue):
        if queue == 0:
            queue = [0.0, 0.0, 0.0, 1.0]
        elif queue == 1:
            queue = [0.0, 0.0, 1.0, 0.0]
        elif queue == 2:
            queue = [0.0, 1.0, 0.0, 0.0]
        elif queue == 3:
            queue = [1.0, 0.0, 0.0, 0.0]
        else:
            queue = [0.0, 0.0, 0.0, 0.0]
        return queue