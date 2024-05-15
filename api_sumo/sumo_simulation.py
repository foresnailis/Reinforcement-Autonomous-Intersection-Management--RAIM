#!/usr/bin/env python
"""

"""

import subprocess
import sys
import platform
import traceback
import pickle

import pandas as pd
import numpy as np

import traci
import traci.constants as tc

from IntersectionManager import IntersectionManager
from sumolib import checkBinary  # noqa
from collections import defaultdict, namedtuple
from xml.dom import minidom

__author__ = "Bryan Alexis Freire Viteri. Mod by Antonio Guillen Perez"
__version__ = "3.0"
__email__ = "bryanfv95@gmail.com antonio.guillen@edu.upct.es"

'''
**成员函数**：

1. `__init__`：
   - 输入：各种模拟参数
   - 输出：无
   - 功能：初始化模拟器，设置参数。

2. `change_graph`、`change_scenario`、`change_algorithm`：
   - 输入：图形、场景、算法对象
   - 输出：无
   - 功能：改变当前模拟的图形、场景和算法。

3. `run_simulation`：
   - 输入：无
   - 输出：模拟结果、训练记录、状态、动作、碰撞信息
   - 功能：运行模拟，执行交通流和奖励的计算，并返回结果。

4. `run_test_simulation`：
   - 输入：无
   - 输出：无
   - 功能：运行测试模拟。

5. `obtain_loss_time`：
   - 输入：无
   - 输出：损失时间
   - 功能：获取损失时间的计算。

6. `getTripinfo`：
   - 输入：无
   - 输出：模拟统计信息
   - 功能：获取模拟的一些统计信息，如旅行次数、总耗时等。

7. `reset_statistics`：
   - 输入：无
   - 输出：无
   - 功能：重置模拟的统计信息。

8. `init_simulation`：
   - 输入：无
   - 输出：无
   - 功能：初始化模拟。

9. `close_simulation`：
   - 输入：无
   - 输出：无
   - 功能：关闭模拟。
'''

class SumoSimulation(object):
    '''
    成员对象：
    sg、ss、sa：分别代表图形、场景和算法的对象，应该是对SumoAlgorithm、SumoGraph、SumoScenario等的调用吧
    sgC、ssC、saC：布尔值，用于跟踪是否更改了以上对象。
    nc、smg、sm、ng：字符串，分别表示netconvert、sumo-gui、sumo和SUMO可执行文件的路径。
    lanes、ncols、nrows、leng、timer：道路车道数、网格列数、网格行数、道路长度、定时器。
    gui、process、port、seed、flow：是否启用gui、控制SUMO进程的启动和关闭、与SUMO通信的端口号（存疑）、模拟的随机种子、车辆的流量。
    im：IntersectionManager实例，管理交叉口的状态。
    running_reward、rewards、training_records、i_ep：此次模拟总奖励值、奖励历史记录列表、训练记录列表、当前模拟的迭代次数。
    tripinfo_file、simulation_duration：存储模拟结果的文件名、simulation_duration模拟的持续时间。
    _traci：与SUMO交互的 traci 对象。
    '''
    def __init__(self,
                 sg=None,
                 ss=None,
                 sa=None,
                 gui=False,
                 sm='sumo',
                 smg='sumo-gui',
                 nc='netconvert',
                 traci_folder='D:\Sumo\tools',
                 lanes=2,
                 ncols=4,
                 nrows=1,
                 leng=300,
                 timer=0,
                 nport=65000,
                 seed=0,
                 i_ep=0,
                 flow=100,
                 simulation_duration=5*60):
        self.sg = sg
        self.ss = ss
        self.sa = sa

        self.sgC = True
        self.ssC = True
        self.saC = True

        self.nc = nc
        plt = platform.system()
        if plt == "Windows":
            print("Your system is Windows")
            self.ng = checkBinary('netgenerate.exe')
            self.smg = checkBinary('sumo-gui.exe')
            self.sm = checkBinary('sumo.exe')

        else:
            print("Your system is Linux")
            self.ng = checkBinary('netgenerate')
            self.smg = checkBinary('sumo-gui')
            self.sm = checkBinary('sumo')

        # sys.path.append(traci_folder)
        import traci

        self._traci = traci
        self.lanes = lanes
        self.ncols = ncols
        self.nrows = nrows
        self.leng = leng
        self.timer = timer

        self.gui = gui
        self.process = None
        self.port = nport
        self.seed = seed
        self.flow = flow

        self.im = IntersectionManager('A0', 'pppqw', seed = self.seed) #'pppqw'好像没用
        self.running_reward = -1000
        self.rewards = []
        self.training_records = []
        self.i_ep = i_ep

        self.tripinfo_file = 'results/tripinfo_'
        self.simulation_duration = simulation_duration

    @property
    def traci(self):
        return self._traci

    @property
    def time(self):
        return self._time

    def change_graph(self,sg): # Graph更新与相关结构更新
        self.sg = sg
        self.sgC = True
        self.saC = True

    def change_scenario(self,ss): # Scenario更新与相关结构更新
        self.ss = ss
        self.ssC = True
        self.saC = True

    def change_algorithm(self,sa): # Algorithm更新与相关结构更新
        self.sa = sa
        self.saC = True

    def run_simulation(self): # 模拟函数
        if not (self.sg and self.ss and self.sa): # 是否实例化必要结构
            raise ValueError('Graph,Scenario and Algorithm are needed')

        '''
        运行sumo.exe
        模拟网络和车流文件是其参数
        然后其他地方使用traci.juntion或者traci.vehicle等调用SUMOapi
        '''
        self.init_simulation()
        if self.saC: # 初始化saC判断
            # self.sa.prepare_algorithm(self)
            self.saC = False
        # else:
            # self.sa.reset_algorithm(self)

        TrainingRecord = namedtuple('TrainingRecord', ['ep', 'reward', 'score']) # 训练记录：轮次与奖励

        self.reset_statistics() # 重置数据
        
        self._traci.simulationStep() # 向前一个时间步

        # 纯纯记录用的
        states = []
        actions = []
        collisions = []
        
        # 初始化状态
        states.append(self.im.first_state()) # 状态更新，拿到当前交叉口车辆所有状态
        self.im.control_tls()  # 更改信号灯
        self.im.reset_values()  # 重置值
        self.im.score = 0 # 初始化分数

        

        '''
        不断循环以下过程
        根据当前状态选择动作
        执行动作，更新状态
        计算这些动作的reward
        '''
        while self._traci.simulation.getMinExpectedNumber() > 0: # 模拟直至车辆消耗完 
            if self._traci.simulation.getTime() % 25 == 0: # 隔一段时间打印时间信息
                print(f'Simulation: {self.i_ep}; Time: {self._traci.simulation.getTime()}')

            # Select action based on state
            actions.append(self.im.select_actions()) # 根据状态选择动作，并添加到动作列表

            # Perform actions based on state
            self.im.perform_actions() # 执行动作

            # 这里是报车流碰撞Warning的地方
            # Warning: Vehicle 'right0_left0.25'; junction collision with vehicle 'left0_top0.5', lane=':A0_12_1', gap=-1.00, time=192.75 stage=move.
            self._traci.simulationStep() # 模拟时间步前行

            states.append(self.im.update_state()) # 更新下一步状态
            collisions.append(self.im.obtain_collisions()) # 更新碰撞
            r = self.im.obtain_reward() # 更新奖励
            if r: # 奖励非空，就扩展奖励列表
                self.rewards.append([self._traci.simulation.getTime(), r])
            # states.append(self.im.update_state())
            self.im.swap_states() # 更新当前状态

#            print(self._time)
            self._time = self._time + traci.simulation.getDeltaT() # 更新时间
            # a.append(self.obtain_loss_time())

            if self._traci.simulation.getTime() > 50000: # 如果模拟时间过长，清除待处理事件
                self._traci.simulation.clearPending()

        
        # running_reward=-1000,为了尽可能快的让所有车通行
        score = self.im.score # 拿到分数 
        self.running_reward = self.running_reward * 0.9 + score * 0.1 # 更新运行奖励
        self.training_records.append(TrainingRecord(self.i_ep, self.running_reward, score))

        try:
            if self.i_ep % 20 == 0: # 每隔20步，保存权重
                # self.im.agent.save_checkpoint(str(self.flow))
                self.im.agent.save_weights('ckpt/TD3-LSTM')

            # with open('log/ppo_training_records.pkl', 'wb') as f:
                    # pickle.dump(self.training_records, f)

        except Exception as e:
            print("type error: " + str(e))
            print("run_simulation问题："+ traceback.format_exc())

        finally: # 最终结束模拟
            self.close_simulation()
            return [self.rewards, TrainingRecord(self.i_ep, self.running_reward, score), states, actions, collisions, self.im.agent.Q1loss, self.im.agent.Q2loss, self.im.agent.Aloss, self.im.agent.Q1, self.im.agent.Q2]

    def run_test_simulation(self, weight_path): # 测试模拟
        self.init_simulation()
        self._traci.simulationStep()
        self.im.update_state()
        self.im.control_tls()
        self.im.score = 0  

        try:
            self.im.agent.load_weights(weight_path)
            collisions = []
            while self._traci.simulation.getMinExpectedNumber() > 0:
                if self._traci.simulation.getTime() % 30 == 0:
                    print(f'Simulation: {self.i_ep}; Time: {self._traci.simulation.getTime()}')
                self.im.select_actions()
                self.im.perform_actions()
                self._traci.simulationStep()
                self.im.update_state()
                collisions.append(self.im.obtain_collisions())

                if self._traci.simulation.getTime() > 50000:
                    self._traci.simulation.clearPending()

        except Exception as e:
            print("type error: " + str(e))
            print("run_test_simulation问题:" + traceback.format_exc())

        finally:
            self.close_simulation()
            return np.sum(collisions)



    def obtain_loss_time(self): # 时间损失，由于迟到而造成的负奖励上升
        try:
            junctionID = 'A0'
            dist = 200.0

            # https://sumo.dlr.de/pydoc/traci.constants.html
            traci.junction.subscribeContext(junctionID, tc.CMD_GET_VEHICLE_VARIABLE, dist, [tc.VAR_SPEED, tc.VAR_ALLOWED_SPEED, tc.VAR_ROAD_ID, tc.VAR_DISTANCE])

            # https://sumo.dlr.de/daily/pydoc/traci._vehicle.html
            # traci.vehicle.subscribeContext(str(veh_id), tc.CMD_GET_VEHICLE_VARIABLE, 0.0, [tc.VAR_SPEED])
            # traci.junction.addSubscriptionFilterLanes([-1,0,1], noOpposite=True, downstreamDist=100, upstreamDist=50)

            # traci.vehicle.addSubscriptionFilterDownstreamDistance(50.0)
            # traci.vehicle.addSubscriptionFilterLanes(lanes, noOpposite=True, downstreamDist=100, upstreamDist=50)

            # Eliminar los vehículos que se alejan de la intersección, en función de su ruta?

            stepLength = traci.simulation.getDeltaT()
            scResults = traci.junction.getContextSubscriptionResults(junctionID)
            halting = 0

            a = tc.VAR_SPEED # Current speed
            b = tc.VAR_ALLOWED_SPEED # Maximum speed
            timeLoss = 0
            print(f'\n Found {len(scResults)} vehicles before filtering')
            print(f'\t Keys: {scResults.keys()}')

            scResults = self._remove_moving_away(junctionID, scResults) # If vehicles are approaching the intersection

            if scResults:
                print(f' After filtering {len(scResults)} vehicles')
                print(f'\t Keys: {scResults.keys()}')
                print(f'{scResults}')
                relSpeeds = [d[a] / d[b] for d in scResults.values()]
                # compute values corresponding to summary-output
                running = len(relSpeeds)
                halting = len([1 for d in scResults.values() if d[tc.VAR_SPEED] < 0.1]) # number of vehicles waiting
                meanSpeedRelative = np.mean(relSpeeds)
                # Due to that the vehicles are under the maximum speed,
                # the loss time in the last simulationStep is:
                timeLoss = (1 - meanSpeedRelative) * running * stepLength

            # print(f'Simulation time: {traci.simulation.getTime()}; Timeloss: {timeLoss}; Halting: {halting}')
            return timeLoss
        except Exception as e:
            print("type error: " + str(e))
            print("obtain_time_loss的错误："+traceback.format_exc())

    def _remove_moving_away(self, j, v):
        return {key:val for key, val in v.items() if (val[tc.VAR_ROAD_ID][-2:] == j) or (val[tc.VAR_ROAD_ID][:1+len(j)] == f':{j}')} # If vehicles are approaching the intersection

#
    def getTripinfo(self): # 从tripinfo中取得行程信息
        total_trips = 0
        total_timeloss = 0
        total_duration = 0
        total_wtime = 0

        average_relative_timeloss = 0
        average_duration = 0
        average_timeloss = 0
        average_wtime = 0
        max_timeloss = 0

        try:
            with open('results/tripinfo_.xml') as f:
                content = f.readlines()
            for line in content:
                if "<tripinfo id=" in line:
                    total_trips += 1
                    xml_string = "<tripinfos>"
                    xml_string = "".join((xml_string, line))
                    xml_string = "/n".join((xml_string, "</tripinfos>"))
                    xml_string = xml_string.replace('>\n', '/>\n')
                    open_data = minidom.parseString(xml_string)
                    intervals_open = open_data.getElementsByTagName('tripinfo')
                    timeloss = float(intervals_open[0].getAttribute('timeLoss'))
                    duration = float(intervals_open[0].getAttribute('duration'))
                    wtime = float(intervals_open[0].getAttribute('waitingTime'))
                    max_timeloss = np.maximum(max_timeloss, timeloss)

                    total_timeloss += timeloss
                    total_duration += duration
                    total_wtime += wtime

                    relative_timeloss = timeloss / duration
                    average_relative_timeloss = ((average_relative_timeloss * (
                            total_trips - 1) + relative_timeloss) / total_trips)

            average_duration = total_duration / total_trips
            average_timeloss = total_timeloss / total_trips
            average_wtime = total_wtime / total_trips

        except Exception as e:
            print("type error: " + str(e))
            print("getTripinfo的错误："+ traceback.format_exc())

        finally:
            return [total_trips, total_timeloss, total_duration, total_wtime,
                    average_relative_timeloss, average_duration, average_wtime,
                    average_timeloss, max_timeloss]
        return self.co2em
    def reset_statistics(self):
        self._time = 0
        self.cars = {}
        self.wt = {}
        self.co2em = {} # 每条边的co2排放
        for edge in self.sg.iter_edges():
            self.co2em[edge.id] = [0,0,0,0,0,0,0]
        self.num_veh = 0
        self.num_ped = 0
        self.tov = []
        self.df = np.empty([0,6])
        self.y = []
    def _get_statistics(self):



        for edge in self.sg.iter_edges():
#        if edge.id in self.co2em:
            self.co2em[edge.id][0] += self._traci.edge.getCO2Emission(edge.id)
#            self.co2em[edge.id][1] += self._traci.edge.getCOEmission(edge.id)
#            self.co2em[edge.id][2] += self._traci.edge.getHCEmission(edge.id)
#            self.co2em[edge.id][3] += self._traci.edge.getPMxEmission(edge.id)
#            self.co2em[edge.id][4] += self._traci.edge.getNOxEmission(edge.id)
#            self.co2em[edge.id][5] += self._traci.edge.getFuelConsumption(edge.id)
#            self.co2em[edge.id][6] += self._traci.edge.getNoiseEmission(edge.id)

#        else:
#            self.co2em[edge.id] = self._traci.edge.getCO2Emission(edge.id)

#        # Estados de los semáforos en cada instante
#        tlsID = self._traci.trafficlight.getIDList()
#        y_NS = self._traci.trafficlight.getRedYellowGreenState(tlsID[0])
#        if y_NS[1] == 'G':
#            self.y.append(1)
#        else:
#            self.y.append(0)


    def init_simulation(self): # 初始化模拟

        # *Deprecated* =======================================================
        # self.__create_files()
        # subprocess.call([self.nc,
        #                   '-n=sumodata/nodes_'+str(self.port)+'.xml',
        #                   '-e=sumodata/edges_'+str(self.port)+'.xml',
        #                   '-o=sumodata/net_'+str(self.port)+'.xml',
        #                   '-L='+str(self.lanes),
        #                   '--no-left-connections=True',
        #                   '--no-turnarounds=True',
        #                   '--walkingareas=True',
        #                   '--offset.disable-normalization=True',
        #                   '--no-internal-links=False',
        #                   '--junctions.corner-detail=5',
        #                   '--junctions.limit-turn-speed=5.5',
        #                   '--rectangular-lane-cut=False',
        #                   '--sidewalks.guess=True',
        #                   '--crossings.guess=True',
        #                   '--default.junctions.radius=10',
        #                   '--default.junctions.keep-clear=True',
        #                   '--default.sidewalk-width=6',
        #                   '--default.crossing-width=6'
        #                   ])
        # ====================================================================

        # print('Creating network:')
        command = [self.ng, '-g',
              '--grid.x-number', str(self.ncols),
              '--grid.y-number', str(self.nrows),
              '--grid.length', str(self.leng),
              '--grid.attach-length', str(self.leng),
              '-L', str(self.lanes),
              '--default.sidewalk-width', '3.0',
              '--sidewalks.guess', 'true',
              '--crossings.guess', 'true',
              '--walkingareas', 'true',
              '--bikelanes.guess', 'true',
              '--verbose', 'true',
              '--tls.guess', 'true',
              '--seed', str(self.seed),
              '-o', 'sumodata/net_'+ str(self.nrows)+'_'+str(self.ncols)+'.xml']
        '''
        拿到进行路网生成的参数，包括行列数、长度。。
        '''
        # p = subprocess.run(command, check=True, stdout=subprocess.PIPE, universal_newlines=True)
        # print(p.stdout)
        # print('Done!')
        self.create_route_files_v2() # 创建路网文件
        # =============================================================================
        #         generate the pedestrians for this simulation

        #         subprocess.call([self.nc,'-n=sumodata/nodes.xml','-e=sumodata/edges.xml','-o=sumodata/net.xml','--no-left-connections=True','--no-turnarounds.except-deadend=True'])
        # =============================================================================
        # -L <int>
        #         The default number of lanes in an edge; default: 1

        # --no-turnarounds.except-deadend <BOOL>
        #       Disables building turnarounds except at dead end junctions; default: false
        #         Hay problemas con los semáforos cuando introducimos este parámetro

        # --no-left-connections <BOOL>
        #       Disables building connections to left; default: false
        #         Hay problemas con los semáforos cuando introducimos este parámetro
        # =============================================================================
        #         import time
        #         time = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
        
        '''
        等效于
        >>sumo.exe -n=sumodata/net_......
        或者
        >>sumo-gui.exe -n=sumodata/net_......
        这里用到了sumodata的net_1_1.xml和veh_routes_1_1.xml
        其中veh_routes_1_1.xml还是上一行代码self.create_route_files_v2()创建的
        '''
        if not self.gui: # 创建一个新线程执行sumo cmd或者sumo gui
            self.process = subprocess.Popen([self.sm,
                                             '-n=sumodata/net_'+ str(self.nrows)+'_'+str(self.ncols)+'.xml',
                                              '-r=sumodata/veh_routes_'+str(self.nrows)+'_'+str(self.ncols)+'.xml',
                                              '-X=never',
                                              '--seed=' + str(self.seed),
                                              '--junction-taz',
                                              '--step-length=0.25',
                                              '--remote-port='+str(self.port),
                                              '--tripinfo-output=results/tripinfo_.xml',
                                             '--tripinfo-output.write-unfinished=True',
                                             '--device.emissions.probability=1',
                                             '--waiting-time-memory=1000',
                                             '--collision.check-junctions=true',
                                             '--collision.action=warn'])
#    '--emission-output=results/emission_'+str(self.timer)+'.xml'
        else:
            # ' '.join([self.smg,
            #           '-n=sumodata/net.xml',
            #           '-r=sumodata/routes.xml',
            # '-a=sumodata/peds_cycl_routes_'+str(self.nrows)+'_'+str(self.ncols)+'.xml, sumodata/induction_loops.add.xml',

            #           '--random',
            #           '--remote-port='+str(self.port),
            #           '--tripinfo-output=results/tripinfo.xml'])
            self.process = subprocess.Popen([self.smg,
                                              '-n=sumodata/net_'+ str(self.nrows)+'_'+str(self.ncols)+'.xml',
                                              '-r=sumodata/veh_routes_'+str(self.nrows)+'_'+str(self.ncols)+'.xml',
                                              '-X=never',
                                              '--seed=' + str(self.seed),
                                              '--junction-taz',
                                              '--step-length=0.25',
                                              '--remote-port='+str(self.port),
                                              '--tripinfo-output=results/tripinfo_.xml',
                                             '--tripinfo-output.write-unfinished=True',
                                             '--device.emissions.probability=1',
                                             '--waiting-time-memory=1000',
                                             '--collision.check-junctions=true',
                                             '--collision.action=warn'])

            # self.process = subprocess.Popen([self.smg,
            #                                  '-n=sumodata/net_'+str(self.port)+'.xml',
            #                                   '-r=sumodata/trips.trips.xml',
            #                                   '-a=sumodata/add.xml',
            #                                   '-X=never',
            #                                   '--remote-port='+str(self.port)])

        self._traci.init(self.port) # 初始化traci控制器与sumo进程交互

    def close_simulation(self): # 关闭模拟，杀死进程
        """Close the simulation."""
        self._traci.close()
        self.process.terminate()
        self.process.kill()

    def create_route_files_v2(self): # 车辆路线文件生成
        # print('Creating routes')
        self.process = subprocess.Popen([self.sm,
                                 '-n=sumodata/net_'+ str(self.nrows)+'_'+str(self.ncols)+'.xml',
                                 '--remote-port='+str(9999),
                                 ]) # 用一个子进程执行路网文件的加载
        self._traci.init(9999) # 初始化traci和这个进程的通信

        junctions = traci.junction.getIDList() # 获取交叉路口列表
        borders = [] # 拿到边界路口
        for j in junctions:
            if str(j).startswith(('top', 'bottom', 'left', 'right')):
                borders.append(j)

        self.__create_vehicles_route_file(borders) # 创建路线
        # edges = traci.edge.getIDList()
        # borders = []
        # for e in edges:
        #     if not str(e).startswith(':'):
        #         borders.append(e)
        # self.__create_peds_route_file()
        self._traci.close() # 关闭连接
        # print('Routes created')

    def __create_vehicles_route_file(self, borders):
        with open('sumodata/veh_routes_' + str(self.nrows) + '_' +\
                  str(self.ncols) + '.xml', 'w') as r: # 打开路线文件
            r.write('<routes>\n') # 起始标记
            '''
            vType部分
            '''
            for ct in self.ss.iter_car_types(): # 遍历车辆
                r.write(repr(ct)) # 车辆的定义类型

            # dic = {'N': 'top',
            #        'S': 'bottom',
            #        'E': 'right',
            #        'W': 'left'}

            # for fl in self.ss.iter_flows():
            #     if fl['type'] == 'typedist1': # Solo vehículos, los peatones van aparte
            #         ide = fl.id
            #         o = dic[ide[4]]
            #         d = dic[ide[5]]
            #         fl['route'] = None
            #         fl['fromJunction'] = o
            #         fl['toJunction'] = d
            #         r.write(repr(fl))

            # ================================================================
            # probability:float([0,1]). Probability for emitting a vehicle each
            # second (not together with personsPerHour or period),
            # When this attribute is used, a vehicle will be emitted randomly
            # with the given probability each second. This results in a
            # binomially distributed flow (which approximates a Poisson
            # Distribution for small probabilities).
            # ================================================================
            # Si prob=1 entonces el flujo que se está simulando es 3600veh/h

            duration = self.simulation_duration #5*60 模拟持续时间
            # dens = [200,300,400,500,600,700,800]
            dens = self.flow  # veh/h/route 车流密度

            if dens > 3600: # 不得超过3600
                print('WARNING: la densidad no puede ser mayor de 3600, set:3600')
                dens = 3600

            '''
            flow部分
            '''
            prob = dens/3600  # (veh/h)/(veh/h) se queda un ratio adimensional
            for o in borders:
                for d in borders:
                    if not o == d:
                        fid = o+'_'+d # 编码路线
                        r.write('\t<flow id="' + fid +
                                '" begin="' + str(0) +
                                '" end="' + str(duration) +
                                '" probability="' + str(prob) +
                                '" type="car_gasoline" departSpeed="max"' +
                                ' fromJunction="' + o +
                                '" toJunction="' + d + '" />\n') # 写入路线信息
            r.write('</routes>') # 结束符号

    # 没用到
    # randomTrips.py的调用，生成net_1_1.xml和peds_cycl_routes_1_1.xml
    def __create_peds_route_file(self):
        command = ['api_sumo/randomTrips.py',
                   '-n', 'sumodata/net_'+ str(self.nrows)+'_'+str(self.ncols)+'.xml',
                   '-o', 'sumodata/peds_cycl_routes_' + str(self.nrows) +
                         '_' + str(self.ncols) + '.xml',
                   '--persontrips',
                   '--trip-attributes', 'modes="bicycle"',
                   '-e', '1',
                   '--seed', str(self.seed),
                   '--verbose', 'True']

        process = subprocess.run(command, check=True, stdout=subprocess.PIPE,
                                 universal_newlines=True)
        output = process.stdout
        print(output)


