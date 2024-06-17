import os
import subprocess
import platform
import traceback

import numpy as np

import traci
import traci.constants as tc

from IntersectionManager import IntersectionManager
from sumolib import checkBinary
from collections import defaultdict, namedtuple


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
                 ss=None,
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
                 simulation_duration=5*60,
                 weight_path='ckpt',
                 policy_noise=True,
                 cf=False,
                 model_name='Test',
                 agent='TD3',
                 map='Default',
                 tl = False):
        self.ss = ss

        self.sgC = True
        self.ssC = True
        self.saC = True

        self.nc = nc
        plt = platform.system()
        if plt == "Windows":
            print("Your system is Windows")
            self.nc = checkBinary('netconvert')
            self.pc = checkBinary('polyconvert')
            self.ng = checkBinary('netgenerate.exe')
            self.smg = checkBinary('sumo-gui.exe')
            self.sm = checkBinary('sumo.exe')

        else:
            print("Your system is Linux")
            self.nc = checkBinary('netconvert')
            self.pc = checkBinary('polyconvert')
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

        self.policy_noise = policy_noise
        self.cf = cf
        self.model_name = model_name
        self.agent = agent
        self.tl = tl

        self.running_reward = -1000
        self.rewards = []
        self.training_records = []
        self.i_ep = i_ep

        self.tripinfo_file = 'results/tripinfo_'
        self.simulation_duration = simulation_duration
        self.weight_path = weight_path

        self.junction_id_list = ['A0']

        self.im = IntersectionManager(self.junction_id_list[-1], 'pppqw',
                                      seed=self.seed, policy_noise=self.policy_noise, cf=cf, model_name=self.model_name, agent=agent)
        
        self.map = map

        self.map_file = os.path.join('sumodata', self.map+'.osm')
        self.poly_file = os.path.join('sumodata', self.map+'.poly.xml')

        if self.map != 'Default':
            self.net_file = os.path.join('sumodata', self.map+'.net.xml')
            self.rou_file = os.path.join('sumodata', self.map+'.rou.xml')
        else:
            self.net_file = 'sumodata/net_'+ str(self.nrows)+'_'+str(self.ncols)+'.xml'
            self.rou_file = 'sumodata/veh_routes_'+str(self.nrows)+'_'+str(self.ncols)+'.xml'

    @property
    def traci(self):
        return self._traci

    @property
    def time(self):
        return self._time

    def change_agent(self, agent, cf=False):
        self.im = IntersectionManager(self.junction_id_list[-1], 'pppqw',
                                      seed=self.seed, policy_noise=self.policy_noise, cf=cf, model_name=self.model_name, agent=agent)

    def change_maxSpeed(self, speed=30):
        self.im.change_maxSpeed(speed)

    def change_graph(self, sg):  # Graph更新与相关结构更新
        self.sg = sg
        self.sgC = True
        self.saC = True

    def change_scenario(self, ss):  # Scenario更新与相关结构更新
        self.ss = ss
        self.ssC = True
        self.saC = True

    def run_simulation(self):  # 模拟函数
        if not (self.sg and self.ss):  # 是否实例化必要结构
            raise ValueError('Graph,Scenario and Algorithm are needed')

        '''
        运行sumo.exe
        模拟网络和车流文件是其参数
        然后其他地方使用traci.juntion或者traci.vehicle等调用SUMOapi
        '''
        self.init_simulation()

        TrainingRecord = namedtuple(
            'TrainingRecord', ['ep', 'reward', 'score'])  # 训练记录：轮次与奖励

        self.reset_statistics()  # 重置数据

        self._traci.simulationStep()  # 向前一个时间步

        # 纯纯记录用的
        states = []
        actions = []
        collisions = []

        # 初始化状态
        states.append(self.im.first_state())  # 状态更新，拿到当前交叉口车辆所有状态
        self.im.control_tls()  # 更改信号灯
        self.im.reset_values()  # 重置值
        self.im.score = 0  # 初始化分数

        '''
        不断循环以下过程
        根据当前状态选择动作
        执行动作，更新状态
        计算这些动作的reward
        '''
        while self._traci.simulation.getMinExpectedNumber() > 0:  # 模拟直至车辆消耗完
            if self._traci.simulation.getTime() % 25 == 0:  # 隔一段时间打印时间信息
                print(
                    f'Simulation: {self.i_ep}; Time: {self._traci.simulation.getTime()}')

            actions.append(self.im.select_actions())  # 根据状态选择动作，并添加到动作列表

            self.im.perform_actions()  # 执行动作

            # 这里是报车流碰撞Warning的地方
            # Warning: Vehicle 'right0_left0.25'; junction collision with vehicle 'left0_top0.5', lane=':A0_12_1', gap=-1.00, time=192.75 stage=move.
            self._traci.simulationStep()  # 模拟时间步前行

            states.append(self.im.update_state())  # 更新下一步状态
            collisions.append(self.im.obtain_collisions())  # 更新碰撞
            r = self.im.obtain_reward()  # 更新奖励
            if r:  # 奖励非空，就扩展奖励列表
                self.rewards.append([self._traci.simulation.getTime(), r])
            self.im.swap_states()  # 更新当前状态

            self._time = self._time + traci.simulation.getDeltaT()  # 更新时间

            if self._traci.simulation.getTime() > 50000:  # 如果模拟时间过长，清除待处理事件
                self._traci.simulation.clearPending()

        # running_reward=-1000是为了尽可能快的让所有车通行
        score = self.im.score  # 拿到分数
        self.running_reward = self.running_reward * 0.9 + score * 0.1  # 更新运行奖励
        self.training_records.append(TrainingRecord(
            self.i_ep, self.running_reward, score))

        try:
            if self.i_ep % 20 == 0:  # 每隔20步，保存权重
                self.im.agent.save_weights(self.weight_path)

        except Exception as e:
            print("type error: " + str(e))
            print("run_simulation问题：" + traceback.format_exc())

        finally:  # 最终结束模拟
            self.close_simulation()
            return [self.rewards, TrainingRecord(self.i_ep, self.running_reward, score), states, actions, collisions, self.im.agent.Q1loss, self.im.agent.Q2loss, self.im.agent.Aloss, self.im.agent.Q1, self.im.agent.Q2]

    def run_test_simulation(self, weight_path='ckpt', is_agent=True):  # 测试模拟
        self.init_simulation()
        self.reset_statistics()
        self._traci.simulationStep()
        states = []
        actions = []
        collisions = []

        states.append(self.im.first_state())  # 状态更新，拿到当前交叉口车辆所有状态
        if self.tl == False:
            if self.map=='Default':
                self.im.control_tls()  # 更改信号灯
        self.im.reset_values()  # 重置值
        self.im.score = 0  # 初始化分数

        try:
            self.im.agent.load_weights(weight_path)
            while self._traci.simulation.getMinExpectedNumber() > 0:
                if self._traci.simulation.getTime() % 30 == 0:
                    print(
                        f'Simulation: {self.i_ep}; Time: {self._traci.simulation.getTime()}')
                actions.append(self.im.select_actions())
                self.im.perform_actions()
                self._traci.simulationStep()
                self.im.update_state()
                collisions.append(self.im.obtain_collisions())
                self.im.obtain_reward()
                if is_agent:
                    self.im.swap_states()

                if self._traci.simulation.getTime() > 50000:
                    self._traci.simulation.clearPending()

        except Exception as e:
            print("type error: " + str(e))
            print("run_test_simulation问题:" + traceback.format_exc())

        finally:
            self.close_simulation()
            return np.sum(collisions)

    def obtain_loss_time(self):  # 时间损失，由于迟到而造成的负奖励上升
        try:
            junctionID = 'A0'
            dist = 200.0

            traci.junction.subscribeContext(junctionID, tc.CMD_GET_VEHICLE_VARIABLE, dist, [
                                            tc.VAR_SPEED, tc.VAR_ALLOWED_SPEED, tc.VAR_ROAD_ID, tc.VAR_DISTANCE])

            stepLength = traci.simulation.getDeltaT()
            scResults = traci.junction.getContextSubscriptionResults(
                junctionID)

            a = tc.VAR_SPEED  # 当前速度
            b = tc.VAR_ALLOWED_SPEED  # 最大限速
            timeLoss = 0
            print(f'\n Found {len(scResults)} vehicles before filtering')
            print(f'\t Keys: {scResults.keys()}')

            # 车辆接近十字路口
            scResults = self._remove_moving_away(junctionID, scResults)

            if scResults:
                print(f' After filtering {len(scResults)} vehicles')
                print(f'\t Keys: {scResults.keys()}')
                print(f'{scResults}')
                relSpeeds = [d[a] / d[b] for d in scResults.values()]
                running = len(relSpeeds)
                meanSpeedRelative = np.mean(relSpeeds)
                timeLoss = (1 - meanSpeedRelative) * running * stepLength
            return timeLoss
        except Exception as e:
            print("type error: " + str(e))
            print("obtain_time_loss的错误："+traceback.format_exc())

    def _remove_moving_away(self, j, v):
        return {key: val for key, val in v.items() if (val[tc.VAR_ROAD_ID][-2:] == j) or (val[tc.VAR_ROAD_ID][:1+len(j)] == f':{j}')}

#
    def getTripinfo(self):  # 从tripinfo中取得行程信息
        total_trips = 0

        total_timeloss = 0
        total_duration = 0
        total_wtime = 0

        avg_relative_timeloss = 0
        avg_duration = 0
        avg_timeloss = 0
        avg_wtime = 0
        max_timeloss = 0

        total_CO_abs = 0
        total_CO2_abs = 0
        total_HC_abs = 0
        total_PMx_abs = 0
        total_NOx_abs = 0
        total_fuel_abs = 0

        try:
            import xml.dom.minidom
            # 读取XML文件内容
            with open('results/tripinfo_.xml', 'r') as file:
                xml_content = file.read()

            # 检查并添加缺失的关闭标签
            if not xml_content.strip().endswith('</tripinfos>'):
                xml_content += '</tripinfos>'
            # 解析XML文件
            dom = xml.dom.minidom.parseString(xml_content)
            tripinfos = dom.getElementsByTagName('tripinfo')

            for tripinfo in tripinfos:
                duration = float(tripinfo.getAttribute('duration'))
                timeloss = float(tripinfo.getAttribute('timeLoss'))
                waitingtime = float(tripinfo.getAttribute('waitingTime'))

                emissions = tripinfo.getElementsByTagName('emissions')[0]
                CO_abs = float(emissions.getAttribute('CO_abs'))
                CO2_abs = float(emissions.getAttribute('CO2_abs'))
                HC_abs = float(emissions.getAttribute('HC_abs'))
                PMx_abs = float(emissions.getAttribute('PMx_abs'))
                NOx_abs = float(emissions.getAttribute('NOx_abs'))
                fuel_abs = float(emissions.getAttribute('fuel_abs'))

                # 累加值
                total_duration += duration
                total_timeloss += timeloss
                total_wtime += waitingtime
                total_CO_abs += CO_abs
                total_CO2_abs += CO2_abs
                total_HC_abs += HC_abs
                total_PMx_abs += PMx_abs
                total_NOx_abs += NOx_abs
                total_fuel_abs += fuel_abs
                total_trips += 1

                max_timeloss = np.maximum(max_timeloss, timeloss)

                relative_timeloss = timeloss / duration
                avg_relative_timeloss = ((avg_relative_timeloss * (
                    total_trips - 1) + relative_timeloss) / total_trips)

            # 计算平均值
            avg_duration = total_duration / total_trips
            avg_timeloss = total_timeloss / total_trips
            avg_wtime = total_wtime / total_trips

        except Exception as e:
            print("type error: " + str(e))
            print("getTripinfo的错误：" + traceback.format_exc())

        finally:
            return [total_trips, total_timeloss, total_duration, total_wtime,
                    avg_relative_timeloss, avg_duration, avg_wtime,
                    avg_timeloss, max_timeloss, total_CO_abs / total_trips,
                    total_CO2_abs / total_trips, total_HC_abs / total_trips, total_PMx_abs / total_trips, total_NOx_abs / total_trips, total_fuel_abs / total_trips]

    def reset_statistics(self):
        self._time = 0
        self.cars = {}
        self.wt = {}
        self.co2em = {}  # 每条边的co2排放
        for edge in self.sg.iter_edges():
            self.co2em[edge.id] = [0, 0, 0, 0, 0, 0, 0]
        self.num_veh = 0
        self.num_ped = 0
        self.tov = []
        self.df = np.empty([0, 6])
        self.y = []

    def init_simulation(self):  # 初始化模拟
        if self.map == 'Default':
            self.create_route_files_v2()
        else:
            self.create_net_rou_poly_files()
        '''
        等效于
        >>sumo.exe -n=sumodata/net_......
        或者
        >>sumo-gui.exe -n=sumodata/net_......
        这里用到了sumodata的net_1_1.xml和veh_routes_1_1.xml
        其中veh_routes_1_1.xml还是上一行代码self.create_route_files_v2()创建的
        '''
        if not self.gui:  # 创建一个新线程执行sumo cmd或者sumo gui
            self.process = subprocess.Popen([self.sm,
                                             '-n=' + self.net_file,
                                             '-r=' + self.rou_file,
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
                                             '--collision.action=warn'] + (['-a=' + self.poly_file] if self.map != 'Default' else []))
        else:
            self.process = subprocess.Popen([self.smg,
                                             '-n=' + self.net_file,
                                             '-r=' + self.rou_file,
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
                                             '--collision.action=warn'] + (['-a=' + self.poly_file] if self.map != 'Default' else []))
        
        self._traci.init(self.port)  # 初始化traci控制器与sumo进程交互

    def close_simulation(self):  # 关闭模拟，杀死进程
        """Close the simulation."""
        self._traci.close()
        self.process.terminate()
        self.process.kill()

    def create_net_rou_poly_files(self):
        # 根据osm生成路网文件
        result_nc = subprocess.run([self.nc, 
                                    '--osm-files', self.map_file, 
                                    '--junctions.join', 'true', 
                                    '--junctions.join-dist', '50', 
                                    '-o', self.net_file,
                                    '--keep-edges.by-type', 'highway.primary,highway.primary_link,highway.secondary,highway.secondary_link,highway.tertiary,highway.tertiary_link',
                                    '--tls.discard-loaded', 'true'], check=True, stdout=subprocess.PIPE, universal_newlines=True)
        print(result_nc.stdout)  # 输出运行结果

        # 根据osm和路网生成poly文件
        result_pc = subprocess.run([self.pc, 
                                    '--net-file', self.net_file, 
                                    '--osm-files', self.map_file,
                                    '-o', self.poly_file], check=True, stdout=subprocess.PIPE, universal_newlines=True)
        print(result_pc.stdout)  # 输出运行结果
        
        self.__create_peds_route_file()

        junction, _ = self.get_junction()

        self.junction_id_list = junction

        print(f"Junciton ID: {self.junction_id_list[-1]}")

        self.im._id = self.junction_id_list[-1]

    def get_junction(self):
        self.process = subprocess.Popen([self.sm,
                                 '-n=' + self.net_file,
                                 '--remote-port='+str(9999),
                                 ])

        self._traci.init(9999)  # 初始化traci和这个进程的通信

        junctions = traci.junction.getIDList()  # 获取交叉路口列表

        borders = []  # 拿到边界路口
        for j in junctions:
            if str(j).startswith(('top', 'bottom', 'left', 'right')):
                borders.append(j)

        self.close_simulation()

        return junctions, borders
    
    # randomTrips.py的调用，生成route文件
    def __create_peds_route_file(self):
        command = ['python',
                   'api_sumo/randomTrips.py',
                   '-n', self.net_file,
                   '-r', self.rou_file,
                   '-o', 'sumodata/trips.trips.xml',
                   '-e', '1000']

        process = subprocess.run(command, check=True, stdout=subprocess.PIPE, universal_newlines=True)
        output = process.stdout
        print(output)

    def create_route_files_v2(self):  # 车辆路线文件生成
        print('Creating routes')
        _, borders = self.get_junction()
        self.__create_vehicles_route_file(borders)  # 创建路线
        print('Routes created')

    def __create_vehicles_route_file(self, borders):
        with open(self.rou_file, 'w') as r:  # 打开路线文件
            r.write('<routes>\n')  # 起始标记
            '''
            vType部分
            '''
            for ct in self.ss.iter_car_types():  # 遍历车辆
                r.write(repr(ct))  # 车辆的定义类型

            duration = self.simulation_duration  # 5*60 模拟持续时间
            # dens = [200,300,400,500,600,700,800]
            dens = self.flow  # veh/h/route 车流密度

            if dens > 3600:  # 不得超过3600
                print('WARNING: la densidad no puede ser mayor de 3600, set:3600')
                dens = 3600

            '''
            flow部分
            '''
            prob = dens/3600  # (veh/h)/(veh/h) se queda un ratio adimensional
            for o in borders:
                for d in borders:
                    if not o == d:
                        fid = o+'_'+d  # 编码路线
                        r.write('\t<flow id="' + fid +
                                '" begin="' + str(0) +
                                '" end="' + str(duration) +
                                '" probability="' + str(prob) +
                                '" type="car_gasoline" departSpeed="max"' +
                                ' fromJunction="' + o +
                                '" toJunction="' + d + '" />\n')  # 写入路线信息
            r.write('</routes>')  # 结束符号
