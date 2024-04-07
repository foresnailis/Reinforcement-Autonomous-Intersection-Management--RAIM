#!/usr/bin/env python
"""

"""

from collections import defaultdict

from api_sumo import SumoAlgorithm

__author__ = "Bryan Alexis Freire Viteri"
__version__ = "3.0"
__email__ = "bryanfv95@gmail.com"

class ManhattanAlgorithm(SumoAlgorithm):
    """
    ManhattanAlgorithm 类用于定义基于曼哈顿交通网络的交通信号灯控制算法。

    Attributes:
        wq (float): 车辆等待时间权重。
        lanes (int): 交通道数。
        pgm (list): 交通信号灯控制程序的列表，包含了各个阶段的信号状态。
        keypoints (list): 关键时刻的列表，用于触发特定操作。
        waittimes (list): 每个交通灯处的车辆等待时间。
        queues (list): 每个交通灯处的车辆排队长度。
        lastemptyqueue (list): 每个交通灯处最后一次排队为空的时间。

    Methods:
        reset_algorithm(sm): 重置算法状态，重新初始化与 SUMO 仿真环境的连接。
        prepare_algorithm(sm): 初始化算法，设置与 SUMO 仿真环境的连接。
        _when_NS(index): 当纵向流量触发时的操作。
        _when_NS_pre(index): 在纵向流量之前的操作。
        _when_WE(index): 当横向流量触发时的操作。
        _when_WE_pre(index): 在横向流量之前的操作。
        _when(pointer, index): 在特定时刻触发的操作。

    Raises:
        NotImplementedError: 如果子类未实现某些方法，则会引发 NotImplementedError 异常。
    """
    def __init__(self,wq,lanes):
        """
        初始化 ManhattanAlgorithm 类的实例。

        Args:
            wq (float): 车辆等待时间权重。
            lanes (int): 交通道数。
        """
        self.lanes = lanes
        n = self.lanes
        NSGREEN = ("G"+"G"*n+"r"+"r"*n)*(2) + 'rrrr'
        NSYELLOW = ("y"+"y"*n+"r"+"r"*n)*(2) + 'ryry'
        WEGREEN = ("r"+"r"*n+"G"+"G"*n)*(2) + 'rrrr'
        WEYELLOW = ("r"+"r"*n+"y"+"y"*n)*(2) + 'yryr'
        CLEAR = ("r"+"r"*n+"r"+"r"*n)*(2) + 'rrrr'

        # Se repiten los estado en yellow tres veces, ya que son el número de
        # segundos que el semáforo está en amarillo

#        Se repiten el estado de clear dos veces ya que se dan dos segundos para
#        que se limpie la intersección

#        program   = [WEGREEN,WEYELLOW, WEYELLOW, WEYELLOW,CLEAR,CLEAR, NSGREEN, NSYELLOW, NSYELLOW, NSYELLOW,CLEAR,CLEAR]
        program = [WEGREEN,
                   WEYELLOW,
                   WEYELLOW,
                   WEYELLOW,
                   CLEAR,
                   CLEAR,
                   NSGREEN,
                   NSYELLOW,
                   NSYELLOW,
                   NSYELLOW,
                   CLEAR,
                   CLEAR,
                   CLEAR]

#        super(ManhattanAlgorithm,self).__init__(program,[0,6])
        super(ManhattanAlgorithm,self).__init__(program,[0,6])

        self.wq = wq

#    def get_statistics(self):
#        return  sum(map(sum,self.waittimes))/(self.sm.num_veh),\
#                sum(map(sum,self.waittimes_ped))/(self.sm.num_ped),\
#                sum(map(sum,self.queues))/(4*len(self.queues))

    def reset_algorithm(self,sm):
        """
        重置算法状态，重新初始化与 SUMO 仿真环境的连接。

        Args:
            sm (SUMOManager): 与 SUMO 仿真环境的连接管理器。
        """
        super(ManhattanAlgorithm,self).reset_algorithm(sm)
        for index in range(len(self.ids)):
            for index2 in range(4):
                self.waittimes[index][index2] = 0

                self.queues[index][index2] = 0
                self.lastemptyqueue[index][index2] = 0

    def prepare_algorithm(self,sm):
        """
        初始化算法，设置与 SUMO 仿真环境的连接。

        Args:
            sm (SUMOManager): 与 SUMO 仿真环境的连接管理器。
        """
        super(ManhattanAlgorithm,self).prepare_algorithm(sm)
        self.waittimes = [[0]*4 for i in range(len(self.ids))]
        self.queues = [[0]*4 for i in range(len(self.ids))]
        self.lastemptyqueue = [[0]*4 for i in range(len(self.ids))]

    def _when_NS(self,index):
        raise NotImplementedError('when_NS is not implemented')

    def _when_NS_pre(self,index):
        raise NotImplementedError('when_NS is not implemented')

    def _when_WE(self,index):
        raise NotImplementedError('when_WE is not implemented')

    def _when_WE_pre(self,index):
        raise NotImplementedError('when_WE is not implemented')

    def _when(self,pointer,index):
        """
        在特定时刻触发的操作。

        Args:
            pointer (int): 当前程序指针的位置。
            index (int): 信号灯的索引。
        """
        # 根据信号灯索引获取其位置信息
        i,j = list(map(int,self.ids[index].split('.')))
        fn = '{}.{}/{}.{}'.format
        fp = ':{}.{}_w{}'.format

        # 获取与当前信号灯相邻的东西南北四个方向的位置信息 
        n = fn(i-1,j,i,j)
        s = fn(i+1,j,i,j)
        w = fn(i,j-1,i,j)
        e = fn(i,j+1,i,j)

#        ret = []
        if pointer == 0:
            # 检查西侧车道的车辆排队情况
            queue1  = self.sm.traci.edge.getLastStepHaltingNumber(w)
            if queue1 == 0:
                # 如果西侧车道没有车辆排队
                # 更新队列情况并记录最后清空队列的时间
                self.queues[index][2] *= pow(1-self.wq,self.sm.time-self.lastemptyqueue[index][2])
                self.lastemptyqueue[index][2] = self.sm.time
            else:
                # 如果西侧车道有车辆排队，根据权重计算队列情况
                self.queues[index][2]*=1-self.wq
                self.queues[index][2]+=self.wq*queue1

            queue2  = self.sm.traci.edge.getLastStepHaltingNumber(e)

            # 检查东侧车道的车辆排队情况
            if queue2 == 0:
                self.queues[index][3] *= pow(1-self.wq,self.sm.time-self.lastemptyqueue[index][3])
                self.lastemptyqueue[index][3] = self.sm.time
            else:
                self.queues[index][3]*=1-self.wq
                self.queues[index][3]+=self.wq*queue2

            self._when_WE(index)

#        if pointer == 1:
#            self._when_WE(index)
#        elif pointer == 0:
#            self._when_WE_pre(index)

        if pointer == 6:
            # 检查北侧车道的车辆排队情况
            queue1  = self.sm.traci.edge.getLastStepHaltingNumber(n)

            if queue1 == 0:
                self.queues[index][0] *= pow(1-self.wq,self.sm.time-self.lastemptyqueue[index][0])
                self.lastemptyqueue[index][0] = self.sm.time
            else:
                self.queues[index][0]*=1-self.wq
                self.queues[index][0]+=self.wq*queue1

            # 检查南侧车道的车辆排队情况
            queue2  = self.sm.traci.edge.getLastStepHaltingNumber(s)

            if queue2 == 0:
                self.queues[index][1] *= pow(1-self.wq,self.sm.time-self.lastemptyqueue[index][1])
                self.lastemptyqueue[index][1] = self.sm.time
            else:
                self.queues[index][1]*=1-self.wq
                self.queues[index][1]+=self.wq*queue2

            self._when_NS(index)


