#!/usr/bin/env python
"""

"""
import numpy as np
__author__ = "Bryan Alexis Freire Viteri"
__version__ = "3.0"
__email__ = "bryanfv95@gmail.com"

class SumoAlgorithm(object):
    """
    SumoAlgorithm 类用于定义基本的 SUMO (Simulation of Urban MObility) 算法。

    Attributes:
        pgm (list): 表示交通信号灯控制程序的列表，包含了各个阶段的信号状态。
        keypoints (list): 表示关键时刻的列表，用于触发特定操作。

    Methods:
        get_statistics(): 抽象方法，用于获取算法的统计信息。
        prepare_algorithm(sm): 初始化算法，设置与 SUMO 仿真环境的连接。
        reset_algorithm(sm): 重置算法状态，重新初始化与 SUMO 仿真环境的连接。
        step(): 执行算法的一步，根据当前时间更新信号灯状态。
        _when(pointer, index): 抽象方法，定义关键时刻触发的操作。
    """

    def __init__(self,program,keypoints):
        self.pgm = program
        self.kps = keypoints

    def get_statistics(self):
        """
        抽象方法，用于获取算法的统计信息。

        Raises:
            NotImplementedError: 如果子类未实现此方法，则会引发 NotImplementedError 异常。
        """
        raise NotImplementedError('get_statistics is not implemented')

    def prepare_algorithm(self,sm):
        """
        初始化算法，设置与 SUMO 仿真环境的连接。

        Args:
            sm (SUMOManager): 与 SUMO 仿真环境的连接管理器。
        """
        self.sm = sm
        self.traci = sm.traci
        self.ids = self.traci.trafficlights.getIDList()
        self.pointers = [0 for i in range(len(self.ids))]
        self.changestate = [0 for i in range(len(self.ids))]

    def reset_algorithm(self,sm):
        """
        重置算法状态，重新初始化与 SUMO 仿真环境的连接。

        Args:
            sm (SUMOManager): 与 SUMO 仿真环境的连接管理器。
        """
        self.sm = sm
        self.traci = sm.traci
        self.ids = self.traci.trafficlights.getIDList()
        for index in range(len(self.ids)):
            self.pointers[index] = 0
            self.changestate[index] = 0

    def step(self):
        """
        执行算法的一步，根据当前时间更新信号灯状态。
        """
        for index,id in enumerate(self.ids):
            if self.sm.time == self.changestate[index]:
                self.pointers[index] += 1
                self.pointers[index] %= len(self.pgm)
                self.traci.trafficlights.setRedYellowGreenState(id,\
                    self.pgm[self.pointers[index]])
                if self.pointers[index] in self.kps:
                    self._when(self.pointers[index],index)
                else:
                    self.changestate[index] += 1

    def _when(self,pointer,index):
        """
        抽象方法，定义关键时刻触发的操作。

        Args:
            pointer (int): 当前程序指针的位置。
            index (int): 信号灯的索引。
        """
        raise NotImplementedError('_when is not implemented')