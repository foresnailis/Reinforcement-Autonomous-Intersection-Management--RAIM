
from manhattan_algorithm import ManhattanAlgorithm

__author__ = "Bryan Alexis Freire Viteri"
__version__ = "3.0"
__email__ = "bryanfv95@gmail.com"

# 继承自 ManhattanAlgorithm 类
class FixedAlgorithm(ManhattanAlgorithm): 
    def __init__(self,greentime=20,wq=0.5,lanes=1):
        super(FixedAlgorithm,self).__init__(wq,lanes)
        self.greentime = greentime

    def _when_NS(self,index): # 纵向流量（南北方向）的绿灯持续时间
        self.changestate[index] += self.greentime

    def _when_WE(self,index): # 横向流量（西东方向）的绿灯持续时间
        self.changestate[index] += self.greentime