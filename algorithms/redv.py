

from random import uniform

from manhattan_algorithm import ManhattanAlgorithm



class REDVAlgorithm(ManhattanAlgorithm):
    """
        初始化 REDVAlgorithm 类的实例。

        Args:
            cycle (int): 信号灯周期的总时间。
            min_green_time (int): 最小绿灯时间。
            min_red_time (int): 最小红灯时间。
            minth (float): 最小车辆排队长度。
            maxth (float): 最大车辆排队长度。
            delta (int): 改变绿灯时间的增量。
            maxp (float): 最大绿灯时延长周期的比例。
            avgmode (function): 用于计算平均排队长度的函数。
            wq (float): 车辆等待时间权重。
            lanes (int): 交通道数。
            greentime_pre (int): 绿灯预留时间。
    """
    def __init__(self,cycle,min_green_time,min_red_time,minth,maxth,delta,\
                maxp,avgmode,wq=0.5,lanes=1, greentime_pre=15):
        super(REDVAlgorithm,self).__init__(wq,lanes)
        self.cycle = cycle
        self.min_green_time = min_green_time
        self.min_red_time = min_red_time
        self.minth = minth
        self.maxth = maxth
        self.delta = delta
        self.maxp = maxp
        self.avgmode = avgmode
        self.limitup = cycle - min_red_time - 10
        self.greentime_pre =greentime_pre

    def prepare_algorithm(self,sm):
        super(REDVAlgorithm,self).prepare_algorithm(sm)
        self.count = [[-1]*2 for _ in range(len(self.ids))]
        self.lastgreentime = [[(self.cycle-10)//2]*2 for _ in range(len(self.ids))]

    def reset_algorithm(self,sm):
        super(REDVAlgorithm,self).reset_algorithm(sm)
        for i in range(len(self.ids)):
            for j in range(2):
                self.count[i][j] = -1
                self.lastgreentime[i][j] = (self.cycle-10)//2

    def _when_green(self,index,num):
        avg = self.avgmode((self.queues[index][0 if num == 0 else 2],\
                            self.queues[index][1 if num == 0 else 3]))
        print("Average queue is: ", avg)
        if avg < self.minth:
            print('avg: {} < minth: {}'.format(avg,self.minth))
            self.count[index][num] = -1
            pa = 0
        elif avg > self.maxth:
            print('avg: {} > maxth: {}'.format(avg,self.maxth))
            self.count[index][num] = 0
            pa = 1
        else:
            print('minth: {} < avg: {} < maxth: {}'.format(self.minth,avg,self.maxth))
            self.count[index][num] += 1
            print('count[{}][{}]: {}'.format(index,num,self.count[index][num]))
            pb = self.maxp*(avg-self.minth)/(self.maxth-self.minth) + 1e-12
            print('pb: {}'.format(pb))

            pa = pb / (1-self.count[index][num]*pb)
            print('pa before correction: {}'.format(pa))
            pa = 1 if pa > 1 else pa
            print('pa after correction: {}'.format(pa))

        gt = self.lastgreentime[index][num]

        print('Green time before decision: {}'.format(gt))
        dec=uniform(0,1)

        print('Checking if dec:{} < pa:{}'.format(dec,pa))
        if dec < pa:
            gt += self.delta
            self.count[index][num] = 0
        print('Green time after decision: {}'.format(gt))

        if gt > self.limitup:
            gt = self.limitup
        elif gt < self.min_green_time:
            gt = self.min_green_time
        #self.lastgreentime[index][num] = gt
        self.lastgreentime[index][1-num] = self.cycle - 20 - gt
        self.changestate[index]   = self.sm.time + gt


    def _when_NS_pre(self,index):
        self.changestate[index] += self.greentime_pre

    def _when_NS(self,index):
        self._when_green(index,0)

    def _when_WE_pre(self,index):
        self.changestate[index] += self.greentime_pre

    def _when_WE(self,index):
        self._when_green(index,1)