
import random

from scenario import AsymmetricVariableFlowsManhattan


class ScenarioEight(AsymmetricVariableFlowsManhattan):
    def __init__(self,sg_mh):
        begin = 0
        end = 1800
        dur = 1800
        inter = 6*3
        flow = ()
        WE = 1
        for interval in range(inter):
            for flows in range(20):
                if WE == 1:
                    if flows == 4:
                        flow = flow + (begin,end,1000)
                    else:
                        flow = flow + (begin,end,100)

                elif WE == 2:
                    if flows == 4 or flows == 6:
                        flow = flow + (begin,end,1000)
                    else:
                        flow = flow + (begin,end,100)

                elif WE == 3:
                    if flows == 4 or flows == 6:
                        flow = flow + (begin,end,1000)
                    else:
                        flow = flow + (begin,end,100)

                elif WE == 4:
                    flow = flow + (begin,end,500)

                elif WE == 5:
                    flow = flow + (begin,end,250)

                else:
                    flow = flow + (begin,end,10)
            WE = WE%6 +1
            begin = end + 1
            end += dur

        super(ScenarioEight,self).__init__(sg_mh,20,*flow)