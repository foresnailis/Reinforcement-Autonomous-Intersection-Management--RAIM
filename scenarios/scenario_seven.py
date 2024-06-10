import random

from scenario import AsymmetricVariableFlowsManhattan

class ScenarioSeven(AsymmetricVariableFlowsManhattan):
    def __init__(self,sg_mh):
        begin = 0
        end = 3600
        dur = 3600
        inter = 5
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
                    if flows == 6:
                        flow = flow + (begin,end,1000)
                    else:
                        flow = flow + (begin,end,100)

                elif WE == 3:
                    flow = flow + (begin,end,500)

                elif WE == 4:
                    flow = flow + (begin,end,250)

                else:
                    flow = flow + (begin,end,10)
            WE = WE%5 +1
            begin = end + 1
            end += dur

        super(ScenarioSeven,self).__init__(sg_mh,20,*flow)