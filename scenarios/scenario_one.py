from scenario import AsymmetricVariableFlowsManhattan

"""
通行5小时
共500辆车
"""
class ScenarioOne(AsymmetricVariableFlowsManhattan):
    def __init__(self,sg_mh,prob=500.0,begin=0,end=3600*5):
        super(ScenarioOne,self).__init__(sg_mh,1,begin,end,prob)
        self.begin = begin