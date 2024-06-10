from scenario import AsymmetricVariableFlowsManhattan

"""
通行2小时
每小时1500辆车
"""
class ScenarioTwo(AsymmetricVariableFlowsManhattan):
    def __init__(self,sg_mh,prob=1500.0/3600,begin=0,end=7200):
        super(ScenarioTwo,self).__init__(sg_mh,1,begin,end,prob)