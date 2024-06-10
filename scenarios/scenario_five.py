from scenario import AsymmetricVariableFlowsManhattan

"""
通行2小时
北500/h
南250/h
西500/h
东250/h
"""
class ScenarioFive(AsymmetricVariableFlowsManhattan):
    def __init__(self,sg_mh):
        super(ScenarioFive,self).__init__(sg_mh,4,\
            0,7200,500.0/3600,0,7200,250.0/3600,\
            0,7200,500.0/3600,0,7200,250.0/3600)