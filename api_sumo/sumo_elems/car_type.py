"""
定义车辆类型的属性和行为
属性列表：
accel: 最大加速度。
decel: 最大减速度。
sigma: 驾驶员的驾驶不确定性。
tau: 车辆间的最小时间间隔。
length: 车辆长度。
minGap: 车辆间的最小间隔。
maxSpeed: 车辆的最大速度。
speedFactor: 速度因子，用于调整速度限制的遵守情况。
speedDev: 速度偏差，表示驾驶员实际速度相对于速度限制的变化。
color: 车辆在图形用户界面（GUI）中的显示颜色。
vClass: 车辆类别。
emissionClass: 排放类别，用于模拟环境影响。
guiShape: 车辆在GUI中的形状。
width: 车辆宽度。
imgFile: 车辆的图像文件路径，用于GUI展示。
impatience: 驾驶员的不耐烦程度，影响车辆的驾驶行为。
laneChangeModel: 车道变换模型。
carFollowModel: 跟车模型，定义车辆如何跟随前车。
personCapacity: 人员容量，对公交车等车辆类型而言。
containerCapacity: 容器容量，对货运车辆类型而言。
boardingDuration: 上车所需时间。
loadingDuration: 装载所需时间。
latAlignment: 横向对齐，影响车辆在车道上的位置。
minGapLat: 横向最小间隔。
maxSpeedLat: 横向最大速度。
probability: 车辆生成概率，可能用于控制某些特定车辆类型的生成频率。
"""

from elem import Elem

class CarType(Elem):
    def __init__(self,_id,\
                    accel=None,\
                    decel=None,\
                    sigma=None,\
                    tau=None,\
                    length=None,\
                    minGap=None,\
                    maxSpeed=None,\
                    speedFactor=None,\
                    speedDev=None,\
                    color=None,\
                    vClass=None,\
                    emissionClass=None,\
                    guiShape=None,\
                    width=None,\
                    imgFile=None,\
                    impatience=None,\
                    laneChangeModel=None,\
                    carFollowModel=None,\
                    personCapacity=None,\
                    containerCapacity=None,\
                    boardingDuration=None,\
                    loadingDuration=None,\
                    latAlignment=None,\
                    minGapLat=None,\
                    maxSpeedLat=None,\
                    probability=None):
        # Set the basic attributes of a car type
        attr= dict()
        attr['accel']=accel
        attr['decel']=decel
        attr['sigma']=sigma
        attr['tau']=tau
        attr['length']=length
        attr['minGap']=minGap
        attr['maxSpeed']=maxSpeed
        attr['speedFactor']=speedFactor
        attr['speedDev']=speedDev
        attr['color']=color
        attr['vClass']=vClass
        attr['emissionClass']=emissionClass
        attr['guiShape']=guiShape
        attr['width']=width
        attr['imgFile']=imgFile
        attr['impatience']=impatience
        attr['laneChangeModel']=laneChangeModel
        attr['carFollowModel']=carFollowModel
        attr['personCapacity']=personCapacity
        attr['containerCapacity']=containerCapacity
        attr['boardingDuration']=boardingDuration
        attr['loadingDuration']=loadingDuration
        attr['latAlignment']=latAlignment
        attr['minGapLat']=minGapLat
        attr['maxSpeedLat']=maxSpeedLat
        attr['probability']=probability
        super(CarType,self).__init__('vType',_id,attr)