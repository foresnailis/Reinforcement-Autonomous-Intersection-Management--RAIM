
"""
交通流
属性列表：
begin：交通流开始的时间（模拟时间单位）。
end：交通流结束的时间（模拟时间单位）。
vehsPerHour：每小时的车辆数，定义交通流的密度。
period：发车间隔时间，另一种定义交通流密度的方式。
probability：每个时间单位有车辆发车的概率，是定义交通流的另一种方法。
number：总车辆数，定义了整个模拟过程中将会发出的车辆总数。
tpe：交通流中车辆的类型。
route：车辆行驶的路线。
color：车辆的颜色（可能用于可视化）。
departLane：车辆出发时所在的车道。
departPos：车辆在车道上的起始位置。
departSpeed：车辆出发时的速度。
arrivalLane：车辆到达时所在的车道。
arrivalPos：车辆在车道上的到达位置。
arrivalSpeed：车辆到达时的速度。
line：用于公共交通线路的标识。
personNumber：每辆车中人的数量（可能用于乘客流量模拟）。
containerNumber：车辆携带的容器数量。
reroute：是否在路线中途重新规划路线。
departPosLat：车辆出发的纬度位置（用于更精确的地理位置模拟）。
arrivalPosLat：车辆到达的纬度位置。
fromJunction：交通流起始的交叉口。
toJunction：交通流结束的交叉口。
"""

from elem import Elem



class Flow(Elem):
    def __init__(self,_id,\
                begin,\
                end,\
                vehsPerHour=None,\
                period=None,\
                probability=None,\
                number=None,\
                tpe=None,\
                route=None,\
                color=None,\
                departLane=None,\
                departPos=None,\
                departSpeed=None,\
                arrivalLane=None,\
                arrivalPos=None,\
                arrivalSpeed=None,\
                line=None,\
                personNumber=None,\
                containerNumber=None,\
                reroute=None,\
                departPosLat=None,\
                arrivalPosLat=None,\
                fromJunction=None,\
                toJunction=None):
        if vehsPerHour==None and period==None and\
            probability==None and number==None:
                raise ValueError('vehsPerHour,period,probability or number must be defined')
        # Set the basic attributes of a flow
        attr= dict()
        attr['begin']=begin
        attr['end']=end
        attr['vehsPerHour']=vehsPerHour
        attr['period']=period
        attr['probability']=probability
        attr['number']=number
        attr['type']=tpe
        attr['route']=route
        attr['color']=color
        attr['departLane']=departLane
        attr['departPos']=departPos
        attr['departSpeed']=departSpeed
        attr['arrivalLane']=arrivalLane
        attr['arrivalPos']=arrivalPos
        attr['arrivalSpeed']=arrivalSpeed
        attr['line']=line
        attr['personNumber']=personNumber
        attr['containerNumber']=containerNumber
        attr['reroute']=reroute
        attr['departPosLat']=departPosLat
        attr['arrivalPosLat']=arrivalPosLat
        attr['fromJunction']=fromJunction
        attr['toJunction']=toJunction
        super(Flow,self).__init__('flow',_id,attr)