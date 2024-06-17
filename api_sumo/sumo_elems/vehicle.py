
"""
具体车辆
属性列表：
depart: 车辆的出发时间（模拟时间单位）。
tpe: 车辆的类型，通常引用一个 CarType 实例的 ID。
route: 车辆将要行驶的路线的标识符。
color: （可选）车辆在图形用户界面（GUI）中的显示颜色。
departLane: （可选）车辆出发时所在的车道。
departPos: （可选）车辆在车道上的起始位置。
departSpeed: （可选）车辆出发时的速度。
arrivalLane: （可选）车辆到达时所在的车道。
arrivalPos: （可选）车辆在车道上的到达位置。
arrivalSpeed: （可选）车辆到达时的速度。
line: （可选）用于公共交通线路的标识。
personNumber: （可选）每辆车中人的数量，对于公交车或其他载人车辆而言。
containerNumber: （可选）车辆携带的容器数量，对货运车辆而言。
reroute: （可选）是否在路线中途重新规划路线。
departPosLat: （可选）车辆出发的纬度位置（用于更精确的地理位置模拟）。
arrivalPosLat: （可选）车辆到达的纬度位置。
"""

from elem import Elem



class Vehicle(Elem):
    def __init__(self,_id,\
                depart,\
                tpe,\
                route,\
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
                arrivalPosLat=None):
        attr= dict()
        attr['type']=tpe
        attr['route']=route
        attr['color']=color
        attr['depart']=depart
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
        super(Vehicle,self).__init__('vehicle',_id,attr)