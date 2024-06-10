
"""
路径
edges 组成路径的边列表
color 颜色
"""

from elem import Elem



class Route(Elem):
    def __init__(self,_id,edges,color=None):
        # Set the basic attributes of a route
        attr         = dict()
        attr['edges'] = edges
        attr['color']   = color
        super(Route,self).__init__('route',_id,attr)