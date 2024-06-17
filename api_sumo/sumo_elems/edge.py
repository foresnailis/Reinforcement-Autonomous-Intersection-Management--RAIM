"""
有向边
属性列表：
from 起点对象id
to 终点对象id
"""

from elem import Elem

class Edge(Elem):
    def __init__(self,_id,frm,to):
        attr         = dict()
        attr['from'] = frm.id
        attr['to']   = to.id
        # attr['sidewalkWidth'] = 6
        super(Edge,self).__init__('edge',_id,attr)
        frm.connections[to.id] = frm.get_weight(to)