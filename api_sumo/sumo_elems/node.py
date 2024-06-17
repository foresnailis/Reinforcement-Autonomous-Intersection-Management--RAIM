
"""
点
除了属性列表外，额外维护一个字典connections，key为其他点的id，value为距离
属性列表：
x,y 坐标
type traffic_light或None
"""
from math import sqrt
from elem import Elem



class Node(Elem):
    def __init__(self,_id,x,y,typ=0):
        # Set the basic attributes of a node
        attr         = dict()
        attr['x']    = x
        attr['y']    = y
        attr['type'] = 'traffic_light' if typ else None
        super(Node,self).__init__('node',_id,attr)
        super(Node,self).__set_attr__('connections',dict())
    
    def get_connections(self):
        '''Returns ids of all neighbors of this node'''
        return self.connections.keys()
        
    def get_weight(self,nbr):
        '''Return the distance between this node and other'''
        if nbr.id not in self.connections:
            return sqrt((self.x-nbr.x)**2 + (self.y-nbr.y)**2)
        return self.connections[nbr.id]