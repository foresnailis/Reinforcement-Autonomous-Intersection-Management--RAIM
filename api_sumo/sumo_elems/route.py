#!/usr/bin/env python
"""
路径
edges 组成路径的边列表
color 颜色
"""

from elem import Elem

__author__ = "Bryan Alexis Freire Viteri"
__version__ = "3.0"
__email__ = "bryanfv95@gmail.com"

class Route(Elem):
    def __init__(self,_id,edges,color=None):
        # Set the basic attributes of a route
        attr         = dict()
        attr['edges'] = edges
        attr['color']   = color
        super(Route,self).__init__('route',_id,attr)