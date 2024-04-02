#!/usr/bin/env python
"""
车辆类型分布
通过初始化这个类的实例，你可以创建一个车辆类型分布，这在创建复杂的交通模拟场景时尤其有用，比如，当你需要模拟由多种不同类型的车辆组成的交通流时。
属性列表：
vTypes: 车辆类型列表，每个元素通常是一个字典，定义了车辆类型及其相应的比例或数量。这个参数是可选的，如果不提供，默认为 None。
"""

from elem import Elem

__author__ = "Bryan Alexis Freire Viteri"
__version__ = "3.0"
__email__ = "bryanfv95@gmail.com"

class VehDist(Elem):
    def __init__(self,_id,\
                    vTypes=None):
        # Set the basic attributes of a car type
        attr= dict()
        attr['vTypes']=vTypes
        super(VehDist,self).__init__('vTypeDistribution',_id,attr)