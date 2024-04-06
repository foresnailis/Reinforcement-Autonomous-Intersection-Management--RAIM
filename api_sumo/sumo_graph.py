#!/usr/bin/env python
"""
sumo_graph.py
构建SumoGraph基类，使用sumo_elems中的边和节点，定义了一些待实现或已实现的方法。总的来说，是一个管理模拟环境中图设计的类。
init: 初始化方法，构建一个边字典和节点字典

add_node: 参数是节点，在字典中增加一个键、值对应输入节点的节点
get_node: 用节点id查找节点
in_node: 输入为节点，判断图内是否有该节点
get_nodes: 得到当前图中所有节点id所构成的列表
iter_nodes: 得到一个迭代器，遍历所有节点的值

add_edge: 输入为一个边，添加边
get_edge: 用边id查找边
in_edge: 输入为一个边，判断图内是否有该边
get_edges: 得到当前图中所有边id所构成的列表
iter_edges: 得到一个迭代器，遍历所有边的值
"""
from sumo_elems import Edge,Node

__author__ = "Bryan Alexis Freire Viteri"
__version__ = "3.0"
__email__ = "bryanfv95@gmail.com"

class SumoGraph(object):
    def __init__(self):
        self.node_list = dict()
        self.edge_list = dict()
    
    def add_node(self,n):
#        if type(n) != Node:
#            raise TypeError('Arguments must be a node.')
        self.node_list[n.id] = n
    
    def get_node(self,id):
        return self.node_list[id]
    
    def in_node(self,n):
        return n in self.node_list
    
    def get_nodes(self):
        return self.node_list.keys()
    
    def iter_edges(self):
        return iter(self.edge_list.values())
    
    def add_edge(self,e):
#        if type(e) != Edge:
#            raise TypeError('Arguments must be a edge.')
        if not (e['from'] in self.node_list and e.to in self.node_list):
            raise ReferenceError('Edge nodes must be in the graph before.')
        self.edge_list[e.id] = e
    
    def get_edge(self,id):
        return self.edge_list[id]

    def in_edge(self,n):
        return n in self.edge_list
        
    def get_edges(self):
        return self.edge_list.keys()
    
    def iter_nodes(self):
        return iter(self.node_list.values())
    