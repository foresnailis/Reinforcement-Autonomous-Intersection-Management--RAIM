"""
manhattan_graph.py
构建ManhattenGraph类，基类是SumoGraph，完成了一个基于曼哈顿距离来定义的图类。
图的大体构造是设置几条横向和纵向的街道，在所有街道交叉点位置设置一个节点（经典曼哈顿街道似乎每个街道的两个端点处也都分别设置一个节点，但是这个没有）。
这些节点有四个属性：（id，x坐标，y坐标，节点类型）。x和y坐标是按照节点数量来排的，都是整形，横向第几个节点，就是几。节点类型分为边缘节点和内部节点，
具体判别方法是看这个节点是不是在一条两侧都有相邻街道的街道上，如果不是，那这个节点（这个街道上的所有节点）都是外部节点，否则是内部节点。之后检查该节点
对应的其他节点，连接成边。

继承了SumoGraph中的基础操作。
重构了：
init方法，在原初始化基础上传入了图的行列数以及行列长度，调用_create方法来完成图构建。
定义了：
rows, cols, length: 属性方法，得到行数、列数和边长的值。
_create_graph(self): 私有方法，用于创建 ManhattanGraph 的节点和边。先定义节点和边的格式化字符串nodef和edgef。
然后按照上面陈述的逻辑创建节点，方法是嵌套循环遍历行和列。
之后创建边。
首先针对内部节点，检查其上下左右相邻节点，分别和自身连接成边；
然后将每个内部行和内部列中，最靠近边缘的端点之间连接成边。（如：a b c d e f，连接a-b，e-f）由于是有向边，在3*3图里就是四个简单环
"""

from api_sumo import SumoGraph,Node,Edge

class ManhattanGraph(SumoGraph):
    def __init__(self,rows=3,cols=3,length=100):
        super(ManhattanGraph,self).__init__()
        self._rows = rows
        self._cols = cols
        self._lgth = length
        self._create_graph()

    @property
    def rows(self): return self._rows
    @property
    def cols(self): return self._cols
    @property
    def length(self): return self._lgth

    def _create_graph(self):
        nodef = '{}.{}'.format
        edgef = '{}/{}'.format

        #Crea un nodo en cada punto (fila,columna), excepto en las cuatro esquinas de la malla
        for i in range(self.rows):
            for j in range(self.cols):
                l = 1 if (0 < i < self.rows-1) and (0 < j < self.cols-1) else 0
                n = Node(nodef(i,j),
                         self.length*j,
                         self.length*(self.rows-1-i),
                         l)
                self.add_node(n)

        for i in range(1,self.rows-1):
            for j in range(1,self.cols-1):
                n0 = self.get_node(nodef(i,j))
                nN = self.get_node(nodef(i-1,j))
                nS = self.get_node(nodef(i+1,j))
                nW = self.get_node(nodef(i,j-1))
                nE = self.get_node(nodef(i,j+1))
                self.add_edge(Edge(edgef(n0.id,nN.id),n0,nN))
                self.add_edge(Edge(edgef(n0.id,nS.id),n0,nS))
                self.add_edge(Edge(edgef(n0.id,nW.id),n0,nW))
                self.add_edge(Edge(edgef(n0.id,nE.id),n0,nE))

        for i in range(1,self.rows-1):
            n0 = self.get_node(nodef(i,0))
            n1 = self.get_node(nodef(i,1))
            n2 = self.get_node(nodef(i,self.cols-2))
            n3 = self.get_node(nodef(i,self.cols-1))
            self.add_edge(Edge(edgef(n0.id,n1.id),n0,n1))
            self.add_edge(Edge(edgef(n3.id,n2.id),n3,n2))

        for i in range(1,self.cols-1):
            n0 = self.get_node(nodef(0,i))
            n1 = self.get_node(nodef(1,i))
            n2 = self.get_node(nodef(self.rows-2,i))
            n3 = self.get_node(nodef(self.rows-1,i))
            self.add_edge(Edge(edgef(n0.id,n1.id),n0,n1))
            self.add_edge(Edge(edgef(n3.id,n2.id),n3,n2))
