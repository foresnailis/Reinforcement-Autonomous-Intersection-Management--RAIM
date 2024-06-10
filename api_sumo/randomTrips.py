'''
sumo官方提供的路网生成代码
'''
from __future__ import print_function
from __future__ import absolute_import
import os
import sys
import random
import bisect
import subprocess
from collections import defaultdict
import math
import optparse

'''
导入SUMO相关的库，让SUMO工具包能被找到，从而直接调用SUMO提供的功能接口，避免指定SUMO的完整路径和手动添加路径的操作。
'''
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import sumolib  # noqa sumo工具包
from sumolib.miscutils import euclidean  # noqa 欧式距离相关工具
from sumolib.geomhelper import naviDegree, minAngleDegreeDiff  # noqa 
'''
naviDegree计算从起始点到目标点的导航角度。导航角度通常指的是一个方向向量与水平轴之间的夹角，用于指示从一个点到另一个点的方向。
接受起始点和目标点的坐标，返回起始点指向目标点的角度。
minAngleDegreeDiff计算两个角度之间的最小角度差，就是两个角度的差值（360度以内）。
接受两个角度值，返回它们之间的最小角度差值。
'''

'''
DUAROUTER似乎是SUMO路由，生成车辆在交通网络中的行车路线，生成的东西能作为模拟的输入。
后面是默认后缀，src，dst，via分别是起点、终点和途径点。
'''
DUAROUTER = sumolib.checkBinary('duarouter')

SOURCE_SUFFIX = ".src.xml"
SINK_SUFFIX = ".dst.xml"
VIA_SUFFIX = ".via.xml"

'''
接受控制台参数，并将其构造成一个列表。根据gpt：
1. `-n`, `--net-file`: 指定网络文件，是必需的参数。
2. `-a`, `--additional-files`: 指定要由路由器加载的额外文件。
3. `-o`, `--output-trip-file`: 指定输出的行程文件名，默认为 "trips.trips.xml"。
4. `-r`, `--route-file`: 生成使用 DUAROUTER 的路由文件。
5. `--vtype-output`: 将生成的车辆类型存储到单独的文件中。
6. `--weights-prefix`: 从指定的文件加载作为源、目的地和途经边的概率。
7. `--weights-output-prefix`: 生成用于可视化的权重文件。
8. `--pedestrians`: 创建行人行程文件而不是车辆行程文件。
9. `--persontrips`: 创建人员行程文件而不是车辆行程文件。
10. `--personrides`: 使用指定的行程创建人员行程文件。
11. `--prefix`: 行程ID的前缀。
12. `-t`, `--trip-attributes`: 额外的行程属性。
13. `--fringe-start-attributes`: 在边缘开始时的额外行程属性。
14. `-b`, `--begin`: 开始时间。
15. `-e`, `--end`: 结束时间，默认为 3600。
16. `-p`, `--period`: 使用等距离出发时间生成车辆。
17. `-s`, `--seed`: 随机种子。
18. `-l`, `--length`: 按长度加权边的概率。
19. `-L`, `--lanes`: 按车道数加权边的概率。
20. `--edge-param`: 使用给定的边参数作为边的因素。
21. `--speed-exponent`: 按速度的指数加权边的概率。
22. `--angle`: 按相对于网络中心的角度加权边的概率。
23. `--angle-factor`: 角度的最大权重因子。
24. `--fringe-factor`: 将边缘边的权重乘以指定的因子。
25. `--fringe-threshold`: 仅将速度高于指定阈值的边视为边缘边。
26. `--allow-fringe`: 允许从离开网络的边出发，并到达进入网络的边。
27. `--allow-fringe.min-length`: 允许从离开网络的边出发，并到达进入网络的边，如果它们至少具有给定的长度。
28. `--min-distance`: 要求每个行程的起始边和结束边之间的距离至少为指定值。
29. `--max-distance`: 要求每个行程的起始边和结束边之间的距离最多为指定值。
30. `-i`, `--intermediate`: 生成指定数量的中间路径点。
31. `--flows`: 生成指定数量的流量，以指定的周期输出车辆。
32. `--jtrrouter`: 生成无目的地的流量，作为 jtrrouter 的输入。
33. `--maxtries`: 满足距离约束的行程尝试的最大次数。
34. `--binomial`: 如果设置了此选项，每秒出发的车辆数将从二项分布中绘制。
35. `-c`, `--vclass`, `--edge-permission`: 限制允许的车辆类型。
36. `--vehicle-class`: 分配给生成行程的车辆类别。
37. `--remove-loops`: 删除路线开始和结束时的循环。
38. `--junction-taz`: 将行程写入带有 fromJunction 和 toJunction 的文件。
39. `--validate`: 是否生成已经检查过连通性的行程输出。
40. `-v`, `--verbose`: 输出详细的执行信息。
'''
def get_options(args=None):
    optParser = optparse.OptionParser()
    optParser.add_option("-n", "--net-file", dest="netfile",
                         help="define the net file (mandatory)")
    optParser.add_option("-a", "--additional-files", dest="additional",
                         help="define additional files to be loaded by the router")
    optParser.add_option("-o", "--output-trip-file", dest="tripfile",
                         default="trips.trips.xml", help="define the output trip filename")
    optParser.add_option("-r", "--route-file", dest="routefile",
                         help="generates route file with duarouter")
    optParser.add_option("--vtype-output", dest="vtypeout",
                         help="Store generated vehicle types in a separate file")
    optParser.add_option("--weights-prefix", dest="weightsprefix",
                         help="loads probabilities for being source, destination and via-edge from the files named " +
                         "<prefix>.src.xml, <prefix>.sink.xml and <prefix>.via.xml")
    optParser.add_option("--weights-output-prefix", dest="weights_outprefix",
                         help="generates weights files for visualisation")
    optParser.add_option("--pedestrians", action="store_true",
                         default=False, help="create a person file with pedestrian trips instead of vehicle trips")
    optParser.add_option("--persontrips", action="store_true",
                         default=False, help="create a person file with person trips instead of vehicle trips")
    optParser.add_option("--personrides", help="create a person file with rides using STR as lines attribute")
    optParser.add_option("--persontrip.transfer.car-walk", dest="carWalkMode",
                         help="Where are mode changes from car to walking allowed " +
                         "(possible values: 'ptStops', 'allJunctions' and combinations)")
    optParser.add_option("--persontrip.walkfactor", dest="walkfactor",
                         help="Use FLOAT as a factor on pedestrian maximum speed during intermodal routing")
    optParser.add_option("--prefix", dest="tripprefix",
                         default="", help="prefix for the trip ids")
    optParser.add_option("-t", "--trip-attributes", dest="tripattrs",
                         default="", help="additional trip attributes. When generating pedestrians, attributes for " +
                         "<person> and <walk> are supported.")
    optParser.add_option("--fringe-start-attributes", dest="fringeattrs",
                         default="", help="additional trip attributes when starting on a fringe.")
    optParser.add_option("-b", "--begin", type="float", default=0, help="begin time")
    optParser.add_option("-e", "--end", type="float", default=3600, help="end time (default 3600)")
    optParser.add_option(
        "-p", "--period", type="float", default=1, help="Generate vehicles with equidistant departure times and " +
        "period=FLOAT (default 1.0). If option --binomial is used, the expected arrival rate is set to 1/period.")
    optParser.add_option("-s", "--seed", type="int", help="random seed")
    optParser.add_option("-l", "--length", action="store_true",
                         default=False, help="weight edge probability by length")
    optParser.add_option("-L", "--lanes", action="store_true",
                         default=False, help="weight edge probability by number of lanes")
    optParser.add_option("--edge-param", dest="edgeParam",
                         help="use the given edge parameter as factor for edge")
    optParser.add_option("--speed-exponent", type="float", dest="speed_exponent",
                         default=0.0, help="weight edge probability by speed^<FLOAT> (default 0)")
    optParser.add_option("--angle", type="float", dest="angle",
                         default=90.0, help="weight edge probability by angle [0-360] relative to the network center")
    optParser.add_option("--angle-factor", type="float", dest="angle_weight",
                         default=1.0, help="maximum weight factor for angle")
    optParser.add_option("--fringe-factor", type="float", dest="fringe_factor",
                         default=1.0, help="multiply weight of fringe edges by <FLOAT> (default 1")
    optParser.add_option("--fringe-threshold", type="float", dest="fringe_threshold",
                         default=0.0, help="only consider edges with speed above <FLOAT> as fringe edges (default 0)")
    optParser.add_option("--allow-fringe", dest="allow_fringe", action="store_true",
                         default=False, help="Allow departing on edges that leave the network and arriving on edges " +
                         "that enter the network (via turnarounds or as 1-edge trips")
    optParser.add_option("--allow-fringe.min-length", type="float", dest="allow_fringe_min_length",
                         help="Allow departing on edges that leave the network and arriving on edges " +
                         "that enter the network, if they have at least the given length")
    optParser.add_option("--min-distance", type="float", dest="min_distance",
                         default=0.0, help="require start and end edges for each trip to be at least <FLOAT> m apart")
    optParser.add_option("--max-distance", type="float", dest="max_distance",
                         default=None, help="require start and end edges for each trip to be at most <FLOAT> m " +
                         "apart (default 0 which disables any checks)")
    optParser.add_option("-i", "--intermediate", type="int",
                         default=0, help="generates the given number of intermediate way points")
    optParser.add_option("--flows", type="int",
                         default=0, help="generates INT flows that together output vehicles with the specified period")
    optParser.add_option("--jtrrouter", action="store_true",
                         default=False, help="Create flows without destination as input for jtrrouter")
    optParser.add_option("--maxtries", type="int",
                         default=100, help="number of attemps for finding a trip which meets the distance constraints")
    optParser.add_option("--binomial", type="int", metavar="N",
                         help="If this is set, the number of departures per seconds will be drawn from a binomial " +
                         "distribution with n=N and p=PERIOD/N where PERIOD is the argument given to " +
                         "option --period. Tnumber of attemps for finding a trip which meets the distance constraints")
    optParser.add_option(
        "-c", "--vclass", "--edge-permission", default="passenger",
        help="only from and to edges which permit the given vehicle class")
    optParser.add_option(
        "--vehicle-class", help="The vehicle class assigned to the generated trips (adds a standard vType definition " +
        "to the output file).")
    optParser.add_option("--remove-loops", dest="remove_loops", action="store_true",
                         default=False, help="Remove loops at route start and end")
    optParser.add_option("--junction-taz", dest="junctionTaz", action="store_true",
                         default=False, help="Write trips with fromJunction and toJunction")
    optParser.add_option("--validate", default=False, action="store_true",
                         help="Whether to produce trip output that is already checked for connectivity")
    optParser.add_option("-v", "--verbose", action="store_true",
                         default=False, help="tell me what you are doing")
    (options, args) = optParser.parse_args(args=args)
    if not options.netfile:
        optParser.print_help()
        sys.exit(1)

    if options.persontrips or options.personrides:
        options.pedestrians = True

    if options.pedestrians:
        options.vclass = 'pedestrian'
        if options.flows > 0:
            print("Error: Person flows are not supported yet", file=sys.stderr)
            sys.exit(1)

    if options.validate and options.routefile is None:
        options.routefile = "routes.rou.xml"

    if options.period <= 0:
        print("Error: Period must be positive", file=sys.stderr)
        sys.exit(1)

    if options.jtrrouter and options.flows <= 0:
        print("Error: Option --jtrrouter must be used with option --flows", file=sys.stderr)
        sys.exit(1)

    if options.vehicle_class:
        if options.tripprefix:
            options.vtypeID = "%s_%s" % (options.tripprefix, options.vehicle_class)
        else:
            options.vtypeID = options.vehicle_class

        if 'type=' in options.tripattrs:
            print("Error: trip-attribute 'type' cannot be used together with option --vehicle-class", file=sys.stderr)
            sys.exit(1)

    return options

'''
InvalidGenerator：用来被抛出的异常
'''
class InvalidGenerator(Exception):
    pass

# assigns a weight to each edge using weight_fun and then draws from a discrete
# distribution with these weights

'''
RandomEdgeGenerator：随机边生成器
'''
class RandomEdgeGenerator:
    # 初始化
    def __init__(self, net, weight_fun):
        self.net = net # 路网对象，随机边在此之上生成
        self.weight_fun = weight_fun # 边权计算（分配）函数
        self.cumulative_weights = [] # 累计边权列表，感觉跟边列表差不多
        self.total_weight = 0 # 总权重 
        for edge in self.net._edges: # 分配边权，同时变更总权重和累计边权列表
            # print edge.getID(), weight_fun(edge)
            self.total_weight += weight_fun(edge)
            self.cumulative_weights.append(self.total_weight)
        if self.total_weight == 0: # 无效图异常
            raise InvalidGenerator()

    # get: 生成一个随机权重边r，它的权重在当前总权重和0之间，然后把它插入累计边权表中，同时保持表有序。之后从表中返回这个边权。
    def get(self): 
        r = random.random() * self.total_weight
        index = bisect.bisect(self.cumulative_weights, r)
        return self.net._edges[index]

    # 把整个图中的所有边信息（边id和边权）按照特定xml格式写入fname
    def write_weights(self, fname):
        # normalize to [0,100]
        normalizer = 100.0 / max(1, max(map(self.weight_fun, self.net._edges))) # 映射器，把边权映射到100以内
        weights = [(self.weight_fun(e) * normalizer, e.getID()) for e in self.net.getEdges()] # 应用映射器，结果存到weights
        weights.sort(reverse=True) # 降序排序
        with open(fname, 'w+') as f: # 按照格式写入
            f.write('<edgedata>\n')
            f.write('    <interval begin="0" end="10">\n')
            for weight, edgeID in weights:
                f.write('        <edge id="%s" value="%0.2f"/>\n' %
                        (edgeID, weight))
            f.write('    </interval>\n')
            f.write('</edgedata>\n')

'''
RandomTripGenerator：随机路线生成器
'''
class RandomTripGenerator:

    # 构造函数，接受三个生成器和两个值作为参数：起点边生成器、终点边生成器、途径边生成器；途径边数量、当前生成的行人路径还是车辆路径
    def __init__(self, source_generator, sink_generator, via_generator, intermediate, pedestrians):
        self.source_generator = source_generator
        self.sink_generator = sink_generator
        self.via_generator = via_generator
        self.intermediate = intermediate
        self.pedestrians = pedestrians

    # 生成路线的函数，参数是：路线最短长度、最长长度（为None无限制），生成次数（这个生成方法类似于启发式搜索，所以会有这个）
    def get_trip(self, min_distance, max_distance, maxtries=100):
        for _ in range(maxtries): # 不断尝试生成次数
            source_edge = self.source_generator.get() 
            intermediate = [self.via_generator.get()
                            for i in range(self.intermediate)]
            sink_edge = self.sink_generator.get() # 通过生成器获取起始边、终点边和对应个数的途径边
            if self.pedestrians: # 如果是生成行人路径，就用getFromNode方法（感觉是终点边的起始节点）得到终点坐标；如果不是，就用getToNode方法（感觉是终点边的结束节点）
                destCoord = sink_edge.getFromNode().getCoord()
            else:
                destCoord = sink_edge.getToNode().getCoord()

            coords = ([source_edge.getFromNode().getCoord()] +
                      [e.getFromNode().getCoord() for e in intermediate] +
                      [destCoord]) # 得到节点列表：起点边的起始坐标、途径边的所有起始节点坐标、终点坐标
            distance = sum([euclidean(p, q)
                            for p, q in zip(coords[:-1], coords[1:])]) # 计算距离：所有节点，按序两两距离相加
            if distance >= min_distance and (max_distance is None or distance < max_distance): # 若符合标准，返回所有相关边
                return source_edge, sink_edge, intermediate
        raise Exception("no trip found after %s tries" % maxtries) # 未找到路径，抛出异常

# get_prob_fun：构造一个边概率函数并返回，输入是刚才构造的参数列表、。返回的函数用来在路径生成中选边，其概率就是选取概率。
def get_prob_fun(options, fringe_bonus, fringe_forbidden):
    # fringe_bonus None generates intermediate way points
    def edge_probability(edge): # 内部定义的函数，将在初始化后被返回
        if options.vclass and not edge.allows(options.vclass): # option边界判断
            return 0  # not allowed
        if fringe_bonus is None and edge.is_fringe() and not options.pedestrians:
            return 0  # not suitable as intermediate way point
        if (fringe_forbidden is not None and edge.is_fringe(getattr(edge, fringe_forbidden)) and
                not options.pedestrians and
                (options.allow_fringe_min_length is None or edge.getLength() < options.allow_fringe_min_length)):
            return 0  # the wrong kind of fringe
        
        prob = 1 # 基础概率
        if options.length:  # 若设置了该策略，则p = p * l，更长的边容易被选择
            prob *= edge.getLength()
        if options.lanes:   # 若设置了该策略，则p = p * lanes，车道更多的边更容易被选择
            prob *= edge.getLaneNumber()
        prob *= (edge.getSpeed() ** options.speed_exponent) # 若设置了该策略，则p = p * v^\mu，其中\mu是速度权重，更快的边容易被选择
        if (options.fringe_factor != 1.0 and # 边缘参数不为1.0，即要对边缘边的选择概率做出调整
                not options.pedestrians and # 不生成行人
                fringe_bonus is not None and # 存在边缘激励
                edge.getSpeed() > options.fringe_threshold and # 边速度大于边缘阈值
                edge.is_fringe(getattr(edge, fringe_bonus))): # 被计算的边是边缘边
            prob *= options.fringe_factor # 若满足上述条件，则p = p * ff
        if options.edgeParam is not None: #  若考虑'edgeParam'参数(?)，就乘上它
            prob *= float(edge.getParam(options.edgeParam, 1.0))
        if options.angle_weight != 1.0 and fringe_bonus is not None: # 考虑边的方向权重，则乘上计算结果
            xmin, ymin, xmax, ymax = edge.getBoundingBox()
            ex, ey = ((xmin + xmax) / 2, (ymin + ymax) / 2)
            nx, ny = options.angle_center
            edgeAngle = naviDegree(math.atan2(ey - ny, ex - nx))
            angleDiff = minAngleDegreeDiff(options.angle, edgeAngle)
            # print("e=%s nc=%s ec=%s ea=%s a=%s ad=%s" % (
            #    edge.getID(), options.angle_center, (ex,ey), edgeAngle,
            #    options.angle, angleDiff))
            # relDist = 2 * euclidean((ex, ey), options.angle_center) / max(xmax - xmin, ymax - ymin)
            # prob *= (relDist * (options.angle_weight - 1) + 1)
            if fringe_bonus == "_incoming":
                # source edge
                prob *= (angleDiff * (options.angle_weight - 1) + 1)
            else:
                prob *= ((180 - angleDiff) * (options.angle_weight - 1) + 1)

        return prob
    return edge_probability

'''
LoadedProps: 加载类，读取前面写入的边权文件，文件里有整个图的边id和边权，写入成员列表weights
'''
class LoadedProps:

    def __init__(self, fname):
        self.weights = defaultdict(lambda: 0)
        for edge in sumolib.output.parse_fast(fname, 'edge', ['id', 'value']):
            self.weights[edge.id] = float(edge.value)

    def __call__(self, edge):
        return self.weights[edge.getID()]

'''
前述RandomTripGenerator类的构建函数，输入网格和参数列表。中间过程是构造类的那几个生成器，生成器都是实例化的RandomEdgeGenerator,
实例化方法就是分别个性化地配置一个随机选边函数来选边。
'''
def buildTripGenerator(net, options):
    try:
        forbidden_source_fringe = None if options.allow_fringe else "_outgoing"
        forbidden_sink_fringe = None if options.allow_fringe else "_incoming"
        source_generator = RandomEdgeGenerator(
            net, get_prob_fun(options, "_incoming", forbidden_source_fringe))
        sink_generator = RandomEdgeGenerator(
            net, get_prob_fun(options, "_outgoing", forbidden_sink_fringe))
        if options.weightsprefix:
            if os.path.isfile(options.weightsprefix + SOURCE_SUFFIX):
                source_generator = RandomEdgeGenerator(
                    net, LoadedProps(options.weightsprefix + SOURCE_SUFFIX))
            if os.path.isfile(options.weightsprefix + SINK_SUFFIX):
                sink_generator = RandomEdgeGenerator(
                    net, LoadedProps(options.weightsprefix + SINK_SUFFIX))
    except InvalidGenerator:
        print("Error: no valid edges for generating source or destination. Try using option --allow-fringe",
              file=sys.stderr)
        return None

    try:
        via_generator = RandomEdgeGenerator(
            net, get_prob_fun(options, None, None))
        if options.weightsprefix and os.path.isfile(options.weightsprefix + VIA_SUFFIX):
            via_generator = RandomEdgeGenerator(
                net, LoadedProps(options.weightsprefix + VIA_SUFFIX))
    except InvalidGenerator:
        if options.intermediate > 0:
            print(
                "Error: no valid edges for generating intermediate points", file=sys.stderr)
            return None
        else:
            via_generator = None

    return RandomTripGenerator(
        source_generator, sink_generator, via_generator, options.intermediate, options.pedestrians)

# 检查属性字符串是否包含与‘walk’相关的属性
def is_walk_attribute(attr):
    for cand in ['arrivalPos', 'speed=', 'duration=', 'busStop=']:
        if cand in attr:
            return True
    return False


# 检查属性字符串是否包含与人员路径相关的属性
def is_persontrip_attribute(attr):
    for cand in ['vTypes', 'modes']:
        if cand in attr:
            return True
    return False


# 检查属性字符串是否包含与人员相关的属性
def is_person_attribute(attr):
    for cand in ['departPos', 'type']:
        if cand in attr:
            return True
    return False


# 检查属性字符串是否包含与车辆相关的属性
def is_vehicle_attribute(attr):
    for cand in ['depart', 'arrival', 'line', 'Number', 'type']:
        if cand in attr:
            return True
    return False


# 切分路径属性的函数，输入包括整个属性字符串、行人选项、属性字符串里面有没有类型属性。
# 结果是将属性字符串切分成四个字符串：车辆类型、车辆属性、人员属性、其他属性
def split_trip_attributes(tripattrs, pedestrians, hasType):
    # handle attribute values with a space
    # assume that no attribute value includes an '=' sign
    allattrs = []
    for a in tripattrs.split():
        if "=" in a:
            allattrs.append(a)
        else:
            if len(allattrs) == 0:
                print("Warning: invalid trip-attribute '%s'" % a)
            else:
                allattrs[-1] += ' ' + a

    # figure out which of the tripattrs belong to the <person> or <vehicle>,
    # which belong to the <vType> and which belong to the <walk> or <persontrip>
    vehicleattrs = []
    personattrs = []
    vtypeattrs = []
    otherattrs = []
    for a in allattrs:
        if pedestrians:
            if is_walk_attribute(a) or is_persontrip_attribute(a):
                otherattrs.append(a)
            elif is_person_attribute(a):
                personattrs.append(a)
            else:
                vtypeattrs.append(a)
        else:
            if is_vehicle_attribute(a):
                vehicleattrs.append(a)
            else:
                vtypeattrs.append(a)

    if not hasType:
        if pedestrians:
            personattrs += vtypeattrs
        else:
            vehicleattrs += vtypeattrs
        vtypeattrs = []

    return (prependSpace(' '.join(vtypeattrs)),
            prependSpace(' '.join(vehicleattrs)),
            prependSpace(' '.join(personattrs)),
            prependSpace(' '.join(otherattrs)))

# 在字符串开头：若无空格，添加一个。如果有就不动。
def prependSpace(s):
    if len(s) == 0 or s[0] == " ":
        return s
    else:
        return " " + s


def main(options):
    if options.seed:
        random.seed(options.seed)

    net = sumolib.net.readNet(options.netfile)
    if options.min_distance > net.getBBoxDiameter() * (options.intermediate + 1):
        options.intermediate = int(
            math.ceil(options.min_distance / net.getBBoxDiameter())) - 1
        print(("Warning: setting number of intermediate waypoints to %s to achieve a minimum trip length of " +
               "%s in a network with diameter %.2f.") % (
            options.intermediate, options.min_distance, net.getBBoxDiameter()))

    if options.angle_weight != 1:
        xmin, ymin, xmax, ymax = net.getBoundary()
        options.angle_center = (xmin + xmax) / 2, (ymin + ymax) / 2

    trip_generator = buildTripGenerator(net, options)
    idx = 0

    vtypeattrs, options.tripattrs, personattrs, otherattrs = split_trip_attributes(
        options.tripattrs, options.pedestrians, options.vehicle_class)

    vias = {}

    def generate_one(idx):
        label = "%s%s" % (options.tripprefix, idx)
        try:
            source_edge, sink_edge, intermediate = trip_generator.get_trip(
                options.min_distance, options.max_distance, options.maxtries)
            combined_attrs = options.tripattrs
            if options.fringeattrs and source_edge.is_fringe(source_edge._incoming):
                combined_attrs += " " + options.fringeattrs
            if options.junctionTaz:
                attrFrom = ' fromJunction="%s"' % source_edge.getFromNode().getID()
                attrTo = ' toJunction="%s"' % sink_edge.getToNode().getID()
            else:
                attrFrom = ' from="%s"' % source_edge.getID()
                attrTo = ' to="%s"' % sink_edge.getID()
            via = ""
            if len(intermediate) > 0:
                via = ' via="%s" ' % ' '.join(
                    [e.getID() for e in intermediate])
                if options.validate:
                    vias[label] = via
            if options.pedestrians:
                fouttrips.write(
                    '    <person id="%s" depart="%.2f"%s>\n' % (label, depart, personattrs))
                if options.persontrips:
                    fouttrips.write(
                        '        <personTrip%s%s%s/>\n' % (attrFrom, attrTo, otherattrs))
                elif options.personrides:
                    fouttrips.write(
                        '        <ride from="%s" to="%s" lines="%s"%s/>\n' % (
                            source_edge.getID(), sink_edge.getID(), options.personrides, otherattrs))
                else:
                    fouttrips.write(
                        '        <walk%s%s%s/>\n' % (attrFrom, attrTo, otherattrs))
                fouttrips.write('    </person>\n')
            else:
                if options.jtrrouter:
                    attrTo = ''
                combined_attrs = attrFrom + attrTo + via + combined_attrs
                if options.flows > 0:
                    if options.binomial:
                        for j in range(options.binomial):
                            fouttrips.write(('    <flow id="%s#%s" begin="%s" end="%s" probability="%s"%s/>\n') % (
                                label, j, options.begin, options.end, 1.0 / options.period / options.binomial,
                                combined_attrs))
                    else:
                        fouttrips.write(('    <flow id="%s" begin="%s" end="%s" period="%s"%s/>\n') % (
                            label, options.begin, options.end, options.period * options.flows, combined_attrs))
                else:
                    fouttrips.write('    <trip id="%s" depart="%.2f"%s/>\n' % (
                        label, depart, combined_attrs))
        except Exception as exc:
            print(exc, file=sys.stderr)
        return idx + 1

    with open(options.tripfile, 'w') as fouttrips:
        sumolib.writeXMLHeader(fouttrips, "$Id$", "routes")  # noqa
        if options.vehicle_class:
            fouttrips.write('    <vType id="%s" vClass="%s"%s/>\n' %
                            (options.vtypeID, options.vehicle_class, vtypeattrs))
            options.tripattrs += ' type="%s"' % options.vtypeID
            personattrs += ' type="%s"' % options.vtypeID
        depart = options.begin
        if trip_generator:
            if options.flows == 0:
                while depart < options.end:
                    if options.binomial is None:
                        # generate with constant spacing
                        idx = generate_one(idx)
                        depart += options.period
                    else:
                        # draw n times from a Bernoulli distribution
                        # for an average arrival rate of 1 / period
                        prob = 1.0 / options.period / options.binomial
                        for _ in range(options.binomial):
                            if random.random() < prob:
                                idx = generate_one(idx)
                        depart += 1
            else:
                for _ in range(options.flows):
                    idx = generate_one(idx)

        fouttrips.write("</routes>\n")

    # call duarouter for routes or validated trips
    args = [DUAROUTER, '-n', options.netfile, '-r', options.tripfile, '--ignore-errors',
            '--begin', str(options.begin), '--end', str(options.end), '--no-step-log']
    if options.additional is not None:
        args += ['--additional-files', options.additional]
    if options.carWalkMode is not None:
        args += ['--persontrip.transfer.car-walk', options.carWalkMode]
    if options.walkfactor is not None:
        args += ['--persontrip.walkfactor', options.walkfactor]
    if options.remove_loops:
        args += ['--remove-loops']
    if options.vtypeout is not None:
        args += ['--vtype-output', options.vtypeout]
    if options.junctionTaz:
        args += ['--junction-taz']
    if not options.verbose:
        args += ['--no-warnings']
    else:
        args += ['-v']

    if options.routefile:
        args2 = args + ['-o', options.routefile]
        print("calling ", " ".join(args2))
        subprocess.call(args2)

    if options.validate:
        # write to temporary file because the input is read incrementally
        tmpTrips = options.tripfile + ".tmp"
        args2 = args + ['-o', tmpTrips, '--write-trips']
        print("calling ", " ".join(args2))
        subprocess.call(args2)
        os.remove(options.tripfile)  # on windows, rename does not overwrite
        os.rename(tmpTrips, options.tripfile)

    if options.weights_outprefix:
        trip_generator.source_generator.write_weights(
            options.weights_outprefix + SOURCE_SUFFIX)
        trip_generator.sink_generator.write_weights(
            options.weights_outprefix + SINK_SUFFIX)
        if trip_generator.via_generator:
            trip_generator.via_generator.write_weights(
                options.weights_outprefix + VIA_SUFFIX)

    # return wether trips could be generated as requested
    return trip_generator is not None


if __name__ == "__main__":
    if not main(get_options()):
        sys.exit(1)
