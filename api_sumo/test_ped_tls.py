activeRequest = False # 行人是否有请求
greenTimeSoFar = 0  # 绿灯亮了多长时间

WALKINGAREAS = [':A0_w0', ':A0_w1', ':A0_w2', ':A0_w3'] # N, E, S, W 行人分别区域
CROSSINGS = [':A0_c0', ':A0_c1', ':A0_c2', ':A0_c3'] # N, E, S, W 斑马线


TLSID = self._traci.trafficlight.getIDList()[0] # 交通灯id
MIN_GREEN_TIME = 10 # 最小绿灯时长
if not activeRequest: 
    activeRequest = checkWaitingPersons(WALKINGAREAS) # 看各个区域有没有人在等

greenTimeSoFar = greenTimeSoFar + traci.simulation.simualtionstep() # 绿灯时长的增加

if greenTimeSoFar > MIN_GREEN_TIME and activeRequest: # 如果有请求且绿灯时长达到最小要求，转变交通灯的相位
    traci.trafficlight.setRedYellowGreenState(
        TLSID, 'GGrrrrrrGGrrrrrrGGrrrrrrGGrrrrrrGGGGrr') # 灯组状况  一个r代表一个红灯，一个G代表一个绿灯？

    activeRequest = False
    greenTimeSoFar = 0

else:
    traci.trafficlight.setRedYellowGreenState(
        TLSID, 'rrGGGGGGrrGGGGGGrrGGGGGGrrGGGGGGrrrrGG') # 灯组状况

def checkWaitingPersons(WALKINGAREAS): # 检查有没有人要过街
    for edge in WALKINGAREAS:
        peds = traci.edge.getLastStepPersonIDs(edge)
        print(peds)
        for ped in peds:
            if (traci.person.getWaitingTime(ped) >= 1 and
                    traci.person.getNextEdge(ped) in CROSSINGS):
                numWaiting = traci.trafficlight.getServedPersonCount(TLSID, 2)
                print("%s: pedestrian %s pushes the button (waiting: %s)" %
                      (traci.simulation.getTime(), ped, numWaiting))
                return True
    return False
