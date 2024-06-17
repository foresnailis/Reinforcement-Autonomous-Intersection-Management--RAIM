"""
返回值：生成车辆行进路径的边集合
起点(i1, j1)
终点(i2, j2)
(ps: 左上角(0,0); 右下角(row-1, col-1))
straight: 是否直行
pedestrians: 行人/车辆
"""
def get_edges(i1,j1,i2,j2,rows,cols,straight,pedestrians):
    edgef = '{}.{}/{}.{}'.format
    edgef_ped_w = ':{}.{}_w{}'.format
    edgef_ped_c = ':{}.{}_c{}'.format

    # 存储车辆行进路径的边集合
    s = []
    # 车辆模式
    if pedestrians == False:
        # 直行
        if straight == True:
            # 不变道
            if i1==i2: # 东西走向
                i = j2-j1 if j2>j1 else j1-j2
                d = 1 if j2 > j1 else -1
                s.append(edgef(i1,j1,i1,j1+d))
                for j in range(1,i):
                    s.append(edgef(i1,j1+d*j,i1,j1+(1+j)*d))
            elif j1==j2: # 南北走向
                j = i2-i1 if i2>i1 else i1-i2
                d = 1 if i2 > i1 else -1
                s.append(edgef(i1,j1,i1+d,j1))
                for i in range(1,j):
                    s.append(edgef(i1+d*i,j1,i1+d*(i+1),j1))
            else: # 变道
                # 车辆位于车道最右侧，先变道再直行
                # 否则先直行再变道
                print("不考虑")
                # if i1 == 0 or i1 == rows-1: 
                #     s.expand(getRoute(i1,j1,i2,j1,nrows,ncols).expand(getRoute(i2,j1,i2,j2,nrows,ncols)))
                # else:
                #     s.expand(getRoute(i1,j1,i1,j2,nrows,ncols).expand(getRoute(i1,j2,i2,j2,nrows,ncols)))
        else:
            # 南北走向来车，右转
            # 先直走，再直角转弯，直走
            if i1 == 0 or i1 == rows-1:
                d = 1 if i1 == 0 else -1

                prev_i = i1
                prev_j = j1

                for i in range(i1,i2,d):
                    s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
    #                print(prev_i,prev_j,prev_i+d,prev_j)
                    prev_i += d
                for i in range(j1,j2,-d):
                    s.append(edgef(prev_i,prev_j,prev_i,prev_j-d))
    #                print(prev_i,prev_j,prev_i,prev_j-d)
                    prev_j -= d

            # 东西走向来车，右转
            # 先直走，再直角转弯，直走
            elif j1 == 0 or j1 == cols-1:
                d = 1 if j1 == 0 else -1

                prev_i = i1
                prev_j = j1

                for i in range(j1,j2,d):
                    s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
    #                print(prev_i,prev_j,prev_i,prev_j+d)
                    prev_j += d

                for i in range(i1,i2,d):
                    s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
    #                print(prev_i,prev_j,prev_i+d,prev_j)
                    prev_i += d
    else: # 行人模式
        if straight == True:

            # 东西走向
            if i1==i2:
                d = 1 if j1 == 0 else -1
                """
                edgef_ped_w的第三个参数代表路人行走路径的边缘
                可取值0,1,2,3, 分别代表四个错开的在斑马线上行走的行人
                    23
                12      03
                    01
                edgef_ped_c尚不清楚什么意思
                """
                w = 1 if d == -1 else 3
                prev_i = i1
                prev_j = j1
                for i in range(j1,j2-d,d):
                    s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w))
                    s.append(edgef_ped_c(prev_i,prev_j+d,w-1))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w-1))
                    prev_j += d
                s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))

            # 南北走向
            else:
                d = 1 if i1 == 0 else -1
                w0 = 0 if d == 1 else 2
                w1 = 3 if d == 1 else 1
                prev_i = i1
                prev_j = j1
                for i in range(i1,i2-d,d):
                    s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w0))
                    s.append(edgef_ped_c(prev_i+d,prev_j,w1))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w1))
                    prev_i += d
                s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
        else: # 拐弯
            # NW
            if i1 == 0 and j2 == 0:
                d = 1
                w0 = 0
                w1 = 3
                prev_i = i1
                prev_j = j1

                for i in range(i1,i2-d,d):
                    s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w0))
                    s.append(edgef_ped_c(prev_i+d,prev_j,w1))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w1))
                    prev_i += d

                s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
                s.append(edgef_ped_w(prev_i+d,prev_j,w0))

                prev_i = prev_i+d
                prev_j = prev_j
                d = -1
                w = 1

                for i in range(j1,j2-d,d):
                    s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w))
                    s.append(edgef_ped_c(prev_i,prev_j+d,w-1))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w-1))
                    prev_j += d
                s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))

            # SE
            elif i1 == rows-1 and j2 == cols-1:
                d = -1
                w0 = 2
                w1 = 1
                prev_i = i1
                prev_j = j1

                for i in range(i1,i2-d,d):
                    s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w0))
                    s.append(edgef_ped_c(prev_i+d,prev_j,w1))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w1))
                    prev_i += d
                s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
                s.append(edgef_ped_w(prev_i+d,prev_j,w0))

                prev_i = prev_i+d
                prev_j = prev_j
                d = 1
                w = 3

                for i in range(j1,j2-d,d):
                    s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w))
                    s.append(edgef_ped_c(prev_i,prev_j+d,w-1))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w-1))
                    prev_j += d
                s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))

            # WS
            elif j1 == 0 and i2 == rows-1:
                d = 1
                w = 3
                prev_i = i1
                prev_j = j1

                for i in range(j1,j2-d,d):
                    s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w))
                    s.append(edgef_ped_c(prev_i,prev_j+d,w-1))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w-1))
                    prev_j += d
                s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
                s.append(edgef_ped_w(prev_i,prev_j+d,w))

                prev_i = prev_i
                prev_j = prev_j+d
                d = 1
                w0 = 0
                w1 = 3

                for i in range(i1,i2-d,d):
                    s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w0))
                    s.append(edgef_ped_c(prev_i+d,prev_j,w1))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w1))
                    prev_i += d

                s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))

            # EN
            elif j1 == cols-1 and i2 == 0:
                d = -1
                w = 1
                prev_i = i1
                prev_j = j1


                for i in range(j1,j2-d,d):
                    s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w))
                    s.append(edgef_ped_c(prev_i,prev_j+d,w-1))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w-1))
                    prev_j += d
                s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
                s.append(edgef_ped_w(prev_i,prev_j+d,w))

                prev_i = prev_i
                prev_j = prev_j+d
                d = -1
                w0 = 2
                w1 = 1

                for i in range(i1,i2-d,d):
                    s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w0))
                    s.append(edgef_ped_c(prev_i+d,prev_j,w1))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w1))
                    prev_i += d

                s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))

            # NE
            elif i1 == 0 and j2 == cols-1:
                d = 1
                w0 = 0
                w1 = 3
                prev_i = i1
                prev_j = j1

                for i in range(i1,i2-1,d):
                    s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w0))
                    s.append(edgef_ped_c(prev_i+d,prev_j,w1))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w1))
                    prev_i += d
                s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
                s.append(edgef_ped_w(prev_i+d,prev_j,w0))
                s.append(edgef_ped_c(prev_i+d,prev_j,w1))

                prev_i = prev_i+d
                prev_j = prev_j
                d = 1
                w = 3

                for i in range(j1,j2,d):
                    s.append(edgef_ped_w(prev_i,prev_j,w))
                    s.append(edgef_ped_c(prev_i,prev_j,w-1))
                    s.append(edgef_ped_w(prev_i,prev_j,w-1))
                    s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
                    prev_j += d

            # SW
            elif i1 == rows-1 and j2 == 0:
                d = -1
                w0 = 2
                w1 = 1
                prev_i = i1
                prev_j = j1


                for i in range(i1,i2-d,d):
                    s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w0))
                    s.append(edgef_ped_c(prev_i+d,prev_j,w1))
                    s.append(edgef_ped_w(prev_i+d,prev_j,w1))
                    prev_i += d

                s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
                s.append(edgef_ped_w(prev_i+d,prev_j,w0))
                s.append(edgef_ped_c(prev_i+d,prev_j,w1))

                prev_i = prev_i+d
                prev_j = prev_j
                d = -1
                w = 1

                for i in range(j1,j2,d):
                    s.append(edgef_ped_w(prev_i,prev_j,w))
                    s.append(edgef_ped_c(prev_i,prev_j,w-1))
                    s.append(edgef_ped_w(prev_i,prev_j,w-1))
                    s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
                    prev_j += d

            # WN
            elif j1 == 0 and i2 == 0:
                d = 1
                w = 3
                prev_i = i1
                prev_j = j1

                for i in range(j1,j2-d,d):
                    s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w))
                    s.append(edgef_ped_c(prev_i,prev_j+d,w-1))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w-1))
                    prev_j += d
                s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
                s.append(edgef_ped_w(prev_i,prev_j+d,w))
                s.append(edgef_ped_c(prev_i,prev_j+d,w-1))

                prev_i = prev_i
                prev_j = prev_j+d
                d = -1
                w0 = 2
                w1 = 1

                for i in range(i1,i2,d):
                    s.append(edgef_ped_w(prev_i,prev_j,w0))
                    s.append(edgef_ped_c(prev_i,prev_j,w1))
                    s.append(edgef_ped_w(prev_i,prev_j,w1))
                    s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
                    prev_i += d

            # ES
            elif j1 == cols-1 and i2 == rows-1:
                d = -1
                w = 1
                prev_i = i1
                prev_j = j1

                for i in range(j1,j2-d,d):
                    s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w))
                    s.append(edgef_ped_c(prev_i,prev_j+d,w-1))
                    s.append(edgef_ped_w(prev_i,prev_j+d,w-1))
                    prev_j += d
                s.append(edgef(prev_i,prev_j,prev_i,prev_j+d))
                s.append(edgef_ped_w(prev_i,prev_j+d,w))
                s.append(edgef_ped_c(prev_i,prev_j+d,w-1))

                prev_i = prev_i
                prev_j = prev_j+d
                d = 1
                w0 = 0
                w1 = 3

                for i in range(i1,i2,d):
                    s.append(edgef_ped_w(prev_i,prev_j,w0))
                    s.append(edgef_ped_c(prev_i,prev_j,w1))
                    s.append(edgef_ped_w(prev_i,prev_j,w1))
                    s.append(edgef(prev_i,prev_j,prev_i+d,prev_j))
                    prev_i += d

    return ' '.join(s)