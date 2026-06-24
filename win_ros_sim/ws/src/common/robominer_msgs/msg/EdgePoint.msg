#与道路边界点提取相关
std_msgs/Header header

geometry_msgs/Point[] rectanglePoints           #最大的长方形方框的4个顶点坐标
geometry_msgs/Point[] edgeSelectedPoints        #存储在最大的长方形方框内的点云

geometry_msgs/Point[] blockSelectedPoints       #聚类点个数需要显示的位置
uint8[] blockNum                                #聚类点的个数
bool[] valid                                    #表明当前聚类是否有效，为1表明有效，为0表示无效
uint8[] recPointsNum                            #聚类周围绘制一个小的正方形，这个正方形方框内点云的个数

geometry_msgs/Point[] blockPointCoordinate      #聚类周围绘制一个小的正方形，这个小正方形的顶点坐标
geometry_msgs/Point[] edgeResultPoints          #最终的路径边界点

