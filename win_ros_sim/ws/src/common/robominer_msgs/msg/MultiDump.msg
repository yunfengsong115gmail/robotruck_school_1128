#
std_msgs/Header header

MultiDumpCell[] multi_dump_cell
MultiDumpPoint[] multi_dump_point
geometry_msgs/Point[] points        #每一瓣中最终符合条件的点

geometry_msgs/Point center_point    #通过360度整圆中的esdf障碍物计算得到中心点的xy坐标
float64 divide_angle                #每一瓣的角度大小