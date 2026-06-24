# 矿区2.5D高度地图，由定位高度数据10点滑动均值得到，大约为5Hz，珊格精度为20cm
std_msgs/Header header #header.stamp 地图最后一次更新高度时间
geometry_msgs/Point origin_blh #矿区原点纬经高
nav_msgs/MapMetaData info #info.origin 表示本子图的左下角当地东北天坐标系
float64[] data