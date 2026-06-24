std_msgs/Header                 header

################ 3D BB
geometry_msgs/Pose              pose  # 预测的障碍物的位置姿态
# geometry_msgs/PointStamped      centroid_point   #障碍物的质心点
# geometry_msgs/Vector3           dimensions    #障碍物的长宽高
# geometry_msgs/Twist             velocity      #障碍物的速度
# geometry_msgs/Twist             acceleration  #障碍物的加速度

# sensor_msgs/PointCloud2         pointcloud    #障碍物的点云

geometry_msgs/PolygonStamped    convex_hull   #预测位置处的障碍物的凸包
float32 predicted_time  # 表示从当前时间开始，多少秒以后的时间点,单位秒
float64 estimated_angle   # 估计的车头朝向，单位radian,弧度
float64 estimated_radius    # 预测的半径，值越大，说明轨迹越直，预测的可信度越高
