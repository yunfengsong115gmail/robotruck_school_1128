std_msgs/Header header

float32 dist_long   #目标纵坐标
float32 dist_lat    #目标横坐标
float32 vrel_long   #纵向相对速度
float32 vrel_lat    #横向相对速度
uint8 dyn_prop      #运动状态 0x0:moving  0x1:stationary  0x2:oncoming  0x3:stationary candidate  0x4:unknown  0x5:crossing stationary  0x6:crossing moving  0x7:stopped
uint8 class_object  #类别
float32 orientation_angle   #方位角
float32 length      #目标长度
float32 width       #目标宽度
float32 rcs         # 雷达反射面积
uint8 meas_state    # 测量状态：在新的测量周期内 object 是否有效，是否被 clusters 确认。0x0:deleted 0x1:new 0x2:measured 0x3: predicted 0x4:deleted for merge 0x5:new from merge
uint8 prob_of_exist # 存在的概率0x0:invalid  0x1:<25%  0x2:<50%  0x3:<75%  0x4:<90%  0x5:<99%  0x6:<99.9%  0x7:<=100%
