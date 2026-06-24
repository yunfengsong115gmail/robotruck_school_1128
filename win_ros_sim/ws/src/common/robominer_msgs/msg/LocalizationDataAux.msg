std_msgs/Header header

uint8 ins_status #惯性导航状态  
                # 0 INS_INACTIVE 对准未激活
                # 1 INS_ALIGNING 正在进行粗对准
                # 2 INS_HIGH_VARIANCE 较高协方差，姿态估计未收敛
                # 3 WAITING_AZIMUTH 等待航向角
                # 4 INS_SOLUTION_FREE 卫星结果较差不可用
                # 5 INS_ALIGNMENT_COMPLETE 粗对准完成
                # 6 DETERMINING_ORIENTATION 正在确定 IMU 轴与重力对齐
                # 7 WAITING_INITIALPOS 等待位置解
                # 8 INS_SOLUTION_GOOD 对准完成结果较好 
                # 9 INITIALIZING_BIASES 在静态数据前 10 秒内估计初始偏差
                # 10 MOTION_DETECT 尚未完全对准，但已检测到运动
uint8 pos_type   #定位状态
                # 0 NONE 未解算
                # 1 INS_PSRSP 伪距单点解
                # 2 INS_PSRDIFF 伪距差分
                # 3 INS_RTKFLOAT RTK浮点解
                # 4 INS_RTKFIXED RTK固定解


float64 heading_length  #两天线之间的距离
float64 angle_difference #航迹角与航向角的角度差 当天线位置变动那这个定值就会变化，按照变动幅度大小设定阈值为5度较为合适
float64 undulation    #高程异常值