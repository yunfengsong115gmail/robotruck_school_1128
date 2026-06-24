std_msgs/Header header

float64 working_time
float64 rtk_diffage   #差分龄期
float64 gnssant_feedV   #天线馈电电压检测

uint8 utc_status
uint8 pps_status
uint8 ins_solstatus  #INS 结算状态
uint8 gnss_status  #GNSS 定位检查 惯导正常工作且双天线有查分时为0000000
uint8 gnssant_shortcheck #天线短路检测 0：正常   1：该次上电发生短路
uint8 ws_check        #轮速CAN信号检查:0:接入正常 1:未接入，无效或超时
uint8 SatsNum_Status  #双天线搜星状态检测:0:正常 1: 副天线搜星不可用2:主天线定位搜星不可用
LocalizationStatus imu_status