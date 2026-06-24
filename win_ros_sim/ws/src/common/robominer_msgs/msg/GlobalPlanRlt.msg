std_msgs/Header header
uint8 result

uint8 IDLE=0 #待命

uint8 STRAT_PLAN=1 #开始规划
uint8 POINTS_TOO_SHORT=2 #点数太少
uint8 COSTMAP_EMPTY=3 #栅格图为空
uint8 COSTMAP_TIME_OUT=4 #栅格图更新超时
uint8 CLOSE_TO_GOAL=5 #接近目标点
uint8 START_POS_OUT_OF_RANGE=6 #起点超出规划范围
uint8 START_POS_OBSTCALE=7 #起点障碍
uint8 END_POS_OUT_OF_RANGE=8 #终点超出规划范围
uint8 END_POS_OBSTCALE=9 #终点障碍
uint8 WAVEFRONT_UNREACH=10 #wavefront搜索不可达
uint8 EXCEND_TIME_LIMIT=11 #规划超时
uint8 OPEN_LIST_IS_EMPTY=12 #列表为空
uint8 CANCLE_GOAL=13 #取消目标

uint8 OK=14 #规划成功
