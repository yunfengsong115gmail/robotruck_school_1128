std_msgs/Header header

std_msgs/Header startPlan
std_msgs/Header stopPlan
std_msgs/Header pausePlan
std_msgs/Header restartPlan
std_msgs/Header updateGoal
std_msgs/Header updatePlan

# plan state machine
uint8 PP_INITIALIZE =0
uint8 PP_IDLE       =1
uint8 PP_PLANNING   =2
uint8 PP_FORWORDING =3
uint8 PP_BACKWORDING=4
uint8 PP_STOPED     =5
uint8 PP_DONE       =6
uint8 PP_STATUS_MAX =7

uint8 processStatus
std_msgs/Time processUptime

#goal state
uint8 GE_INITIALIZE =0
uint8 GE_NEWGOAL    =1
uint8 GE_PLANING    =2
uint8 GE_PLAN_DONE  =3
uint8 GE_RUNNING    =4
uint8 GE_RUN_DONE   =5
uint8 GE_RUN_FAILED =6
uint8 GE_PLAN_FAILED=7
uint8 GE_LOAD_FAILED=8
uint8 GE_STATUS_MAX =9

uint8 moveStatus
std_msgs/Time moveUptime
robominer_msgs/PlanGoal goal
float64 currentFS
float64 currentFD
float64 currentFYaw #axes frame  base is lane

#obstacle state
uint8 obstacleStatus
std_msgs/Time obstacleUptime

robominer_msgs/PlannerObstStatus[] pathObstacle

#pause status
uint8 PE_STATUS_IDLE=0
uint8 PE_PAUSING    =1
uint8 PE_PAUSED     =2
uint8 PE_STATUS_MAX =3

uint8 pauseStatus
std_msgs/Time pauseUptime

#cusp status
uint8 CUSP_STATUS_IDLE=0
uint8 CUSP_PAUSING  =1
uint8 CUSP_PAUSED   =2
uint8 CUSP_STATUS_MAX=3

uint8 cuspStatus
std_msgs/Time cuspUptime

#stop status
uint8 STE_STATUS_IDLE=0
uint8 STE_STOPING   =1
uint8 STE_STOPED    =2
uint8 STE_STATUS_MAX=3

uint8 stopStatus
std_msgs/Time stopUptime

#locate status
uint8 LE_UNINITIAL  =0
uint8 LE_LOCATE_WELL=1
uint8 LE_LOCATE_FAIL=2
uint8 LE_STATUS_MAX =3

uint8 locateStatus
std_msgs/Time locateUptime
float64 currentSpeed
float64 planSpeed

#speed status
uint8 SE_UNINITIAL  =0
uint8 SE_SPEED_RUN  =1
uint8 SE_SPEED_STOP =2
uint8 SE_STATUS_MAX =3

uint8 speedStatus
std_msgs/Time speedUptime

#costmap status
uint8 CE_UNINITIAL  =0
uint8 CE_COSTMAP_WELL=1
uint8 CE_COSTMAP_FAIL=2
uint8 CE_STATUS_MAX =3

uint8 costmapStatus
std_msgs/Time costmapUptime

#logic code status
uint8 LES_VERSION_ERR=0
uint8 LES_FS_ERROR  =1
uint8 LES_FD_ERROR  =2
uint8 LES_PARK_FAIL =3
uint8 LES_ERROR_MAX =4

uint8[] error

#area flags
uint8 LOAD_AREA_FLAG =0          #装料区
uint8 DUMP_AREA_FLAG =1          #卸料区
uint8 MAIN_LANE_FLAG =2          #主干道区
uint8 MAIN_TAIL_FLAG =3          #主干道末尾排队区
uint8 END_REVERSE_FLAG =4        #最后一段倒车路径
uint8 CROSS_PREV_FLAG =5         #会车区前部
uint8 CROSS_LANE_FLAG =6         #会车区
uint8 SCOPE_UP_PREV_FLAG =7      #上坡前
uint8 SCOPE_UP_FLAG =8           #上坡段
uint8 SCOPE_DOWN_PREV_FLAG =9    #下坡前
uint8 SCOPE_DOWN_FLAG =10        #下坡段
uint8 END_ALLCHECK_FLAG =11      # 末尾全程检测
uint8 END_ALLCHECK_PREV_FLAG =12 # 末尾全程检测缓冲段 长度默认3米
uint8 END_IGNORE_FLAG =13        # 末尾忽律段
uint8 CURVE_LEVEL1_FLAG =14      # 1类弯道
uint8 CURVE_LEVEL2_FLAG =15      # 2类弯道
uint8 CURVE_LEVEL3_FLAG =16      # 3类弯道
uint8 CURVE_LEVEL4_FLAG =17      # 4类弯道
uint8 CURVE_LEVEL5_FLAG =18      # 5类弯道
uint8 CURVE_LEVEL6_FLAG =19      # 6类弯道
uint8 CURVE_LEVEL7_FLAG =20      # 7类弯道
uint8 CURVE_LEVEL8_FLAG =21      # 8类弯道
uint8 CURVE_LEVEL9_FLAG =22      # 9类弯道
uint8 CURVE_LEVEL10_FLAG=23      # 10类弯道
uint8 BOARD_SMALLER_FLAG=24      # 边框收窄标志，默认在最后一段距离进行收窄
uint8 AREA_FLAG_MAX =25
