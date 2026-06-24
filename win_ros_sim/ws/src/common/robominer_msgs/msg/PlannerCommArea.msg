std_msgs/Header header      #消息构建时间戳
int32 area_type             #区域类型，默认无效区域，与云端保持一致
int32 AREA_NONE     =0      #无效区域
int32 AREA_MAIN     =1      #主干道
int32 AREA_DUMP     =2      #装料区
int32 AREA_LOAD     =3      #卸料区
int32 AREA_CROSS    =4      #会车区
int32 AREA_ENDALL   =5      #末端全检测
int32 AREA_GOAL     =6      #目标点全检测
int32 AREA_OBST     =7      #障碍全检测区域
int32 AREA_RUT      =8      #限速障碍忽略区
int32 AREA_SPEED    =9      #限速区
int32 AREA_SLOPEUP  =10     #上坡区域
int32 AREA_SLOPEDOWN=11     #下坡区域
int32 AREA_QUEUE    =12     #排队区域
int32 AREA_ENDREVERSE=13    #末端倒车段
int32 AREA_REVERSEPOINT=14  #人字形顶点障碍忽略区域
int32 AREA_COMMON_AREA=15   #云端规划通用功能区
int32 AREA_FORBIDDEN=16     #云端规划禁停区
int32 AREA_PARKING  =17     #云端规划停车区
int32 AREA_WAITING  =18     #云端规划等待区
int32 AREA_AUTOCROSS=19     #规划自动会车区
int32 AREA_WASTEDUSMP=20     #云端规划排土场区
int32 AREA_MAX      =21     #区域类型最大值

bool load_active            #装料任务区域生效标志 默认true生效
bool dump_active            #卸料任务区域生效标志 默认true生效

string name                 #区域名称，和云端保持一直
string id                   #区域资源id，和云端保持一直
string version              #资源版本号，和云端保持一直

geometry_msgs/Polygon poly  #区域多边形,和云端保持一直
geometry_msgs/Polygon buff  #区域入口多边形,车端自动计算

#path index: 0...startIndex...enterIndex...exitIndex...end
int32 startIndex            #进入区域之前缓冲区的全局路径索引点
int32 enterIndex            #进入区域的全局路径索引点
int32 exitIndex             #退出区域的全局路径索引点

# startIndex enterIndex exitIndex在单边交叉时有用
# 禁停区存在多次进出，所以设计一个树组结构，后续其他区域也会保存多次进出的结果
int32[] start_idxs      #进入区域之前缓冲区的全局路径索引点
int32[] enter_idxs      #进入区域的全局路径索引点
int32[] exit_idxs       #退出区域的全局路径索引点

float64 ignore_height       #本区域障碍忽律高度,默认0.0 不启用障碍忽略，和云端保持一直
bool obstacle_active        #本区域遇障状态，车端动态数据
time obstacle_active_tm     #本区域遇障状态最后起效时间，车端动态数据

float64 speed_limit         #本区域限速默认值，默认0.0 不限速 和云端保持一直

float64 stop_height         #本区域超过高度则阻止进入,默认0.0 不启用障碍阻止 和云端保持一直
time stop_height_tm         #障碍阻止最后起效时间，车端动态数据

bool replan                 #本区域是否允许绕障 换道行驶 默认false 不允许 和云端保持一直
float64 replan_duration     #允许换道行驶触发间隔 默认5秒，车端配置参数

float64 obst_stop_distance  #本区域障碍停止距离,默认15米，车端配置参数
float64 obst_restart_distance #本区域障碍再次起步距离,默认18米，车端配置参数

bool area_pause             #区域是否时是暂停状态，车端动态数据，来自云端区域暂停指令
time area_pause_tm          #区域暂停最后起效时间，车端动态数据

bool navi_broadcast         #如果true，表示进入区域并且播放过导航信息,默认false ，车端动态数据

float64 ignore_height_upboard #本区域中外框忽略高度 车端动态数据

GearData gear_data          #锁档限制 和云端保持一直
bool limit_up               #是否上坡限速 和云端保持一直
bool limit_down             #是否下坡限速 和云端保持一直

float64 fs_limit        #区域纵差限制，车端配置参数
float64 fd_limit        #区域横差限制，车端配置参数