uint8 status
VehicleError[] errors

uint8 INIT=0
uint8 CHKING=1

uint8 OK=2 #为兼容之前枚举定义,后续会取消该值，改为下面自动驾驶的各种具体状态

uint8 WARN=3 #为兼容之前枚举定义
uint8 ERROR=4 #为兼容之前枚举定义

uint8 IDLE=10
uint8 PREPARE=11 #下载资源等操作，为出发做准备
uint8 DRIVING=12 #行驶中
uint8 CUSP=13 #人字形倒车点
uint8 PAUSE=14 #暂停，通常是中控发了暂停
uint8 LOADING=15 #装载中
uint8 DUMPING=16 #卸料中
uint8 DUMPED=17 #卸料完成，预留
uint8 QUEUING=18 #排队中, 预留
uint8 OBSTACLE=19 #遇障停车

uint8 TELE=100

uint8 MANU=200


#uint8 ERROR_STOP=100 #异常导致的停车停车