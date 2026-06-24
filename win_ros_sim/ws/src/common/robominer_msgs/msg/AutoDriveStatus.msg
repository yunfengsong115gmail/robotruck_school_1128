uint8 status
# 0 "idle"; 1 "driving"; 2 "loading"; 3 "dumping"; 4 "waiting"
# 修改
uint8 IDLE=0#待命
uint8 READY=1#准备出发
uint8 DRIVING=2#运行中
uint8 LOADING=3#装载中
uint8 PREPAREDUMP=4#待卸料
uint8 DUMPING=5#卸料中
uint8 WAITING=6#排队中
uint8 OBSTACLE=7#遇障停车
uint8 PAUSE=8#任务暂停
uint8 ERROR=9#任务被打断