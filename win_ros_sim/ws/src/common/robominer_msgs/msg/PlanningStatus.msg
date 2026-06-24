uint8 status
string msg #FAIL时会附带失败原因描述
uint8 IDLE=0
#uint8 STABAY=1 #V3.5删掉
uint8 PAUSED=2
#uint8 RESTARTED=3  #V3.5删掉
uint8 RUNNING=4
uint8 DONE=5
uint8 FAIL=6
uint8 PAUSING=7
uint8 STOPING=8
uint8 OBSTACLE=9
uint8 CUSP=10 #人字形倒车的顶点
uint8 PLANNING=11 #车停止后，并且全局规划未出现出结果时，给出规划中状态
uint8 STOP_YIELD=12  #车辆主动停车让行，车辆能够前进，只是优先级低而让行
uint8 PASS_YIELD=13  #车辆被动停车让行，用于多车之间调度协调，表示当前车辆被其他车辆阻挡无法前进