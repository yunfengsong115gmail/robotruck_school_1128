uint8 rlt

uint8 OK=0
uint8 COMMON_FAIL=1

uint8 WRONG_STATUS=2            #call startPlan时plan的当前状态不对
uint8 JUST_PLANNING=3           #仍在处理上一次startPlan
uint8 LOAD_PATHFILE_FAIL=4      #加载路径文件失败
uint8 GOAL_TOO_NEAR=5           #终点距离当前位置太近
uint8 LAST_GOAL_RUNNING=6       #上次目标仍在运行
uint8 STOP_AREA_UPFAIL=7        #停车区不更新目标点位姿
