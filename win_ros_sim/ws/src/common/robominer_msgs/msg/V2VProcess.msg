uint8 at
std_msgs/Time stamp     # 状态变换时间
uint8 v2v_status_idle=0 # 自车与他车路线不相交
uint8 v2v_crossing=1    # 自车与他车路线相交，但不是跟车模式
uint8 v2v_following=2   # 自车与他车路线相交，自车在跟车模式
uint8 v2v_followed=3    # 自车与他车路线相交，他车在跟车模式
uint8 v2v_stopyield=4   # 自车在停车让行状态
