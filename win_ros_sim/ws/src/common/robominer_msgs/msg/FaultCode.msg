std_msgs/Header header

FaultLevel fault_level #故障等级 可不传

uint8 fault_code #故障码 必传
uint8 fault_action #故障动作 必传

string fault_msg #附带的描述信息，可不传

FaultModule fault_module #can_driver模块专用 vcu故障模块标识

###故障上报动作描述
uint8 FAULT_ADD = 1      #新增持续类故障，当故障消失时，需要撤销
uint8 FAULT_RESET = 2    #撤销持续性故障
uint8 MSG_ADD = 3        #上报一条顺时故障，不需要撤销，指明当前由于什么原因动作执行失败

#上报故障时，可只上传故障码，故障等级可传可不传
#必须上传动作
#