std_msgs/Header header

FaultCode[] fault_codes #故障码列表



# 使用说明：
# topic名称：/fault_code/xxx_fault xxx为localization、can_driver、guardian、planner、control、perception
# 上报规则：只上报变化
# 每次只上报新增或者删除的故障信息