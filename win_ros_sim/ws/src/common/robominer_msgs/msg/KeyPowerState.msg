uint8 key_power_state

uint8 OFF=0     
uint8 ACC=1     #钥匙三个位置（acc,on,start）acc:附件通电档（收音机等）
uint8 ON=2      #on档：全车通电，车辆正常行驶时钥匙会保持在这个位置
uint8 START=3   #start档：启动车辆（松开后，钥匙会自动回到ON档）