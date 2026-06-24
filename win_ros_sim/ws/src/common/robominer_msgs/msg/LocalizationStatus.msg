uint8 status              #定位状态

uint8 RTKINIT=0             #定位不可用
uint8 APOSIT=1               #单点定位
uint8 RTCMDIFF=2          #差分定位
uint8 NOTUSED=3          #定位不可用
uint8 RTKFIX=4                #固定解
uint8 RTKFLO=5                #浮点解
uint8 RTKOTHER=6         #航迹推算
uint8 DATATIMOUT=11      #其它