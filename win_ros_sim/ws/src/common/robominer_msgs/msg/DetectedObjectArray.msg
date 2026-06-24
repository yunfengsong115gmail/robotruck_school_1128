std_msgs/Header header
uint8 sensor_status  #传感器状态 0:all ok, 1:左雷达出问题, 2: 右雷达出问题， 4：中间雷达出问题， 8：后雷达出问题，16：前毫米波问题，32：后毫米波问题
DetectedObject[] objects    #障碍物列表
PredictedObject[] self_predicted_objects    #运动预测的位置列表