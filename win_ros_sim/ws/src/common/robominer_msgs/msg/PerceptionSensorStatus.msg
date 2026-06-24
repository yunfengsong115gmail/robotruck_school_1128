std_msgs/Header header
uint8 sensor_status  #传感器状态 0:all ok, 1:左雷达出问题, 2: 右雷达出问题， 4：中间雷达出问题， 8：后雷达出问题，16: top雷达出问题，32：前毫米波问题，64：后毫米波问题, 128:顶摄像头出问题
uint8 left_lidar_hz
uint8 right_lidar_hz
uint8 middle_lidar_hz
uint8 top_lidar_hz
uint8 back_lidar_hz
uint8 front_radar_hz
uint8 back_radar_hz
uint8 top_camera_hz
float32 left_right_lidar_angle
float32 left_top_lidar_angle
float32 top_right_lidar_angle