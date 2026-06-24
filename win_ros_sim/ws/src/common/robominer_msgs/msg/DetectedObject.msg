std_msgs/Header                 header

uint32                          id     #障碍物编号
string                          label  #障碍物分类
float32                         score   # 0~1.0, 用来表示障碍物的置信度，值越大，置信度越高
bool                            valid   # 内部用，表示障碍物是否valid
uint8                           object_source  # 内部用，0--lidar, 1--radar, 2--otherdevice

################ 3D BB
geometry_msgs/Pose              pose  # 障碍物的位置姿态
geometry_msgs/PointStamped      centroid_point   #障碍物的质心点
geometry_msgs/Vector3           dimensions    #障碍物的长宽高
geometry_msgs/Twist             velocity      #障碍物的速度
geometry_msgs/Twist             acceleration  #障碍物的加速度

sensor_msgs/PointCloud2         pointcloud    #障碍物的点云

geometry_msgs/PolygonStamped    convex_hull   #障碍物的凸包
#robominer_msgs/LaneArray        candidate_trajectories

bool                            pose_reliable   # 位姿是否可靠
bool                            velocity_reliable    # 速度是否可靠
bool                            acceleration_reliable   # 加速度是否可靠

float32 height_from_ground      # 障碍物离地面的最高点高度
float32 minpoint_height_from_ground   # 障碍物离地面的最低点高度
int32                           lifeCount   #被观测次数

PredictedObject[] predicted_objects    #运动预测的位置列表
bool    in_same_lane   		# 对于vehicle类型，告知是否在同一条道上，如果是在同一条道上，可以不作predicted_objects的碰撞检测，避免运输效率的降低
string    overlapped_other_device_id   	# 对于vehicle类型，告知是否是和other device重叠，如果重叠，填上other device的device_sn，碰撞检测可以根据任务状态决定用predicted_objects或者other device的local path

float64 estimated_angle
float32 distance_to_center

#### 下面的字段暂时不用 ####
############### 2D Rect
string                          image_frame # Image coordinate Frame,        Required if x,y,w,h defined
int32                           x           # X coord in image space(pixel) of the initial point of the Rect
int32                           y           # Y coord in image space(pixel) of the initial point of the Rect
int32                           width       # Width of the Rect in pixels
int32                           height      # Height of the Rect in pixels
float32                         angle       # Angle [0 to 2*PI), allow rotated rects

sensor_msgs/Image               roi_image

############### Indicator information
uint8                          indicator_state # INDICATOR_LEFT = 0, INDICATOR_RIGHT = 1, INDICATOR_BOTH = 2, INDICATOR_NONE = 3

############### Behavior State of the Detected Object
uint8                           behavior_state # FORWARD_STATE = 0, STOPPING_STATE = 1, BRANCH_LEFT_STATE = 2, BRANCH_RIGHT_STATE = 3, YIELDING_STATE = 4, ACCELERATING_STATE = 5, SLOWDOWN_STATE = 6

#
string[]                        user_defined_info

std_msgs/ColorRGBA              color   # Define this object specific color
string                          space_frame #3D Space coordinate frame of the object, required if pose and dimensions are defines

geometry_msgs/Vector3           variance

uint8 vehicle_drive_mode        # 车辆驾驶模式 0 "人工驾驶"; 2 "自动驾驶"
