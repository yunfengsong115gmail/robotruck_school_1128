#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 车辆质量与路面坡度估计节点
订阅：/vcu/vcu_data、/localization/localization_data
发布：/estimation/mass_slope_est (30Hz)
功能包: estimation
"""
import rospy
import numpy as np
from std_msgs.msg import Header
from robominer_msgs.msg import LocalizationData, VcuData
from estimation.msg import MassSlopeEst

class MassSlopeEKF:
    def __init__(self):
        rospy.init_node('mass_slope_ekf_node', anonymous=False)
        rospy.loginfo("Mass and Slope Estimation Node Started")

        # 车辆参数
        self.ig_1 = 4.504
        self.ig_2 = 2.889
        self.ig_3 = 1.0
        self.i0 = 18.722
        self.wheel_r = 0.83
        self.p = 1.18
        self.A = 18.0
        self.Cd = 6.5
        self.f = 0.02
        self.g = 9.81
        self.t = 0.02  # 采样时间

        # EKF参数
        self.Q = np.diag([10, 10, 1])
        self.R = np.array([[1e-2]])
        self.H = np.array([[1, 0, 0]])
        self.I = np.eye(3)

        # 状态初始化
        init_v = 4.0 / 3.6
        init_m = 84000.0
        init_beta = 0.0
        self.X = np.array([[init_v], [init_m], [init_beta]])
        self.P = np.diag([0.01**2, (2.75e9)**2, 0.1**2])

        # 数据缓存
        self.latest_vcu = None
        self.latest_loc = None
        self.loc_receive_time = rospy.Time(0)  
        self.vcu_receive_time = rospy.Time(0)
        self.data_timeout = 0.1 

        # 订阅与发布
        self.vcu_sub = rospy.Subscriber('/vcu/vcu_data', VcuData, self.vcu_callback, queue_size=10)
        self.loc_sub = rospy.Subscriber('/localization/localization_data', LocalizationData, self.loc_callback, queue_size=10)
        self.est_pub = rospy.Publisher('/estimation/mass_slope_est', MassSlopeEst, queue_size=10)

        # 30Hz定时器触发处理
        self.rate = 30.0
        self.timer = rospy.Timer(rospy.Duration(1/self.rate), self.timer_callback)

    def vcu_callback(self, msg):
        self.latest_vcu = msg
        self.vcu_receive_time = rospy.Time.now()
        # rospy.loginfo("received vcu data")  # 测试消息

    def loc_callback(self, msg):
        self.latest_loc = msg
        self.loc_receive_time = rospy.Time.now()
        # rospy.loginfo("received loc data")

    def timer_callback(self, event):
        current_time = rospy.Time.now()
        loc_delay = (current_time - self.loc_receive_time).to_sec()
        vcu_delay = (current_time - self.vcu_receive_time).to_sec()

        if loc_delay > self.data_timeout:
            rospy.logwarn_throttle(1, f"Localization data timeout (latest {loc_delay:.2f}s ago)")
            return
        if vcu_delay > self.data_timeout:
            rospy.logwarn_throttle(1, f"VCU data timeout (latest {vcu_delay:.2f}s ago)")
            return

        # 提取topic数据
        vcu = self.latest_vcu
        loc = self.latest_loc
        vx_meas = loc.vx
        ax_meas = loc.ax
        engine_torque = vcu.engine_torque
        gear_data = vcu.gear_data
        vehicle_speed = loc.speed

        # 计算驱动力
        if gear_data == 1:
            ig = self.ig_1
        elif gear_data == 2:
            ig = self.ig_2
        elif gear_data == 3:
            ig = self.ig_3
        else:
            ig = 0
        F_drive = engine_torque * ig * self.i0 / self.wheel_r if ig != 0 else 0.0

        # 计算空气阻力
        F_air = 0.5 * self.p * self.Cd * self.A * (vehicle_speed ** 2)

        # 计算纵向合力
        Fx = F_drive + F_air - (ax_meas * self.wheel_r / 6)

        # EKF估计
        self.ekf_predict_update(vx_meas, Fx)

        # 坡度转换
        beta = self.X[2, 0]
        beta_clipped = np.clip(beta, -0.99, 0.99)
        slope_tangent = beta_clipped / np.sqrt(1 - beta_clipped ** 2)

        # 发布
        self.publish_result(slope_tangent)

    def ekf_predict_update(self, v_meas, Fx):
        v = self.X[0, 0]
        m = self.X[1, 0]
        beta = self.X[2, 0]

        # 预测
        F = np.array([
            [1 - self.t * self.p * self.Cd * self.A * v / m,
             self.t * (self.Cd * self.A * self.p * v**2 * self.wheel_r - 2 * Fx * self.wheel_r) / (2 * m**2 * self.wheel_r),
             -self.g * self.t],
            [0, 1, 0],
            [0, 0, 1]
        ])

        f_x = np.array([[
            -self.p * self.Cd * self.A * v**2 / (2 * m) - self.g * beta - self.g * self.f + Fx / m,
            0.0,
            0.0
        ]]).T
        X1 = self.X + self.t * f_x
        P1 = np.dot(np.dot(F, self.P), F.T) + self.Q

        # 更新
        H_T = self.H.T
        denominator = np.dot(np.dot(self.H, P1), H_T) + self.R
        K = np.dot(np.dot(P1, H_T), np.linalg.inv(denominator))

        Z = np.array([[v_meas]])
        residual = Z - np.dot(self.H, X1)
        self.X = X1 + np.dot(K, residual)
        self.P = np.dot((self.I - np.dot(K, self.H)), P1)

        # 估计范围物理约束
        self.X[1, 0] = np.clip(self.X[1, 0], 61000, 152000)
        self.X[2, 0] = np.clip(self.X[2, 0], -0.3, 0.3)

    def publish_result(self, slope_tangent):
        msg = MassSlopeEst()
        msg.header.stamp = rospy.Time.now()
        msg.mass = float(self.X[1, 0])
        msg.slope = float(slope_tangent)
        self.est_pub.publish(msg)

        # rospy.loginfo_throttle(0.5, f"Estimation: Mass={self.X[1,0]:.0f}kg, Slope(tan)={slope_tangent:.4f}")

if __name__ == "__main__":
    try:
        node = MassSlopeEKF()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mass and Slope Estimation Node Shutdown")
