#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 车辆参数估计节点
订阅：/vcu/vcu_data、/localization/localization_data
发布：/estimation/veh_params
功能包: estimation
"""
import rospy
import numpy as np
from std_msgs.msg import Header
from robominer_msgs.msg import LocalizationData, VcuData
from estimation.msg import VehParamEst  # 自定义消息


class TruckParams:
    """车辆参数类（空载/满载模式）"""
    def __init__(self, work_mode: str = "empty"):
        self.a_m = 4.2  # 前轴到中轴距离(m)
        self.m_r = 1.9  # 中轴到后轴距离(m)
        self.L = self.a_m + self.m_r  # 总轴距(m)
        self.B_f = 3.184  # 前轮距(m)
        self.B_m = 3.324  # 中轮距(m)
        self.B_r = 3.324  # 后轮距(m)
        self.R = 0.845  # 轮胎半径(m)
        self.g = 9.81  # 重力加速度(m/s²)
        
        if work_mode == "empty":  # 空载参数
            # 初始刚度（作为估计初值）
            self.init_Cf = 1.4e5    # 前轴侧偏刚度
            self.init_Cr = 1.2e5    # 后轴侧偏刚度
            self.init_Cxf = 1.2e5   # 前轴纵向刚度
            self.init_Iz = 6.5e5    # 横摆惯量
            
            # 质心位置
            self.a = 2.2    # 质心到前轴距离
            self.b = self.a_m - self.a  # 质心到中轴距离
            self.c = self.L - self.a    # 质心到后轴距离
            self.h_g = 1.7     # 质心高度
            self.m_total = 61000  # 总质量
            
            # 轴荷比例
            self.axle_load_ratio = {
                'front': 0.573,   
                'middle': 0.213,  
                'rear': 0.213 
            }
        else:  # 满载模式
            # 初始刚度（作为估计初值）
            self.init_Cf = 2.5e5    # 前轴侧偏刚度
            self.init_Cr = 3.6e5    # 后轴侧偏刚度
            self.init_Cxf = 1.6e5   # 前轴纵向刚度
            self.init_Iz = 2.0e6    # 横摆惯量
            
            # 质心位置
            self.a = 3.884    # 质心到前轴距离
            self.b = self.a_m - self.a  # 质心到中轴距离
            self.c = self.L - self.a    # 质心到后轴距离
            self.h_g = 2.2     # 质心高度
            self.m_total = 152000  # 总质量
            
            # 轴荷比例
            self.axle_load_ratio = {
                'front': 38400 / 152000,   # 0.2526
                'middle': 58900 / 152000,  # 0.3875
                'rear': 58900 / 152000     # 0.3875
            }


class VehParamEKF:
    def __init__(self):
        rospy.init_node('veh_param_est', anonymous=False)
        rospy.loginfo("Vehicle Parameter Estimation Node Started")

        # 加载工作模式参数
        self.work_mode = rospy.get_param("~work_mode", "empty")
        self.truck_params = TruckParams(self.work_mode)
        rospy.loginfo(f"Using {self.work_mode} parameters")

        self.xf = self.truck_params.a       # 前轴到质心距离
        self.xm = -self.truck_params.b      # 中轴到质心距离
        self.xr = -self.truck_params.c      # 后轴到质心距离
        self.dt = 0.02                      # 采样时间

        # UKF_TireForces初始化
        self.ukf_xk = np.array([[0.0], [0.0], [0.0], [0.0]])  # [Ff_b; Fm_b; Fr_b; Fx_f]
        self.ukf_Pk = np.diag([1e6, 1e6, 1e6, 1e6])
        self.ukf_K_nom = np.array([
            [self.truck_params.init_Cf],    # Cf初值
            [self.truck_params.init_Cr],    # Cm初值（=Cr）
            [self.truck_params.init_Cr],    # Cr初值
            [self.truck_params.init_Cxf]    # Cxf初值
        ])
        self.ukf_tau_proc = 0.1
        self.ukf_Qk = np.diag([1e5, 1e5, 1e5, 1e5])
        self.ukf_Rk = np.diag([0.5**2, 0.2**2])
        self.ukf_Iz_nom = self.truck_params.init_Iz  
        self.ukf_r_prev = 0.0
        self.ukf_rdot_f = 0.0
        self.ukf_fc_d = 6
        self.ukf_alpha_d = np.exp(-2 * np.pi * self.ukf_fc_d * self.dt)

        # EKF初始化
        self.ekf_S = np.diag([5e5, 5e5, 2e5, 2e5])  # 缩放矩阵
        self.ekf_theta_phys0 = np.array([
            [self.truck_params.init_Cf],    # Cf初值
            [self.truck_params.init_Cr],    # Cr初值
            [self.truck_params.init_Cxf],   # Cxf初值
            [self.truck_params.init_Iz]     # Iz初值
        ])
        # 初始EKF协方差
        self.ekf_P_phys0 = np.diag([
            (0.3 * self.truck_params.init_Cf)**2,
            (0.3 * self.truck_params.init_Cr)** 2,
            (0.3 * self.truck_params.init_Cxf)**2,
            (0.3 * self.truck_params.init_Iz)** 2
        ])
        self.ekf_P_s = np.linalg.inv(self.ekf_S)
        self.ekf_P_s = np.dot(np.dot(self.ekf_P_s, self.ekf_P_phys0), np.linalg.inv(self.ekf_S.T))
        self.ekf_Q_s = np.diag([1e-2*(5e-3)**2, 1e-4*(5e-3)** 2, (5e-3)**2, (1e-4)** 2])
        self.ekf_R_base = np.diag([(1e5)**2, (1e5)** 2, (1e5)**2, 0.2**2, (5e4)** 2])
        self.ekf_theta_s = np.dot(np.linalg.inv(self.ekf_S), self.ekf_theta_phys0)
        self.ekf_r_prev = 0.0
        self.ekf_rdot_f = 0.0
        self.ekf_fc = 6
        self.ekf_alpha_d = np.exp(-2 * np.pi * self.ekf_fc * self.dt)

        # 数据缓存与标志
        self.latest_vcu = None
        self.latest_loc = None
        self.loc_receive_time = rospy.Time(0)  
        self.vcu_receive_time = rospy.Time(0) 
        self.data_ready = False
        self.data_timeout = 0.1

        # 订阅与发布
        self.vcu_sub = rospy.Subscriber('/vcu/vcu_data', VcuData, self.vcu_callback, queue_size=10)
        self.loc_sub = rospy.Subscriber('/localization/localization_data', LocalizationData, self.loc_callback, queue_size=10)
        self.est_pub = rospy.Publisher('/estimation/veh_params', VehParamEst, queue_size=10)

        # 30Hz定时器
        self.rate = 30.0
        self.timer = rospy.Timer(rospy.Duration(1/self.rate), self.timer_callback)

    def vcu_callback(self, msg):
        self.latest_vcu = msg
        self.vcu_receive_time = rospy.Time.now()

    def loc_callback(self, msg):
        self.latest_loc = msg
        self.loc_receive_time = rospy.Time.now()

    def timer_callback(self, event):
        # current_time = rospy.Time.now()
        # loc_delay = (current_time - self.loc_receive_time).to_sec()
        # vcu_delay = (current_time - self.vcu_receive_time).to_sec()

        # if loc_delay > self.data_timeout:
        #     rospy.logwarn_throttle(1, f"Localization data timeout (latest {loc_delay:.2f}s ago)")
        #     return
        # if vcu_delay > self.data_timeout:
        #     rospy.logwarn_throttle(1, f"VCU data timeout (latest {vcu_delay:.2f}s ago)")
        #     return
        # 提取数据
        vcu = self.latest_vcu
        loc = self.latest_loc
        vx = loc.vx
        vy = loc.vy
        r = loc.wz
        ay_meas = loc.ay
        wf = vcu.wheel_speed_fl  # 左前轮速
        delta = vcu.steering_angle  # 前轮转角
        m = self.truck_params.m_total  # 总质量

        # UKF估计轮胎力
        Ff_b, Fm_b, Fr_b, Fx_f = self.ukf_tire_forces(vx, vy, r, ay_meas, wf, delta, m)

        # EKF估计参数（Cf, Cr, Cxf, Iz）
        Cf_est, Cr_est, Cxf_est, Iz_est = self.ekf_params(
            Ff_b, Fm_b, Fr_b, Fx_f, vx, vy, r, ay_meas, wf, delta, m
        )

        # 计算其他参数
        Cm_est = Cr_est  # 中轴侧偏刚度=后轴侧偏刚度
        # 中后轴纵向刚度（基于前轴纵向刚度和轴荷比）
        ratio_front = self.truck_params.axle_load_ratio['front']
        ratio_middle = self.truck_params.axle_load_ratio['middle']
        ratio_rear = self.truck_params.axle_load_ratio['rear']
        Cxm_est = Cxf_est * (ratio_middle / ratio_front)  # 中轴纵向刚度
        Cxr_est = Cxf_est * (ratio_rear / ratio_front)    # 后轴纵向刚度

        # 发布所有参数
        self.publish_result(Cf_est, Cm_est, Cr_est, Cxf_est, Cxm_est, Cxr_est, Iz_est)

    # UKF估计轮胎力
    def ukf_tire_forces(self, vx, vy, r, ay_meas, wf, delta, m):
        # 避免vx为0
        vx_safe = max(abs(vx), 0.1)

        # 计算侧偏角与纵向滑移率
        alpha_f = (vy + self.xf * r) / vx_safe - delta  # 前轴侧偏角
        alpha_m = (vy + self.xm * r) / vx_safe          # 中轴侧偏角
        alpha_r = (vy + self.xr * r) / vx_safe          # 后轴侧偏角

        Vwf = vx * np.cos(delta) + vy * np.sin(delta)   # 前轴地速
        Vwf_safe = np.sign(Vwf) * max(abs(Vwf), 0.1)
        s_f = (wf - Vwf) / Vwf_safe                     # 前轴纵向滑移率

        # 计算rdot_meas（低通滤波差分）
        dr_raw = (r - self.ukf_r_prev) / self.dt
        self.ukf_rdot_f = self.ukf_alpha_d * self.ukf_rdot_f + (1 - self.ukf_alpha_d) * dr_raw
        self.ukf_r_prev = r
        rdot_meas = self.ukf_rdot_f

        # UKF参数配置
        n = 4  # 状态维度
        kappa = 0
        alpha_ukf = 1e-3
        beta = 2
        lam = alpha_ukf**2 * (n + kappa) - n
        gamma = np.sqrt(n + lam)
        Wm = np.array([lam/(n+lam)] + [1/(2*(n+lam))]*(2*n))  # 均值权重
        Wc = Wm.copy()
        Wc[0] += (1 - alpha_ukf**2 + beta)  # 协方差权重

        # 生成sigma点
        S = np.linalg.cholesky(self.ukf_Pk + 1e-9 * np.eye(n)).T  # 下三角分解
        Xsigma = np.zeros((n, 2*n + 1))
        Xsigma[:, 0] = self.ukf_xk.flatten()  # 中心sigma点
        for i in range(n):
            Xsigma[:, 1 + i] = self.ukf_xk.flatten() + gamma * S[:, i]
            Xsigma[:, 1 + n + i] = self.ukf_xk.flatten() - gamma * S[:, i]

        # 预测步
        Xpred = np.zeros_like(Xsigma)
        for i in range(2*n + 1):
            xi = Xsigma[:, i].reshape(-1, 1)  # (4,1) 
            xstat = np.array([
                [-self.ukf_K_nom[0, 0] * alpha_f],
                [-self.ukf_K_nom[1, 0] * alpha_m],
                [-self.ukf_K_nom[2, 0] * alpha_r],
                [self.ukf_K_nom[3, 0] * s_f]
            ]).reshape(-1, 1) 
            Xpred[:, i] = (xi + (self.dt / self.ukf_tau_proc) * (xstat - xi)).flatten()

        # 预测均值与协方差
        xpred = np.sum(Xpred * Wm.reshape(1, -1), axis=1).reshape(-1, 1)  # 加权求和
        Ppred = self.ukf_Qk.copy()
        for i in range(2*n + 1):
            dx = Xpred[:, i].reshape(-1, 1) - xpred
            Ppred = Ppred + Wc[i] * np.dot(dx, dx.T)

        # 观测预测（横向加速度+横摆角加速度）
        Zsigma = np.zeros((2, 2*n + 1))
        for i in range(2*n + 1):
            f, fm, fr, fx = Xpred[:, i]
            ay_sim = (f + fm + fr) / m  # 横向加速度模拟
            rdot_sim = (self.xf * f + self.xm * fm + self.xr * fr) / self.ukf_Iz_nom  # 横摆角加速度模拟
            Zsigma[:, i] = [ay_sim, rdot_sim]

        # 观测均值与协方差
        zpred = np.sum(Zsigma * Wm.reshape(1, -1), axis=1).reshape(-1, 1)
        Pzz = self.ukf_Rk.copy()
        for i in range(2*n + 1):
            dz = Zsigma[:, i].reshape(-1, 1) - zpred
            Pzz = Pzz + Wc[i] * np.dot(dz, dz.T)

        # 交叉协方差
        Pxz = np.zeros((n, 2))
        for i in range(2*n + 1):
            dx = Xpred[:, i].reshape(-1, 1) - xpred
            dz = Zsigma[:, i].reshape(-1, 1) - zpred
            Pxz = Pxz + Wc[i] * np.dot(dx, dz.T)

        # 更新步
        Kukf = np.dot(Pxz, np.linalg.inv(Pzz + 1e-9 * np.eye(2)))  # 卡尔曼增益
        zmeas = np.array([[ay_meas], [rdot_meas]])
        if not np.all(np.isfinite(zmeas)): 
            zmeas = zpred
        self.ukf_xk = xpred + np.dot(Kukf, (zmeas - zpred))  # 状态更新
        # 协方差更新
        self.ukf_Pk = Ppred - np.dot(np.dot(Kukf, Pzz), Kukf.T)
        self.ukf_Pk = (self.ukf_Pk + self.ukf_Pk.T) / 2 + 1e-6 * np.eye(n)

        # 物理约束范围
        Fmax = 5e6
        self.ukf_xk[0, 0] = np.clip(self.ukf_xk[0, 0], -Fmax, Fmax)
        self.ukf_xk[1, 0] = np.clip(self.ukf_xk[1, 0], -Fmax, Fmax)
        self.ukf_xk[2, 0] = np.clip(self.ukf_xk[2, 0], -Fmax, Fmax)
        self.ukf_xk[3, 0] = np.clip(self.ukf_xk[3, 0], -Fmax, Fmax)

        return self.ukf_xk[0, 0], self.ukf_xk[1, 0], self.ukf_xk[2, 0], self.ukf_xk[3, 0]

    # EKF估计车辆参数
    def ekf_params(self, Ff_b, Fm_b, Fr_b, Fx_f, vx, vy, r, ay_meas, wf, delta, m):
        # 避免vx为0
        vx_safe = max(abs(vx), 0.05)

        # 计算侧偏角与纵向滑移率
        alpha_f = (vy + self.xf * r) / vx_safe - delta  # 前轴侧偏角
        alpha_m = (vy + self.xm * r) / vx_safe          # 中轴侧偏角
        alpha_r = (vy + self.xr * r) / vx_safe          # 后轴侧偏角

        Vwf = vx * np.cos(delta) + vy * np.sin(delta)   # 前轴速
        Vwf_safe = np.sign(Vwf) * max(abs(Vwf), 0.02)
        s_f = (wf - Vwf) / Vwf_safe                     # 前轴纵向滑移率

        # 计算rdot_meas
        dr_raw = (r - self.ekf_r_prev) / self.dt
        self.ekf_rdot_f = self.ekf_alpha_d * self.ekf_rdot_f + (1 - self.ekf_alpha_d) * dr_raw
        self.ekf_r_prev = r
        rdot_meas = self.ekf_rdot_f

        # 测量向量（轮胎力+横摆角加速度）
        z = np.array([[Ff_b], [Fm_b], [Fr_b], [rdot_meas], [Fx_f]])

        # EKF预测
        theta_s_pred = self.ekf_theta_s
        P_s_pred = self.ekf_P_s + self.ekf_Q_s

        theta_phys = np.dot(self.ekf_S, theta_s_pred)
        Cf, Cr, Cxf, Iz = theta_phys[0, 0], theta_phys[1, 0], theta_phys[2, 0], theta_phys[3, 0]

        # 测量预测
        Ff_pred = Cxf * s_f * np.sin(delta) - Cf * alpha_f * np.cos(delta)  # 前轴横向力
        Fm_pred = - Cr * alpha_m  # 中轴横向力
        Fr_pred = - Cr * alpha_r  # 后轴横向力
        Fx_f_pred = Cxf * s_f  # 前轴纵向力
        My = self.xf * Ff_pred + self.xm * Fm_pred + self.xr * Fr_pred  # 横摆力矩
        rdot_pred = My / Iz if Iz != 0 else 0.0  # 横摆角加速度
        h = np.array([[Ff_pred], [Fm_pred], [Fr_pred], [rdot_pred], [Fx_f_pred]])

        # 雅可比矩阵（5x4）
        H11 = -alpha_f * np.cos(delta)  # dFf_pred/dCf
        H12 = 0                         # dFf_pred/dCr
        H13 = s_f * np.sin(delta)       # dFf_pred/dCxf
        H14 = 0                         # dFf_pred/dIz

        H21 = 0                         # dFm_pred/dCf
        H22 = -alpha_m / 2              # dFm_pred/dCr（Cm=Cr）
        H23 = 0                         # dFm_pred/dCxf
        H24 = 0                         # dFm_pred/dIz

        H31 = 0                         # dFr_pred/dCf
        H32 = -alpha_r / 2              # dFr_pred/dCr
        H33 = 0                         # dFr_pred/dCxf
        H34 = 0                         # dFr_pred/dIz

        # 横摆角加速度对参数的导数
        dMy_dCf = self.xf * H11 + self.xm * H21 + self.xr * H31
        dMy_dCr = self.xf * H12 + self.xm * H22 + self.xr * H32
        dMy_dCxf = self.xf * H13 + self.xm * H23 + self.xr * H33
        H41 = dMy_dCf / Iz if Iz != 0 else 0.0  # d(rdot)/dCf
        H42 = dMy_dCr / Iz if Iz != 0 else 0.0  # d(rdot)/dCr
        H43 = dMy_dCxf / Iz if Iz != 0 else 0.0 # d(rdot)/dCxf
        H44 = -My / (Iz**2) if Iz != 0 else 0.0 # d(rdot)/dIz

        H51 = 0                         # dFx_f_pred/dCf
        H52 = 0                         # dFx_f_pred/dCr
        H53 = s_f                       # dFx_f_pred/dCxf
        H54 = 0                         # dFx_f_pred/dIz

        # 雅可比矩阵
        H_phys = np.array([
            [H11, H12, H13, H14],
            [H21, H22, H23, H24],
            [H31, H32, H33, H34],
            [H41, H42, H43, H44],
            [H51, H52, H53, H54]
        ])
        H_s = np.dot(H_phys, self.ekf_S)

        # 测量噪声
        minVarVec = np.array([1e3, 1e3, 1e3, 1e-4, 1e3])
        R = np.diag(np.maximum(np.diag(self.ekf_R_base), minVarVec))

        # 更新步
        nu = z - h
        S_k = np.dot(np.dot(H_s, P_s_pred), H_s.T) + R
        S_k = (S_k + S_k.T) / 2 + 1e-9 * np.eye(5)
        NIS = np.dot(np.dot(nu.T, np.linalg.inv(S_k)), nu)
        do_update = np.isfinite(NIS) and (NIS < 1e8)

        if do_update:
            K = np.dot(np.dot(P_s_pred, H_s.T), np.linalg.inv(S_k))  # 卡尔曼增益
            theta_s_upd = theta_s_pred + np.dot(K, nu)  # 状态更新
            # Joseph形式协方差更新
            I_KH = np.eye(4) - np.dot(K, H_s)
            P_s_upd = np.dot(np.dot(I_KH, P_s_pred), I_KH.T) + np.dot(np.dot(K, R), K.T)
        else:
            theta_s_upd = theta_s_pred
            P_s_upd = P_s_pred

        # 协方差对称化
        P_s_upd = (P_s_upd + P_s_upd.T) / 2 + 1e-9 * np.eye(4)

        # 物理域参数与约束
        theta_phys_upd = np.dot(self.ekf_S, theta_s_upd)
        theta_phys_upd[0, 0] = np.clip(theta_phys_upd[0, 0], 
            0.8 * self.truck_params.init_Cf, 1.5 * self.truck_params.init_Cf)
        theta_phys_upd[1, 0] = np.clip(theta_phys_upd[1, 0], 
            0.8 * self.truck_params.init_Cr, 1.5 * self.truck_params.init_Cr)
        theta_phys_upd[2, 0] = np.clip(theta_phys_upd[2, 0], 
            0.8 * self.truck_params.init_Cxf,1.5 * self.truck_params.init_Cxf)
        theta_phys_upd[3, 0] = np.clip(theta_phys_upd[3, 0], 
            0.8 * self.truck_params.init_Iz, 1.5 * self.truck_params.init_Iz)

        # 更新状态
        self.ekf_theta_s = np.dot(np.linalg.inv(self.ekf_S), theta_phys_upd)
        self.ekf_P_s = P_s_upd

        return theta_phys_upd[0, 0], theta_phys_upd[1, 0], theta_phys_upd[2, 0], theta_phys_upd[3, 0]

    # 发布估计参数
    def publish_result(self, Cf, Cm, Cr, Cxf, Cxm, Cxr, Iz):
        msg = VehParamEst()
        msg.header.stamp = rospy.Time.now()
        msg.Cf = Cf
        msg.Cm = Cm
        msg.Cr = Cr
        msg.Cxf = Cxf
        msg.Cxm = Cxm
        msg.Cxr = Cxr
        msg.Iz = Iz
        self.est_pub.publish(msg)


if __name__ == "__main__":
    try:
        node = VehParamEKF()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Vehicle Parameter Estimation Node Shutdown")
