#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 路面附着系数估计节点
订阅：/localization/localization_data、/vcu/vcu_data
发布：/estimation/adhesion_est
功能包: estimation
"""
import sys
import rospy
import numpy as np
from scipy.linalg import cholesky, inv, LinAlgError
from typing import Tuple
from std_msgs.msg import Header
from robominer_msgs.msg import LocalizationData, VcuData
from estimation.msg import AdhesionEst


# 矿车参数
class TruckParams:
    def __init__(self, work_mode: str = "empty"):
        self.a_m = 4.2  # 前轴到中轴距离
        self.m_r = 1.9  # 中轴到后轴距离
        self.L = self.a_m + self.m_r  # 总轴距
        self.B_f = 3.184  # 前轮距
        self.B_m = 3.324  # 中轮距
        self.B_r = 3.324  # 后轮距
        self.R = 0.845  # 轮胎半径（m）
        self.g = 9.81  # 重力加速度（m/s²）
        
        if work_mode == "empty":  # 空载参数
            self.Cx_f = 1.2e5  # 前轴纵向刚度
            self.Cx_m = 0.9e5  # 中轴纵向刚度
            self.Cx_r = 1.1e5  # 后轴纵向刚度
            self.Cy_f = 1.4e5  # 前轴侧偏刚度
            self.Cy_m = 1.2e5  # 中轴侧偏刚度
            self.Cy_r = 1.2e5  # 后轴侧偏刚度
            self.a = 2.2    # 质心到前轴距离（前向为正）
            self.b = self.a_m - self.a  # 质心到中轴距离
            self.c = self.L - self.a    # 质心到后轴距离
            self.h_g = 1.7     # 质心高度（m）
            self.Iz = 6.5e5    # 横摆转动惯量（kg·m²）
            self.m_total = 61000  # 总质量（kg）
            
            # 轴荷比例（空载）
            self.axle_load_ratio = {
                'front': 0.573,   
                'middle': 0.213,  
                'rear': 0.213 
            }
        else:  # 满载模式
            self.Cx_f = 1.6e5  # 前轴纵向刚度
            self.Cx_m = 1.8e5  # 中轴纵向刚度
            self.Cx_r = 2.1e5  # 后轴纵向刚度
            self.Cy_f = 2.5e5  # 前轴侧偏刚度
            self.Cy_m = 3.3e5  # 中轴侧偏刚度
            self.Cy_r = 3.6e5  # 后轴侧偏刚度
            self.a = 3.884    # 质心到前轴距离
            self.b = self.a_m - self.a  # 质心到中轴距离
            self.c = self.L - self.a    # 质心到后轴距离
            self.h_g = 2.2     # 质心高度（m）
            self.Iz = 2.0e6    # 横摆转动惯量（kg·m²）
            self.m_total = 152000  # 总质量（kg）
            
            # 轴荷比例（满载）
            self.axle_load_ratio = {
                'front': 38400 / 152000,   # 0.2526
                'middle': 58900 / 152000,  # 0.3875
                'rear': 58900 / 152000     # 0.3875
            }


# 轮胎动力学计算
class TireDynamics:
    def __init__(self, params: TruckParams):
        self.params = params
        self.tire_states = {
            'Fx0': {'fl': 0.0, 'fr': 0.0, 'ml': 0.0, 'mr': 0.0, 'rl': 0.0, 'rr': 0.0},
            'Fy0': {'fl': 0.0, 'fr': 0.0, 'ml': 0.0, 'mr': 0.0, 'rl': 0.0, 'rr': 0.0},
            'alpha': {'fl': 0.0, 'fr': 0.0, 'ml': 0.0, 'mr': 0.0, 'rl': 0.0, 'rr': 0.0},
            'lambda': {'fl': 0.0, 'fr': 0.0, 'ml': 0.0, 'mr': 0.0, 'rl': 0.0, 'rr': 0.0},
            'Fz': {'fl': 0.0, 'fr': 0.0, 'ml': 0.0, 'mr': 0.0, 'rl': 0.0, 'rr': 0.0},
            'locate_gamma_dot': 0.0,
            'a_long': 0.0,
            'a_lat': 0.0,
            'delta_f': 0.0  # 前轮转角
        }
        self.prev_wz = 0.0  
        self.prev_time = None 

    def update(self, loc_data: LocalizationData, vcu_data: VcuData):
        vx_enu = loc_data.vx
        vy_enu = loc_data.vy
        yaw_deg = loc_data.yaw
        yaw_rad = np.radians(yaw_deg)
        omega_z = np.radians(loc_data.wz)
        ax_enu = loc_data.ax
        ay_enu = loc_data.ay
        # current_time = loc_data.header.stamp.to_sec()  # no need for header
        current_time = rospy.Time.now().to_sec()
        v_long = vx_enu * np.cos(yaw_rad) + vy_enu * np.sin(yaw_rad)
        v_lat = -vx_enu * np.sin(yaw_rad) + vy_enu * np.cos(yaw_rad)

        # 计算横摆角加速度
        if self.prev_time is not None and (current_time - self.prev_time) > 1e-6:
            dt = current_time - self.prev_time
            self.tire_states['locate_gamma_dot'] = (omega_z - self.prev_wz) / dt
        self.prev_wz = omega_z
        self.prev_time = current_time

        # 前轮转角
        steering_angle_deg = np.nan_to_num(vcu_data.steering_angle, nan=0.0)
        self.tire_states['delta_f'] = np.radians(steering_angle_deg)

        # 轮速
        wheel_speeds = {
            'fl': np.nan_to_num(vcu_data.wheel_speed_fl, nan=0.1),
            'fr': np.nan_to_num(vcu_data.wheel_speed_fr, nan=0.1),
            'ml': np.nan_to_num(vcu_data.wheel_speed_fl, nan=0.1),
            'mr': np.nan_to_num(vcu_data.wheel_speed_fr, nan=0.1),
            'rl': np.nan_to_num(vcu_data.wheel_speed_fl, nan=0.1),
            'rr': np.nan_to_num(vcu_data.wheel_speed_fr, nan=0.1)
        }

        # 计算轮心速度
        v_x_tire = {}
        v_y_tire = {}
        delta_f = self.tire_states['delta_f']
        for tire in ['fl', 'fr', 'ml', 'mr', 'rl', 'rr']:
            if tire in ['fl', 'fr']:
                y_pos = self.params.B_f / 2 if tire == 'fl' else -self.params.B_f / 2
                # 前轮考虑转向角的纵向速度修正
                v_x_tire[tire] = (v_long - y_pos * omega_z) * np.cos(delta_f) - (v_lat + self.params.a * omega_z) * np.sin(delta_f)
                v_y_tire[tire] = (v_long - y_pos * omega_z) * np.sin(delta_f) + (v_lat + self.params.a * omega_z) * np.cos(delta_f)
            elif tire in ['ml', 'mr']:
                y_pos = self.params.B_m / 2 if tire == 'ml' else -self.params.B_m / 2
                v_x_tire[tire] = v_long - y_pos * omega_z
                v_y_tire[tire] = v_lat - self.params.b * omega_z
            else:
                y_pos = self.params.B_r / 2 if tire == 'rl' else -self.params.B_r / 2
                v_x_tire[tire] = v_long - y_pos * omega_z
                v_y_tire[tire] = v_lat - self.params.c * omega_z

        # 计算侧偏角
        for tire in ['fl', 'fr', 'ml', 'mr', 'rl', 'rr']:
            v_x_safe = np.maximum(np.abs(v_x_tire[tire]), 0.1)
            alpha = np.arctan2(v_y_tire[tire], v_x_safe)
            self.tire_states['alpha'][tire] = alpha

        # 计算滑移率
        for tire in ['fl', 'fr', 'ml', 'mr', 'rl', 'rr']:
            v_wheel = wheel_speeds[tire]
            v_wx = v_x_tire[tire]
            max_speed = np.maximum(np.abs(v_wheel), np.abs(v_wx))
            denominator = np.maximum(max_speed, 0.1)
            lambda_val = (v_wheel - v_wx) / denominator
            self.tire_states['lambda'][tire] = np.clip(lambda_val, -1.0, 1.0)

        # 计算垂向载荷
        total_weight = self.params.m_total * self.params.g
        Fz_front = total_weight * self.params.axle_load_ratio['front']
        Fz_middle = total_weight * self.params.axle_load_ratio['middle']
        Fz_rear = total_weight * self.params.axle_load_ratio['rear']
        self.tire_states['Fz']['fl'] = self.tire_states['Fz']['fr'] = Fz_front / 2
        self.tire_states['Fz']['ml'] = self.tire_states['Fz']['mr'] = Fz_middle / 2
        self.tire_states['Fz']['rl'] = self.tire_states['Fz']['rr'] = Fz_rear / 2

        # 动态载荷转移
        a_long = self.tire_states['a_long']
        a_lat = self.tire_states['a_lat']
        ax_smoothed = np.clip(a_long, -self.params.g, self.params.g)
        ay_smoothed = np.clip(a_lat, -self.params.g, self.params.g)
        for tire in ['fl', 'fr']:
            self.tire_states['Fz'][tire] *= np.clip(1 - 0.2 * ax_smoothed / self.params.g, 0.8, 1.2)
        for tire in ['ml', 'mr', 'rl', 'rr']:
            self.tire_states['Fz'][tire] *= np.clip(1 + 0.2 * ax_smoothed / self.params.g, 0.8, 1.2)
        for tire in ['fl', 'ml', 'rl']:
            self.tire_states['Fz'][tire] *= np.clip(1 - 0.15 * ay_smoothed / self.params.g, 0.85, 1.15)
        for tire in ['fr', 'mr', 'rr']:
            self.tire_states['Fz'][tire] *= np.clip(1 + 0.15 * ay_smoothed / self.params.g, 0.85, 1.15)
        for tire in self.tire_states['Fz']:
            self.tire_states['Fz'][tire] = np.maximum(self.tire_states['Fz'][tire], 100.0)

        # Dugoff模型计算轮胎力
        def dugoff_force(Fz, alpha, lambd, Cx, Cy):
            alpha_clipped = np.clip(alpha, -0.3, 0.3)
            lambd_clipped = np.clip(lambd, -0.5, 0.5)
            numerator = (1 - np.abs(lambd_clipped))
            denominator = 2 * np.sqrt((Cx * lambd_clipped)**2 + (Cy * np.tan(alpha_clipped))** 2 + 1e-6)
            denominator = np.maximum(denominator, 1e-6)
            L = np.clip(numerator / denominator, 0, 1.5)
            f_L = L * (2 - L) if L < 1 else 1.0
            denom_term = np.maximum(1 - np.abs(lambd_clipped) + 1e-6, 0.3)
            Fx0 = Cx * (lambd_clipped / denom_term) * f_L
            Fy0 = Cy * (np.tan(alpha_clipped) / denom_term) * f_L
            Fx_max = 0.8 * Fz
            Fy_max = 0.8 * Fz
            return np.clip(Fx0, -Fx_max, Fx_max), np.clip(Fy0, -Fy_max, Fy_max)

        # 前轴轮胎力
        for tire in ['fl', 'fr']:
            Fx0, Fy0 = dugoff_force(
                self.tire_states['Fz'][tire],
                self.tire_states['alpha'][tire],
                self.tire_states['lambda'][tire],
                self.params.Cx_f, self.params.Cy_f
            )
            self.tire_states['Fx0'][tire] = Fx0
            self.tire_states['Fy0'][tire] = Fy0
        # 中轴轮胎力
        for tire in ['ml', 'mr']:
            Fx0, Fy0 = dugoff_force(
                self.tire_states['Fz'][tire],
                self.tire_states['alpha'][tire],
                self.tire_states['lambda'][tire],
                self.params.Cx_m, self.params.Cy_m
            )
            self.tire_states['Fx0'][tire] = Fx0
            self.tire_states['Fy0'][tire] = Fy0
        # 后轴轮胎力
        for tire in ['rl', 'rr']:
            Fx0, Fy0 = dugoff_force(
                self.tire_states['Fz'][tire],
                self.tire_states['alpha'][tire],
                self.tire_states['lambda'][tire],
                self.params.Cx_r, self.params.Cy_r
            )
            self.tire_states['Fx0'][tire] = Fx0
            self.tire_states['Fy0'][tire] = Fy0

    def get_states(self) -> dict:
        return self.tire_states.copy()

# UKF附着系数估计
class UKF_AttachEst:
    def __init__(self, params: TruckParams):
        self.params = params
        self.n_x = 6  # 6个轮胎附着系数
        self.n_z = 3  # 纵向加速度、侧向加速度、横摆角加速度
        # UKF参数
        self.alpha = 0.8
        self.beta = 2.0
        self.lambda_ = self.alpha**2 * (self.n_x + 3) - self.n_x
        # 权重
        self.Wm = np.zeros(2 * self.n_x + 1)
        self.Wc = np.zeros(2 * self.n_x + 1)
        self.Wm[0] = self.lambda_ / (self.n_x + self.lambda_)
        self.Wc[0] = self.Wm[0] + (1 - self.alpha**2 + self.beta)
        for i in range(1, 2 * self.n_x + 1):
            self.Wm[i] = 1 / (2 * (self.n_x + self.lambda_))
            self.Wc[i] = self.Wm[i]
        # 噪声协方差
        self.Q = np.diag([5e-6] * 2 + [3e-6] * 2 + [5e-6] * 2)
        self.R = 0.3e-4 * np.diag([(2.0)**2, (2)** 2, (2)**2])  # 观测噪声对估计结果影响较大
        # 稳定性参数
        self.max_covariance = 1e2
        self.min_eigenvalue = 1e-5
        self.max_mu_jump = 0.1
        self.sigma_pred = None
        # 初始化
        self.x_est = np.array([0.6, 0.6, 0.55, 0.55, 0.5, 0.5])
        self.P_est = np.diag([0.01**2] * 6)

    def ensure_positive_definite(self, P: np.ndarray) -> np.ndarray:
        P = np.nan_to_num(P, nan=0.0, posinf=self.max_covariance, neginf=-self.max_covariance)
        try:
            eigvals, eigvecs = np.linalg.eig(P)
        except LinAlgError:
            return np.diag(np.full(self.n_x, self.min_eigenvalue))
        eigvals = np.maximum(eigvals, self.min_eigenvalue)
        eigvals = np.nan_to_num(eigvals, nan=self.min_eigenvalue)
        try:
            return eigvecs @ np.diag(eigvals) @ np.linalg.inv(eigvecs)
        except LinAlgError:
            return np.diag(np.full(self.n_x, self.min_eigenvalue))

    def generate_sigma(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        x = np.nan_to_num(x, nan=0.5)
        x = np.clip(x, 0.05, 0.95)
        P_stable = self.ensure_positive_definite(P)
        P_scaled = (self.n_x + self.lambda_) * P_stable
        P_scaled = np.clip(P_scaled, -self.max_covariance, self.max_covariance)
        try:
            sqrt_P = cholesky(P_scaled, lower=True)
        except LinAlgError:
            eigvals, eigvecs = np.linalg.eig(P_scaled)
            eigvals = np.maximum(eigvals, self.min_eigenvalue)
            sqrt_P = eigvecs @ np.diag(np.sqrt(eigvals))
        sqrt_P = np.nan_to_num(sqrt_P, nan=0.0)
        sigma = np.zeros((self.n_x, 2 * self.n_x + 1))
        sigma[:, 0] = x
        for i in range(self.n_x):
            delta = np.clip(sqrt_P[:, i], -0.2, 0.2)
            sigma[:, i+1] = x + delta
            sigma[:, i+1+self.n_x] = x - delta
        sigma = np.clip(sigma, 0.05, 0.95)
        return np.nan_to_num(sigma, nan=0.5)

    def predict_state(self, sigma: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        sigma_pred = sigma.copy()
        sigma_pred = np.nan_to_num(sigma_pred, nan=0.5)
        x_pred = np.dot(sigma_pred, self.Wm)
        x_pred = np.clip(x_pred, 0.05, 0.95)
        P_pred = self.Q.copy()
        for i in range(2 * self.n_x + 1):
            diff = sigma_pred[:, i] - x_pred
            diff = np.clip(diff, -0.3, 0.3)
            P_pred += self.Wc[i] * np.outer(diff, diff)
        P_pred = self.ensure_positive_definite(P_pred)
        return x_pred, P_pred, sigma_pred

    def predict_observation(self, sigma_pred: np.ndarray, tire_states: dict, delta_f: float) -> np.ndarray:
        z_sigma = np.zeros((self.n_z, 2 * self.n_x + 1))
        # 轮胎力
        Fx0 = np.array([
            tire_states['Fx0']['fl'], tire_states['Fx0']['fr'],
            tire_states['Fx0']['ml'], tire_states['Fx0']['mr'],
            tire_states['Fx0']['rl'], tire_states['Fx0']['rr']
        ])
        Fy0 = np.array([
            tire_states['Fy0']['fl'], tire_states['Fy0']['fr'],
            tire_states['Fy0']['ml'], tire_states['Fy0']['mr'],
            tire_states['Fy0']['rl'], tire_states['Fy0']['rr']
        ])
        Fx0 = np.nan_to_num(Fx0, nan=0.0)
        Fy0 = np.nan_to_num(Fy0, nan=0.0)

        # 矿车参数
        m, Iz = self.params.m_total, self.params.Iz
        a, b, c = self.params.a, self.params.b, self.params.c
        B_f, B_m, B_r = self.params.B_f, self.params.B_m, self.params.B_r

        # 对每个sigma点计算观测值
        for i in range(2 * self.n_x + 1):
            mu = sigma_pred[:, i]
            mu = np.clip(mu, 0.05, 0.95)
            Fx = mu * Fx0
            Fy = mu * Fy0

            try:
                # 纵向加速度
                ax = (1/m) * np.sum([
                    Fx[0] * np.cos(delta_f) - Fy[0] * np.sin(delta_f),
                    Fx[1] * np.cos(delta_f) - Fy[1] * np.sin(delta_f),
                    Fx[2], Fx[3], Fx[4], Fx[5]
                ])
                # 侧向加速度
                ay = (1/m) * np.sum([
                    Fx[0] * np.sin(delta_f) + Fy[0] * np.cos(delta_f),
                    Fx[1] * np.sin(delta_f) + Fy[1] * np.cos(delta_f),
                    Fy[2], Fy[3], Fy[4], Fy[5]
                ])
                # 横摆角加速度
                term_fl = a * (Fx[0] * np.sin(delta_f) + Fy[0] * np.cos(delta_f)) - 0.5*B_f * (Fx[0] * np.cos(delta_f) - Fy[0] * np.sin(delta_f))
                term_fr = a * (Fx[1] * np.sin(delta_f) + Fy[1] * np.cos(delta_f)) - 0.5*B_f * (Fx[1] * np.cos(delta_f) - Fy[1] * np.sin(delta_f))
                term_ml = -b * Fy[2] - 0.5*B_m * Fx[2]
                term_mr = -b * Fy[3] + 0.5*B_m * Fx[3]
                term_rl = -c * Fy[4] - 0.5*B_r * Fx[4]
                term_rr = -c * Fy[5] + 0.5*B_r * Fx[5]
                gamma_dot = (1/Iz) * (term_fl + term_fr + term_ml + term_mr + term_rl + term_rr)
            except Exception as e:
                rospy.logwarn_throttle(1, f"UKF observation error: {str(e)}, using default")
                ax, ay, gamma_dot = 0.0, 0.0, 0.0

            z_sigma[0, i] = np.clip(np.nan_to_num(ax, nan=0.0), -8, 8)
            z_sigma[1, i] = np.clip(np.nan_to_num(ay, nan=0.0), -5, 5)
            z_sigma[2, i] = np.clip(np.nan_to_num(gamma_dot, nan=0.0), -1.5, 1.5)
        return z_sigma

    def update_state(self, x_pred: np.ndarray, P_pred: np.ndarray, 
                    z_sigma: np.ndarray, z_meas: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        z_meas = np.nan_to_num(z_meas, nan=0.0)
        z_meas = np.clip(z_meas, [-8, -5, -1.5], [8, 5, 1.5])
        z_pred = np.dot(z_sigma, self.Wm)
        z_pred = np.nan_to_num(z_pred, nan=0.0)

        # 计算协方差
        P_zz = self.R.copy()
        P_xz = np.zeros((self.n_x, self.n_z))
        for i in range(2 * self.n_x + 1):
            z_diff = z_sigma[:, i] - z_pred
            z_diff = np.clip(z_diff, -5, 5)
            P_zz += self.Wc[i] * np.outer(z_diff, z_diff)

            x_diff = self.sigma_pred[:, i] - x_pred
            x_diff = np.clip(x_diff, -0.2, 0.2)
            P_xz += self.Wc[i] * np.outer(x_diff, z_diff)

        P_zz = self.ensure_positive_definite(P_zz)

        # 卡尔曼增益
        try:
            K = np.dot(P_xz, inv(P_zz))
        except LinAlgError:
            K = np.dot(P_xz, np.linalg.pinv(P_zz))
        K = np.clip(K, -2, 2)

        # 状态更新
        x_update_raw = x_pred + np.dot(K, z_meas - z_pred)
        delta = x_update_raw - self.x_est
        delta_clipped = np.clip(delta, -self.max_mu_jump, self.max_mu_jump)
        x_update = self.x_est + delta_clipped
        x_update = np.clip(x_update, 0.05, 0.95)

        # 协方差更新
        try:
            P_update = P_pred - np.dot(K, np.dot(P_zz, K.T))
        except LinAlgError:
            P_update = P_pred
        P_update = self.ensure_positive_definite(P_update)

        return x_update, P_update

    def step(self, tire_states: dict, loc_data: LocalizationData) -> np.ndarray:
        sigma = self.generate_sigma(self.x_est, self.P_est)
        x_pred, P_pred, self.sigma_pred = self.predict_state(sigma)
        # 实际前轮转角
        delta_f = tire_states['delta_f']
        z_sigma = self.predict_observation(self.sigma_pred, tire_states, delta_f)
        # 观测向量
        z_meas = np.array([
            tire_states['a_long'],
            tire_states['a_lat'],
            tire_states['locate_gamma_dot']
        ])
        self.x_est, self.P_est = self.update_state(x_pred, P_pred, z_sigma, z_meas)
        return self.x_est.copy()


# ROS节点
class AdhesionEstNode:
    def __init__(self):
        rospy.init_node('adhesion_est_node', anonymous=False)
        rospy.loginfo("Adhesion Estimation Node Started")

        # 参数配置
        self.work_mode = rospy.get_param('~work_mode', 'empty')
        self.pub_rate = rospy.get_param('~pub_rate', 30)
        self.data_timeout = 0.1

        # 模块初始化
        self.truck_params = TruckParams(work_mode=self.work_mode)
        self.tire_dynamics = TireDynamics(self.truck_params)
        self.ukf_estimator = UKF_AttachEst(self.truck_params)
        # 数据缓存
        self.latest_loc = None  
        self.latest_vcu = None  
        self.loc_receive_time = rospy.Time(0)  
        self.vcu_receive_time = rospy.Time(0) 

        # 话题订阅
        rospy.Subscriber('/localization/localization_data', LocalizationData, self.loc_callback, queue_size=10)
        rospy.Subscriber('/vcu/vcu_data', VcuData, self.vcu_callback, queue_size=10)

        # 发布
        self.est_pub = rospy.Publisher('/estimation/adhesion_est', AdhesionEst, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1/self.pub_rate), self.main_loop)

    def loc_callback(self, msg: LocalizationData):
        self.latest_loc = msg
        self.loc_receive_time = rospy.Time.now()
        # rospy.loginfo("received loc data")

    def vcu_callback(self, msg: VcuData):
        self.latest_vcu = msg
        self.vcu_receive_time = rospy.Time.now()
        # rospy.loginfo("received vcu data")

    def main_loop(self, event):
        # current_time = rospy.Time.now()
        # loc_delay = (current_time - self.loc_receive_time).to_sec()
        # vcu_delay = (current_time - self.vcu_receive_time).to_sec()

        # if loc_delay > self.data_timeout:
        #     rospy.logwarn_throttle(1, f"Localization data timeout (latest {loc_delay:.2f}s ago)")
        #     return
        # if vcu_delay > self.data_timeout:
        #     rospy.logwarn_throttle(1, f"VCU data timeout (latest {vcu_delay:.2f}s ago)")
        #     return

        # 更新轮胎动力学状态
        self.tire_dynamics.update(self.latest_loc, self.latest_vcu)
        tire_states = self.tire_dynamics.get_states()

        # ENU_2_body
        yaw_deg = self.latest_loc.yaw
        yaw_rad = np.radians(yaw_deg)
        enu_ax = self.latest_loc.ax
        enu_ay = self.latest_loc.ay
        tire_states['a_long'] = enu_ax * np.cos(yaw_rad) + enu_ay * np.sin(yaw_rad)
        tire_states['a_lat'] = -enu_ax * np.sin(yaw_rad) + enu_ay * np.cos(yaw_rad)

        # UKF估计
        mu_est = self.ukf_estimator.step(tire_states, self.latest_loc)

        # 发布结果
        self.publish_result(mu_est, self.latest_loc)
        rospy.loginfo_throttle(0.5, 
            f"Adhesion: fl={mu_est[0]:.3f}, fr={mu_est[1]:.3f}, "
            f"ml={mu_est[2]:.3f}, mr={mu_est[3]:.3f}, rl={mu_est[4]:.3f}, rr={mu_est[5]:.3f}"
        )

    def publish_result(self, mu_est: np.ndarray, loc_data: LocalizationData):
        msg = AdhesionEst()
        # msg.header = loc_data.header  # no need for header
        msg.mu_fl = mu_est[0]
        msg.mu_fr = mu_est[1]
        msg.mu_ml = mu_est[2]
        msg.mu_mr = mu_est[3]
        msg.mu_rl = mu_est[4]
        msg.mu_rr = mu_est[5]
        msg.speed_mps = np.sqrt(loc_data.vx**2 + loc_data.vy** 2)
        self.est_pub.publish(msg)

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Adhesion Estimation Node Shutdown")
        finally:
            self.timer.shutdown()


if __name__ == "__main__":
    node = AdhesionEstNode()
    node.run()
