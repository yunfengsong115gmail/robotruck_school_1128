#!/usr/bin/env python3
# -*- coding: utf-8 -*
"""
MPC控制器
"""
import rospy
import numpy as np
import matplotlib.pyplot as plt
# import cvxpy as cp

from math import *
# from scipy.linalg import inv

from std_msgs.msg import String

from robominer_msgs.msg import *
from robominer_msgs.srv import *
from estimation.msg import VehParamEst  # 估计模块自定义消息

from visualization_msgs.msg import Marker

from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

# PID纵向速度控制器
class PIDController:
    def __init__(self):
        # 初始化 PID 参数
        self.Kp = 10
        self.Ki = 3
        self.Kd = 1

        # 误差项
        self.integral = 0
        self.prev_error = 0

        # 目标速度与当前速度
        self.vx_ref = 5
        self.vx = 0
    
    def compute_pid(self, dt):
        rospy.loginfo("PID controller running...")
        error = self.vx_ref-self.vx
        # 积分项
        self.integral += error * dt
        # 微分项
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        # PID控制输出
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        # 保存上次误差用于导数项
        self.prev_error = error
        return u
    
    def update(self, vx):
        self.vx = vx

class MPCController:
    def __init__(self):
        # MPC参数
        self.N = 50            # 预测时域长度
        self.dt = 1/30          # 时间步长，频率30Hz
        self.nx = 4            # 状态量 x [y deviation, yaw deviation]
        self.nu = 1            # 控制量 u steering angle(rad)
        self.vx = 20           # 纵向速度
        self.kf = 140000           # 前轮刚度
        self.km = 120000           # 中轮刚度
        self.kr = 120000           # 后轮刚度
        self.m = 61000           # 质量
        self.Iz = 23665           # 横摆转动惯量
        self.a = 2.2           # 三个轴距
        self.b = 2
        self.c = 3.9

        # discrete matrices A and B
        self.disA = np.array([[1, self.dt*self.vx, self.dt, 0],
                           [0, 1, 0, self.dt],
                           [0, 0, 1-self.dt*(self.kf+self.km+self.kr)/(self.m*self.vx), 
                            -self.dt*(self.a*self.kf-self.b*self.km-self.c*self.kr)/(self.m*self.vx)-self.vx*self.dt],
                           [0, 0, -self.dt*(self.a*self.kf-self.b*self.km-self.c*self.kr)/(self.Iz*self.vx), 
                            1-self.dt*(self.a**2*self.kf+self.b**2*self.km+self.c**2*self.kr)/(self.Iz*self.vx)]])
        self.disB = np.array([[0],
                           [0],
                           [self.dt*self.kf/self.m],
                           [self.dt*self.a*self.kf/self.Iz]])

        # 权重矩阵
        self.Q = np.diag([100, 10, 1, 1])
        self.R = np.diag([1])

        # 当前与目标状态
        self.state = np.zeros((self.nx, 1))
        self.state_ref = np.zeros((self.nx, 1))  # [横向参考，侧偏角参考，0，0] 

    def solve_mpc(self):
        rospy.loginfo("MPC controller running...")
        # 构造 Psi 和 Theta
        Psi = np.zeros((self.N * self.nx, self.nx))
        Theta = np.zeros((self.N * self.nx, self.N * self.nu))
        tmp = np.eye(self.nx)

        for i in range(1, self.N+1):
            # 注意slice的范围
            rows = slice((i-1)*self.nx, i*self.nx)
            if i == 1:
                Theta[rows, 0:self.nu] = tmp @ self.disB
            else:
                Theta[rows, :] = np.hstack([tmp @ self.disB, Theta[(i-2)*self.nx:(i-1)*self.nx,
                                                                   0:(self.N-1)*self.nu]])
            tmp = self.disA @ tmp
            Psi[rows, :] = tmp

        # 权重矩阵扩展
        Q_bar = np.kron(np.eye(self.N), self.Q)
        R_bar = np.kron(np.eye(self.N), self.R)

        # MPC相关矩阵参数计算
        E = Theta.T @ Q_bar @ Psi
        H = Theta.T @ Q_bar @ Theta + R_bar

        # 计算控制序列
        # U_k = -H^-1 * E * x
        x = self.state-self.state_ref
        U_k = -np.linalg.inv(H) @ E @ x

        # 当前时刻控制量（延迟补偿）
        steer = U_k[self.nu-1, 0]
        return steer
    
    def update(self, curState, curRef, vx):
        self.state = curState
        self.state_ref = curRef
        self.vx = vx

    def vehparam_update(self,Cf,Cm,Cr,Iz):
        self.kf = Cf
        self.km = Cm
        self.kr = Cr
        self.Iz = Iz

class pathfollowing:
    def __init__(self):
        self.vx = 0
        self.vy = 0
        self.x = 0
        self.y = 0
        self.yref = 0
        self.yaw = 0  # [-pi,pi]
        self.yawref = 0
        # parameters from estimation part
        self.Cf = 140000           # 前轮刚度
        self.Cm = 120000           # 中轮刚度
        self.Cr = 120000           # 后轮刚度
        self.Iz = 23665
        # plots variables(Additional)
        self.x_hist = [0]
        self.y_hist = [0]
        self.yref_hist = [0]

    def sysState_callback(self, data):
        # 位置和速度指令消息回调
        # 此处暂且不设置参考点
        self.x = data.x
        self.y = data.y
        glovx = data.vx
        glovy = data.vy
        self.yaw = data.yaw*pi/180
        # 速度按车辆坐标系变换[cos sin; -sin cos]
        T = np.array([[np.cos(self.yaw), np.sin(self.yaw)],[-np.sin(self.yaw), np.cos(self.yaw)]])
        glov = np.array([[glovx],[glovy]])
        localv = T @ glov
        self.vx = localv[0]
        self.vy = localv[1]
        rospy.loginfo("current longitudinal speed: %.1f" % (self.vx))

    def vehparam_callback(self, data):
        # MPC需要的车辆参数更新回调
        self.Cf = data.Cf
        self.Cm = data.Cm
        self.Cr = data.Cr
        self.Iz = data.Iz

    def RefGenerator(self):
        # 备选路径
        '''
        r1 = 0.096*(self.x-60)-1.2
        r2 = 0.096*(self.x-120)-1.2
        dm1 = dm2 = 25
        dn1 = dn2 = 3.6
        self.yref = dn1/2*(1+np.tanh(r1))-dn2/2*(1+np.tanh(r2))
        self.yawref = np.arctan(dn1*(1/np.cosh(r1))**2*(1.2/dm1)
                        -dn2*(1/np.cosh(r2))**2*(1.2/dm2))
        '''
        # 三角曲线路径
        A = 2
        omega = 1/5
        self.yref = A*np.sin(omega*self.x)
        self.yawref = np.arctan(A*omega*np.cos(omega*self.x))
        # 根式路径
        # self.yref = (self.x+1)**(2/3)-1
        # self.yawref = np.arctan(2/3*(self.x+1)**(-1/3))
    
    def move_truck(self):
        # 初始化节点
        rospy.init_node('mpc_control_node')
    
        # ROS 通信
        cmd_pub = rospy.Publisher('/guardian/control_cmd', ControlCmd, queue_size=1)
        cmd_aux_pub = rospy.Publisher('/guardian/control_cmd_aux', ControlCmdAux, queue_size=1)
        rate = rospy.Rate(30)
        # self.vcu_sub = rospy.Subscriber('/vcu/vcu_data', VcuData)
        rospy.Subscriber('/localization/localization_data', LocalizationData, self.sysState_callback)
        rospy.Subscriber('/estimation/veh_params', VehParamEst, self.vehparam_callback, queue_size=10)

        # PID prev_time and MPC update timer initialize
        prev_time = rospy.get_time()
        MPC_timer = rospy.get_time()

        # 打开图窗(Additional)
        plt.figure(figsize=(8, 5))
        
        while not rospy.is_shutdown():
            # 自身参数更新
            self.RefGenerator()
            
            # PID进程控制纵向速度
            pid = PIDController()
            # 计算时间差
            curr_time = rospy.get_time()
            dt = curr_time - prev_time
            prev_time = curr_time
            
            # PID控制输出
            pid.update(self.vx)
            acceler = pid.compute_pid(dt)
            # 输出调整
            if acceler >= 0:
                th = acceler
                br = 0
            elif acceler < 0:
                th = 0
                br = -acceler
            else: 
                br = 0 
                th = 0
            
            # MPC进程控制转向角
            mpc = MPCController()
            # 更新MPC控制参数
            if curr_time - MPC_timer > 10:
                mpc.vehparam_update(self.Cf,self.Cm,self.Cr,self.Iz)
                rospy.loginfo("MPC parameters updated: Cf=%.1f, Cm=%.1f, Cr=%.1f, Iz=%.1f" %
                               (mpc.kf, mpc.km, mpc.kr, mpc.Iz))
                MPC_timer = curr_time
            # 从里程计中获取位置与速度（这里只取x方向）
            State = np.array([[self.y],[self.yaw],[0],[0]])
            refState = np.array([[self.yref],[self.yawref],[0],[0]])
            mpc.update(State,refState,self.vx)
            steer = mpc.solve_mpc()*180/pi

            # 发布控制指令
            # initial values for th, br and st: [40, 0, 30]
            controlCmd = ControlCmd()
            controlCmd.header.stamp = rospy.Time.now()
            controlCmd.target_drive_mode.mode = 2
            controlCmd.target_throttle = th
            controlCmd.target_brake = br
            controlCmd.target_steering = steer
            controlCmd.target_gear.gear = Gear.D
            cmd_pub.publish(controlCmd)

            controlCmdAux = ControlCmdAux()
            controlCmdAux.header.stamp = rospy.Time.now()
            controlCmdAux.target_epb.epb = 0
            cmd_aux_pub.publish(controlCmdAux)

            # Plots(Additional)
            plt.ion()
            self.x_hist.append(self.x)
            self.y_hist.append(self.y)
            self.yref_hist.append(self.yref)
            xhis = self.x_hist
            yhis = self.y_hist
            yrefhis = self.yref_hist

            plt.clf()
            plt.plot(xhis, yrefhis, label='Reference Path')
            plt.plot(xhis, yhis, label="Actual Output Path")
            plt.xlabel("X position (m)")
            plt.ylabel("Y position (m)")
            plt.legend()
            plt.grid(True)
            plt.title("MPC Path Tracking Simulation")
            plt.show()
            plt.pause(0.03)
            plt.ioff()
            # 结束plot
    
            rate.sleep()
 
if __name__ == '__main__':
    try:
        movetruck = pathfollowing()
        movetruck.move_truck()
    except rospy.ROSInterruptException:
        pass