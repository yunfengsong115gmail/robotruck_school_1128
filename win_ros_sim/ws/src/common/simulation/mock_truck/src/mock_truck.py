#!/usr/bin/env python3
# -*- coding: utf-8 -*
"""
按基本运动学模型模拟矿车

"""
import rospy

import time
import json
import random
import inspect
import traceback

from math import *

from std_msgs.msg import String

from robominer_msgs.msg import *
from robominer_msgs.srv import *

from visualization_msgs.msg import Marker

from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


def mat_n2e(lat, lon, alt):

    si = sin(radians(lat))
    ci = cos(radians(lat))
    sj = sin(radians(lon))
    cj = cos(radians(lon))
    return [[-sj, -si * cj, ci * cj], [cj, -si * sj, ci * sj], [0, ci, si]]

def blh2ecef(lat, lon, alt):  # blh 为大地测量球面坐标系<纬度，经度，高程>，ecef 为地心地固坐标系；
    WGS84_A = 6378137.0
    f = (1.0 / 298.257223563)
    WGS84_E = sqrt(2.0 * f - f * f)
    WGS84_E2 = WGS84_E * WGS84_E
    sB = sin(radians(lat))
    cB = cos(radians(lat))
    sL = sin(radians(lon))
    cL = cos(radians(lon))
    N = WGS84_A / sqrt(1.0 - WGS84_E2 * sB * sB)
    return ((N + alt) * cB * cL, (N + alt) * cB * sL,
            (N * (1.0 - WGS84_E2) + alt) * sB)

def enu2ecef(x, y, z, lat, lon, alt):  # enu 为站心坐标系，即东北天坐标系
    mat = mat_n2e(lat, lon, alt)
    delta_x = mat[0][0] * x + mat[0][1] * y + mat[0][2] * z
    delta_y = mat[1][0] * x + mat[1][1] * y + mat[1][2] * z
    delta_z = mat[2][0] * x + mat[2][1] * y + mat[2][2] * z
    x, y, z = blh2ecef(lat, lon, alt)
    return (delta_x + x, delta_y + y, delta_z + z)

def ecef2blh(x, y, z):
    Re = 6378137.0
    f = (1.0 / 298.257223563)
    s = sqrt(x * x + y * y)
    Rp = (1.0 - f) * Re
    ep = sqrt(Re * Re - Rp * Rp) / Rp
    ep2 = ep * ep
    e = sqrt(2 * f - f * f)
    e2 = e * e
    theta = atan2(z * Re, s * Rp)
    s3 = sin(theta)
    c3 = cos(theta)
    s3 = s3 * s3 * s3
    c3 = c3 * c3 * c3

    if (s < (6378137.0 * 1.0 * pi / 180.0)):
        print("error")
        return (0, 0, 0)

    L = atan2(y, x)
    B = atan2(z + ep2 * Rp * s3, s - e2 * Re * c3)
    sB = sin(B)
    cB = cos(B)
    N = Re / sqrt(1 - e2 * sB * sB)
    return (degrees(B), degrees(L), s / cB - N)


#enu转blh，前三个参数为中心点坐标
def enu2blh(lat_c, lon_c, alt_c, x, y, z):
    ecef_x, ecef_y, ecef_z = enu2ecef(x, y, z, lat_c, lon_c, alt_c)
    return ecef2blh(ecef_x, ecef_y, ecef_z)

class Params:
    time_progress_start = time.time()
    # 车辆类型
    vehicle_type = rospy.get_param('guardian_execute/vehicle_type', 0)


class MockTruck(object):
    time_start = {}     # 记录各指令变更时间

    def __init__(self):
        self._vcu_data = VcuData(**{
            'key_power_state': KeyPowerState(KeyPowerState.ON),
            'epower_state': EpowerState(EpowerState.READY)
        })
        self.__goalX, self.__goalY, self.__goalZ = 0, 0, 0
        self.__goalYaw = 0
        self.__pathfileDir = rospy.get_param('/mock_truck/path_file_dir', "")
        self.__steeringLimit = rospy.get_param('/mock_truck/steering_limit',
                                               32.0)
        self.__liftLimit = rospy.get_param('/mock_truck/lift_limit', 50.0)
        self.__speedLimit = rospy.get_param('/mock_truck/speed_limit', 10.0)
        self.__wheelBase = rospy.get_param('/mock_truck/wheel_base', 4.0)
        self.__baseFrameId = rospy.get_param('/mock_truck/base_frame_id',
                                             'base_link')
        self.__worldFrameId = rospy.get_param('/mock_truck/world_frame_id',
                                              'world')
        self.__lat0 = rospy.get_param('/mock_truck/origin_latitude', 31.0)
        self.__lon0 = rospy.get_param('/mock_truck/origin_longitude', 121.0)
        self.__alt0 = rospy.get_param('/mock_truck/origin_altitude', 0.0)
        self.__startX = rospy.get_param('/mock_truck/start_x', 0.0)
        self.__startY = rospy.get_param('/mock_truck/start_y', 0.0)
        self.__startZ = rospy.get_param('/mock_truck/start_z', 0.0)
        self.__startYaw = rospy.get_param('/mock_truck/start_yaw', 0.0)

        self.__x = self.__startX
        self.__y = self.__startY
        self.__z = self.__startZ
        self.__lastZ = self.__startZ
        self.__vx = 0.0
        self.__vy = 0.0
        self.__vz = 0.0
        self.__ax = 0.0
        self.__ay = 0.0
        self.__az = 0.0
        self.__qx = 0.0
        self.__qy = 0.0
        self.__qz = 0.0
        self.__qw = 1.0
        self.__roll = 0.0
        self.__pitch = 0.0
        self.__ggaage = 0
        self.__yaw = self.__startYaw
        self.__wx = 0.0
        self.__wy = 0.0
        self.__wz = 0.0
        self.__a = 0.0  # 车身坐标下纵向加速度

        # TODO 暂时处理
        # 缓存以下rosparam
        self._params = {}

        # epb状态，主要用于判断epb切换状态
        self.epb_pressure_mock = {
            "epb_from": Epb.ON,
            "epb_to": Epb.ON,
            "epb_switch_start_time": 0.0  # epb气压的变化开始时间
        }
        self.MAX_EPB_PRESSURE_DURATION = 3.2  # epb气压的变化过程时间

        # 如果起始位不在原点，更新gps
        (self.__lon, self.__lat, self.__alt) = self.gps_from_cartesian_offset(self.__lon0, self.__lat0,
                                                      self.__alt0, self.__x,
                                                      self.__y, self.__z)

        self.__pto = Pto.NONE
        self.drive_status = DriveStatus.AUTO
        self.__fuelLevel = 10099.9
        self.__speed = 0.0
        self.__odograph = 0.0
        self.__currentThrottle = 0.0
        self.__engineTorque = 0.0
        self.__engineSpeed = 0.0
        self.__engineFault = 0
        self.__steeringAngle = 0.0  # 左正右负
        self.__steeringWheelTorque = 0.0
        self.__steeringWheelSpeed = 0.0
        self.__steeringFault = 0
        self.__steeringMotorTorque = 0.0
        self.__gear = Gear.N
        self.__transmissionFault = 0
        self.__epb = Epb.ON
        self.__epbFault = 0
        self.__currentBrake = 0.0
        self.__lastBrake = 0.0
        self.__brakeFault = 0
        self.__liftAngle = 0.0
        self.__liftFault = 0
        self.__engineOilTemperature = 50.0
        self.__engineFuelTemperature = 50.0
        self.__engineCoolWaterTemperature = 50.0
        self.__wheelSpeedRR = 0.0  # m/s
        self.__wheelSpeedRL = 0.0  # m/s
        self.__wheelSpeedFR = 0.0  # m/s
        self.__wheelSpeedFL = 0.0  # m/s

        self.__lastUpdateTime = time.time()
        self.__lastNotBrakeTime = time.time()

        # 增加人工模式的操作
        self.__manuModeChange = 0

        rospy.Subscriber('guardian/control_cmd', ControlCmd, self.on_control_cmd_callback)
        rospy.Subscriber('guardian/control_cmd_aux', ControlCmdAux, self.on_control_cmd_aux_callback)
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.start_pose_callback)


        self.__localizationDataPub = rospy.Publisher( "/localization/localization_data", LocalizationData, queue_size=1)
        self.__vcuDataPub = rospy.Publisher("/vcu/vcu_data",VcuData,     queue_size=1)
        self.__tfBroadcaster = TransformBroadcaster()

        self.__markerPub = rospy.Publisher("visualization_marker",
                                           Marker,
                                           queue_size=3)

        self.__planningStatus = 0
        # 增加钥匙位的概念
        self.__keyPosition = 3

        # 设置一个怠速
        self.__defaultSpeed = rospy.get_param('/mock_truck/default_speed',
                                              0.25)

        # 增加定位健康度、定位心跳以及vcu心跳的控制
        self.__localizationState = 1
        self.__vcuState = 1


        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.__currentBrake > 0:
                if time.time() - self.__lastNotBrakeTime > 0.8:
                    self.__lastBrake = self.__currentBrake
            else:
                self.__lastNotBrakeTime = time.time()
                self.__lastBrake = 0.0

            # 计算加速度
            if self.__currentBrake > 0:
                self.__a = -self.__lastBrake * 0.1
            else:
                if self.__gear in [Gear.R, Gear.D]:
                    self.__a = self.__currentThrottle * 0.05
                    if self.__currentThrottle < 40 and self.__speed > self.__defaultSpeed:
                        self.__a = -self.__currentThrottle * 0.05

            if self.__keyPosition == 0:
                self.__a = -5

            if self.__speed < 0.3 and self.__a < 0:
                self.__a = 0
                self.__speed = 0

            # 发动机转速
            if self.__keyPosition == 0:
                self.__engineSpeed = self.__speed * 300 + 600 + int(
                    random.random() * 30)
            else:
                if self.__gear in (Gear.D, Gear.R):
                    self.__engineSpeed = self.__speed * 300 + 600 + int(
                        random.random() * 30)
                    if self.__currentThrottle == 0 and self.__currentBrake == 0:
                        if self.__speed > self.__defaultSpeed + 0.1:
                            self.__a = -0.5
                        elif self.__speed < self.__defaultSpeed - 0.1:
                            self.__a = 0.5
                        else:
                            self.__a = 0

                else:
                    self.__engineSpeed = 600 + int(
                        random.random() * 30) + self.__currentThrottle * 400

            # 计算速度
            dv = self.__a * (time.time() - self.__lastUpdateTime)
            v = self.__speed + dv
            if self.__gear == Gear.D:
                if v < 0:
                    v = 0
                if v > self.__speedLimit:
                    v = self.__speedLimit
            elif self.__gear == Gear.R:
                v *= -1
                if v > 0:
                    v = 0
                if v < -self.__speedLimit:
                    v = -self.__speedLimit
            self.__speed = abs(v) if not self.__epb else 0
            self.__wheelSpeedRR = self.__speed
            self.__wheelSpeedRL = self.__speed
            self.__wheelSpeedFR = self.__speed
            self.__wheelSpeedFL = self.__speed

            # 简单的运动学模型，更新车辆位置
            yaw_radian = radians(self.__yaw)  # deg2rad
            self.__ax = self.__a * cos(yaw_radian)
            self.__ay = self.__a * sin(yaw_radian)
            self.__vx = v * cos(yaw_radian) * (1 if not self.get_ros_param('is_debug_skidding', False) else -1)
            self.__vy = v * sin(yaw_radian) * (1 if not self.get_ros_param('is_debug_skidding', False) else -1)
            dx = self.__vx * (time.time() - self.__lastUpdateTime)
            dy = self.__vy * (time.time() - self.__lastUpdateTime)
            dz = self.__z - self.__lastZ

            self.__wz = degrees(v * tan(radians(self.__steeringAngle)) /
                                self.__wheelBase)
            self.__yaw += self.__wz * (time.time() - self.__lastUpdateTime)
            self.__yaw = (self.__yaw + 180.0) % 360.0 - 180.0

            # 横摆运动姿态-四元数
            self.__qz = sin(0.5 * radians(self.__yaw))
            self.__qw = cos(0.5 * radians(self.__yaw))

            if not self.__epb:  # Electronic Parking Brake
                self.__x += dx
                self.__y += dy
            (self.__lat, self.__lon, self.__alt) = self.gps_from_cartesian_offset(
                self.__lat0, self.__lon0, self.__alt0, self.__x, self.__y,
                self.__z)

            # 里程
            self.__odograph += sqrt(dx * dx + dy * dy + dz * dz) * 0.001

            # 车斗举升
            if self.__pto == Pto.UP:
                self.__liftAngle += 2.0 * (time.time() - self.__lastUpdateTime)
            if self.__pto == Pto.DOWN:
                self.__liftAngle -= 2.0 * (time.time() - self.__lastUpdateTime)
            if self.__liftAngle > self.__liftLimit:
                self.__liftAngle = self.__liftLimit + 1e-5
            elif self.__liftAngle < 0:
                self.__liftAngle = 0.0

            # 油耗
            if self.__keyPosition != 0:
                self.__fuelLevel -= 0.01 * (time.time() -
                                            self.__lastUpdateTime)
                if self.__fuelLevel < 0:
                    self.__fuelLevel = 0.0

            # 更新时间
            self.__lastUpdateTime = time.time()
            # 更新z值
            self.__lastZ = self.__z

            # 发布数据
            localization_data = LocalizationData()
            localization_data.header.stamp = rospy.Time.now()
            localization_data.latitude = self.__lat
            localization_data.longitude = self.__lon
            localization_data.altitude = self.__alt
            localization_data.x = self.__x
            localization_data.y = self.__y
            localization_data.z = self.__z
            localization_data.vx = self.__vx
            localization_data.vy = self.__vy
            localization_data.vz = self.__vz
            localization_data.ax = self.__ax
            localization_data.ay = self.__ay
            localization_data.az = self.__az
            localization_data.roll = self.__roll
            localization_data.pitch = self.__pitch
            localization_data.gga_age = self.__ggaage
            localization_data.yaw = self.__yaw
            localization_data.wx = self.__wx
            localization_data.wy = self.__wy
            localization_data.wz = self.__wz
            localization_data.speed = self.__speed
            localization_data.is_valid = True
            localization_data.rtk_status.status = LocalizationStatus.RTKFIX
            if self.__localizationState == 2:
                localization_data.is_valid = False
                localization_data.rtk_status.status = LocalizationStatus.RTKINIT

            if self.__localizationState != 0:
                self.__localizationDataPub.publish(localization_data)

            vcu_data = VcuData()
            vcu_data.header.stamp = rospy.Time.now()
            vcu_data.drive_status.status = self.drive_status
            vcu_data.fuel_level = self.__fuelLevel
            vcu_data.speed = self.__speed
            vcu_data.deceleration = self.__ax
            vcu_data.odograph = self.__odograph
            vcu_data.throttle_pos = self.__currentThrottle
            vcu_data.engine_torque = self.__engineTorque
            vcu_data.engine_speed = self.__engineSpeed
            vcu_data.engine_fault = self.__engineFault
            vcu_data.steering_angle = self.__steeringAngle
            vcu_data.steering_wheel_torque = self.__steeringWheelTorque
            vcu_data.steering_wheel_speed = self.__steeringWheelSpeed
            vcu_data.steering_fault = self.__steeringFault
            vcu_data.steering_motor_torque = self.__steeringMotorTorque
            vcu_data.gear.gear = self.__gear
            vcu_data.transmission_fault = self.__transmissionFault
            vcu_data.epb.epb = self.__epb
            vcu_data.epb_fault = self.__epbFault
            #vcu_data.brake_pos = self.__currentBrake
            vcu_data.brake_fault = self.__brakeFault
            vcu_data.lift_angle = self.__liftAngle
            vcu_data.lift_fault = self.__liftFault
            vcu_data.engine_oil_temperature = self.__engineOilTemperature + (
                    random.random() * 1.0 - 0.5)
            vcu_data.engine_fuel_temperature = self.__engineFuelTemperature + (
                    random.random() * 1.0 - 0.5)
            vcu_data.engine_cool_water_temperature = self.mock_engine_cool_water_temperature()
            vcu_data.wheel_speed_rr = self.__wheelSpeedRR
            vcu_data.wheel_speed_rl = self.__wheelSpeedRL
            vcu_data.wheel_speed_fr = self.__wheelSpeedFR
            vcu_data.wheel_speed_fl = self.__wheelSpeedFL
            vcu_data.total_fuel_consumption = 100 - self.__fuelLevel

            # 增加一下定值
            vcu_data.fuel_rate = 6.8
            vcu_data.eps_presesure = 89
            vcu_data.engine_oil_pressure = 168
            vcu_data.engine_torque = 120
            vcu_data.steering_wheel_torque = 30
            vcu_data.steering_wheel_speed = 0.1
            vcu_data.steering_motor_torque = 0.1
            vcu_data.oil_filter_presesur = 2
            vcu_data.oiltemperature_brake = 15

            vcu_data.brake_light_state = 1 if self.__currentBrake > 0.1 else 0
            vcu_data.back_light_state = 1 if self.__gear == Gear.R else 0
            vcu_data.ac_switch_state = self._vcu_data.ac_switch_state
            vcu_data.transmission_oil_temperature = 23.0

            if self.__currentBrake > 0:
                vcu_data.break_pressure_fr = self.__lastBrake * 12.5
                vcu_data.break_pressure_fl = self.__lastBrake * 12.5
                vcu_data.break_pressure_r1r = self.__lastBrake * 12.5
                vcu_data.break_pressure_r1l = self.__lastBrake * 12.5
                vcu_data.break_pressure_r2r = self.__lastBrake * 12.5
                vcu_data.break_pressure_r2l = self.__lastBrake * 12.5

            vcu_data.pressure_fa = 835 + random.random() * 2.0
            vcu_data.pressure_ra = 835 + random.random() * 2.0
            vcu_data.epb_pressure = self.mock_epb_pressure(vcu_data)
            vcu_data.key_power_state = self._vcu_data.key_power_state
            vcu_data.epower_state = self._vcu_data.epower_state
            vcu_data.pto_state.pto_state = PtoState.NO_PRESS
            vcu_data.xbr_state = 0
            vcu_data.soc = 80.

            vcu_data.low_beam_state = self._vcu_data.low_beam_state
            vcu_data.high_beam_state = self._vcu_data.high_beam_state
            vcu_data.left_light_state = self._vcu_data.left_light_state
            vcu_data.right_light_state = self._vcu_data.right_light_state

            self.mock_light_status(vcu_data)    # 模拟状态

            # 当驾驶模式变成手动的时候，油门变为方向盘
            if self.drive_status == DriveStatus.MANU:
                vcu_data.steering_angle = max(
                    min(vcu_data.steering_angle + random.random() * 1.0 - 0.5,
                        self.__steeringLimit), -self.__steeringLimit)

            if self.__vcuState == 1:
                self.__vcuDataPub.publish(vcu_data)

            self.__tfBroadcaster.sendTransform(
                (self.__x, self.__y, 0.0),
                (self.__qx, self.__qy, self.__qz, self.__qw), rospy.Time.now(),
                self.__baseFrameId, self.__worldFrameId)
            r.sleep()

    @staticmethod
    def gps_from_cartesian_offset(latitude0, longitude0, altitude0, x, y, z):
        lat, lon, alt = enu2blh(latitude0, longitude0, altitude0, x, y, z)
        return lat, lon, alt

    def on_control_cmd_callback(self, data):
        """自动驾驶控制指令消息回调"""
        self.__currentThrottle = data.target_throttle
        self.__currentBrake = data.target_brake
        self.__gear = self.mock_gear(self.__gear, data.target_gear.gear)
        self.__steeringAngle = max(min(data.target_steering, self.__steeringLimit), -self.__steeringLimit)
        rospy.loginfo("control_cmd set to %.1f,%.1f,%.1f" %  (self.__currentThrottle, self.__currentBrake, self.__steeringAngle))

    def on_control_cmd_aux_callback(self, data):
        """自动驾驶辅助控制指令消息回调"""
        self.__epb = self.mock_epb(self.__epb, data.target_epb.epb)
        self.__pto = data.target_pto.pto
        self._vcu_data.ac_switch_state = data.target_airconditioner
        self._vcu_data.low_beam_state = data.target_low_beam
        self._vcu_data.high_beam_state = data.target_high_beam
        self._vcu_data.left_light_state = data.target_left_light
        self._vcu_data.right_light_state = data.target_right_light
        self._vcu_data.key_power_state.key_power_state = KeyPowerState.ON #if data.target_engine else KeyPowerState.OFF
        self._vcu_data.epower_state.epower_state = EpowerState.READY #if data.target_engine else EpowerState.LOW_VOLTAGE



    def start_pose_callback(self, data):
        self.__startX = data.pose.pose.position.x
        self.__startY = data.pose.pose.position.y
        self.__startZ = data.pose.pose.position.z
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        (roll, pitch,  self.__startYaw) = euler_from_quaternion([qx, qy, qz, qw])
        self.__startYaw = self.__startYaw * 180 / pi
        self.__x = self.__startX
        self.__y = self.__startY
        self.__z = self.__startZ
        self.__yaw = self.__startYaw
        (self.__lon, self.__lat, self.__alt) = self.gps_from_cartesian_offset(self.__lon0, self.__lat0,
                                                      self.__alt0, self.__x,
                                                      self.__y, self.__z)
        rospy.loginfo("startpose set to %.2f,%.2f,%.2f" %
                      (self.__startX, self.__startY, self.__startYaw))


    def _get_delayed_value(self, value_from, value_to, key=None, delay=3.5):
        """获取延迟后的值"""
        # 记录变更时间点
        key = key or inspect.currentframe().f_code.co_name
        if time.time() - self.time_start.get(key, 0) > 9:
            if value_from != value_to:
                self.time_start[key] = time.time()

        # 若切换时间超过n秒，则返回变更后的值
        if time.time() - self.time_start.get(key, 0) > delay:
            return value_to
        return value_from


    def mock_epb_pressure(self, vcu_data):
        """模拟epb压力
        :vcu_data: 车底盘数据
        :return: epb_pressure（epb压力）
        """
        epb_pressure = 105
        if vcu_data.epb.epb == Epb.OFF:
            epb_pressure = 780


        if not self.get_ros_param('mock_epb', False):  # 未开启epb mock时，不继续mock曲线
            return epb_pressure

        if time.time() - Params.time_progress_start < 5:  # 不处理程序刚启动的情况
            return epb_pressure

        if self.epb_pressure_mock['epb_to'] != vcu_data.epb.epb:  # 换档时刻
            self.epb_pressure_mock['epb_to'] = vcu_data.epb.epb
        else:  # 其他时刻则更新epb的from/to状态为最新状态即可
            self.epb_pressure_mock['epb_from'] = vcu_data.epb.epb
            self.epb_pressure_mock['epb_to'] = vcu_data.epb.epb

        if self.epb_pressure_mock['epb_from'] == Epb.OFF \
                and self.epb_pressure_mock['epb_to'] == Epb.ON:  # 拉起手刹
            self.epb_pressure_mock['epb_switch_start_time'] = time.time()  # 拉起手刹时刻

        if time.time() - self.epb_pressure_mock['epb_switch_start_time'] < self.MAX_EPB_PRESSURE_DURATION:
            rate = (time.time() - self.epb_pressure_mock['epb_switch_start_time']) / self.MAX_EPB_PRESSURE_DURATION
            rate = 2 / (rate + 1) - 1  # 反比例函数变换式，x = 0, y = 1; x = 5, y = 0
            epb_pressure_base = epb_pressure  # 记录一下基础值，仅用于日志输出
            epb_pressure *= rate
            rospy.loginfo_throttle(0.35, "正在模拟epb压力降低，基础值：{}，模拟值：{}，比例：{}"
                                   .format(epb_pressure_base, epb_pressure, round(rate, 2)))

        return epb_pressure

    def mock_engine_cool_water_temperature(self):
        """模拟发动机冷却液温度"""
        if self.get_ros_param('mock_engine_cool_water_temperature', False):
            return 96.0  # 模拟一个高温值
        return self.__engineCoolWaterTemperature + (random.random() * 1.0 - 0.5)

    def mock_epb(self, value_from, value_to):
        """模拟手刹"""
        return self._get_delayed_value(value_from, value_to, delay=1.5)

    def mock_gear(self, value_from, value_to):
        """模拟档位"""
        return self._get_delayed_value(value_from, value_to, delay=3.5)

    @staticmethod
    def mock_light_status(vcu_data):
        """模拟状态"""
        if False:
            vcu_data.left_light_state = 1  # 左转向灯
            vcu_data.right_light_state = 1  # 右转向灯
            vcu_data.pos_light_state = 1  # 位置灯
            vcu_data.int_light_state = 1  # 室内灯

            vcu_data.low_beam_state = 1  # 近光灯
            vcu_data.high_beam_state = 1  # 远光灯
            vcu_data.lift_warn_state = 1  # 告警灯
            vcu_data.back_light_state = 1  # 倒车灯
            vcu_data.ac_switch_state = 1  # 压缩机工作状态
            vcu_data.smokealarmsw_state = 1  # 烟雾告警开关
            vcu_data.airconditioner_state = 1  # 空调工作状态

            vcu_data.horn_state = 1  # 喇叭
            vcu_data.wiper_slow_state = 1   # 雨刷
            vcu_data.wiper_fast_state = 1
            vcu_data.wiper_clean_state = 1
    
    def get_ros_param(self, key, default_value=None):
        """ 防止因为get_param导致mock_truck """
        if key not in self._params.keys():
            self._params[key] = rospy.get_param(key, default_value)
        return self._params[key]

if __name__ == '__main__':
    rospy.init_node('mock_truck')
    mockTruck = MockTruck()
