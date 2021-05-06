from __future__ import print_function

import sys
import time
import termios
import logging
import threading
import math

import yaml  # ros的话题接收数据的处理
import rospy
import geometry_msgs.msg as geometry_msgs
import geometry_msgs.msg as Accel

import numpy as np
import transformations as trans
from cflib import crazyflie, crtp
from cflib.crazyflie.log import LogConfig
from cflib.drivers.crazyradio import Crazyradio

# Set a channel - if set to None, the first available crazyflie is used
# URI = 'radio://0/101/2M'
URI = 'radio://0/80/2M/E7E7E7E702'
URI2 = None
count = 0
timeout_sum = 0.0
mode = 0  # 0 悬停模式 1 轨迹跟踪模式 2 降落模式

command_x = [0]
command_y = [0]
command_z = [0]
l = []
x = y = z = x0 = y0 = z0 = 0
ex = 0.0
ey = 0.0
ez = 0.0
flag = 1
control_flag = 0
seq = 0

des_vz = 1  # 高度方向速度期望值
des_vx = 0  # 水平方向ｘｙ速度期望值
des_vy = 0

# 位置环控制参数
# 位置参数
kp_x = 1  # 水平
kp_y = 1

# 速度参数
kd_v_x = 0
kd_v_y = 0
kd_v_z = 0

# 速度环控制参数
kp_vx = 8
kp_vy = 10
ki_vx = 0.002
ki_vy = 0.002
kd_vx = 0
kd_vy = 0

kp_z = 1  # 高度
kp_vz = 4
ki_vz = 0.05

ki_x = 0  # 积分参数
ki_y = 0
ki_z = 0

kd_x = 0  # 微分参数
kd_y = 0
kd_z = 0
kd_vz = 0

err_x = 0  # 误差参数
err_y = 0
err_z = 0

integral_vx = 0  # 积分累计
integral_vy = 0
integral_vz = 0

err_last_x = 0  # 过去误差保留
err_last_y = 0
err_last_z = 0
err_last_vx = 0
err_last_vy = 0
err_last_vz = 0


def A_parameter(t):
    a = np.array([[math.sin(t), -math.cos(t)], [math.cos(t), math.sin(t)]])

    return a


def sat_xy(x, a):
    if max(abs(x[0]), abs(x[1])) <= a:
        angular = x
    else:
        angular = a * x / max(abs(x[0]), abs(x[1]))
    return angular


def sat(x, a):
    if abs(x) <= a:
        thrust = x
    else:
        if x < 0:
            thrust = -a
        else:
            thrust = a
    return thrust


def callback(data):
    global x0, y0, z0, ex, ey, ez, seq
    seq = data.header.seq
    pose_x = data.pose.position.x
    pose_y = data.pose.position.y
    pose_z = data.pose.position.z

    ex = pose_x - x0
    ey = pose_y - y0
    ez = pose_z - z0

    print(seq, ex, ey, ez)

    x0 = pose_x
    y0 = pose_y
    z0 = pose_z


def trans_quaternion(q0, q1, q2, q3):
    matrix = np.zeros((3, 3))
    matrix[0, 0] = square(q0) + square(q1) - square(q2) - square(q3)
    matrix[0, 1] = 2 * (q1 * q2 - q0 * q3)
    matrix[0, 2] = 2 * (q1 * q3 + q0 * q2)
    matrix[1, 0] = 2 * (q1 * q2 + q0 * q3)
    matrix[1, 1] = square(q0) - square(q1) + square(q2) - square(q3)
    matrix[1, 2] = 2 * (q2 * q3 - q0 * q1)
    matrix[2, 0] = 2 * (q1 * q3 - q0 * q2)
    matrix[2, 1] = 2 * (q2 * q3 + q0 * q1)
    matrix[2, 2] = square(q0) - square(q1) - square(q2) + square(q3)
    return matrix


def square(a):
    s = math.pow(a, 2)
    return s


class PoseGenerator():

    def __init__(self):
        self._sub_pose = rospy.Subscriber('/vrpn_client_node/cf1/pose', geometry_msgs.PoseStamped, callback,
                                          queue_size=1)
        # self._filename = filename

    # def _write_file(self):
    #     ways = {}
    #     ways['header'] = self._waypoints
    #     with open(self._filename, 'w') as f:
    #         f.write(yaml.dump(ways, default_flow_style=False))
    def spin(self):
        rospy.spin()
        # self._write_file()

def read_input(file=sys.stdin):
    """Registers keystrokes and yield these every time one of the
    *valid_characters* are pressed.
    从键盘读取输入值
	"""

    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        while True:
            try:
                yield sys.stdin.read(1)
            except (KeyboardInterrupt, EOFError):
                break
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)


def extract(data, t, starting):  # 返回x或y或z调用的起始和终止序列
    start = data.index(t, starting)
    long = len(data)
    for b in range(start, long):
        end = b
        if data[b] == '\n':
            return start, end


#
# def extract(id_str, t, start):  # 返回x或y或z调用的起始和终止序列
#     starting = l[id_str].index(t, start)
#     long = len(l[id_str])
#     for b in range(starting, long):
#         end = b
#         if l[id_str][b] == '\n':
#             return starting, end


def read_error(starting, ending, data):  # 读取数据,返回当前时刻ex、ey、ez的数值数据
    """
     从列表中取两组数据，得差后返回偏差ex,ey,ez，因此尽管能取n组数据，但是只能返回n-1个偏差
    """
    global x0, y0, z0, x, y, z, ex, ey, ez, seq
    for i in range(starting, ending):
        (start1, end1) = extract(i, 'x')
        (start2, end2) = extract(i, 'y')
        (start3, end3) = extract(i, 'z')
        (start4, end4) = extract(i, 's')
        x = float(''.join(data[i][start1 + 3:end1]))
        y = float(''.join(data[i][start2 + 3:end2]))
        z = float(''.join(data[i][start3 + 3:end3]))
        seq = float(''.join(data[i][start4 + 4:end4]))
        ex = x - x0
        ey = y - y0
        ez = z - z0
        x0 = x
        y0 = y
        z0 = z
    return ex, ey, ez, seq


class ControllerThread(threading.Thread):
    period_in_ms = 20  # Control period. [ms] 每20ms发送一次控制指令
    thrust_step = 5e3  # Thrust step with W/S. [65535 = 100% PWM duty cycle]
    thrust_initial = 0
    thrust_limit = (0, 65535)
    roll_limit = (-30.0, 30.0)
    pitch_limit = (-30.0, 30.0)
    yaw_limit = (-200.0, 200.0)
    g = 9.8
    m = 40
    enabled = False

    def __init__(self, cf, ax, ay, az, des_x, des_y, des_z):  # 重写init
        super(ControllerThread, self).__init__()
        self.cf = cf
        self.ax = ax
        self.ay = ay
        self.az = az
        self.des_x = [0.0] * 100
        self.des_x = des_x
        self.des_y = [0.0] * 100
        self.des_y = des_y
        self.des_z = des_z
        # Reset state
        self.disable(stop=False)

        # Keeps track of when we last printed
        self.last_time_print = 0.0

        # Connect some callbacks from the Crazyflie API
        self.cf.connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)
        self.send_setpoint = self.cf.commander.send_setpoint

        # Pose estimate from the Kalman filter
        self.pos = np.r_[0, 0, 0]
        self.vel = np.r_[0, 0, 0]
        self.attq = np.r_[0.0, 0.0, 0.0, 1.0]
        self.R = np.eye(3)

        # Attitide (roll, pitch, yaw) from stabilizer
        self.stab_att = np.r_[0.0, 0.0, 0.0]

        # This makes Python exit when this is the only thread alive.
        self.daemon = True

    def _connected(self, link_uri):
        print('Connected to', link_uri)

        log_stab_att = LogConfig(name='Stabilizer', period_in_ms=self.period_in_ms)
        log_stab_att.add_variable('stabilizer.roll', 'float')
        log_stab_att.add_variable('stabilizer.pitch', 'float')
        log_stab_att.add_variable('stabilizer.yaw', 'float')
        self.cf.log.add_config(log_stab_att)

        log_pos = LogConfig(name='Kalman Position', period_in_ms=self.period_in_ms)
        log_pos.add_variable('kalman.stateX', 'float')
        log_pos.add_variable('kalman.stateY', 'float')
        log_pos.add_variable('kalman.stateZ', 'float')
        self.cf.log.add_config(log_pos)

        log_vel = LogConfig(name='Kalman Velocity', period_in_ms=self.period_in_ms)
        log_vel.add_variable('kalman.statePX', 'float')
        log_vel.add_variable('kalman.statePY', 'float')
        log_vel.add_variable('kalman.statePZ', 'float')
        self.cf.log.add_config(log_vel)

        log_att = LogConfig(name='Kalman Attitude',
                            period_in_ms=self.period_in_ms)
        log_att.add_variable('kalman.q0', 'float')
        log_att.add_variable('kalman.q1', 'float')
        log_att.add_variable('kalman.q2', 'float')
        log_att.add_variable('kalman.q3', 'float')
        self.cf.log.add_config(log_att)

        if log_stab_att.valid and log_pos.valid and log_vel.valid and log_att.valid:
            log_stab_att.data_received_cb.add_callback(self._log_data_stab_att)
            log_stab_att.error_cb.add_callback(self._log_error)
            log_stab_att.start()

            log_pos.data_received_cb.add_callback(self._log_data_pos)
            log_pos.error_cb.add_callback(self._log_error)
            log_pos.start()

            log_vel.error_cb.add_callback(self._log_error)
            log_vel.data_received_cb.add_callback(self._log_data_vel)
            log_vel.start()

            log_att.error_cb.add_callback(self._log_error)
            log_att.data_received_cb.add_callback(self._log_data_att)
            log_att.start()
        else:
            raise RuntimeError('One or more of the variables in the configuration was not'
                               'found in log TOC. Will not get any position data.')

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)

    def _log_data_stab_att(self, timestamp, data, logconf):
        self.stab_att = np.r_[data['stabilizer.roll'],
                              data['stabilizer.pitch'],
                              data['stabilizer.yaw']]

    def _log_data_pos(self, timestamp, data, logconf):
        self.pos = np.r_[data['kalman.stateX'],
                         data['kalman.stateY'],
                         data['kalman.stateZ']]

    def _log_data_vel(self, timestamp, data, logconf):
        vel_bf = np.r_[data['kalman.statePX'],
                       data['kalman.statePY'],
                       data['kalman.statePZ']]
        self.vel = np.dot(self.R, vel_bf)

    # self.vel=[kalman.statex,kalman.statey,kalman.statez]

    def _log_data_att(self, timestamp, data, logconf):
        # NOTE q0 is real part of Kalman state's quaternion, but
        # transformations.py wants it as last dimension.
        self.attq = np.r_[data['kalman.q1'], data['kalman.q2'],
                          data['kalman.q3'], data['kalman.q0']]
        # Extract 3x3 rotation matrix from 4x4 transformation matrix
        self.R = trans.quaternion_matrix(self.attq)[:3, :3]
        # r, p, y = trans.euler_from_quaternion(self.attq)    从四元数中得到欧拉角

    def _log_error(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    def make_position_sanity_check(self):
        # We assume that the position from the LPS should be
        # [-20m, +20m] in xy and [0m, 5m] in z
        if np.max(np.abs(self.pos[:2])) > 20 or self.pos[2] < 0 or self.pos[2] > 5:
            raise RuntimeError('Position estimate out of bounds', self.pos)

    def run(self):  # 控制环 20ms发送一次控制指令
        """Control loop definition"""
        global x, y, z, x0, y0, z0, ex, ey, ez, flag, control_flag, seq, kp_x, kp_y, ki_x, ki_y, err_x, err_y, \
            integral_x, integral_y, err_last_x, err_last_y, integral_vz, err_last_vz, kp_z, \
            des_vz, integral_vx, integral_vy, err_last_vx, err_last_vy, des_vx, des_vy, kp_vx, kp_vy, ki_vx, ki_vy, kd_vx, kd_vy, command_x, command_y, command_z

        index_x = 0.0
        index_y = 0.0
        flag_position = 10
        time_total_start = 0
        count_times = 0
        times = 0
        count_ending_trajectory = 0
        j = 0
        q = [0.0] * 4
        w = [0.0] * 4
        q1 = [0]
        q2 = [0]
        q3 = [0]
        q4 = [0]
        p = []
        pose_qua = []

        ev_x = 0
        ev_y = 0
        evz = 0
        old_x = 0  # 过去值
        old_y = 0
        old_z = 0

        vel_x = 0  # 实际值
        vel_y = 0
        vel_z = 0

        old_vel_x = 0  # 上一组速度值
        old_vel_y = 0
        old_vel_z = 0

        pose_x = 0
        pose_y = 0
        pose_z = 0

        gap = 0
        t = 0
        seq = 0

        pose_x_old = 0
        pose_y_old = 0
        pose_z_old = 0

        q1_old = 0
        q2_old = 0
        q3_old = 0
        q4_old = 0.1

        track = 0

        while not self.cf.is_connected():
            time.sleep(0.2)
        print('Waiting for position estimate to be good enough...')
        self.reset_estimator()

        self.make_position_sanity_check()

        # Set the current reference to the current positional estimate, at a
        # slight elevation
        self.pos_ref = np.r_[self.pos[:2], 1.0]
        self.yaw_ref = 0.0
        print('Initial positional reference:', self.pos_ref)
        print('Initial thrust reference:', self.thrust_r)
        print('Ready! Press e to enable motors, h for help and Q to quit')
        # pose = rospy.wait_for_message('/vrpn_client_node/cf2/pose', geometry_msgs.PoseStamped, 1)
        while True:
            time_start = time.time()
            # des_x = self.des_x[j]  # 期望值
            # des_y = self.des_y[j]
            # des_z = self.des_z

            if self.enabled:
                times += 1
                print('***************************************starting*********************************************')
                print('times:', times)

                print('time: ({})\n'.format(time.time()))
                # 文本处理方法嵌入点
                with open("/home/l/cf2.txt", 'r') as file:  # 打开数据文件，将一维列表按照包拆分成二维列表，每个数据包作为其中一行
                    try:
                        l = file.read()
                        data = l
                        # data = l.split('---')  # 包含数据的索引范围为0-（len(data)-2）  因为数据包的结尾是---，所以多分一个空数据包
                        # for i in range(0, len(data) - 1):  # 0-(len-1),取不到len-1 ，最高索引值len-2
                        (start1, end1) = extract(data, 'x', 0)
                        (start2, end2) = extract(data, 'y', 0)
                        (start3, end3) = extract(data, 'z', 0)

                        (start4, end4) = extract(data, 'x', end1)
                        (start5, end5) = extract(data, 'y', end2)
                        (start6, end6) = extract(data, 'z', end3)
                        (start7, end7) = extract(data, 'w', end6)       # 四元数的实轴

                        pose_x = float(data[start1 + 3:end1])   # pose_x代表由动捕系统传入的X位置坐标
                        pose_y = float(data[start2 + 3:end2])
                        pose_z = float(data[start3 + 3:end3])

                        q1 = float(data[start4 + 3:end4])               # x
                        q2 = float(data[start5 + 3:end5])               # y
                        q3 = float(data[start6 + 3:end6])               # z
                        q4 = float(data[start7 + 3:end7])               # w

                    except Exception:
                        track += 1
                        print('track:', track)
                        pose_x = pose_x_old
                        pose_y = pose_y_old
                        pose_z = pose_z_old

                        q1 = q1_old
                        q2 = q2_old
                        q3 = q3_old
                        q4 = q4_old

                pose_x_old = pose_x
                pose_y_old = pose_y
                pose_z_old = pose_z

                q1_old = q1
                q2_old = q2
                q3_old = q3
                q4_old = q4

                time_data = time.time()
                time_data_deal = time_data - time_start
                print('time_data_deal:', time_data_deal)
                # time_now_s = pose.header.stamp.secs
                # time_now_ns = pose.header.stamp.nsecs
                # seq = pose.header.seq
                pose_real_x = pose_x
                pose_real_y = pose_z
                pose_real_z = pose_y
                q[0] = q4
                q[1] = q1
                q[2] = q2
                q[3] = q3

                w[0] = q1
                w[1] = q2
                w[2] = q3
                w[3] = q4
                time_now = time.time()

                pitch, roll, yaw = trans.euler_from_quaternion(w)  # 从四元数提取欧拉角
                roll = math.degrees(roll)
                pitch = math.degrees(pitch)
                yaw = math.degrees(yaw)

                p = np.ones((3, 1))
                p[0] = pose_x
                p[1] = pose_y
                p[2] = pose_z
                # rotate_matrix = trans_quaternion(q[0], q[1], q[2], q[3])       # 构建四元数旋转矩阵
                # pose_qua = np.dot(rotate_matrix, p)
                rotate_matrix = trans_quaternion(q[0], q[1], q[2], q[3])  # 构建四元数旋转矩阵

                pose_qua = np.dot(rotate_matrix, p)
                pose_qua_x = pose_qua[0]
                pose_qua_y = pose_qua[2]
                pose_qua_z = pose_qua[1]

                if t == 1:  # 差分得到速度
                    gap = time_now - time_old
                    vel_x = (pose_qua_x - old_x) / gap  # 当前时刻x轴速度
                    vel_y = (pose_qua_y - old_y) / gap  # 当前时刻y轴速度
                    vel_z = (pose_real_z - old_z) / gap  # 当前时刻z轴速度

                time_old = time_now
                old_x = pose_qua_x  # 保存上一组位置信息
                old_y = pose_qua_y
                old_z = pose_real_z

                old_vel_x = vel_x  # 保存上一组速度信息
                old_vel_y = vel_y
                old_vel_z = vel_z

                t = 1

                if mode == 0:  # 悬停模式
                    if control_flag == 0:
                        time_total_start = time.time()
                    print('------------------->mode:', mode)
                    des_x = 0  # 期望值
                    des_y = 0
                    des_z = 0.5
                    des_v_x = 0
                    des_v_y = 0
                    des_v_z = 0

                    ex = des_x - pose_qua_x  # 得到位置偏差
                    ey = des_y - pose_qua_y
                    ez = pose_real_z - des_z
                    if track > 0:     # track是什么？
                        err_vx = des_vx - old_vel_x  # 得到速度偏差
                        err_vy = des_vy - old_vel_y
                        err_vz = des_vz - old_vel_z
                        track = 0
                    else:
                        err_vx = des_vx - vel_x  # 得到速度偏差
                        err_vy = des_vy - vel_y
                        err_vz = des_vz - vel_z                                          # 两次得到的没有区别？？？？？？？？？？？？？？？？
                    if control_flag == 1:
                        flag_position += 1
                        # if flag_position >= 2:  # 外环位置环 40ms 计算期望速度 P控制
                        #     des_vx = kp_x * (des_x - pose_qua_x)  # ｘ轴方向
                        #     des_vy = kp_y * (des_y - pose_qua_y)  # y轴方向
                        #     des_vz = kp_z * sat((des_z - pose_real_z), 10)  # 高度
                        #     flag_position = 0

                        if flag_position >= 2:  # 外环位置环 40ms 计算期望速度 P控制
                            des_vx = kp_x * sat((des_x - pose_qua_x), 5)  # ｘ轴方向
                            des_vy = kp_y * sat((des_y - pose_qua_y), 10)  # y轴方向
                            des_vz = kp_z * sat((des_z - pose_real_z), 10)  # 高度
                            flag_position = 0

                        # if flag_position >= 2:  # 外环位置环 40ms 计算期望速度 PD控制
                        #     des_vx = sat(kp_x * (des_x - pose_qua_x)+kd_v_x * (des_v_x-vel_x), 2)  # ｘ轴方向
                        #     des_vy = sat(kp_y * (des_y - pose_qua_y)+kd_v_y * (des_v_y-vel_y), 2)  # y轴方向
                        #     # des_vz = -kp_z * sat((pose_real_z - des_z), 10)  # 高度
                        #     des_vz = sat(kp_z * (des_z-pose_real_z)+kd_v_z * (des_v_z-vel_z), 2)  # 高度
                        #     flag_position = 0

                        # # ..............................变积分..............................
                        # if abs(integral_vx) < 100:  # x轴方向变积分
                        #     index_x = 1
                        # elif 100 < abs(integral_vx) < 140:
                        #     index_x = (160 - abs(integral_vx)) / 40
                        # else:
                        #     index_x = 0
                        #
                        # if abs(integral_vy) < 100:  # y轴方向变积分
                        #     index_y = 1
                        # elif 100 < abs(integral_vy) < 140:
                        #     index_y = (160 - abs(integral_vy)) / 40
                        # else:
                        #     index_y = 0

                        # ..............................消除目标位置附近的积分累加..............................
                        if square(err_vx)+square(err_vy)+square(err_vz) <= 0.03*0.03:  # x轴方向变积分
                            index_x = 1
                            integral_vx = 0
                        else:
                            index_x = 1

                        if square(err_vx)+square(err_vy)+square(err_vz) <= 0.03*0.03:  # y轴方向变积分
                            index_y = 1
                            integral_vy = 0
                        else:
                            index_y = 1

                        # ..............................速度控制环..............................
                        integral_vz += err_vz  # z轴方向内环 20ms 计算控制输出thrust
                        self.thrust_r = sat(
                            self.m * 100 * (
                                        self.g + kp_vz * err_vz + ki_vz * integral_vz + kd_vz * (err_vz - err_last_vz)),
                            0xfffe)
                        err_last_vz = err_vz

                        integral_vx += err_vx  # ｘ轴方向内环 20ms 计算控制输出pitch
                        a = (kp_vx * err_vx) + (index_x * ki_vx * integral_vx) + (kd_vx * (err_vx - err_last_vx))
                        self.pitch_r = sat(a, 2)
                        err_last_vx = err_vx

                        integral_vy += err_vy  # y轴方向内环 20ms 计算控制输出roll
                        b = (kp_vy * err_vy) + (index_y * ki_vy * integral_vy) + (kd_vy * (err_vy - err_last_vy))
                        self.roll_r = sat(b, 2)
                        err_last_vy = err_vy

                        # 增加前馈补偿
                        self.pitch_r = sat(a, 1.5) - 1.3
                        self.roll_r = sat(b, 1.5) - 0.4
                        # self.pitch_r = -1.5
                        # self.roll_r = -0.1

                        print('flag_position:', flag_position)
                        print('position_des: ({}, {}, {})'.format(des_x, des_y, des_z))
                        print('position_qua: ({}, {}, {})'.format(pose_qua_x, pose_qua_y, pose_qua_z))
                        print('error_pos: ({}, {}, {})'.format(ex, ey, ez))
                        print('position: ({}, {}, {})'.format(pose_x, pose_y, pose_real_z))
                        print('kp_xyz:({}, {}, {})'.format(kp_x, kp_y, kp_z))
                        print('ki_xyz:({}, {}, {})'.format(ki_x, ki_y, ki_z))
                        print('velocity_des: ({}, {}, {})'.format(des_vx, des_vy, des_vz))
                        print('velocity: ({}, {}, {})'.format(vel_x, vel_y, vel_z))
                        print('error_vel: ({}, {}, {})'.format(err_vx, err_vy, err_vz))
                        print('error_proportion: ({}, {}, {})'.format(kp_vx * err_vx,
                                                                      kp_vy * err_vy,
                                                                      kp_vz * err_vz))
                        print('error_integral: ({}, {}, {})'.format(integral_vx, integral_vy, integral_vz))
                        print('error_difference: ({}, {}, {})'.format(kd_vx * (err_vx - err_last_vx),
                                                                      kd_vy * (err_vy - err_last_vy),
                                                                      kd_vz * (err_vz - err_last_vz)))
                        print('total_time:', (time.time() - time_total_start))
                        print('---------->former_control: ({}, {}, {}, {})<----------'.format(sat(a, 2), sat(b, 2),
                                                                                              self.yawrate_r,
                                                                                              self.thrust_r))
                    else:
                        pass
                    control_flag = 1
                if mode == 1:  # 轨迹跟踪模式
                    if control_flag == 0:
                        time_total_start = time.time()
                    print('------------------->mode:', mode)
                    des_x = self.des_x[j]  # 期望值
                    des_y = self.des_y[j]
                    des_z = self.des_z

                    ex = des_x - pose_qua_x  # 得到位置偏差
                    ey = des_y - pose_qua_y
                    ez = pose_real_z - des_z

                    if control_flag == 1:
                        flag_position += 1
                        if flag_position > 2:  # 外环位置环 100ms 计算期望速度 P控制
                            count_times += 1
                            if count_times >= 2:  # 每个位置点进行2次位置PID的调节
                                count_times = 0
                                j += 1
                                if j >= 199:
                                    j = 0
                                    count_ending_trajectory += 1     # 计算圈数
                            des_vx = kp_x * (des_x - pose_qua_x)  # ｘ轴方向
                            des_vy = kp_y * (des_y - pose_qua_y)  # y轴方向
                            des_vz = -kp_z * sat((pose_real_z - des_z), 10)  # 高度
                            flag_position = 0              # flag_position？？？？？？？？？？？？？

                        # if times == 2:
                        #     j += 1
                        #     times = 0
                        #     print('**************des_j**************:', j)
                        # else:
                        #     pass
                        # if j >= 999:
                        #     j = 0
                        err_vz = des_vz - vel_z  # z轴方向内环 20ms 计算控制输出thrust
                        integral_vz += err_vz
                        self.thrust_r = sat(
                            self.m * 100 * (
                                    self.g + kp_vz * err_vz + ki_vz * integral_vz + kd_vz * (err_vz - err_last_vz)),
                            0xfffe)
                        err_last_vz = err_vz

                        err_vx = des_vx - vel_x  # ｘ轴方向内环 20ms 计算控制输出pitch
                        integral_vx += err_vx
                        a = (kp_vx * err_vx) + (ki_vx * integral_vx) + (kd_vx * (err_vx - err_last_vx))
                        self.pitch_r = sat(a, 2) - 1.3
                        err_last_vx = err_vx

                        err_vy = des_vy - vel_y  # y轴方向内环 20ms 计算控制输出roll
                        integral_vy += err_vy
                        b = (kp_vy * err_vy) + (ki_vy * integral_vy) + (kd_vy * (err_vy - err_last_vy))
                        self.roll_r = sat(b, 2) - 0.4
                        err_last_vy = err_vy
                        print('position_des: ({}, {}, {})'.format(des_x, des_y, des_z))
                        print('position_qua: ({}, {}, {})'.format(pose_qua_x, pose_qua_y, pose_qua_z))
                        print('error_pos: ({}, {}, {})'.format(ex, ey, ez))
                        print('position: ({}, {}, {})'.format(pose_x, pose_y, pose_real_z))
                        print('kp_xyz:({}, {}, {})'.format(kp_x, kp_y, kp_z))
                        print('ki_xyz:({}, {}, {})'.format(ki_x, ki_y, ki_z))
                        print('velocity_des: ({}, {}, {})'.format(des_vx, des_vy, des_vz))
                        print('velocity: ({}, {}, {})'.format(vel_x, vel_y, vel_z))
                        print('error_vel: ({}, {}, {})'.format(err_vx, err_vy, err_vz))
                        print('error_integral: ({}, {}, {})'.format(integral_vx, integral_vy, integral_vz))
                        print('total_time:', (time.time() - time_total_start))
                        print('count_ending_trajectory, j:', count_ending_trajectory, j)
                    else:
                        pass
                    control_flag = 1

                if mode == 2:  # 降落模式
                    self.thrust_r = 45000
                    if pose_real_z < 0.1:
                        self.thrust_r = 0
                sp = (self.roll_r, self.pitch_r, self.yawrate_r, int(self.thrust_r))
                self.send_setpoint(*sp)
                print('---------->control: ({}, {}, {}, {})<----------'.format(self.pitch_r, self.roll_r,
                                                                               self.yawrate_r,
                                                                               self.thrust_r))
                print('successfully send data to crazyflie :({})\n'.format(self.cf))
                time_end = time.time()
                print('control_time', time_end - time_start)
                print('***************************************ending*********************************************\n\n')
                self.loop_sleep(time_start)

    def calc_control_signals(self):
        '''
            This is where you should put your control code that outputs the refrence values for roll
        ,pitch,yawrate and thrust which are taken care of by the onboard control loops
        '''
        # THIS IS WHERE YOU SHOULD PUT YOUR CONTROL CODE
        # THAT OUTPUTS THE REFERENCE VALUES FOR
        # ROLL PITCH, YAWRATE AND THRUST
        # WHICH ARE TAKEN CARE OF BY THE ONBOARD CONTROL LOOPS

        roll, pitch, yaw = trans.euler_from_quaternion(self.attq)

        # Compute control errors in posi tion
        # ex, ey, ez = self.pos_ref - self.pos
        '''
              The code below will simply send the thrust that you can set using the keyboard and put all other control
               signals to zero. It also shows how, using numpy, you can threshold the signals to be between the lower and 
               upper limits defined by the arrays *_limit
        '''

        '''
        parameter setting for cf
        '''

        # self.roll_r = np.clip(0.0, *self.roll_limit)
        # self.pitch_r = np.clip(0.0, *self.pitch_limit)
        # self.yawrate_r = np.clip(0.0, *self.yaw_limit)
        self.thrust_r = np.clip(self.thrust_r, *self.thrust_limit)

        # self.ax = 5
        # self.ay = 5
        # self.az = 5

        # self.pitch_r = (self.flag_g * self.ax * 100) * 20  # rotation around Y axis
        # self.roll_r = (-self.flag_g * self.ay * 100) * 20  # rotation around X axis
        # self.yawrate_r = 0  # rotation around Z axis
        # self.thrust_r = (27 * self.az * 100 + 27 * self.g)
        if control_flag == 0:
            self.pitch_r = 0
            self.roll_r = 0
            self.thrust_r = 45000
            self.yawrate_r = 0  # rotation around Z axis
        else:
            pass
        message = ('code of crazyflie : ({})\n'.format(self.cf) +
                   'time : ({})\n'.format(time.ctime()) +
                   'ref: ({}, {}, {}, {})\n'.format(self.pos_ref[0], self.pos_ref[1], self.pos_ref[2], self.yaw_ref) +
                   'pos: ({}, {}, {}, {})\n'.format(self.pos[0], self.pos[1], self.pos[2], yaw) +
                   'vel: ({}, {}, {})\n'.format(self.vel[1], self.vel[1], self.vel[2]) +
                   'error: ({}, {}, {})\n'.format(ex, ey, ez) +
                   'control: ({}, {}, {}, {})\n'.format(self.pitch_r, self.roll_r, self.yawrate_r, self.thrust_r) +
                   'accelerate({},{},{})\n'.format(self.ax, self.ay, self.az)
                   )

        self.print_at_period(2.0, message)

    def print_at_period(self, period, message):
        """ Prints the message at a given period """
        if (time.time() - period) > self.last_time_print:
            self.last_time_print = time.time()
            print(message)

    def reset_estimator(self):
        self.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.cf.param.set_value('kalman.resetEstimation', '0')
        # Sleep a bit, hoping that the estimator will have converged
        # Should be replaced by something that actually checks...
        time.sleep(1.5)

    def disable(self, stop=True):
        if stop:
            self.send_setpoint(0.0, 0.0, 0.0, 0)
        if self.enabled:
            print('Disabling controller')
        self.enabled = False
        self.roll_r = 0.0
        self.pitch_r = 0.0
        self.yawrate_r = 0.0
        self.thrust_r = self.thrust_initial

    def enable(self):
        if not self.enabled:
            print('Enabling controller')
        # Need to send a zero setpoint to unlock the controller.
        self.send_setpoint(0.0, 0.0, 0.0, 0)
        self.enabled = True

    def loop_sleep(self, time_start):
        """ Sleeps the control loop to make it run at a specified rate """
        global count, timeout_sum
        delta_time = 1e-3 * self.period_in_ms - (time.time() - time_start)
        if delta_time > 0:
            time.sleep(delta_time)
        else:
            timeout_sum += -delta_time
            count += 1
            print('Deadline missed by', -delta_time, 'seconds. ''Too slow control loop!'
                                                     '.........................................', 'total_time:',
                  timeout_sum, 'count:', count)

    def increase_thrust(self):
        self.thrust_r += self.thrust_step
        self.thrust_r = min(self.thrust_r, 0xffff)

    def decrease_thrust(self):
        self.thrust_r -= self.thrust_step
        self.thrust_r = max(0, self.thrust_r)


def handle_keyboard_input(control1):
    pos_step = 0.1  # [m]
    yaw_step = 5  # [deg]
    global mode, control_flag
    for ch in read_input():
        if ch == 'h':
            print('Key map:')
            print('>: Increase thrust (non-control mode)')
            print('<: Decrease thrust (non-control mode)')
            print('Q: quit program')
            print('e: Enable motors')
            print('q: Disable motors')
            print('w: Increase x-reference by ', pos_step, 'm.')
            print('s: Decrease x-reference by ', pos_step, 'm.')
            print('a: Increase y-reference by ', pos_step, 'm.')
            print('d: Decrease y-reference by ', pos_step, 'm.')
            print('i: Increase z-reference by ', pos_step, 'm.')
            print('k: Decrease z-reference by ', pos_step, 'm.')
            print('j: Increase yaw-reference by ', yaw_step, 'm.')
            print('l: Decrease yaw-reference by ', yaw_step, 'deg.')
        elif ch == '>':
            control1.increase_thrust()
            print('Increased thrust to', control1.thrust_r)
        elif ch == '<':
            control1.decrease_thrust()
            print('Decreased thrust to', control1.thrust_r)
        elif ch == 'w':
            # control1.pos_ref[0] += pos_step
            # print('Reference position changed to :', control1.pos_ref)
            mode = 2  # 降落模式
        elif ch == 's':
            control1.pos_ref[0] -= pos_step
            print('Reference position changed to :', control1.pos_ref)
        elif ch == 'a':
            control1.pos_ref[1] += pos_step
            print('Reference position changed to :', control1.pos_ref)
        elif ch == 'd':
            control1.pos_ref[1] -= pos_step
            print('Reference position changed to :', control1.pos_ref)
        elif ch == 'i':
            control1.pos_ref[2] += pos_step
            print('Reference position changed to :', control1.pos_ref)
        elif ch == 'k':
            control1.pos_ref[2] -= pos_step
            print('Reference position changed to :', control1.pos_ref)
        elif ch == 'j':
            control1.yaw_ref += np.radians(yaw_step)
            print('Yaw reference changed to :',
                  np.degrees(control1.yaw_ref), 'deg.')
        elif ch == 'l':
            # control1.yaw_ref -= np.radians(yaw_step)
            # print('Yaw reference changed to :',
            #       np.degrees(control1.yaw_ref), 'deg.')
            mode = 1  # 轨迹跟踪模式
            control_flag = 0
        elif ch == ' ':
            control1.pos_ref[2] = 0.0
            print('Reference position changed to :', control1.pos_ref)
        elif ch == 'e':
            control1.enable()
            # control2.enable()
        elif ch == 'q':
            if not control1.enabled:
                print('Uppercase Q quits the program')
            control1.disable()
            # if not control2.enabled:
            #     print('Uppercase Q quits the program')
            # control2.disable()
        elif ch == 'Q':
            control1.disable()
            # control2.disable()
            print('Bye!')
            break
        else:
            print('Unhandled key', ch, 'was pressed')


if __name__ == "__main__":
    end = 2 * math.pi
    r = 0.5
    t = np.linspace(0, end, 200)
    trajectory_x = [0.0] * 200
    trajectory_y = [0.0] * 200
    for i in range(0, 200):
        trajectory_x[i] = r * math.cos(t[i])
        trajectory_y[i] = r * math.sin(t[i])
    # print('trajectory_x', trajectory_x)
    # print('trajectory_y', trajectory_y)

    # trajectory_x = [0.0] * 500     # y=x * x *x 的轨迹 起始点（-1,-1），终止点（1,1）
    # end = 1
    # t = np.linspace(0, end, 500)
    # trajectory_y = [0.0] * 500
    # for i in range(0, 500):
    #     trajectory_x[i] = t[i]
    #     trajectory_y[i] = t[i] * t[i] * t[i]
    # print('trajectory_x', trajectory_x)
    # print('trajectory_y', trajectory_y)

    rospy.init_node('Pose_generator2')
    # g = PoseGenerator()
    # g.spin()

    logging.basicConfig()
    crtp.init_drivers(enable_debug_driver=False)

    cf1 = crazyflie.Crazyflie(rw_cache='./cache')
    # cf2 = crazyflie.Crazyflie(rw_cache='./cache')
    control1 = ControllerThread(cf1, 1, 0.1, 1650, trajectory_x, trajectory_y, 0.8)
    control1.start()

    # control2 = ControllerThread(cf2, 100, 100, 100)
    # control2.start()

    if URI is None:
        print('Scanning for Crazyflies...')
        available = crtp.scan_interfaces(0xE7E7E7E702)
        if available:
            print('Found Crazyflies:')
            for i in available:
                print('-', i[0])
            URI = available[0][0]
        else:
            print('No Crazyflies found!')
            sys.exit(1)

    # if URI2 is None:
    #     print('Scanning for Crazyflies...')
    #     available2 = crtp.scan_interfaces(0xE7E7E7E7E2)
    #     print(available2)
    #     crazy2 = Crazyradio()
    #     crazy2.set_channel(100)
    #     if available2:
    #         print('Found Crazyflies:')
    #         for i in available2:
    #             print('-', i[0])
    #         URI2 = available2[0][0]
    #     else:
    #         print('No Crazyflies found!')
    #         sys.exit(1)

    print('Connecting to', URI)
    # print('Connecting to', URI2)

    cf1.open_link(URI)
    # cf2.open_link(URI2)

    handle_keyboard_input(control1)  # handle_keyboard_input(control1, control2)

    cf1.close_link()
    # cf2.close_link()
