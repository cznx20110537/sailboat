# -*- coding：utf-8 -*-
# import math
import numpy as np
import xlsxwriter
import datetime
from scipy import interpolate

# from numpy.linalg import *

class SailModel(object):
    def __init__(self):
        # 帆参数
        self.sail_height = 1.3  # 帆高
        self.sail_width = 0.55  # 帆弦长（宽）
        # self.sail_width = 0.55  # 帆弦长（宽）
        self.S_sail = self.sail_height * self.sail_width  # 帆面积
        self.l1 = -0.25  # 桅杆距船体重心水平距离
        self.l2 = self.sail_width / 2  # 帆受力中心距桅杆的水平距离
        self.h1 = self.sail_height / 2 + 0.15  # 帆受力中心距船体重心垂直距离
        self.rho_w = 1.205  # 空气密度
        self.niu_w = 0.1511  # 空气粘度系数*10^4
        # 帆的空气动力系数（升阻力系数）
        self.sail_angle = [-180, -175, -170, -165,
                           -160, -155, -150, -145, -140, -135, -130, -125, -120, -115,
                           -110, -105, -100, -95, -90, -85, -80, -75, -70, -65,
                           -60, -55, -50, -45, -40, -35, -30, -27, -26, -25,
                           -24, -23, -22, -21, -20, -19, -18, -17, -16, -15,
                           -14, -13, -12, -11, -10, -9, -8, -7, -6,
                           0,
                           6, 7, 8, 9, 10, 11, 12, 13, 14,
                           15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
                           25, 26, 27, 30, 35, 40, 45, 50, 55, 60,
                           65, 70, 75, 80, 85, 90, 95, 100, 105, 110,
                           115, 120, 125, 130, 135, 140, 145, 150, 155, 160,
                           165, 170, 175, 180]
        self.sail_cl = [0.000, 0.690, 0.850, 0.675,
                        0.660, 0.740, 0.850, 0.910, 0.945, 0.945, 0.910, 0.840, 0.735, 0.625,
                        0.510, 0.370, 0.220, 0.070, -.070, -.220, -.370, -.515, -.650, -.765,
                        -.8750, -.9650, -1.040, -1.085, -1.075, -1.020, -.9150, -.9646, -.9109, -.8572,
                        -.8034, -.7497, -.6956, -.6414, -.5870, -.5322, -.4768, -.4200, -.3620, -.3082,
                        -.2474, -.1934, -.1406, -.0856, -.0320, 0.0198, 0.0699, 0.1089, 0.0298,
                        .0000,
                        -.0298, -.1089, -.0699, -.0198, .0320, .0856, .1406, .1934, .2474,
                        .3082, .3620, .4200, .4768, .5322, .5870, .6414, .6956, .7497, .8034,
                        .8572, .9109, .9646, .9150, 1.020, 1.075, 1.085, 1.040, .9650, .8750,
                        .7650, .6500, .5150, .3700, .2200, .0700, -.070, -.220, -.370, -.510,
                        -.625, -.735, -.840, -.910, -.945, -.945, -.910, -.850, -.740, -.660,
                        -.675, -.850, -.690, 0.000]
        self.sail_cd = [0.025, 0.055, 0.140, 0.230,
                        0.320, 0.420, 0.575, 0.755, 0.925, 1.085, 1.225, 1.350, 1.465, 1.555,
                        1.635, 1.700, 1.750, 1.780, 1.800, 1.800, 1.780, 1.735, 1.665, 1.575,
                        1.470, 1.345, 1.215, 1.075, 0.920, 0.745, 0.570, 0.473, 0.446, 0.420,
                        0.394, 0.369, 0.344, 0.320, 0.297, 0.274, 0.252, 0.231, 0.210, 0.190,
                        0.171, 0.152, 0.134, 0.076, .0188, .0203, .0185, .0170, .0152,
                        .0103,
                        .0152, .0170, .0185, .0203, .0188, .0760, .1340, .1520, .1710,
                        .1900, .2100, .2310, .2520, .2740, .2970, .3200, .3440, .3690, .3940,
                        .4200, .4460, .4730, .5700, .7450, .9200, 1.075, 1.215, 1.345, 1.470,
                        1.575, 1.665, 1.735, 1.780, 1.800, 1.800, 1.780, 1.750, 1.700, 1.635,
                        1.555, 1.465, 1.350, 1.225, 1.085, .9250, .7550, .5750, .4200, .3200,
                        .2300, .1400, .0550, .0250]
        self.sail_cl_aft_angle = [6, 7, 8, 9, 10, 11, 12, 13, 14]
        self.sail_cl_aft = [-0.0298, -0.1089, -0.0699, -0.0198, 0.032, 0.0856, 0.1406, 0.1934, 0.2474]  # 6-14
        self.sail_cl_fore_angle = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
        self.sail_cl_fore = [0, 0.11, 0.22, 0.33, 0.44, 0.55, 0.66, 0.77, 0.8542, 0.9352, 0.9811, 0.9132]  # 0-11

    # 计算风w相对帆s的速度
    def get_V_ws(self, u, v, uw, vw, phi, psi, r, p, lambda0, dlambda):
        assert np.abs(phi) < np.pi / 2, "phi >= pi/2!"
        _J = np.mat([[np.cos(psi), -np.cos(phi) * np.sin(psi)], [np.sin(psi), np.cos(phi) * np.cos(psi)]])  #
        _J_1 = _J.I  # 船体坐标系到大地坐标系的变换矩阵的逆矩阵
        _V = np.mat([[u], [v]])  # 船速矩阵
        _V_w = np.mat([[uw], [vw]])  # 风速矩阵
        _sa = np.mat([[r * self.l2 * np.sin(lambda0)], [-r * (self.l1 + self.l2 * np.cos(lambda0)) + p * self.h1]])
        _dsa = np.mat([[dlambda * self.l2 * np.sin(lambda0)], [-dlambda * self.l2 * np.cos(lambda0)]])
        _V_ws = _J_1 * _V_w - _V - _sa - _dsa  # 相对速度矩阵
        u_ws = _V_ws[0, 0]  # 读取矩阵中的数值
        v_ws = _V_ws[1, 0]
        return u_ws, v_ws

    def get_awa_aws(self, u, v, uw, vw, phi, psi, r, p, lambda0, dlambda):
        u_ws, v_ws = self.get_V_ws(u, v, uw, vw, phi, psi, r, p, lambda0, dlambda)
        aws = np.sqrt(u_ws ** 2 + v_ws **2)
        awa = np.arctan2(v_ws, u_ws)
        return awa, aws

    def get_beta_ws(self, u, v, uw, vw, phi, psi, r, p, lambda0, dlambda):
        u_ws, v_ws = self.get_V_ws(u, v, uw, vw, phi, psi, r, p, lambda0, dlambda)
        return np.arctan2(v_ws, u_ws)

    @staticmethod
    def angle_transform(angle):  # 将角度转化为[-pi,pi]之间
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle <= -np.pi:
            angle += 2 * np.pi
        return angle

    #  计算攻角
    def get_alpha_s(self, u, v, uw, vw, phi, psi, r, p, lambda0, dlambda):
        beta_ws = self.get_beta_ws(u, v, uw, vw, phi, psi, r, p, lambda0, dlambda)
        alpha_s = beta_ws - lambda0 + np.pi
        alpha_s = self.angle_transform(alpha_s)
        # print('alpha_s:{: .2f}  beta_ws:{: .2f}'.format(alpha_s, beta_ws))
        return alpha_s

    def get_CL(self, alpha_s, u_ws, v_ws):
        wind_speed = np.sqrt((u_ws ** 2 + v_ws ** 2))
        reynold = self.sail_width * wind_speed / self.niu_w
        reynold_fit = [2, 4, 8, 16, 36]
        cl_lose_speed_attack = [5, 6, 7.5, 9, 10.5]
        cl_restore_speed_attack = [6.2, 7.2, 9, 10.1, 13]
        lose_speed_attack = np.interp(reynold, reynold_fit, cl_lose_speed_attack)
        restore_speed_attack = np.interp(reynold, reynold_fit, cl_restore_speed_attack)
        if np.abs(alpha_s * 180 / np.pi) >= restore_speed_attack:
            C_L = np.interp((alpha_s * 180 / np.pi), self.sail_angle, self.sail_cl)
            return C_L
        elif np.abs(alpha_s * 180 / np.pi) <= lose_speed_attack:
            if alpha_s > 0:
                return np.interp((alpha_s * 180 / np.pi), self.sail_cl_fore_angle, self.sail_cl_fore)
            else:
                return -np.interp(np.abs(alpha_s * 180 / np.pi), self.sail_cl_fore_angle, self.sail_cl_fore)
        else:
            if alpha_s >= 0:
                return np.interp((alpha_s * 180 / np.pi), [lose_speed_attack, restore_speed_attack],
                                 [np.interp(lose_speed_attack, self.sail_cl_fore_angle, self.sail_cl_fore),
                                  np.interp(restore_speed_attack, self.sail_angle, self.sail_cl)])
            else:
                return -np.interp(np.abs(alpha_s * 180 / np.pi), [lose_speed_attack, restore_speed_attack],
                                  [np.interp(lose_speed_attack, self.sail_cl_fore_angle, self.sail_cl_fore),
                                   np.interp(restore_speed_attack, self.sail_angle, self.sail_cl)])

        # 风帆受力模型
    def get_force_sail(self, u, v, uw, vw, phi, psi, r, p, lambda0, dlambda):
        alpha_s = self.get_alpha_s(u, v, uw, vw, phi, psi, r, p, lambda0, dlambda)
        beta_ws = self.get_beta_ws(u, v, uw, vw, phi, psi, r, p, lambda0, dlambda)
        u_ws, v_ws = self.get_V_ws(u, v, uw, vw, phi, psi, r, p, lambda0, dlambda)
        C_D = np.interp((alpha_s * 180 / np.pi), self.sail_angle, self.sail_cd)
        C_L = self.get_CL(alpha_s, u_ws, v_ws)
        S_D = 0.5 * C_D * self.rho_w * (u_ws ** 2 + v_ws ** 2) * self.S_sail
        S_L = 0.5 * C_L * self.rho_w * (u_ws ** 2 + v_ws ** 2) * self.S_sail
        X_S = -S_L * np.sin(beta_ws) + S_D * np.cos(beta_ws)
        Y_S = S_L * np.cos(beta_ws) + S_D * np.sin(beta_ws)
        N_S = X_S * self.l2 * np.sin(lambda0) - Y_S * (self.l1 + self.l2 * np.cos(lambda0))
        L_S = self.h1 * Y_S
        return X_S, Y_S, N_S, L_S


class RudderModel(object):
    def __init__(self):
        # 舵参数
        self.l4 = 0.499
        self.h3 = 0.13
        self.rho_c = 1000  # 水密度
        # 舵的水动力系数（实验数据受力大小）
        self.rudder_angle = [-25, -20, -15, -13, -10, -8, -5, 0,
                             5, 10, 13, 15, 18, 20, 25]
        self.rudder_cl = [-12.907, -12.762, -13.967, -14.674, -15.149, -10.711, -7.010, 0,
                          6.353, 11.242, 16.202, 15.245, 14.571, 13.840, 12.902]
        self.rudder_cd = [5.655, 4.266, 2.599, 1.374, 1.034, 0.519, 0.298, 0.011,
                          0.778, 1.218, 2.461, 2.096, 4.273, 4.756, 6.056]
        self.rudder_fy = [-16.202, -15.245, -14.571, -13.84, -12.902, -11.242, -6.353, 0, 7.01, 10.711, 12.762, 12.907,
                          13.967, 14.674, 15.149]  #
        self.rudder_mz = [7.403, 7.035, 6.612, 6.346, 5.962, 5.109, 2.645, 0, -4.065, -5.876, -6.972, -7.078, -7.451,
                          -7.802, -7.985]  #

    @staticmethod  # 计算船体h相对水流c的速度（输入u, v, u_current, v_current, roll_angle, yaw_angle; 输出u_hc, v_hc)
    def get_V_hc(u, v, uc, vc, phi, psi):
        assert np.abs(phi) < np.pi / 2, "phi >= pi/2!"
        _J = np.mat([[np.cos(psi), -np.cos(phi) * np.sin(psi)], [np.sin(psi), np.cos(phi) * np.cos(psi)]])  #
        _J_1 = _J.I  # 船体坐标系到大地坐标系的变换矩阵的逆矩阵
        _V = np.mat([[u], [v]])  # 船速矩阵
        _V_c = np.mat([[uc], [vc]])  # 流速矩阵
        _V_hc = _V - _J_1 * _V_c  # 相对速度矩阵
        u_hc = _V_hc[0, 0]  # 读取矩阵中的数值
        v_hc = _V_hc[1, 0]
        return u_hc, v_hc

    # 计算水流相对舵的速度
    def get_V_cr(self, u, v, uc, vc, phi, psi, r, p):
        u_hc, v_hc = self.get_V_hc(u, v, uc, vc, phi, psi)
        _V_hc = np.mat([[u_hc], [v_hc]])  # 船体相对水流的速度矩阵
        _add = np.mat([[0], [r * self.l4 - p * self.h3]])
        _V_cr = -_V_hc - _add
        u_cr = _V_cr[0, 0]
        v_cr = _V_cr[1, 0]
        return u_cr, v_cr

    def get_beta_cr(self, u, v, uc, vc, phi, psi, r, p):
        u_cr, v_cr = self.get_V_cr(u, v, uc, vc, phi, psi, r, p)
        beta_cr = np.arctan2(v_cr, u_cr)
        return beta_cr

    @staticmethod
    def angle_transform(angle):  # 将角度转化为[-pi,pi]之间
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle <= -np.pi:
            angle += 2 * np.pi
        return angle

    #  计算舵攻角
    def get_alpha_r(self, u, v, uc, vc, phi, psi, r, p, sigma):
        beta_cr = self.get_beta_cr(u, v, uc, vc, phi, psi, r, p)
        alpha_r = beta_cr - sigma + np.pi
        alpha_r = self.angle_transform(alpha_r)
        # print('alpha_r:{: .2f}  beta_cr:{: .2f}'.format(alpha_r, beta_cr))
        return alpha_r

    # 舵受力模型
    def get_force_rudder(self, u, v, uc, vc, phi, psi, r, p, sigma):
        alpha_r = self.get_alpha_r(u, v, uc, vc, phi, psi, r, p, sigma)
        beta_cr = self.get_beta_cr(u, v, uc, vc, phi, psi, r, p)
        C_D = np.interp((alpha_r * 180 / np.pi), self.rudder_angle, self.rudder_cd)
        C_L = np.interp((alpha_r * 180 / np.pi), self.rudder_angle, self.rudder_cl)
        u_cr, v_cr = self.get_V_cr(u, v, uc, vc, phi, psi, r, p)
        R_D = C_D * self.rho_c / 1000 * (u_cr ** 2 + v_cr ** 2)
        R_L = C_L * self.rho_c / 1000 * (u_cr ** 2 + v_cr ** 2)
        X_R = -R_L * np.sin(beta_cr) + R_D * np.cos(beta_cr)
        Y_R = R_L * np.cos(beta_cr) + R_D * np.sin(beta_cr)
        N_R = -0.0029 * Y_R ** 2 - 0.499 * Y_R
        L_R = -self.h3 * Y_R
        # print('X_r:{: .2f}  Y_r:{: .2f}  N_r:{: .2f}  L_r:{: .2f}'.format(X_R, Y_R, N_R, L_R))
        return X_R, Y_R, N_R, L_R

    @staticmethod
    def rudder_angle_limit(rudder_angle):
        if rudder_angle > 75. / 180. * np.pi:
            rudder_angle = 75. / 180. * np.pi
        elif rudder_angle < -75. / 180. * np.pi:
            rudder_angle = -75. / 180. * np.pi
        return rudder_angle

    # 攻角限制（防止失速）
    def alpha_r_limit(self, alpha_r, beta_cr):
        alpha_min = np.pi + beta_cr - 60. / 180. * np.pi
        alpha_max = np.pi + beta_cr + 60. / 180. * np.pi
        alpha_min = self.angle_transform(alpha_min)
        alpha_max = self.angle_transform(alpha_max)
        if alpha_r > alpha_max:
            alpha_r = alpha_max
        elif alpha_r < alpha_min:
            alpha_r = alpha_min
        if alpha_r > 13. / 180. * np.pi:
            alpha_r = 13. / 180. * np.pi
        elif alpha_r < -10. / 180. * np.pi:
            alpha_r = -10. / 180. * np.pi
        return alpha_r


class HullModel(object):
    def __init__(self):
        # 水线间长（m）
        self.Lwl = 1.4900
        # 船宽（m）
        self.B = 0.3760
        # 水线面船宽（m）
        self.Bwl = 0.3510
        # 型深（m）
        self.D = 0.1628
        # 排水量（kg）
        self.disp = 27.2229
        # 吃水（m）
        self.draft = 0.0900
        # 水线面面积（m^2）
        self.waterplane_area = 0.3930
        # 湿表面积（m^2）
        self.wetted_surface = 0.5440
        # 方形系数
        self.Cb = 0.558
        # 初稳性高（m）
        self.GM = 0.154
        # X方向附加质量（kg）
        self.mx = 0.01 * self.disp * (0.398 + 11.97 * self.Cb * (1 + 3.73 * self.draft / self.Bwl) -
                                      2.890 * self.Cb * (self.Lwl / self.Bwl) * (1 + 1.13 * self.draft / self.Bwl) +
                                      0.175 * self.Cb * (self.Lwl / self.Bwl) * (1 + 0.54 * self.draft / self.Bwl) *
                                      (self.Lwl / self.Bwl) - 1.107 * (self.Lwl / self.Bwl) * (self.draft / self.Bwl))
        # Y方向附加质量（kg）
        self.my = 19.192
        # Z方向旋转惯量总和（kg*m^2）IzzJzz = Izz + Jzz
        self.IzzJzz = 2.657
        # X方向旋转惯量总和（kg*m^2）IxxJxx = Ixx + Jxx
        self.IxxJxx = self.disp * (0.4 * self.B) * (0.4 * self.B)
        # 重力加速度（m/s^2)
        self.g = 9.79460
        # 质量（kg）
        self.m = self.disp
        # 重力（N）
        self.W = self.m * self.g
        # X方向水动力导数
        self.Xuu = 3.8776
        self.Xu = -0.7552
        self.Xvv = 70.335
        self.Xvr = self.my * (1.75 * self.Cb - 1.525)
        self.Xrr = 11.519
        # Y方向水动力导数
        self.Yv = -214.996
        self.Yr = -23.524
        self.Yvv = 450.758
        self.Yrr = 68.859
        self.Yvr = 0.3772  #
        self.Yvvr = 495.313
        self.Yvrr = -4044.241
        # N方向水动力导数
        self.Nv = -25.805
        self.Nr = -9.597
        self.Nr_s = -5.9051
        self.Nvv = 24.477
        self.Nrr = 37.738  # ##
        self.Nrr_s = -6.8153
        # self.Nrr = -89.141
        self.Nvr = -0.0132  #
        self.Nvvr = -11.766
        self.Nvrr = -1711.467  # ##
        self.Nphi = -3.081
        self.Nvphi = 54.703
        self.Nrphi = 7.480
        self.Nheel = -0.008  #
        self.Nvheel = 1.72 * self.Nv  #
        self.Nrheel = -2.60 * self.Nr  #
        # L方向水动力导数
        self.phi_m = 0.5
        self.mu_phi = 0.0057 * self.Lwl * (self.Bwl ** 4) * self.phi_m / (2 * self.W * (self.Bwl ** 2 + self.D ** 4))
        self.K_dphi = 2 * self.mu_phi * np.sqrt(self.IxxJxx * self.W * self.GM) * 0.5 * 1000 * self.Lwl ** 5
        self.K_pp = 35.72
        self.GZ_angle = np.array([0, 8.05, 16.23, 26.73, 32.81, 40.24, 49.12, 58.03, 68.31, 78.40, 87.77, 93.32])
        self.GZ_value = np.array([0, 0.020, 0.04, 0.06, 0.067, 0.073, 0.076, 0.077, 0.075, 0.070, 0.063, 0.059])

    @staticmethod  # 计算船体h相对水流c的速度（输入u, v, u_current, v_current, roll_angle, yaw_angle; 输出u_hc, v_hc)
    def get_V_hc(u, v, uc, vc, phi, psi):
        assert np.abs(phi) < np.pi / 2, "phi >= pi/2!"
        _J = np.mat([[np.cos(psi), -np.cos(phi) * np.sin(psi)], [np.sin(psi), np.cos(phi) * np.cos(psi)]])  #
        _J_1 = _J.I  # 船体坐标系到大地坐标系的变换矩阵的逆矩阵
        _V = np.mat([[u], [v]])  # 船速矩阵
        _V_c = np.mat([[uc], [vc]])  # 流速矩阵
        _V_hc = _V - _J_1 * _V_c  # 相对速度矩阵
        u_hc = _V_hc[0, 0]  # 读取矩阵中的数值
        v_hc = _V_hc[1, 0]
        return u_hc, v_hc

    @staticmethod
    def X(u):  # 船体阻力
        if u > 0:
            if np.abs(u) <= 0.5:
                return -0.724 / 0.5 * u
            else:
                return -(5.1964 * u * u - 3.2206 * u + 1.0531)
        else:
            if np.abs(u) <= 0.5:
                return -0.724 / 0.5 * u
            else:
                return 5.1964 * u * u - 3.2206 * np.abs(u) + 1.0531

    def GZ(self, phi):  # 回复力臂
        if phi > 0:
            return np.interp(phi * 180 / np.pi, self.GZ_angle, self.GZ_value)
        else:
            return -np.interp(np.abs(phi) * 180 / np.pi, self.GZ_angle, self.GZ_value)

    # !!!船体受力模型（输入 u, v, uc, vc, psi, phi, r, p; 输出X_H, Y_H, N_H, L_H）
    def get_force_hull(self, u0, v0, uc, vc, psi, phi, r, p):
        u, v = self.get_V_hc(u0, v0, uc, vc, phi, psi)
        # assert u >= 0, "u < 0!"
        X_H = self.X(u) + self.Xvv * v * v + self.Xvr * v * r + self.Xrr * r * r
        Y_H = self.Yv * v + self.Yr * r + self.Yvv * np.abs(v) * v + self.Yrr * np.abs(
            r) * r + self.Yvvr * v * v * r + self.Yvrr * v * r * r
        if np.abs(r) >= 0.2:
            N_H = self.Nv * v + self.Nr_s * r + self.Nvv * np.abs(v) * v + self.Nrr_s * np.abs(
                r) * r + self.Nvvr * v * v * r + self.Nvrr * v * r * r + self.Nphi * phi + self.Nvphi * v * np.abs(
                phi) + self.Nrphi * r * np.abs(phi)
        else:
            N_H = self.Nv * v + self.Nr * r + self.Nvv * np.abs(v) * v + self.Nrr * np.abs(
                r) * r + self.Nvvr * v * v * r + self.Nvrr * v * r * r + self.Nphi * phi + self.Nvphi * v * np.abs(
                phi) + self.Nrphi * r * np.abs(phi)
        # L_H = -self.K_dphi * p - self.W * self.GZ(phi) + Y_H * self.draft/2
        L_H = -self.K_pp * p * np.abs(p) - self.W * self.GZ(phi) + Y_H * self.draft/2
        # print('-u:{: .2f}  v:{: .2f}  r:{: .2f}  p:{: .2f}  psi:{: .2f}  phi:{: .2f}'.format(u, v, r, p, psi, phi))
        return X_H, Y_H, N_H, L_H


class SailboatModel(SailModel, RudderModel, HullModel):
    def __init__(self):
        SailModel.__init__(self)
        RudderModel.__init__(self)
        HullModel.__init__(self)

    # 计算合力
    def get_force_total(self, u, v, uc, vc, uw, vw, psi, phi, r, p, sigma, lambda0, dlambda):
        X_H, Y_H, N_H, L_H = self.get_force_hull(u, v, uc, vc, psi, phi, r, p)
        X_R, Y_R, N_R, L_R = self.get_force_rudder(u, v, uc, vc, phi, psi, r, p, sigma)
        X_S, Y_S, N_S, L_S = self.get_force_sail(u, v, uw, vw, phi, psi, r, p, lambda0, dlambda)
        assert np.abs(Y_H) < 10000, "Y_H is too big to calculate. The sailboat is unstable under this circumstance. "
        X = X_H + X_S + X_R
        Y = Y_H + Y_S + Y_R
        N = N_H + N_S + N_R
        L = L_H + L_S + L_R
        # print("""u:%.2f  v:%.2f  r:%.2f  p:%.2f""" % (u, v, r, p))
        return X, Y, N, L

    # 计算船体加速度
    def get_accelerate(self, u, v, r, p, psi, phi, uc, vc, uw, vw, sigma, lambda0, dlambda, dt=0.01):
        X, Y, N, L = self.get_force_total(u, v, uc, vc, uw, vw, psi, phi, r, p, sigma, lambda0, dlambda)
        du = (X + (self.m + self.my) * v * r) / (self.m + self.mx)
        dv = (Y - (self.m + self.mx) * u * r) / (self.m + self.my)
        dr = N / self.IzzJzz
        dp = L / self.IxxJxx
        dx = u * np.cos(psi) - v * np.sin(psi) * np.cos(phi)
        dy = u * np.sin(psi) + v * np.cos(psi) * np.cos(phi)
        dpsi = r * np.cos(phi)
        dphi = p
        return du, dv, dr, dp, dx, dy, dpsi, dphi

    # 更新状态量(欧拉法)
    def update_state(self, u, v, r, p, x, y, psi, phi, uc, vc, uw, vw, sigma, lambda0, dlambda, dt=0.01):
        du, dv, dr, dp, _, _, _, _ = self.get_accelerate(
            u, v, r, p, psi, phi, uc, vc, uw, vw, sigma, lambda0, dlambda)

        u_new = u + du * dt
        v_new = v + dv * dt
        r_new = r + dr * dt
        p_new = p + dp * dt

        phi_new = self.angle_transform(phi + (p + p_new) * dt / 2)
        psi_new = self.angle_transform(psi + (r * np.cos(phi) + r_new * np.cos(phi_new)) * dt / 2)
        x_new = x + ((u * np.cos(psi) - v * np.sin(psi) * np.cos(phi)) + (
                u_new * np.cos(psi_new) - v_new * np.sin(psi_new) * np.cos(phi_new))) * dt / 2
        y_new = y + ((u * np.sin(psi) + v * np.cos(psi) * np.cos(phi)) + (
                u_new * np.sin(psi_new) + v_new * np.cos(psi_new) * np.cos(phi_new))) * dt / 2
        return u_new, v_new, r_new, p_new, x_new, y_new, psi_new, phi_new

    # 更新状态量(龙格库塔法)
    def update_state_RK(self, u, v, r, p, x, y, psi, phi, uc, vc, uw, vw, sigma, lambda0, dlambda, dt=0.01):
        dt2 = dt3 = dt / 2
        dt4 = dt
        du1, dv1, dr1, dp1, dx1, dy1, dpsi1, dphi1 = self.get_accelerate(
            u, v, r, p, psi, phi, uc, vc, uw, vw, sigma, lambda0, dlambda, 0)
        du2, dv2, dr2, dp2, dx2, dy2, dpsi2, dphi2 = self.get_accelerate(
            u + dt2 * du1, v + dt2 * dv1, r + dt2 * dr1, p + dt2 * dp1, psi + dt2 * dpsi1, phi + dt2 * dphi1,
            uc, vc, uw, vw, sigma, lambda0, dlambda, dt2)
        du3, dv3, dr3, dp3, dx3, dy3, dpsi3, dphi3 = self.get_accelerate(
            u + dt3 * du2, v + dt3 * dv2, r + dt3 * dr2, p + dt3 * dp2, psi + dt3 * dpsi2, phi + dt3 * dphi2,
            uc, vc, uw, vw, sigma, lambda0, dlambda, dt3)
        du4, dv4, dr4, dp4, dx4, dy4, dpsi4, dphi4 = self.get_accelerate(
            u + dt4 * du3, v + dt4 * dv3, r + dt4 * dr3, p + dt4 * dp3, psi + dt4 * dpsi3, phi + dt4 * dphi3,
            uc, vc, uw, vw, sigma, lambda0, dlambda, dt4)
        u_new = u + dt / 6 * (du1 + 2 * du2 + 2 * du3 + du4)
        v_new = v + dt / 6 * (dv1 + 2 * dv2 + 2 * dv3 + dv4)
        r_new = r + dt / 6 * (dr1 + 2 * dr2 + 2 * dr3 + dr4)
        p_new = p + dt / 6 * (dp1 + 2 * dp2 + 2 * dp3 + dp4)
        x_new = x + dt / 6 * (dx1 + 2 * dx2 + 2 * dx3 + dx4)
        y_new = y + dt / 6 * (dy1 + 2 * dy2 + 2 * dy3 + dy4)
        psi_new = self.angle_transform(psi + dt / 6 * (dpsi1 + 2 * dpsi2 + 2 * dpsi3 + dpsi4))
        phi_new = phi + dt / 6 * (dphi1 + 2 * dphi2 + 2 * dphi3 + dphi4)
        if r_new >= 0.3:
            r_new = 0.3
        if r_new <= -0.3:
            r_new = -0.3
        return u_new, v_new, r_new, p_new, x_new, y_new, psi_new, phi_new

    # npd风谱：输入（平均速度，平均风向角，波动风向角，横摇角，时间，时间间隔）输出（合风速u_wind、v_wind、波动风向角、合风速u_z、合风向角）
    def npdwind(self, u_10, beta_wm0, beta_wflux0, phi, t0, dt):
        z = -(self.h1 - 0.15 + self.D - self.draft) * np.cos(phi)
        C = 0.0573 * np.sqrt(1 + 0.15 * u_10)
        u_z = u_10 * (1 + C * np.log(-z / 10))
        N = 50  # 风分量的数目
        k = 1
        while k < (N + 1):
            f_k = 0.4 / N * k
            x_k = 172 * f_k * (-z / 10) ** (2 / 3) / (u_10 / 10) ** 0.75
            S_k = 320 * (u_10 / 10) ** 2 * (-z / 10) ** 0.45 / ((1 + x_k ** 0.468) ** (5 / (3 * 0.468)))
            np.random.seed(int(t0 * 100 + k))
            u_z += np.sqrt(2 * S_k * 0.4 / N) * np.cos(2 * np.pi * f_k * t0 + np.random.rand() * 2 * np.pi)
            k += 1

        omega_wind = np.random.randn()
        beta_wflux0 += omega_wind * dt
        if beta_wflux0 > 5 / 180 * np.pi:
            beta_wflux0 = 5 / 180 * np.pi
        if beta_wflux0 < -5 / 180 * np.pi:
            beta_wflux0 = -5 / 180 * np.pi
        beta_w0 = beta_wm0 + beta_wflux0

        u_w = u_z * np.cos(beta_w0)
        v_w = u_z * np.sin(beta_w0)
        return u_w, v_w, beta_wflux0, u_z, beta_w0

    @staticmethod
    def constwind(u_10, beta_wm0, beta_wflux0, phi, t0, dt):
        u_z = u_10
        beta_w0 = beta_wm0
        u_w = u_z * np.cos(beta_w0)
        v_w = u_z * np.sin(beta_w0)
        return u_w, v_w, beta_wflux0, u_z, beta_w0

