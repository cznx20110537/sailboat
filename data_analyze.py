import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math


def draw_position_plot():
    data_posx = df['Posx'].values
    data_posy = df['Posy'].values
    plt.figure(1)
    plt.title('Position Plot')
    plt.plot(data_posx, data_posy)


def draw_heading_plot():
    data_time = df['seq'].values
    data_heading = df['Yaw'].values
    data_heading = data_heading * 180 / np.pi - 180
    plt.figure(3)
    plt.plot(data_time, data_heading)
    plt.title('Heading Plot')


def draw_wind_speed():
    data_time = df['seq'].values
    data_true_wind_speed = df['TWS'].values
    plt.figure(4)
    plt.plot(data_time, data_true_wind_speed)
    plt.title('Wind Speed')


def draw_wind_direction():
    data_time = df['seq'].values
    data_true_wind_angle = df['TWA'].values
    data_true_wind_angle = data_true_wind_angle * 180 / np.pi
    plt.figure(5)
    plt.plot(data_time, data_true_wind_angle)
    plt.title('Wind Direction')


def draw_course_plot():
    data_time = df['seq'].values
    data_draft_angle = np.arctan2(df['vy'].values * np.cos(df['Roll'].values), df['ux'].values)  # 漂角
    data_course_angle_radius = (df['Yaw'].values + data_draft_angle)  # 航向角
    data_course_angle = np.zeros(len(data_course_angle_radius))
    for i in range(len(data_course_angle_radius)):
        data_course_angle[i] = angle_limit(data_course_angle_radius[i]) * 180 / np.pi
    plt.figure(2)
    plt.plot(data_time, data_course_angle)
    plt.title('Course Plot')


def angle_limit(angle):  # 限制所有角度再-pi到pi
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def aw2tw(aws, awa, u, v, heading):  # 表风到真风
    vx = u * np.cos(heading) - v * np.sin(heading)
    vy = u * np.sin(heading) + v * np.cos(heading)
    twx = aws * math.cos(awa + heading + np.pi) + vx
    twy = aws * math.sin(awa + heading + np.pi) + vy
    return twx, twy


def draw_wind_speed_direction():
    data_time = df['seq'].values
    data_appearance_wind_angle = df['AWA'].values
    data_appearance_wind_speed = df['AWS'].values
    data_ux = df['ux'].values
    data_vy = df['vy'].values
    data_heading = df['Roll'].values
    data_true_wind_angle = np.zeros(len(data_time))
    data_true_wind_speed = np.zeros(len(data_time))
    for i in range(len(data_time)):
        twx, twy = aw2tw(data_appearance_wind_speed[i], data_appearance_wind_angle[i], data_ux[i], data_vy[i],
                         data_heading[i])
        data_true_wind_angle[i] = np.arctan2(twy, twx) * 180 / np.pi
        data_true_wind_speed[i] = math.sqrt(twy * twy + twx * twx)
    plt.figure(6)
    plt.plot(data_time, data_true_wind_speed)
    plt.title('Wind Speed')

    plt.figure(7)
    plt.plot(data_time, data_true_wind_angle)
    plt.title('Wind Direction')


if __name__ == '__main__':
    df = pd.read_csv(r'D:\SJTU大三（上）总\NA322船舶与海洋工程自主创新实验\研究进度\第5周_xlsxRead&write/single_slash_sensor_kalman_msg.csv')
    # df = pd.read_csv(r'D:\SJTU大三（上）总\NA322船舶与海洋工程自主创新实验\实验数据\2020-10-23-14-36-53/_slash_sensor_kalman_msg.csv')

    df = df[10000: 12000]

    draw_position_plot()
    draw_course_plot()
    draw_heading_plot()
    draw_wind_speed()
    draw_wind_direction()
    # draw_wind_speed_direction()
    plt.show()
