from Sailboat_model_NACA import *
import math
from collections import deque

heading_error = deque(maxlen=100)
heading_error.append(0.0)
sail_error = deque(maxlen=100)
sail_error.append(0.0)
para_cfg = [0.2, 0.4, 0.1]
mark = 0
mark_time = 0


def angle_limit(angle):  # 限制所有角度再-pi到pi
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def angel_deg(angle):  # 角度转换rad到deg
    angle = angle / np.pi * 180
    return angle


def get_sa(awa, state, sa):
    skp = 0.2
    skd = 0.1
    ski = 0.01
    u = state[0]
    v = state[1]
    beta = angle_limit(math.atan(v / u))  # 速度漂角rad
    jiajiao = angle_limit(-(beta - awa))
    jiajiao_deg = angel_deg(jiajiao)
    if jiajiao_deg > 50 or jiajiao_deg < -50:
        AWA = [-180, -170, -160, -150, -140, -130, -120, -110, -100, -90, -80, -70, -60, -50, 50, 60, 70, 80, 90, 100,
               110, 120, 130, 140, 150, 160, 170, 180]
        SA = [-90, -88, -81.67, -78.67, -76.67, -74.67, -57, -81.33, -70.67, -61.33, -52.33, -43.67, -34.67, -26.67,
              26.67, 34.67, 43.67, 52.33, 61.33, 70.67, 81.33, 57, 74.67, 76.67, 78.67, 81.67, 88, 90]
        result = int(round(jiajiao_deg, -1))
        index = AWA.index(result)
        if jiajiao_deg < AWA[index]:
            index_1 = index - 1
            x1 = AWA[index_1]
            y1 = SA[index_1]
            x2 = AWA[index]
            y2 = SA[index]
            sa_deg = (y2 - y1) / (x2 - x1) * (jiajiao_deg - x2) + y2
        elif jiajiao_deg > AWA[index]:
            index_1 = index + 1
            x1 = AWA[index]
            y1 = SA[index]
            x2 = AWA[index_1]
            y2 = SA[index_1]
            sa_deg = (y2 - y1) / (x2 - x1) * (jiajiao_deg - x1) + y1
        else:
            sa_deg = SA[index]
    else:
        sa_deg = 0
    sa_rad = sa_deg / 180 * np.pi
    error = sa_rad - sa
    sail_error.append(error)
    a = float(sail_error[-1])
    b = float(sail_error[-1] - sail_error[-2])
    c = float(sum(sail_error))
    sa_goal = sa + skp * a + skd * b + ski * c
    if sa_goal > np.pi / 2:
        sa_goal = np.pi / 2
    if sa_goal < -np.pi / 2:
        sa_goal = -np.pi / 2
    return sa_goal


def get_ra(heading_error, para_cfg):
    ra_limit_0 = 1 / 18 * np.pi
    u_up = 0.3
    u_down = 0.1
    u = state[0]
    if u > u_up:
        ra_limit = ra_limit_0
    elif u > u_down and u < u_up:
        ra_limit = (u - u_down) / (u_up - u_down) * ra_limit_0
    else:
        ra_limit = 0
    rkp = para_cfg[0]
    rkd = para_cfg[1]
    rki = para_cfg[2]
    a = float(heading_error[-1])
    b = float(heading_error[-1] - heading_error[-2])
    c = float(sum(heading_error))
    ra_goal = rkp * a + rkd * b + rki * c
    if ra_goal > ra_limit:
        ra_goal = ra_limit
    elif ra_goal < -ra_limit:
        ra_goal = -ra_limit
    return ra_goal


def tacking(state, goal_point, awa, aws, sa):  # 三点绕标
    u, v, r, p, x, y, psi, phi = state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7]
    global mark, mark_time, awa_list
    awa = angle_limit(awa - np.pi)
    awa_list.append(awa)
    x0 = goal_point[0]
    y0 = goal_point[1]
    psi = angle_limit(psi)
    # heading = psi
    goal_heading = math.atan2((y0 - y), (x0 - x))
    beta = angle_limit(math.atan(v / u))
    heading = psi + beta
    banned_area = 4 / 18 * np.pi
    if mark == 1:
        if abs(awa) < banned_area:
            mark = 11
            mark_time = 1
        if abs(awa) > banned_area:
            mark = 0
        goal_heading = heading + awa + np.pi / 3
        error = goal_heading - heading
    elif mark == 11:
        mark_time += 1
        if mark_time > 400:
            mark = 0
        if abs(awa) > banned_area:
            mark = 0
        goal_heading = heading + awa + np.pi / 3
        error = goal_heading - heading
    elif mark == 2:
        if abs(awa) < banned_area:
            mark = 22
            mark_time = 1
        if abs(awa) > banned_area:
            mark = 0
        goal_heading = heading + awa - np.pi / 3
        error = goal_heading - heading
    elif mark == 22:
        mark_time += 1
        if mark_time > 400:
            mark = 0
        if abs(awa) > banned_area:
            mark = 0
        goal_heading = heading + awa - np.pi / 3
        error = goal_heading - heading
    else:
        if abs(awa) < banned_area:
            gh = goal_heading - heading
            if gh > 0:
                mark = 1
            else:
                mark = 2
        error = angle_limit(goal_heading - heading)
    heading_error.append(error)
    ra = get_ra(heading_error, para_cfg)
    sa = get_sa(awa, state, sa)
    return sa, ra


def heading_control(state, goal_point, awa, aws, sa):  # 艏向控制
    u, v, r, p, x, y, psi, phi = state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7]
    global mark, mark_time, awa_list
    awa = angle_limit(awa - np.pi)
    psi = angle_limit(psi)
    # heading = psi
    goal_heading = angle_limit(math.atan2((goal_point[1] - y), (goal_point[0] - x)))
    beta = angle_limit(math.atan(v / u))
    heading = psi  # + beta
    banned_area = 4 / 18 * np.pi
    if mark == 1:
        if abs(awa) < banned_area:
            mark = 11
            mark_time = 1
        if abs(awa) > banned_area:
            mark = 0
        goal_heading = heading + awa + np.pi / 3
        error = goal_heading - heading
    elif mark == 11:
        mark_time += 1
        if mark_time > 500:
            mark = 0
        if abs(awa) > banned_area:
            mark = 0
        goal_heading = heading + awa + np.pi / 3
        error = goal_heading - heading
    elif mark == 2:
        if abs(awa) < banned_area:
            mark = 22
            mark_time = 1
        if abs(awa) > banned_area:
            mark = 0
        goal_heading = heading + awa - np.pi / 3
        error = goal_heading - heading
    elif mark == 22:
        mark_time += 1
        if mark_time > 500:
            mark = 0
        if abs(awa) > banned_area:
            mark = 0
        goal_heading = heading + awa - np.pi / 3
        error = goal_heading - heading
    else:
        if abs(awa) < banned_area:
            gh = goal_heading - heading
            if gh > 0:
                mark = 1
            else:
                mark = 2
        error = angle_limit(goal_heading - heading)
    heading_error.append(error)
    ra = get_ra(heading_error, para_cfg)
    sa = get_sa(awa, state, sa)
    return sa, ra


def heading_control_test(state, awa, sa, goal_point):  # 没用的
    u, v, r, p, x, y, psi, phi = state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7]
    awa = angle_limit(awa - np.pi)
    goal = angle_limit(math.atan2((goal_point[1] - y), (goal_point[0] - x)))
    heading = angle_limit(psi)
    error = angle_limit(goal - heading)
    heading_error.append(error)
    ra = get_ra(heading_error, para_cfg)
    sa = get_sa(awa, state, sa)
    return sa, ra


if __name__ == '__main__':
    k = 0
    goal_heading = 12 / 18 * np.pi
    awa_list = []
    time_i = []
    state6_list = []
    sailboat = SailboatModel()
    # 初始舵角
    ra = 0
    # 帆角
    sa_angle = 0
    sa = sa_angle / 180 * np.pi
    # 波动风向/勿改数值
    beta_wflux = 0
    # 绝对流速tcs（m/s），绝对流向tca（rad）
    tcs = 0.0
    tca = 0.0
    # 流的 u，v 分量
    u_current = tcs * np.cos(tca)
    v_current = tcs * np.sin(tca)

    # #########################################################################################################
    # 10m平均风速u10_m（m/s），平均风向beta_wm（rad）（沿 x0 轴正向顺时针旋转到风矢时转过的角度，南风0，西风pi/2，北风pi，东风3pi/2）
    u10_m = 5.0
    beta_wm_angle = 180
    beta_wm = beta_wm_angle / 180 * np.pi
    # 循环次数
    times = 10000
    # 计算时间间隔
    delta_t = 0.01
    # 船体径向速度u, 船体横向速度v, 船体首摇角速度r, 船体横摇角速度p
    # 船体重心在世界坐标系下坐标x, 船体重心在世界坐标系下坐标y, 船体首摇角psi, 船体横摇角phi
    # 初始状态  u,   v,   r,   p,   x,   y, psi, phi
    state = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # excel表头
    data = [["time", "u", "v", "Posx", "Posy", "heading[deg]", "phi[deg]", "course[deg]", "WindSpeed", "WindAngle", "ra[deg]",
             "speed", "sa[deg]", "r", "p"]]
    # excel 名称
    excel_name = datetime.datetime.now().strftime('%Y-%m-%d %H_%M_%S ') + '.xlsx'
    for i in range(times):
        t = delta_t * i
        # # 风的 u，v 分量，波动风向
        u_wind, v_wind, beta_wflux, uz, beta_w = sailboat.npdwind(u10_m, beta_wm, beta_wflux, state[7], t, delta_t)
        # u_wind, v_wind, beta_wflux, uz, beta_w = sailboat.constwind(u10_m, beta_wm, beta_wflux, state[7], t, delta_t)
        # get_awa_aws(u, v, uw, vw, phi, psi, r, p, lambda0, dlambda)
        awa, aws = sailboat.get_awa_aws(state[0], state[1], u_wind, v_wind, state[7], state[6], state[2], state[3], 0,
                                        0)
        # # 帆角sa（rad），舵角ra（rad）###############################################################################
        # 帆角sa（rad），舵角ra（rad）需要根据现实情况中传感器可以获得的数据，即帆船速度、位置、姿态、风速、风向进行计算
        # 即,可以使用 state 数组中的全部数据和 awa, aws。state 数组中角度单位均为弧度。
        goal_list = [(200, 0), (50, -20), (30, -10)]
        sa, ra = tacking(state, goal_list[k], awa, aws, sa)
        distance = math.sqrt(
            (goal_list[k][0] - state[4]) * (goal_list[k][0] - state[4]) + (goal_list[k][1] - state[5]) * (
                        goal_list[k][1] - state[5]))
        if distance < 2:
            k = k + 1
        if k > 2:
            k = 0
        ra = -ra
        # goal_angle = (20,20)
        # sa, ra = heading_control(state, goal_angle, awa, aws, sa)
        # ra = -ra
        # # update_state(u, v, r, p, x, y, psi, phi, uc, vc, uw, vw, sigma, lambda0, dlambda, dt=0.01)
        tmp = sailboat.update_state_RK(state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7],
                                       u_current, v_current, u_wind, v_wind, ra, sa, 0, delta_t)
        # # tmp: [  0     1       2       3      4     5       6         7  ]
        # # tmp: [u_new  v_new  r_new  p_new  x_new  y_new  psi_new  phi_new]
        # # state :[0, 1, 2, 3, 4, 5,  6,   7]
        # # state =[u, v, r, p, x, y, psi, phi]
        state[0], state[1], state[2], state[3] = tmp[0], tmp[1], tmp[2], tmp[3]
        state[4], state[5], state[6], state[7] = tmp[4], tmp[5], tmp[6], tmp[7]

        # 采集数据
        drift_angle = np.arctan2(tmp[1] * np.cos(tmp[7]), tmp[0]) #漂角
        course_angle = (tmp[6] + drift_angle) * 180 / np.pi
        speed = np.sqrt(state[0] ** 2 + state[1] ** 2)
        data.append([t, tmp[0], tmp[1], tmp[4], tmp[5], tmp[6] * 180 / np.pi, tmp[7] * 180 / np.pi,
                     course_angle, uz, beta_w * 180 / np.pi, ra * 180 / np.pi, speed, sa * 180 / np.pi,
                     tmp[2], tmp[3]])
        if i % 10 == 0:  # 每0.1秒，计算并采集一次数据
            print('{:%}'.format(i / times))  # 进度提示

    # 将数据插入excel
    print("data exporting ... ...")  # 进度提示
    excel_sailboat = xlsxwriter.Workbook(excel_name)
    sheet_sailboat = excel_sailboat.add_worksheet("sheet_1")
    for row in range(len(data)):  # 将数据插入excel中，每次插入一整行数据
        sheet_sailboat.write_row("A" + str(row + 1), data[row])
    # 绘制轨迹图
    position_chart = excel_sailboat.add_chart({'type': 'scatter'})
    position_chart.add_series({
        'categories': ['sheet_1', 1, 4, len(data) - 1, 4],
        'values': ['sheet_1', 1, 3, len(data) - 1, 3],
        'line': {'none': True},
        'marker': {
            'type': 'circle',
            'size': 2,
            'border': {'color': 'blue'},
            'fill': {'color': 'blue'},
        },
    })
    position_chart.set_title({'name': 'Position Plot'})
    position_chart.set_x_axis({'name': 'East'})
    position_chart.set_y_axis({'name': 'North'})
    sheet_sailboat.insert_chart('A5', position_chart)
    # 绘制航向图
    course_angle_chart = excel_sailboat.add_chart({'type': 'scatter', 'subtype': 'smooth'})
    course_angle_chart.add_series({
        'categories': ['sheet_1', 1, 0, len(data) - 1, 0],
        'values': ['sheet_1', 1, 7, len(data) - 1, 7],
    })
    course_angle_chart.set_title({'name': 'Course Plot'})
    course_angle_chart.set_x_axis({'name': 'time [s]', 'min': 0, 'max': (times * delta_t)})
    course_angle_chart.set_y_axis({'name': 'course angle [deg]'})
    course_angle_chart.set_size({'x_scale': 2})
    sheet_sailboat.insert_chart('A20', course_angle_chart)
    # 绘制艏向图
    heading_chart = excel_sailboat.add_chart({'type': 'scatter', 'subtype': 'smooth'})
    heading_chart.add_series({
        'categories': ['sheet_1', 1, 0, len(data) - 1, 0],
        'values': ['sheet_1', 1, 5, len(data) - 1, 5],
    })
    heading_chart.set_title({'name': 'Heading Plot'})
    heading_chart.set_x_axis({'name': 'time [s]', 'min': 0, 'max': (times * delta_t)})
    heading_chart.set_y_axis({'name': 'heading angle [deg]'})
    heading_chart.set_size({'x_scale': 2})
    sheet_sailboat.insert_chart('A35', heading_chart)
    # 绘制风速图
    wind_speed_chart = excel_sailboat.add_chart({'type': 'scatter', 'subtype': 'smooth'})
    wind_speed_chart.add_series({
        'categories': ['sheet_1', 1, 0, len(data) - 1, 0],
        'values': ['sheet_1', 1, 8, len(data) - 1, 8],
    })
    wind_speed_chart.set_title({'name': 'Wind Speed'})
    wind_speed_chart.set_x_axis({'name': 'time [s]', 'min': 0, 'max': (times * delta_t)})
    wind_speed_chart.set_y_axis({'name': 'wind speed [m/s]'})
    wind_speed_chart.set_size({'x_scale': 2})
    sheet_sailboat.insert_chart('A50', wind_speed_chart)
    # 绘制风向图
    wind_direction_chart = excel_sailboat.add_chart({'type': 'scatter', 'subtype': 'smooth'})
    wind_direction_chart.add_series({
        'categories': ['sheet_1', 1, 0, len(data) - 1, 0],
        'values': ['sheet_1', 1, 9, len(data) - 1, 9],
    })
    wind_direction_chart.set_title({'name': 'Wind Direction'})
    wind_direction_chart.set_x_axis({'name': 'time [s]', 'min': 0, 'max': (times * delta_t)})
    wind_direction_chart.set_y_axis({'name': 'wind direction [deg]'})
    wind_direction_chart.set_size({'x_scale': 2})
    sheet_sailboat.insert_chart('A65', wind_direction_chart)

    excel_sailboat.close()
