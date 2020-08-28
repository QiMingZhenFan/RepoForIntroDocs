import glob
import os
import sys
import pygame

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random
import time
# -------------------------------------------
# @note 读取csv文件相关
# -------------------------------------------
import csv

def read_csv(str):
    with open(str, 'r') as file:
        content = csv.reader(file,dialect='excel')
        data = [[]]
        index = 0   # 从第一行开始，第0行的list作空处理
        for line in content:
            data.append([])
            index += 1
            for element in line:
                data[index].append(float(element))
    return data

def get_path(*filepath):
    path = [[]]
    for i in filepath:
        sub_path = read_csv(i)
        # TODO:二维列表，只能这样循环生成吗
        j = 6   # 从第6项开始是坐标等信息
        while j < len(sub_path):
            path.append(sub_path[j])
            j += 1
    path.pop(0) # 上面二维列表初始化导致最开始元素是一个的一维的空列表
    return path

def get_path_main():
    filepath = []
    filepath.append('/home/mrk/Lab_file/carla_simu/lane_0.csv')
    filepath.append('/home/mrk/Lab_file/carla_simu/lane_1.csv')
    path = get_path(*filepath)
    return path

def draw_target_path(debug,path):
    index = 0
    for i in path:
        location1 = carla.Location(i[0],i[1],i[2])

        if index != 0:
            draw_location_union(debug, location0, location1, green, 60)
        index += 1
        location0 = location1
    pass
# -------------------------------------------
# @note 仿真环境中作图相关，便于debug
# -------------------------------------------
red = carla.Color(255, 0, 0)
green = carla.Color(0, 255, 0)
blue = carla.Color(47, 210, 231)
cyan = carla.Color(0, 255, 255)
yellow = carla.Color(255, 255, 0)
orange = carla.Color(255, 162, 0)
white = carla.Color(255, 255, 255)

def draw_transform(debug, trans, col=carla.Color(255, 0, 0), lt=-1):
    debug.draw_arrow(
        trans.location, trans.location + trans.get_forward_vector(),
        thickness=0.05, arrow_size=0.1, color=col, life_time=lt)


def draw_waypoint_union(debug, w0, w1, color=carla.Color(255, 0, 0), lt=5):
    debug.draw_line(
        w0.transform.location + carla.Location(z=0.25),
        w1.transform.location + carla.Location(z=0.25),
        thickness=0.1, color=color, life_time=lt, persistent_lines=False)
    debug.draw_point(w1.transform.location + carla.Location(z=0.25), 0.1, color, lt, False)

# location版本，不需要输入waypoint
def draw_location_union(debug, location0, location1, color=carla.Color(255, 0, 0), lt=5):
    debug.draw_line(
        location0 + carla.Location(z=0.25),
        location1 + carla.Location(z=0.25),
        thickness=0.05, color=color, life_time=lt, persistent_lines=False)
    debug.draw_point(location1 + carla.Location(z=0.25), 0.08, color, lt, False)

# -------------------------------------------
# @note 获取轴距
# -------------------------------------------
def get_wheelbase(vehicle):
    # 获取轴距
    physics_control = vehicle.get_physics_control()
    wheels = physics_control.wheels
    wheels_pos = []
    wheels_pos.append(wheels[0].position)
    wheels_pos.append(wheels[1].position)
    wheels_pos.append(wheels[2].position)
    wheels_pos.append(wheels[3].position)
    wheelbase = pow(wheels_pos[2].x-wheels_pos[0].x,2) \
                + pow(wheels_pos[2].y-wheels_pos[0].y,2) \
                + pow(wheels_pos[2].z-wheels_pos[0].z,2)
    # 这个wheel.position只是个Vector3D类型，没有说单位是多少
    # 认为单位为厘米（cm）
    return pow(wheelbase,0.5)


# -------------------------------------------
# @note 获取车速（标量）
# -------------------------------------------
def get_speed(vehicle):
    current_v = vehicle.get_velocity()
    v = pow(current_v.x,2) + pow(current_v.y,2) + pow(current_v.z,2)
    return  pow(v,0.5)

# -------------------------------------------
# @note 利用snapshot更新车辆的状态参数
# -------------------------------------------
def get_current_state(world, vehicle_id):
    current_snapshot = world.get_snapshot()
    ActorSnapshot = current_snapshot.find(vehicle_id)
    if ActorSnapshot != None:
        # 获得位置信息
        Actor_transform = ActorSnapshot.get_transform()
        # 获得速度（标量）
        Actor_v = ActorSnapshot.get_velocity().x ** 2 + \
                  ActorSnapshot.get_velocity().y ** 2 + \
                  ActorSnapshot.get_velocity().z ** 2
        Actor_v = pow(Actor_v, 0.5)

        return Actor_transform,Actor_v
    else:
        print('cannot get this actor!')

# -------------------------------------------

# ----------------------------------------
k = 0.1  # 前视距离系数
Lfc = 2.0  # 前视距离
Kp = 1.0  # 速度P控制器系数
dt = 0.1  # 时间间隔，单位：s
L = 2.9  # 车辆轴距，单位：m

import math

def pure_pursuit_control(transform, v, cx, cy, pind):
    ind = calc_target_index(transform, cx, cy, v)

    if pind >= ind:  # 上一次最近的在这一次最近的后面
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:  # 最后一个点
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - transform.location.y, tx - transform.location.x) - math.radians(transform.rotation.yaw)

    if v < 0:  # back
        alpha = math.pi - alpha

    Lf = k * v + Lfc
    # 以弧度为单位
    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def calc_target_index(transform, cx, cy, velocity):
    # 搜索最临近的路点
    dx = [transform.location.x - icx for icx in cx]
    dy = [transform.location.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0

    # print("current index:",ind,'distance:',d)
    # print('velocity:',velocity)
    # cur_ind = ind
    Lf = k * velocity + Lfc
    # 感觉有点问题，是路点之间的线段长度相加？不该是路点到当前点这一个线段的距离吗？
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1
    # print('cur_ind=',cur_ind,"L=",L,'tar_ind=',ind)
    return ind

def pure_pursuit(world,vehicle_id, cx, cy):
    lastIndex = len(cx) - 1
    a_transform, a_v = get_current_state(world, vehicle_id)

    # 这里只用第一次更新target_index，后续更新在循环中完成
    target_ind = calc_target_index(a_transform, cx, cy, a_v)
    while lastIndex > target_ind:
        deltai, target_ind = pure_pursuit_control(a_transform, a_v, cx, cy, target_ind)
        # print("delta",deltai)
        # 这里需要更新控制再次循环获得状态
        time.sleep(0.1)
        a_transform, a_v = get_current_state(world, vehicle_id)
    pass
# ----------------------------------------


def main():
    # actor_list = []

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()
    map = world.get_map()
    print('current map is:',map)

    if map.name != 'Town01' :
        world = client.load_world('Town01')
        time.sleep(5)  # 这个也得延时。不延时就要崩
        map = world.get_map()
        print('current map change to:',map)

    world.set_weather(carla.WeatherParameters.ClearNoon)
    spectator = world.get_spectator()
    debug = world.debug

    # 创建车辆
    blueprint_library = world.get_blueprint_library()
    bp = random.choice(blueprint_library.filter('vehicle.audi.etron'))  # 随机选个车型
    spawn_location = carla.Location(93.0,1.0,1.0)
    spawn_rotation = carla.Rotation(0,180,0)

    # spawn_transform = random.choice(world.get_map().get_spawn_points())  # 随机选个位置
    spawn_transform = carla.Transform(spawn_location,spawn_rotation)
    if bp.has_attribute('color'):   # 随机选个颜色
        color = random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color', color)
    vehicle = world.spawn_actor(bp, spawn_transform)
    vehicle_id = vehicle.id
    print('created：', vehicle)
    # 没这个延时车辆生成到（0,0,0）了
    time.sleep(1)

    # 设置初始观察视角
    spect_transform = spawn_transform
    spect_transform.location += carla.Location(x=0.0, y=0.0, z=50.0)  # 这样修改后两个变量都变了，和C/C++不一样
    # spectator.set_transform(spect_transform)
    wheelbase = get_wheelbase(vehicle)

    # Disable physics, in this example the vehicle is teleported.
    vehicle.set_simulate_physics(True)
    last_w = map.get_waypoint(vehicle.get_location())
    # print('vehicle location:',vehicle.get_location())
    # print('waypoint location:',current_w.transform.location)

    # 控制油门、前轮转角、刹车
    my_vehiclecontrol = carla.VehicleControl(throttle = 0.5)
    vehicle.apply_control(my_vehiclecontrol)

    # 获得目标路线，并画图
    target_path = get_path_main()
    cx = []
    cy = []
    for i in target_path:
        cx.append(i[0])
        cy.append(i[1])
    draw_target_path(debug,target_path)

    try:
        # pure_pursuit(world,vehicle_id,cx,cy)
        lastIndex = len(cx) - 1
        a_transform, a_v = get_current_state(world, vehicle_id)

        # 这里只用第一次更新target_index，后续更新在循环中完成
        target_ind = calc_target_index(a_transform, cx, cy, a_v)
        while lastIndex > target_ind:
            deltai, target_ind = pure_pursuit_control(a_transform, a_v, cx, cy, target_ind)
            # 这里需要更新控制再次循环获得状态
            my_vehiclecontrol.steer = deltai / (math.pi / 3) * 1.0
            if my_vehiclecontrol.steer > 1 : my_vehiclecontrol.steer = 1
            if my_vehiclecontrol.steer < -1 : my_vehiclecontrol.steer = -1


            vehicle.apply_control(my_vehiclecontrol)
            time.sleep(0.1)
            a_transform, a_v = get_current_state(world, vehicle_id)

            # ---------------------------------------------------------
            # currentcontrol = vehicle.get_control()
            # c_steer = currentcontrol.steer
            # print("delta:",deltai,my_vehiclecontrol.steer,'current:',c_steer)
            # print(a_transform)
            # print("target index:",target_ind)
            debug.draw_point(carla.Location(x=target_path[target_ind][0],\
                                            y=target_path[target_ind][1],\
                                            z=0.25), 0.1, red, 30, False)
            # ---------------------------------------------------------

    except KeyboardInterrupt:
        print("ctrl+c pressed!")


    # 获取用户创建的车辆列表
    total_actor_list = world.get_actors()
    vehicle_list = total_actor_list.filter('vehicle.*.*')

    # print(transform)
    print("vehicle_list:",vehicle_list)

    # vehicle.set_autopilot(True)
    # vehicle.set_velocity()
    # time.sleep(5)
    input('type to destroy')
    print('destroying actors')

    # destroy_success = vehicle.destroy()
    # print(destroy_success)

    # --------------------------------------
    # 删除所有车辆
    # --------------------------------------
    # print(vehicle_list)
    for actor in vehicle_list:
        destroy_success = actor.destroy()
        print(destroy_success)
    print('done.')



if __name__ == '__main__':
    main()
