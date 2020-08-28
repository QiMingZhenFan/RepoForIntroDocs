#!/usr/bin/env python
# -*- coding: utf-8 -*-

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


    number = 50
    i = 1
    vehicle_list = []
    try:
        while i <= number :
            i += 1
            vehicle = None
            while vehicle is None:
                spawn_transform = random.choice(world.get_map().get_spawn_points())  # 随机选个位置
                if bp.has_attribute('color'):   # 随机选个颜色
                    color = random.choice(bp.get_attribute('color').recommended_values)
                    bp.set_attribute('color', color)
                vehicle = world.try_spawn_actor(bp, spawn_transform)
            vehicle_id = vehicle.id
            print('created：', vehicle)
            # 没这个延时车辆生成到（0,0,0）了
            vehicle_list.append(vehicle)
            time.sleep(1)


        # 获取用户创建的车辆列表
        # total_actor_list = world.get_actors()
        # vehicle_list = total_actor_list.filter('vehicle.*.*')

        print("Setting vehicle autopilot...")

        for item in vehicle_list:
            # Disable physics, in this example the vehicle is teleported.
            item.set_simulate_physics(True)
            item.set_autopilot(True)

        while True:
            pass
    # except KeyboardInterrupt:
    except:
        print("ctrl+c pressed!")
    finally:




    # print(transform)
    # print("vehicle_list:",vehicle_list)

    # vehicle.set_autopilot(True)
    # vehicle.set_velocity()
    # time.sleep(5)
    # input('type to destroy')
        print('destroying actors')

    # destroy_success = vehicle.destroy()
    # print(destroy_success)

    # --------------------------------------
    # 删除所有车辆
    # --------------------------------------
    # print(vehicle_list)
    # for actor in vehicle_list:
    #     destroy_success = actor.destroy()
    #     print(destroy_success)
    # 此处循环删除出问题，得用这个apply batch一波全波删除
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
        print('done.')



if __name__ == '__main__':
    main()
