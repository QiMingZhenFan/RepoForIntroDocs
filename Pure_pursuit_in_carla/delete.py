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


def main():
    # actor_list = []

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()
    map = world.get_map()
    print('current map is:',map)


    # 获取用户创建的车辆列表
    total_actor_list = world.get_actors()
    vehicle_list = total_actor_list.filter('vehicle.*.*')

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
