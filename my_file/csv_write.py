#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
    将carla中车辆的位置写入csv文件中，用于测试循迹
    2021.06.07

    note: 同步模式下，多clien生成车辆是可以的，但是查询actor相关信息时需要卡在某一个时间点才可以查到（从机）
'''
import csv
import carla
import os

dirname = os.path.dirname(os.path.realpath(__file__))
if dirname[-1] != '/':
    dirname += '/'
filename = dirname + "data.csv"

def main():
    actor_list = []

    client = carla.Client('10.214.143.211', 2000)
    client.set_timeout(2.0)

    world = client.get_world()
    world_snapshot = world.wait_for_tick()
    is_found = False
    actor_id = None
    for actor_snapshot in world_snapshot: # Get the actor and the snapshot information
        actual_actor = world.get_actor(actor_snapshot.id)
        if actual_actor.type_id[0:7] == 'vehicle':
            transform = actor_snapshot.get_transform()
            is_found = True
            actor_id = actual_actor.id
            # actor_snapshot.get_velocity()
            # actor_snapshot.get_angular_velocity()
            # actor_snapshot.get_acceleration()  
            break
    if is_found:
        print(str(actual_actor))
        print(transform)    
    else:
        return
    
    lastLocation = carla.Location()
    with open(filename, 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')

        try:
            while True:
                world_snapshot = world.wait_for_tick()
                actor_snapshot = world_snapshot.find(actor_id)
                transform = actor_snapshot.get_transform()
                distance = transform.location.distance(lastLocation)
                # print('distance: ', distance)
                if distance > 2:
                    writer.writerow([transform.location.x, transform.location.y, transform.location.z] + [0] * 4)
                    lastLocation = transform.location
                    print([transform.location.x, transform.location.y, transform.location.z] + [0] * 4)
                
        except KeyboardInterrupt:
            pass

if __name__ == '__main__':

    main()



