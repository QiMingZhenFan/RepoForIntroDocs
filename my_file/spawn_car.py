#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

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
    actor_list = []

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        world = client.get_world()

        blueprint_library = world.get_blueprint_library()

        bp = random.choice(blueprint_library.filter('vehicle.dodge_charger.police'))

        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)

        transform = random.choice(world.get_map().get_spawn_points())

        vehicle = world.spawn_actor(bp, transform)

        actor_list.append(vehicle)
        print('created %s' % vehicle.type_id)
        print('current transform: ', str(transform))
        # vehicle.set_autopilot(True)

        # camera_bp = blueprint_library.find('sensor.camera.depth')
        # camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        # camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        # actor_list.append(camera)
        # print('created %s' % camera.type_id)
        #
        # cc = carla.ColorConverter.LogarithmicDepth
        # camera.listen(lambda image: image.save_to_disk('_out/%06d.png' % image.frame, cc))

        # location = vehicle.get_location()
        # location.x += 40
        # vehicle.set_location(location)
        # print('moved vehicle to %s' % location)

        # transform.location += carla.Location(x=40, y=-3.2)
        # transform.rotation.yaw = -180.0
        # for _ in range(0, 10):
        #     transform.location.x += 8.0
        #
        #     bp = random.choice(blueprint_library.filter('vehicle'))
        #
        #     # This time we are using try_spawn_actor. If the spot is already
        #     # occupied by another object, the function will return None.
        #     npc = world.try_spawn_actor(bp, transform)
        #     if npc is not None:
        #         actor_list.append(npc)
        #         npc.set_autopilot()
        #         print('created %s' % npc.type_id)
        #
        # time.sleep(5)
        while(1):
            pass
    except KeyboardInterrupt:
        pass
    finally:

        print('destroying actors')
        # camera.destroy()
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('done.')


if __name__ == '__main__':

    main()
