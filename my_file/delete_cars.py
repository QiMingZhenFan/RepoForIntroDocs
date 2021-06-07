#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


import carla

import random
import time


def main():
    actor_list = []

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        world = client.get_world()
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)

        actor_list = world.get_actors()
        vehicle_list = actor_list.filter('vehicle.*.*')

        print('vehicles in carla now: ', str(vehicle_list))

        key = raw_input('press key to destroy all the vehicles!')
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
        print('done.')

    except KeyboardInterrupt:
        pass


if __name__ == '__main__':

    main()
