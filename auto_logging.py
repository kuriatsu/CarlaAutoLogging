#!/usr/bin/env python
# -*- coding:utf-8 -*-

import carla
import random
import time
import csv
import logging

def main():
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)

    world = client.get_world()

    blueprint = world.get_blueprint_library().filter('vehicle.mini*')[0]
    color = random.choice(blueprint.get_attribute('color').recommended_values)
    blueprint.set_attribute('color', color)
    if blueprint.has_attribute('driver_id'):
        driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
        blueprint.set_attribute('driver_id', driver_id)
    blueprint.set_attribute('role_name', 'hero')

    tm = client.get_trafficmanager(8000)
    for actor in client.get_world().get_actors():
        if actor.attributes.get('number_of_wheels') in ['2', '4']:
            tm.vehicle_percentage_speed_difference(actor, 0)
            tm.ignore_lights_percentage(actor, 100)

    for i in range(3, 150):

        ego_vehicle = None

        spawn_points = world.get_map().get_spawn_points()
        transform = random.choice(spawn_points)
        batch = [carla.command.SpawnActor(blueprint, transform).then(carla.command.SetAutopilot(carla.command.FutureActor, True))]

        for response in client.apply_batch_sync(batch):
            if response.error:
                logging.error(response.error)
            else:
                actor = world.get_actor(response.actor_id)
                if actor.attributes.get('role_name') == 'hero':
                    ego_vehicle = actor
        # ego_vehicle = world.spawn_actor(blueprint, transform)
        world.wait_for_tick()
        ego_vehicle.set_autopilot(True)

        if ego_vehicle is None:
            continue

        tm.vehicle_percentage_speed_difference(ego_vehicle, 0)
        tm.ignore_lights_percentage(ego_vehicle, 100)

        # log_file = '/home/kuriatsu/Source/CarlaAutoLogging/carla_data/test_{}.log'.format(str(i))
        # client.start_recorder(log_file)
        # waypoint_file = '/home/kuriatsu/Source/CarlaAutoLogging/carla_data/test_{}.csv'.format(str(i))

        print(log_file, waypoint_file)

        waypoint_step = 0.0
        travel_dist = 0.0
        start_time = time.time()
        last_loc = ego_vehicle.get_location()
        waypoint = []


        while True:

            loc = ego_vehicle.get_location()
            step = (((loc.x - last_loc.x) ** 2 + (loc.y - last_loc.y) ** 2) ** 0.5)
            last_loc = loc
            travel_dist += step
            waypoint_step += step

            if waypoint_step > 0.5:
                trans = ego_vehicle.get_transform()
                vel = ego_vehicle.get_velocity()
                speed = (vel.x ** 2 + vel.y ** 2) ** 0.5
                waypoint.append([trans.location.x, trans.location.y, trans.location.z, -np.radians(trans.rotation.yaw), speed, 0])
                waypoint_step = 0.0

            print(travel_dist)

            if travel_dist > 350:
                break

            if (time.time() - start_time) > 60:
                if travel_dist > 20:
                    break
                else:
                    return

            world.wait_for_tick()

        client.apply_batch([carla.command.DestroyActor(ego_vehicle.id)])
        # client.stop_recorder()

        # with open(waypoint_file, 'w') as f:
        #     writer = csv.writer(f)
        #     writer.writerow(['x', 'y', 'z', 'yaw', 'velocity', 'change_flag'])
        #     writer.writerows(waypoint)

if __name__ == "__main__":

    main()
