#!/usr/bin/env python
# -*- coding:utf-8 -*-

import carla
import random
import time
import csv
import logging
import pickle
import sys

def createEgoBlueprint(world, role_name):

    blueprint = world.get_blueprint_library().filter('vehicle.mini*')[0]
    color = random.choice(blueprint.get_attribute('color').recommended_values)
    blueprint.set_attribute('color', color)
    if blueprint.has_attribute('driver_id'):
        driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
        blueprint.set_attribute('driver_id', driver_id)
    blueprint.set_attribute('role_name', role_name)

    return blueprint


def setupTrafficManager(client, world):

    tm_get = False
    while not tm_get:
        try:
            tm = client.get_trafficmanager(8000)
            tm_get = True
        except:
            tm_get = False

    for actor in client.get_world().get_actors():
        if actor.attributes.get('number_of_wheels') in ['2', '4']:
            tm.vehicle_percentage_speed_difference(actor, 0)
            tm.vehicle_percentage_speed_difference(actor, 0)
            tm.ignore_lights_percentage(actor, 100)
            tm.ignore_lights_percentage(actor, 100)

    print('set traffic manager')


def spawnEgoVehicle(client, world, blueprint):

    spawn_points = world.get_map().get_spawn_points()
    transform = random.choice(spawn_points)
    batch = [carla.command.SpawnActor(blueprint, transform).then(carla.command.SetAutopilot(carla.command.FutureActor, True))]

    ego_vehicle = None
    while ego_vehicle is None:
        for response in client.apply_batch_sync(batch):
            if response.error:
                logging.error(response.error)
            else:
                actor = world.get_actor(response.actor_id)
                if actor.attributes.get('role_name') == 'ego_vehicle':
                    ego_vehicle = actor

    world.wait_for_tick()
    ego_vehicle.set_autopilot(True)
    print('set autopilot')

    tm_get = False
    while not tm_get:
        try:
            tm = client.get_trafficmanager(8000)
            tm_get = True
        except:
            tm_get = False

    tm.vehicle_percentage_speed_difference(ego_vehicle, 0)
    tm.vehicle_percentage_speed_difference(ego_vehicle, 0)
    tm.ignore_lights_percentage(ego_vehicle, 100)
    tm.ignore_lights_percentage(ego_vehicle, 100)
    print('set ego_vehicle traffic manager')

    return ego_vehicle


def carlaVectorToList(vector):
    return [vector.x, vector.y, vector.z]


def getWp(actor):
    trans = actor.get_transform()
    wp = [trans.location.x, -trans.location.y, trans.location.z, -trans.rotation.yaw, actor.get_speed_limit(), 0]
    return wp


def getActorData(actor):
    trans = actor.get_transform()
    vel = actor.get_velocity()
    data = {
        'type' : actor.type_id,
        'pose' : [trans.location.x, -trans.location.y, trans.location.z, -trans.rotation.yaw],
        'speed' : (vel.x ** 2 + vel.y ** 2) ** 0.5,
        'size': carlaVectorToList(actor.bounding_box.extent),
        }
    return data


def getActorListData(ego_vehicle, actor_list):

    data = {}
    data['ego_vehicle'] = getActorData(ego_vehicle)

    for actor in actor_list:
        data[actor.id] = getActorData(actor)

    return data


def driveLoop(world, actor_list, ego_vehicle, dist_step_data, time_step_data, waypoint):

    wp_step_length = 0.0
    wp_interval = 0.5
    save_step_time = 0.0
    save_interval = 0.5

    start_time = world.wait_for_tick().timestamp.elapsed_seconds
    last_tick_loc = ego_vehicle.get_location()

    while True:

        loc = ego_vehicle.get_location()
        wp_step_length += ((loc.x - last_tick_loc.x) ** 2 + (loc.y - last_tick_loc.y) ** 2) ** 0.5
        last_tick_loc = loc
        save_step_time += world.get_snapshot().timestamp.delta_seconds

        if wp_step_length > wp_interval:
            waypoint.append(getWp(ego_vehicle))

            data = {
                'trigger' : getWp(ego_vehicle),
                'actors' : getActorListData(ego_vehicle, actor_list),
                'intervention' : None,
                'collision' : None,
                }

            dist_step_data.append(data)
            wp_step_length = 0.0

        if save_step_time > save_interval:
            data = {
                'time' : world.get_snapshot().timestamp.elapsed_seconds,
                'actors' : getActorListData(ego_vehicle, actor_list),
                'intervention' : None,
                'collision' : None,
                }

            time_step_data.append(data)
            save_step_time = 0.0

        recoad_time = len(time_step_data) * save_interval
        travel_dist = len(dist_step_data) * wp_interval
        sys.stdout.write('\r' + str(travel_dist))
        sys.stdout.flush()

        if recoad_time > 30:
            if travel_dist < 20:
                print('travel distance is less than 20m in 30sec -> exit')
                exit()
            else:
                return

        world.wait_for_tick()

    return


def main():

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    ego_blueprint = createEgoBlueprint(world, 'ego_vehicle')

    actor_list = []
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == 'ego_vehicle':
            continue
        elif actor.type_id.startswith('walker') or actor.type_id.startswith('vehicle'):
            actor_list.append(actor)

    setupTrafficManager(client, world)

    for i in range(int(sys.argv[2]), int(sys.argv[3])):
        log_name = sys.argv[1]
        log_file = '{}_{}.log'.format(log_name, str(i))
        dist_step_file = '{}_{}_dist.pickle'.format(log_name, str(i))
        time_step_file = '{}_{}_time.pickle'.format(log_name, str(i))

        print(log_file)

        for actor in client.get_world().get_actors():
        	if actor.attributes.get('role_name') == 'ego_vehicle':
        		actor.destroy()

        ego_vehicle = None
        while ego_vehicle is None:
            ego_vehicle = spawnEgoVehicle(client, world, ego_blueprint)

        dist_step_data = []
        time_step_data = []
        waypoint = []

        client.start_recorder(log_file)
        driveLoop(world, actor_list, ego_vehicle, dist_step_data, time_step_data, waypoint)
        client.stop_recorder()

        with open(dist_step_file, 'wb') as f:
            pickle.dump(dist_step_data, f)

        time_data = {'data':time_step_data, 'waypoint':waypoint}
        with open(time_step_file, 'wb') as f:
            pickle.dump(time_data, f)


if __name__ == '__main__':
    main()
