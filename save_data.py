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


def setupTrafficManager(client, tm):

    for actor in client.get_world().get_actors():
        if actor.attributes.get('number_of_wheels') in ['2', '4']:
            tm.vehicle_percentage_speed_difference(actor, 0)
            tm.ignore_lights_percentage(actor, 100)


def spawnEgoVehicle(client, world, tm):

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

        world.wait_for_tick()
        ego_vehicle.set_autopilot(True)

        if ego_vehicle is None:
            return None

        tm.vehicle_percentage_speed_difference(ego_vehicle, 0)
        tm.ignore_lights_percentage(ego_vehicle, 100)

        return ego_vehicle


def getWp(actor):
    trans = ego_vehicle.get_transform()
    vel = ego_vehicle.get_velocity()
    speed = (vel.x ** 2 + vel.y ** 2) ** 0.5
    wp = [trans.location.x, -trans.location.y, trans.location.z, -np.degrees(trans.rotation.yaw), speed, 0]


def getDistStepActorData(actor_list)
    data = {}
    for actor in actor_list:
        data[actor.id] = {
            'type' : actor.type_id,
            'waypoint' : getWp(actor),
            'size': actor.bounding_box.extent,
            }

    return data



def getTimeStepActorData(world_data, world, ego_vehicle):

    data{}
    data['ego_vehicle'] = {
        'type' : ego_vehicle.type_id,
        'waypoint' : getWp(ego_vehicle),
        'size': ego_vehicle.bounding_box.extent,
        }

    for actor in actor_list:
        data[actor.id] = {
            'type' : actor.type_id,
            'waypoint' : getWp(actor),
            'size': actor.bounding_box.extent,
            }

    return data

def drive_loop(world, actor_list, ego_vehicle, dist_step_data, time_step_data):

    wp_step_length = 0.0
    wp_interval = 0.5
    save_step_time = 0.0
    save_interval = 0.5

    start_time = world.wait_for_tick().timestamp.elapsed_seconds
    last_tick_position = ego_vehicle.get_location()

    while True:

        loc = ego_vehicle.get_location()
        last_tick_loc = loc
        wp_step_length += (((loc.x - last_tick_loc.x) ** 2 + (loc.y - last_tick_loc.y) ** 2) ** 0.5
        save_step_time += world.get_snapshot().timestamp.delta_seconds

        if wp_step_length > wp_interval:
            data = {
                'waypoint' : getWp(ego_vehicle),
                'actors' : getDistStepActorData(actor_list)
                }
            dist_step_data.append(data)
            wp_step_length = 0.0

        if save_step_time > save_interval:
            data = {
                'time' : world.get_snapshot().timestamp.elapsed_seconds,
                'actors' : getTimeStepActorData(world_data, world, ego_vehicle)
                }
            time_step_data.append(data)
            save_step_time = 0.0

        data_length = len(time_step_data) * save_interval
        travel_dist = len(dist_step_data) * wp_interval

        if data_length > 30:
            if travel_dist > 20:
                exit()
            else:
                return

        world.wait_for_tick()

    return


def main():

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    tm = client.get_trafficmanager(8000)

    ego_blueprint = createegoBlueprint(world, 'ego_vehicle')

    actor_list = []
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == 'ego_vehicle':
            continue
        elif actor.type_id.startswith('walker') or actor.type_id.startswith('vehicle')
            actor_list.append(actor)

    setupTrafficManager(client, tm)

    for i in range(1, 150):
        log_name = sys.args[1]
        log_file = '/home/kuriatsu/Source/CarlaAutoLogging/{}_{}.log'.format(log_name, str(i))
        dist_step_file = '/home/kuriatsu/Source/CarlaAutoLogging/{}_{}_dist.pickle'.format(log_name, str(i))
        time_step_file = '/home/kuriatsu/Source/CarlaAutoLogging/{}_{}_time.pickle'.format(log_name, str(i))

        while ego_vehicle is None:
            ego_vehicle = spawnEgoVehicle(client, world, tm)

        dist_step_data = []
        time_step_data = []

        client.start_recorder(log_file)

        drive_loop(world, actor_list, ego_vehicle, dist_step_data, time_step_data)

        client.stop_recorder()

        client.apply_batch([carla.command.DestroyActor(ego_vehicle.id)])
        ego_vehicle = None

        with open(dist_step_file, 'wb') as f:
            pickle.dump(dist_step_data, f)

        with open(time_step_file, 'wb') as f:
            pickle.dump(time_step_data, f)

