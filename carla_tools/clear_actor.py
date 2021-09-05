#!/usr/bin/env python
# -*- coding:utf-8 -*-
import carla
client = carla.Client('127.0.0.7', 2000)
for actor in client.get_world().get_actors():
	if actor.attributes.get('role_name') == 'ego_vehicle':
		actor.destroy()
