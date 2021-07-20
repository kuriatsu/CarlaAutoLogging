#!/bin/bash

python config.py -m Town01
python spawn_npc.py -n 40 -w 60
python auto_logging.py
