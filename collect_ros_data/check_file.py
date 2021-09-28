#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pickle
import sys

if __name__ == "__main__":
    with open(sys.argv[1], 'rb') as f:
        data = pickle.load(f)

    with open("last_int_distance.txt", 'r') as f_2:
        intervention_mieage = float(f_2.read())

    if len(data.get("drive_data")) > 10:
        if intervention_mieage == 0.0:
            print("no intervention occured")
            sys.exit(0)

        elif intervention_mieage > 0.0:
            print("next intervention trial")
            sys.exit(1)

    else:
        print("data collection failed")
        sys.exit(-1)
