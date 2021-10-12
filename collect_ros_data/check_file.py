#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pickle
import sys

if __name__ == "__main__":
    with open(sys.argv[1], 'rb') as f:
        data = pickle.load(f)

    with open("intervened_target.pickle", 'r') as f_2:
        intervened_target = pickle.load(f_2)

    if len(data.get("drive_data")) > 10:
        if not intervened_target.get("last_list") or intervened_target.get("last_list")[0] in intervened_target.get('no_list'):
            print("no intervention occured")
            sys.exit(0)

        else:
            print("next intervention trial")
            sys.exit(1)

    else:
        print("data collection failed")
        sys.exit(-1)
