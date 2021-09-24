#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pickle
import sys

if __name__ == "__main__":
    with open(sys.argv[1], 'rb') as f:
        data = pickle.load(f)

    if len(data.get('drive_data')) > 10:
        sys.exit(1)
    else:
        print('data collection failed')
        sys.exit(0)
