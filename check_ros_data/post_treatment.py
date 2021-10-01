#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pickle
import sys

if __name__ == "__main__":
    with open(sys.argv[1], 'rb') as f:
        data = pickle.load(f)

    # find intervention point
    first_intervention_index = None
    for i, drive_data in enumerate(data.get("drive_data")):
        if drive_data.get('intervention'):
            first_intervention_index = i
            break

    if first_intervention_index is None:
        print('no intervention')
        exit()

    # find 33 sec before intervention
    start_index = None
    end_index = None
    for i, drive_data in enumerate(data.get("drive_data")):
        if start_index is None and data.get('drive_data')[first_intervention_index].get('time') - drive_data.get('time') < 33.0:
            start_index = i

        if start_index is not None and drive_data.get('time') - data.get('drive_data')[start_index].get('time') > 50.0:
            end_index = i
            break


    if start_index is None or end_index is None:
        print("intervention segment is shorter than 10sec")
        exit()

    out_intervention_data = {'waypoint' : data.get('waypoint')}
    out_intervention_data['drive_data'] = data["drive_data"][start_index:end_index]

    fullpath = sys.argv[1].rsplit('/', 1)
    filename_split = fullpath[1].split('_')
    out_dir = fullpath[0].rsplit('/', 1)[0] + '/extracted_data'
    out_intervention_filename = "{}/{}_{}_{}_int_{}".format(out_dir, filename_split[0], filename_split[1], filename_split[2], filename_split[4])

    with open(sys.argv[2], 'rb') as f:
        no_int_data = pickle.load(f)

    start_index = None
    end_index = None
    for i, drive_data in enumerate(no_int_data.get('drive_data')):
        if start_index is None and drive_data.get('mileage_progress') > out_intervention_data.get('drive_data')[0].get('mileage_progress'):
            start_index = i
        if start_index is not None and drive_data.get('mileage_progress') > out_intervention_data.get('drive_data')[-1].get('mileage_progress'):
            end_index = i
            break

    print(start_index , end_index)

    if start_index is None or end_index is None:
        print('failed to extract no_int data')
        exit()

    out_nointervention_data = {'waypoint' : no_int_data.get('waypoint')}
    out_nointervention_data['drive_data'] = no_int_data["drive_data"][start_index:end_index]
    out_nointervention_filename = "{}/{}_{}_{}_noint_{}".format(out_dir, filename_split[0], filename_split[1], filename_split[2], filename_split[4])
    print(out_nointervention_data.get('drive_data')[-1].get('time') - out_nointervention_data.get('drive_data')[0].get('time'))


    for drive_data in out_intervention_data.get('drive_data'):
        start_mileage = out_intervention_data['drive_data'][0]['mileage_progress']
        total_mileage = out_intervention_data['drive_data'][-1]['mileage_progress'] - out_intervention_data['drive_data'][0]['mileage_progress']
        start_progress = out_intervention_data['drive_data'][0]['simulate_progress']
        total_progress = out_intervention_data['drive_data'][-1]['simulate_progress'] - out_intervention_data['drive_data'][0]['simulate_progress']

        drive_data['mileage_progress'] = (drive_data['mileage_progress'] - start_mileage) / total_mileage
        drive_data['simulate_progress'] = (drive_data['simulate_progress'] - start_progress) / total_progress
        print(drive_data['simulate_progress'], drive_data['mileage_progress'])

    for drive_data in out_nointervention_data.get('drive_data'):
        start_mileage = out_nointervention_data['drive_data'][0]['mileage_progress']
        total_mileage = out_nointervention_data['drive_data'][-1]['mileage_progress'] - out_nointervention_data['drive_data'][0]['mileage_progress']
        start_progress = out_nointervention_data['drive_data'][0]['simulate_progress']
        total_progress = out_nointervention_data['drive_data'][-1]['simulate_progress'] - out_nointervention_data['drive_data'][0]['simulate_progress']

        drive_data['mileage_progress'] = (drive_data['mileage_progress'] - start_mileage) / total_mileage
        drive_data['simulate_progress'] = (drive_data['simulate_progress'] - start_progress) / total_progress
        print(drive_data['simulate_progress'], drive_data['mileage_progress'])


    with open(out_intervention_filename, 'wb') as f:
        pickle.dump(out_intervention_data, f)

    with open(out_nointervention_filename, 'wb') as f:
        pickle.dump(out_nointervention_data, f)
