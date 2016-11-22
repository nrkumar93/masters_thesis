#!/usr/bin/env python
# smoothed_label_overlay.py
#  Created on: Nov 22, 2016
#      Author: Ramkumar Natarjan
#

import os
import sys
import subprocess
import argparse
import time

from mission_directories.mission import Mission
from mission_directories.mission import get_mission
from bnr_map_manager import get_locations_root

def main():
    
    parser = argparse.ArgumentParser(description="Smooth using label landmarks and generate the full information smoothed map.")
    parser.add_argument("config", choices=['ROBOT', 'OFFLOAD', 'NAS', 'CUSTOM'], help="The data source type. Defines the search paths.")
    parser.add_argument("aisle_name", help="The name of the aisle for which label localization has to be tested")
    parser.add_argument("missions", nargs=argparse.REMAINDER, default="", help="The name of list of missions separated by a blank space. In the case of CUSTOM, this is the full path to each of all the mission")
    args = parser.parse_args()

    # List of all missions to be tested with label localization
    missions_list = args.missions
    
    # Create mission_directories object for all the missions
    if args.config == 'CUSTOM':
        mission = [Mission(os.path.basename((os.path.realpath(mis)).rstrip('/')), [os.path.realpath(mis)]) for mis in missions_list]
    else:
        mission = [get_mission(mis, args.config) for mis in missions_list]
    
    # Create a list of aisles. This list will only contain the single aisle specified
    aisle_names = [args.aisle_name]

    results_folder_name = 'indigo-feature-landmark_localization'    # This is temporary. I Cannot find a better way to do it.

    for i in range(0, len(missions_list)):
        
        # Delete any existing smoothed bags
        for smoothed_bag in mission[i].get_preferred_folder("SmoothedBags"):
            if args.aisle_name in os.path.basename(smoothed_bag):
                os.remove(smoothed_bag)

        # Run per-aisle smoothing for a mission
        mission_prior_filename = os.path.join(mission[i].get_preferred_folder('Analytics'), results_folder_name, str(args.aisle_name) + '_prior_landmarks.json')
        mission_observations_filename = os.path.join(mission[i].get_preferred_folder('Analytics'), results_folder_name, str(args.aisle_name) + '_observed_landmarks.json')
        mission_output_filename = os.path.join(mission[i].get_preferred_folder('Analytics'), results_folder_name, str(args.aisle_name) + '_output_landmarks.json')
        command = "smooth_aisles.py " + args.config + " " + missions_list[i]
        command += " --process-metadata false"
        command += " --aisle-names " + ' '.join(aisle_names)
        command += " --landmark-enabled true"
        command += ' --landmark-prior-filename ' + mission_prior_filename
        command += ' --landmark-observations-filename ' + mission_observations_filename
        command += ' --landmark-output-filename ' + mission_output_filename
        command += " --force"
        command += " --verbose"
        command += " --debug"
        subprocess.call(command, shell=True)

        # Convert landmarks to labels again which serves as the prior for next mission.
        command = "rosrun bnr_analytics_map_manager label_from_landmark.py "
        command += ' ' + os.path.join(mission[i].get_preferred_folder('Analytics'), results_folder_name, str(args.aisle_name) + '_output_landmarks.json')
        command += ' ' + os.path.join(mission[i].get_preferred_folder('Analytics'), results_folder_name, '_reverse_map.json')
        subprocess.call(command, shell=True)

if __name__ == '__main__':
    main()
