#!/usr/bin/env python

import os
import sys
import subprocess
import argparse
import time
import shutil
import datetime

from mission_directories.mission import Mission
from mission_directories.mission import get_mission
from bnr_map_manager import get_locations_root

def main():
    
    parser = argparse.ArgumentParser(description="Run analytics pipeline in batch")
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
    
    #bnr_analytics launch params
    fastscan = 'false'

    for i in range(0, len(missions_list)):
		
		# Running Analytics pipeline
		if args.config == 'CUSTOM':
			mode = os.path.dirname(missions_list[i])
			run_label = os.path.basename(missions_list[i])
		else:
			run_label = missions_list[i]
			mode = ''
		command = "roslaunch bnr_analytics_launch server_system.launch "
		command += " run_label:=" + run_label
		command += " aisles:=" + args.aisle_name
		command += " fastscan:=" + fastscan
		if args.config == 'CUSTOM':
			command += " mode:=" + mode
		print '+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++'
		print command
		print '+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++'
		subprocess.call(command, shell=True)
		time.sleep(1)
    

if __name__ == '__main__':
    main()
