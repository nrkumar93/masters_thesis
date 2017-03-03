#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  batch_label_localization_top_level.py
#  
#  Copyright 2016 Prasanna <prasanna@prasanna-ThinkStation-P300>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  

import json
import argparse
import subprocess
import math
import matplotlib.pyplot as plt

def main():

    parser = argparse.ArgumentParser(description="Use label_defs to set params. Open this program to set step size and aspect ratio.")
    parser.add_argument("label_defs", help="The path to label_defs.json file")
    parser.add_argument("input_dir", help="The directory containing symlinks to input images.")
    args = parser.parse_args()
	    
    step_size = 0.02
    aspect_ratio = 1.75
    
    avg_time = []    
    x = []
    yy = []
    diag_dist = []
    for i in range(40):
		label_defs = open(args.label_defs, 'r').read()
		defs = json.loads(label_defs)
		defs['label_definitions'][0]['classifier_types']['source'][0]['scale_factor'] = defs['label_definitions'][0]['classifier_types']['source'][0]['scale_factor'] + (i*step_size)
		
		with open('/tmp/label_defs.json', 'w') as outfile:
			json.dump(defs, outfile)
		
		print defs['label_definitions'][0]['classifier_types']['source'][0]['scale_factor']
		command = 'rosrun bnr_analytics_labels label_detect_tool '
		command += ' --images_path ' + args.input_dir
		command += ' --label_defs_json ' + '/tmp/label_defs.json'
		print '========================================EXECUTING========================================'
		print command
		print '========================================================================================='
		cmd = command.split()
		ret = subprocess.Popen(cmd, stdout=subprocess.PIPE)
		avg_time.append(float(ret.stdout.readlines()[-1].replace('\n', '')))
		diag_dist.append(math.sqrt(defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_height']**2 + defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_width']**2) - math.sqrt(defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_height']**2 + defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_width']**2))
		x.append(defs['label_definitions'][0]['classifier_types']['source'][0]['scale_factor'])
		yy.append(math.log(defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_width']/defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_width'], defs['label_definitions'][0]['classifier_types']['source'][0]['scale_factor']))
		print avg_time
		print x
		if defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_height']*defs['label_definitions'][0]['classifier_types']['source'][0]['scale_factor'] > defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_height'] or defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_width']*defs['label_definitions'][0]['classifier_types']['source'][0]['scale_factor'] > defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_width']:
			x = x[0:len(x)-1]
			avg_time = avg_time[0:len(avg_time)-1]
			break
    plt.plot(x, avg_time, '-o', linewidth=1.0, label='Observed')
    plt.plot(x, yy, '-x', linewidth=1.0, label='Theoretical')
    plt.xlabel('Scale Factor ($\gamma$)')
    plt.ylabel('Time taken (ms)')
    plt.legend(loc='upper left')
    plt.ylim((0, max(avg_time)+3))
    plt.show()
    return 0

if __name__ == '__main__':
	main()
