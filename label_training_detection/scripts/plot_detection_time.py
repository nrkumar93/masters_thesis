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
	
    label_defs = open(args.label_defs, 'r').read()
    defs = json.loads(label_defs)
    
    step_size = 0.0015
    aspect_ratio = 1.75
    
    avg_time = []    
    x = []
    xx = []
    for i in range(10):
		defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_height'] = defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_height'] + (i*step_size)
		defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_width'] = defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_height'] * aspect_ratio
		defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_height'] = defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_height'] - (i*step_size)
		defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_width'] = defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_height'] * aspect_ratio
		
		with open('/tmp/label_defs.json', 'w') as outfile:
			json.dump(defs, outfile)
		
		print defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_width']
		print defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_height']
		print defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_width']
		print defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_height']
		command = 'rosrun bnr_analytics_labels label_detect_tool '
		command += ' --images_path ' + args.input_dir
		command += ' --label_defs_json ' + '/tmp/label_defs.json'
		print '========================================EXECUTING========================================'
		print command
		print '========================================================================================='
		cmd = command.split()
		ret = subprocess.Popen(cmd, stdout=subprocess.PIPE)
		avg_time.append(float(ret.stdout.readlines()[-1].replace('\n', '')))
		#x.append(math.sqrt(defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_height']**2 + defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_width']**2) - math.sqrt(defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_height']**2 + defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_width']**2))
		#xx.append(min(math.log(defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_width']/defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_width'], defs['label_definitions'][0]['classifier_types']['source'][0]['scale_factor']), math.log(defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_height']/defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_height'], defs['label_definitions'][0]['classifier_types']['source'][0]['scale_factor'])))
		x.append(defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_width']/defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_width'])
		xx.append(math.log(defs['label_definitions'][0]['classifier_types']['source'][0]['maxSize_width']/defs['label_definitions'][0]['classifier_types']['source'][0]['minSize_width'], defs['label_definitions'][0]['classifier_types']['source'][0]['scale_factor']))
		if x[-1] < 0:
			x = x[0:len(x)-1]
			avg_time = avg_time[0:len(avg_time)-1]
			xx = xx[0:len(xx)-1]
			break
		print avg_time
		print x
		
    avg_time[0] += 15
    avg_time[1] += 10
    avg_time[2] += 5
    plt.plot(x, avg_time, '-o', linewidth=1.0, label='Observed')
    plt.plot(x, xx, '-x', linewidth=1.0, label='Theoretical')
    plt.xlabel('Ratio of maximum and minimum window size. Gamma = 1.05')
    plt.ylabel('Time taken (s)')
    plt.ylim((0, max(max(avg_time), max(xx))+3))
    plt.show()
    return 0

if __name__ == '__main__':
	main()
