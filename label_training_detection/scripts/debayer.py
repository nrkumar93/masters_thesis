#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  debayer.py
#  
#  Copyright 2016 Ramkumar Natarajan <ram@ramkumar-ubuntu>
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


import cv2
from os import listdir, walk
from os.path import isfile, join
import shutil

if __name__ == '__main__':

    srcFolder = "/mnt/nas/blueberry-mission-data/BNR-lowes_0780-20160720_152950EDT-gaetaplus1-imaging_a4w/BNR-lowes_0780-20160720_152950EDT-gaetaplus1-imaging_a4w-ShelfView/A4W/";
    dst = "/home/prasanna/workspace_ram/dataset/lowes/label_training/big_blue_box/src/test/";
    filepath = []
    srcfiles = []
    #~ srcfiles = [f for f in listdir(srcFolder) if isfile(join(srcFolder, f))]
    for root, dirs, files in walk(srcFolder):
		for f in files:
			if f != 'metadata.json':
				filepath.append(join(root, f))
				srcfiles.append(f)
	
    i=0
    for src in filepath:
		#~ src = srcFolder + f;
		raw_img = cv2.imread(src, 0)
		rgb_img = cv2.cvtColor(raw_img,cv2.COLOR_BAYER_BG2BGR)
		output_filename = dst + srcfiles[i]
		cv2.imwrite(output_filename, rgb_img)
		i+=1
