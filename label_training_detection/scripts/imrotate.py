#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  imrotate.py
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

from PIL import Image
from os import listdir, walk
from os.path import isfile, join
import shutil

if __name__ == '__main__':
	
    srcFolder = "/home/prasanna/workspace_ram/dataset/lowes/label_training/big_blue_box/src/test/";
    dst = "/home/prasanna/workspace_ram/dataset/lowes/label_training/big_blue_box/src/test_rot/";
    filepath = []
    srcfiles = [] 	
    #~ srcfiles = [f for f in listdir(srcFolder) if isfile(join(srcFolder, f))];
    for root, dirs, files in walk(srcFolder):
		for f in files:
			filepath.append(join(root, f))
			srcfiles.append(f)
    i=0
    for f in filepath:
		src =  f;
		img = Image.open(src);
		rot = img.rotate(-90);
		dst_name = dst + srcfiles[i];
		rot.save(dst_name)
		i+=1
	

