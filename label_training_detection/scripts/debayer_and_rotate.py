#!/usr/bin/env python
# debayer_and_rotate.py
#  Created on: Sep 16, 2016
#      Author: Ramkumar Natarjan
#

import cv2
from os import listdir, walk
from os.path import isfile, join
import shutil, argparse

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Debayer and rotate images recursively in a folder")
    parser.add_argument("source_dir", help="The parent folder containing the images to be debayered")
    parser.add_argument("dst_dir", help="The destination directory to save the images")
    args = parser.parse_args()

    srcFolder = args.source_dir;
    dst = args.dst_dir;
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
		center = (rgb_img.shape[1]/2, rgb_img.shape[0]/2)
		M = cv2.getRotationMatrix2D(center, -90, 1.0)
		rgb_rotated = cv2.warpAffine(rgb_img, M, center)
		output_filename = join(dst, srcfiles[i])
		cv2.imwrite(output_filename, rgb_rotated)
		i+=1
