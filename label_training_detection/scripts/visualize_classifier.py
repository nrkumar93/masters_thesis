#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  visualize_classifier.py
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

import os
import cv2
import argparse
import numpy as np
import xml.etree.ElementTree as ET


feats_per_stage = []
feats = []
tilted_per_feat = []
featID_in_stage = []
feat_img = []

def improperXML():
  raise Exception('The classifier XML is not properly formatted. Please verify whether you have given the correct classifier.')

def main():
    parser = argparse.ArgumentParser(description="Visualize the HAAR features obtained from Opencv Cascade Classifier")
    parser.add_argument("classifier", help="The full path to the classifier ")
    parser.add_argument("overlay", help="Image on which the features should be overlaid.")
    parser.add_argument("dst", help="The path to save the visualization.")
    args = parser.parse_args()
    
    if os.path.splitext(args.classifier)[1] != '.xml':
      raise Exception('A non XML classifier is provided. Cannot be parsed! Aborting...')
      
    # Create classifier XML object for parsing through the XML
    obj = ET.parse(args.classifier).getroot()
    if obj.tag != 'opencv_storage':
      improperXML()
    
    '''
    for i in range(len(obj[0])):
      if obj[i].tag == 'cascade':
        for j in range(len(obj[i])):
          if obj[i][j].tag == 'stages':
            for k in range(len(obj[i][j])):
              if obj[i][j][k].tag == '_':
                for l in range(len(obj[i][j][k])):
                  if obj[i][j][k][l].tag == 'maxWeakCount':
                    feats_per_stage.append(int(obj[i][j][k][l].text))
    '''
    
    for i in obj.iter('width'):
      width = int(i.text)    
    for i in obj.iter('height'):
      height = int(i.text)    
    
    # Parse XML to collect weak classifiers per stage
    for i in obj.iter('stages'):
      for j in i.iter('maxWeakCount'):
        feats_per_stage.append(int(j.text))        
    for i in obj.iter('stageNum'):  
      assert(len(feats_per_stage) == int(i.text))

    # Parse XML to collect all the features in the classifier.
    for i in obj.iter('rects'):
      rect_in_feat=[]
      for j in i.iter('_'):
        rect_in_feat.append(j.text.split())
      feats.append(rect_in_feat)
    
    # Parse XML to collect 'tilted' flag per feature 
    for i in obj.iter('tilted'):
      tilted_per_feat.append(int(i.text))

    assert(sum(feats_per_stage) == len(feats))
    assert(sum(feats_per_stage) == len(tilted_per_feat))
    
    for i in feats:
      haar = np.ones((height, width), dtype = 'u1')*127
      for j in i:
        if float(j[-1]) < 0:
          haar[int(j[1]):int(j[1]+j[3]), int(j[0]):int(j[0]+j[2])] = 255
        else:
          haar[int(j[1]):int(j[1]+j[3]), int(j[0]):int(j[0]+j[2])] = 0          
      feat_img.append(haar)
    
    
    for i in feat_img:
      res = cv2.resize(i, None, fx=5, fy=5, interpolation=cv2.INTER_LINEAR)
      cv2.imshow('img', res)
      cv2.waitKey(0)
    
    #~ for i in obj.iter('internalNodes'):
      #~ featID_in_stage.append(i.text.split()[2])
      	
    return 0

if __name__ == '__main__':
	main()

