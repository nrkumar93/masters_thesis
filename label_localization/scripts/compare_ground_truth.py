#!/usr/bin/env python
#
# compare_ground_truth.py
#  Created on: Nov 15, 2016
#      Author: Ramkumar Natarajan
#


import os
import argparse
import json
import matplotlib.pyplot as plt
import numpy as np
import math
import csv
from collections import defaultdict 
from matplotlib.cbook import Sorter

def productIdToPosition(json_data):
    id_to_position = defaultdict(list)
    if 'labels' in json_data:
        json_data = json_data['labels']
    for i in json_data:
        id_to_position[i['product_id']].append(i['location']['bounding_box'])
    return id_to_position

def distance(x, y):
    sq_norm = (x['x1'] - y['x1'])*(x['x1'] - y['x1']) + (x['y1'] - y['y1'])*(x['y1'] - y['y1']) + (x['z1'] - y['z1'])*(x['z1'] - y['z1'])
    return math.sqrt(sq_norm)*100

def bestDistance(product_a, product_b, true_distance):
    dist = []
    aisle_pos_x = []
    for i in product_a:
        for j in product_b:
            dist.append(abs(distance(i,j)-float(true_distance)))
            aisle_pos_x.append(min(i['x1'], j['x1']))
    min_index = dist.index(min(dist))
    return (aisle_pos_x[min_index], dist[min_index])
    
def observationError(groundtruth, smooth_labels):
    error_of_x = []
    for truth in groundtruth:
        if truth[0] in smooth_labels and truth[1] in smooth_labels:
            error_of_x.append(bestDistance(smooth_labels[truth[0]], smooth_labels[truth[1]], truth[2]))
    return error_of_x
    
def removeOutliers(x, y):
    outlier_thresh = 2 * np.std(y)
    data_mean = np.mean(y)
    inlier_x = []
    inlier_y = []
    for i in range(len(x)):
        if abs(y[i] - data_mean) <= outlier_thresh:
            inlier_x.append(x[i])
            inlier_y.append(y[i])
    return inlier_x, inlier_y
    
def main():
    parser = argparse.ArgumentParser(description="Compare smoothed label positions with relative distances between labels measured based on barcodes")
    parser.add_argument("groundtruth", help="Full path to the CSV file containing the ground truth measurements")
    parser.add_argument("output_path", help="Fullpath to save output plot")
    parser.add_argument("smoothed_aisles", nargs=argparse.REMAINDER, help="Full path to the set JSON files containing smoothed label positions of different missions")
    args = parser.parse_args()
    
    with open(args.groundtruth, 'r') as csv_file:
        csv_contents = csv.reader(csv_file, delimiter=',')
        groundtruth = [i[2:5] for i in csv_contents][4:]
        
    transformed_smoothed_data = []
    for aisle in args.smoothed_aisles:
        aisle_file =  open(aisle, 'r').read() 
        transformed_smoothed_data.append(productIdToPosition(json.loads(aisle_file)))
    
    error_per_aisle = []
    for transformed_smoothed_aisle in transformed_smoothed_data:
        error_per_aisle.append(observationError(groundtruth, transformed_smoothed_aisle))

    sort = Sorter()
    for i in range(len(args.smoothed_aisles)):
        sorted = sort(error_per_aisle[i],0)
        aisle_pos_x, err = zip(*sorted)
        aisle_pos_x, err = removeOutliers(list(aisle_pos_x), list(err))
        plt.plot(aisle_pos_x, err, '-o', linewidth=2.0, label=str(len(args.smoothed_aisles) - i - 1) + ' Priors')
        plt.legend(loc='upper left')
    plt.show()

if __name__ == '__main__':
    main()
    