# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Evaluate performance of object detection
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# general package imports
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import torch
from shapely.geometry import Polygon
from operator import itemgetter

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

# object detection tools and helper functions
import misc.objdet_tools as tools


# compute various performance measures to assess object detection
def measure_detection_performance(detections, labels, labels_valid, min_iou=0.5):
    
     # find best detection for each valid label 
    true_positives = 0 # no. of correctly detected objects
    center_devs = []
    ious = []
    for label, valid in zip(labels, labels_valid):
        matches_lab_det = []
        if valid: # exclude all labels from statistics which are not considered valid
            label_corners = tools.compute_box_corners(
                label.box.center_x, label.box.center_y, label.box.width,
                label.box.length, label.box.heading)
            label_poly = Polygon(label_corners)

            for det in detections:
                _, x, y, z, h, w, l, yaw = det
                det_corners = tools.compute_box_corners(x, y, w, l, yaw)
                det_poly = Polygon(det_corners)
                union = label_poly.union(det_poly).area
                if union <= 0:
                    continue
                iou = label_poly.intersection(det_poly).area / union
                if iou > min_iou:
                    dist_x = label.box.center_x - x
                    dist_y = label.box.center_y - y
                    dist_z = label.box.center_z - z
                    matches_lab_det.append([iou, dist_x, dist_y, dist_z])
            
        # find best match and compute metrics
        if matches_lab_det:
            best_match = max(matches_lab_det,key=itemgetter(0)) # retrieve entry with max iou in case of multiple candidates   
            ious.append(best_match[0])
            center_devs.append(best_match[1:])
            true_positives += 1


    # compute positives and negatives for precision/recall
    all_positives = int(np.sum(labels_valid))
    false_negatives = all_positives - true_positives
    false_positives = len(detections) - true_positives
    
    pos_negs = [all_positives, true_positives, false_negatives, false_positives]
    det_performance = [ious, center_devs, pos_negs]
    
    return det_performance


# evaluate object detection performance based on all frames
def compute_performance_stats(det_performance_all):

    # extract elements
    ious = []
    center_devs = []
    pos_negs = []
    for item in det_performance_all:
        ious.append(item[0])
        center_devs.append(item[1])
        pos_negs.append(item[2])
    
    positives = sum(item[0] for item in pos_negs)
    true_positives = sum(item[1] for item in pos_negs)
    false_negatives = sum(item[2] for item in pos_negs)
    false_positives = sum(item[3] for item in pos_negs)

    precision = true_positives / max(true_positives + false_positives, 1)
    recall = true_positives / max(positives, 1)
    print('precision = ' + str(precision) + ", recall = " + str(recall))   

    # serialize intersection-over-union and deviations in x,y,z
    ious_all = [element for tupl in ious for element in tupl]
    devs_x_all = []
    devs_y_all = []
    devs_z_all = []
    for tuple in center_devs:
        for elem in tuple:
            dev_x, dev_y, dev_z = elem
            devs_x_all.append(dev_x)
            devs_y_all.append(dev_y)
            devs_z_all.append(dev_z)
    

    # compute statistics
    stdev__ious = np.std(ious_all) if ious_all else 0.0
    mean__ious = np.mean(ious_all) if ious_all else 0.0

    stdev__devx = np.std(devs_x_all) if devs_x_all else 0.0
    mean__devx = np.mean(devs_x_all) if devs_x_all else 0.0

    stdev__devy = np.std(devs_y_all) if devs_y_all else 0.0
    mean__devy = np.mean(devs_y_all) if devs_y_all else 0.0

    stdev__devz = np.std(devs_z_all) if devs_z_all else 0.0
    mean__devz = np.mean(devs_z_all) if devs_z_all else 0.0
    #std_dev_x = np.std(devs_x)

    # plot results
    data = [precision, recall, ious_all, devs_x_all, devs_y_all, devs_z_all]
    titles = ['detection precision', 'detection recall', 'intersection over union', 'position errors in X', 'position errors in Y', 'position error in Z']
    textboxes = ['', '', '',
                 '\n'.join((r'$\mathrm{mean}=%.4f$' % (mean__devx, ), r'$\mathrm{sigma}=%.4f$' % (stdev__devx, ), r'$\mathrm{n}=%.0f$' % (len(devs_x_all), ))),
                 '\n'.join((r'$\mathrm{mean}=%.4f$' % (mean__devy, ), r'$\mathrm{sigma}=%.4f$' % (stdev__devy, ), r'$\mathrm{n}=%.0f$' % (len(devs_y_all), ))),
                 '\n'.join((r'$\mathrm{mean}=%.4f$' % (mean__devz, ), r'$\mathrm{sigma}=%.4f$' % (stdev__devz, ), r'$\mathrm{n}=%.0f$' % (len(devs_z_all), )))]

    f, a = plt.subplots(2, 3)
    a = a.ravel()
    num_bins = 20
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    for idx, ax in enumerate(a):
        ax.hist(data[idx], num_bins)
        ax.set_title(titles[idx])
        if textboxes[idx]:
            ax.text(0.05, 0.95, textboxes[idx], transform=ax.transAxes, fontsize=10,
                    verticalalignment='top', bbox=props)
    plt.tight_layout()
    plt.show()
