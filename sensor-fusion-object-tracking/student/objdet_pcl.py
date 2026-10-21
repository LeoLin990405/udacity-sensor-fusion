# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Process the point-cloud and prepare it for object detection
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# general package imports
import cv2
import numpy as np
import torch

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

# waymo open dataset reader
from tools.waymo_reader.simple_waymo_open_dataset_reader import utils as waymo_utils
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2, label_pb2

# object detection tools and helper functions
import misc.objdet_tools as tools


# visualize lidar point-cloud
def show_pcl(pcl):
    try:
        import open3d as o3d
    except ImportError:
        print("open3d is not installed; skipping point-cloud visualization.")
        return

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="LiDAR point cloud")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcl[:, 0:3])
    vis.add_geometry(pcd)

    keep_running = {"value": True}

    def close_visualizer(vis_obj):
        keep_running["value"] = False
        return False

    vis.register_key_callback(262, close_visualizer)
    while keep_running["value"]:
        vis.poll_events()
        vis.update_renderer()

    vis.destroy_window()
       

# visualize range image
def show_range_image(frame, lidar_name):

    lidar = waymo_utils.get(frame.lasers, lidar_name)
    ri, _, _ = waymo_utils.parse_range_image_and_camera_projection(lidar)

    ri_range = ri[:, :, 0]
    ri_intensity = ri[:, :, 1]
    ri_range[ri_range < 0] = 0
    ri_intensity[ri_intensity < 0] = 0

    ri_range = ri_range * 255 / max(np.amax(ri_range), 1e-6)
    p1, p99 = np.percentile(ri_intensity, [1, 99])
    ri_intensity = np.clip(ri_intensity, p1, p99)
    ri_intensity = 255 * (ri_intensity - p1) / max(p99 - p1, 1e-6)
    img_range_intensity = np.vstack((ri_range, ri_intensity)).astype(np.uint8)
    
    return img_range_intensity


# create birds-eye view of lidar data
def bev_from_pcl(lidar_pcl, configs):

    # remove lidar points outside detection area and with too low reflectivity
    mask = np.where((lidar_pcl[:, 0] >= configs.lim_x[0]) & (lidar_pcl[:, 0] <= configs.lim_x[1]) &
                    (lidar_pcl[:, 1] >= configs.lim_y[0]) & (lidar_pcl[:, 1] <= configs.lim_y[1]) &
                    (lidar_pcl[:, 2] >= configs.lim_z[0]) & (lidar_pcl[:, 2] <= configs.lim_z[1]))
    lidar_pcl = lidar_pcl[mask]

    if lidar_pcl.shape[0] == 0:
        bev_map = np.zeros((1, 3, configs.bev_height, configs.bev_width))
        return torch.from_numpy(bev_map).to(configs.device, non_blocking=True).float()
    
    # shift level of ground plane to avoid flipping from 0 to 255 for neighboring pixels
    lidar_pcl[:, 2] = lidar_pcl[:, 2] - configs.lim_z[0]  

    # convert sensor coordinates to bev-map coordinates (center is bottom-middle)
    bev_discret = (configs.lim_x[1] - configs.lim_x[0]) / configs.bev_height
    lidar_pcl_cpy = np.copy(lidar_pcl)
    lidar_pcl_cpy[:, 0] = np.int_(np.floor(lidar_pcl_cpy[:, 0] / bev_discret))
    lidar_pcl_cpy[:, 1] = np.int_(np.floor((lidar_pcl_cpy[:, 1] - configs.lim_y[0]) / bev_discret))
    
    
    # Compute intensity layer of the BEV map
    intensity_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    idx_sorted = np.lexsort((-lidar_pcl_cpy[:, 2], lidar_pcl_cpy[:, 1], lidar_pcl_cpy[:, 0]))
    lidar_pcl_cpy = lidar_pcl_cpy[idx_sorted]
    _, idx_unique, counts = np.unique(lidar_pcl_cpy[:, 0:2], axis=0, return_index=True, return_counts=True)
    lidar_pcl_top = lidar_pcl_cpy[idx_unique]
    intensity = lidar_pcl_top[:, 3]
    intensity = (intensity - np.min(intensity)) / max(np.max(intensity) - np.min(intensity), 1e-6)
    intensity_map[np.int_(lidar_pcl_top[:, 0]), np.int_(lidar_pcl_top[:, 1])] = intensity


    # Compute height layer of the BEV map
    height_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    height_map[np.int_(lidar_pcl_top[:, 0]), np.int_(lidar_pcl_top[:, 1])] = \
        lidar_pcl_top[:, 2] / float(np.abs(configs.lim_z[1] - configs.lim_z[0]))

    # Compute density layer of the BEV map
    density_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    _, _, counts = np.unique(lidar_pcl_cpy[:, 0:2], axis=0, return_index=True, return_counts=True)
    normalizedCounts = np.minimum(1.0, np.log(counts + 1) / np.log(64)) 
    density_map[np.int_(lidar_pcl_top[:, 0]), np.int_(lidar_pcl_top[:, 1])] = normalizedCounts
        
    # assemble 3-channel bev-map from individual maps
    bev_map = np.zeros((3, configs.bev_height, configs.bev_width))
    bev_map[2, :, :] = density_map[:configs.bev_height, :configs.bev_width]  # r_map
    bev_map[1, :, :] = height_map[:configs.bev_height, :configs.bev_width]  # g_map
    bev_map[0, :, :] = intensity_map[:configs.bev_height, :configs.bev_width]  # b_map

    # expand dimension of bev_map before converting into a tensor
    s1, s2, s3 = bev_map.shape
    bev_maps = np.zeros((1, s1, s2, s3))
    bev_maps[0] = bev_map

    bev_maps = torch.from_numpy(bev_maps)  # create tensor from birds-eye view
    input_bev_maps = bev_maps.to(configs.device, non_blocking=True).float()
    return input_bev_maps
