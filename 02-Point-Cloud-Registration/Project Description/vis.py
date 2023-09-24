# From:
# https://stackoverflow.com/questions/69184560/visualizing-a-sequence-of-point-clouds-in-open3d-0-10-0-as-a-video


import open3d as o3d
from open3d import *
import numpy as np
import struct
from natsort import natsorted
import os
import time

def convert_kitti_bin_to_pcd(binFilePath):
    # Load binary point cloud
    bin_pcd = np.fromfile(binFilePath, dtype=np.float32)

    # Reshape and drop reflection values
    points = bin_pcd.reshape((-1, 4))[:, 0:3]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    # Convert to Open3D point cloud
    return pcd

input_dir = "00/"
input_files = natsorted(os.listdir(input_dir))
    
# read first ply file
pcd = convert_kitti_bin_to_pcd(os.path.join(input_dir, input_files[0]))

# getting open3d to display the video
vis = o3d.visualization.Visualizer()
vis.create_window()



# iterate through remaining files     
for input_file in input_files[1:]:
    input_file = os.path.join(input_dir, input_file)
    
    # remove previous pointcloud and add new one    
    vis.remove_geometry(pcd,False)
    pcd = convert_kitti_bin_to_pcd(input_file)
        
    # add pcd to visualizer
    vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    time.sleep(0.005) # makes display a little smoother (display will not match fps of video)

vis.destroy_window()