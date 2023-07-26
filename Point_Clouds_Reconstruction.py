import numpy as np
import open3d as o3d
import plotly.graph_objects as go
import cv2
from PIL import Image
import math
import itertools
import copy
import matplotlib.pyplot as plt

demo_icp_pcds = o3d.data.OfficePointClouds()

# Load each partial point clouds
voxel_size = 0.04
radius_normal = voxel_size * 2
radius_feature = voxel_size * 3
distance_threshold = voxel_size

# Source
prev_pcd = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])

# Down Sampling
prev_pcd = prev_pcd.voxel_down_sample(voxel_size=voxel_size)
prev_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

# FPFH
prev_feature = o3d.pipelines.registration.compute_fpfh_feature(
        prev_pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

# Empty point clouds
pcd_global = o3d.geometry.PointCloud()
pcd_global += prev_pcd

for i in range(1, 30):
  # Target
  pcd = o3d.io.read_point_cloud(demo_icp_pcds.paths[i])

  # Down Sampling
  pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
  pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

  # FPFH
  feature = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
  
  # Pose Estimation(RANSAC)
  result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        prev_pcd, pcd, prev_feature, feature, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.3),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(10000000, 0.99))
  
  # ICP Refinement
  T = result.transformation
  refinement_result = o3d.pipelines.registration.registration_icp(prev_pcd, pcd, distance_threshold, T,
           o3d.pipelines.registration.TransformationEstimationPointToPlane())
  T_ref = refinement_result.transformation

  pcd_global = copy.deepcopy(pcd_global).transform(T_ref)
  pcd_global += pcd
  
  prev_pcd = pcd
  prev_feature = feature

# Reconstruction File
o3d.io.write_point_cloud("office.ply", pcd_global)