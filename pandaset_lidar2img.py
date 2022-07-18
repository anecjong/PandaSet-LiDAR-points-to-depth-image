import pandas as pd
from tools.Lidar2Camera import *
from tools.SE3 import *
from tools.Inlier_selection import *
import os
import sys
import yaml
import numpy as np
import open3d as o3d
import cv2
import glob
import json
import shutil

with open("./pandaset_calibration.yaml", "r") as stream:
    try:
        yaml_file = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

cam = yaml_file['front_camera']
lidar = yaml_file['front_gt']

rot_vc_wxyz = [cam['extrinsic']['transform']['rotation'][idx]
               for idx in ['w', 'x', 'y', 'z']]
t_vc = [cam['extrinsic']['transform']['translation'][idx]
        for idx in ['x', 'y', 'z']]

rot_vl_wxyz = [lidar['extrinsic']['transform']['rotation'][idx]
               for idx in ['w', 'x', 'y', 'z']]
t_vl = [lidar['extrinsic']['transform']['translation'][idx]
        for idx in ['x', 'y', 'z']]

D = np.asarray(cam['intrinsic']['D'])
K = np.asarray(cam['intrinsic']['K']).reshape(3, 3)

width = 1920
height = 1080

pandas_dataset_path = "./PandaSet"

for seq_num in sorted(os.listdir(pandas_dataset_path)):
    cam_path = os.path.join(pandas_dataset_path, seq_num,
                            "camera", "front_camera")
    lid_path = os.path.join(pandas_dataset_path, seq_num, "lidar")
    cam_pose_path = os.path.join(cam_path, "poses.json")
    with open(cam_pose_path, 'r') as f:
        cam_pose_data = json.load(f)
    if not os.path.isdir("./img2d/lidar"):
        os.makedirs("./img2d/lidar")
    if not os.path.isdir("./img2d/camera"):
        os.makedirs("./img2d/camera")

    for idx, lidar_pkl in enumerate(sorted(glob.glob(lid_path + "/*.pkl"))):
        cam_img = lidar_pkl.replace(lid_path, cam_path).replace(".pkl", ".jpg")

        cam_pose = cam_pose_data[idx]
        t_wc = [cam_pose['position'][i] for i in ['x', 'y', 'z']]
        rot_wc_wxyz = [cam_pose['heading'][i] for i in ['w', 'x', 'y', 'z']]
        se3_wc = SE3(rot_wc_wxyz, t_wc)
        T_wc = se3_wc.get_SE3_matrix()
        T_cw = se3_wc.get_SE3_inv_matrix()
        l2c = Lidar2Camera(K, T_cw, width, height, 255)

        df = pd.read_pickle(lidar_pkl)
        xyz = df.iloc[:, 0:3].to_numpy()
        # print(xyz)
        # sys.exit()
        # pcd_origin = o3d.geometry.PointCloud()
        # pcd_origin.points = o3d.utility.Vector3dVector(xyz)
        # pcd_origin = pcd_remove_outlier(pcd_origin)
        # pcd_origin = np.array(pcd_origin.points)

        pcd_origin = xyz

        save_path = "./img2d/lidar/" + seq_num + "_" + \
            lidar_pkl.split('/')[-1].replace(".pkl", ".png")
        print(save_path)

        shutil.copy(cam_img, save_path.replace(
            "/lidar/", "/camera/").replace(".png", ".jpg"))
        l2c.pcd_data_to_2d_img(pcd_origin, save_path)
