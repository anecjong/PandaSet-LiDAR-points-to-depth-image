'''
version: 1.4
anecjong
version 1.1: z_norm max added.
version 1.2: for loop to numpy matmul.
version 1.3: npy is not saved
version 1.4: draw points from close points to far points.
if nearby pixel(represented by gap) has value, skip drawing for that point.
'''
import numpy as np
import open3d as o3d
import cv2
import sys


class Lidar2Camera:
    def __init__(self, intrinsic_matrix: np.ndarray, extrinsic_matrix: np.ndarray, width: int, height: int, z_norm: int):
        self.intrinsic_matrix = intrinsic_matrix
        self.extrinsic_matrix = extrinsic_matrix
        self.width = width
        self.height = height
        self.z_norm = z_norm

    def pcd_data_to_camera_frame(self, pcd_origin: np.ndarray):
        pcd_origin = np.append(pcd_origin.transpose(), np.ones(
            [1, pcd_origin.shape[0]], dtype=np.float64), axis=0)
        pcd_camera = self.extrinsic_matrix.astype(
            np.float64)[:3, :].dot(pcd_origin).transpose()
        pcd_camera = pcd_camera[np.where(pcd_camera[:, 2] > 0), :]
        pcd_camera = np.squeeze(pcd_camera)
        pcd_camera = pcd_camera[pcd_camera[:, 2].argsort()[::1]]
        return pcd_camera

    def pcd_data_path_to_camera_frame(self, path: str):
        pcd_origin = np.array(o3d.io.read_point_cloud(path).points)
        return self.pcd_data_to_camera_frame(pcd_origin)

    def pcd_camera_to_2d_img(self, pcd_camera: np.ndarray):
        Z_max = np.max(pcd_camera[:, 2]).item()
        Z_min = np.min(pcd_camera[:, 2]).item()
        Z_norm = (pcd_camera[:, 2] - Z_min)/(Z_max-Z_min)
        img_2d = np.zeros((self.height, self.width), dtype=np.uint8)
        pos_img = self.intrinsic_matrix.astype(
            np.float64).dot(pcd_camera.transpose()).transpose()

        for i in range(pos_img.shape[0]):
            pos_img[i] /= pos_img[i][-1]

            u = int(round(pos_img[i][0], 0))
            v = int(round(pos_img[i][1], 0))

            if 0 <= u < self.width and 0 <= v < self.height:
                gap = 3
                u_min, u_max = max(0, -gap+u), min(self.width - 1, gap+u)
                v_min, v_max = max(0, -gap+v), min(self.height - 1, gap+v)

                check_val = np.sum(
                    img_2d[v_min:v_max+1, u_min:u_max + 1]).item()
                depth_val = min(self.z_norm, round(Z_norm[i]*self.z_norm, 0))

                if check_val == 0:
                    img_2d[v, u] = depth_val
                else:
                    continue

        return img_2d

    def pcd_data_path_to_2d_img(self, path: str, save_path="result_img.png"):
        img_2d = self.pcd_camera_to_2d_img(
            self.pcd_data_path_to_camera_frame(path))
        if save_path != "":
            cv2.imwrite(save_path, img_2d)

    def pcd_data_to_2d_img(self, pcd_origin: np.ndarray, save_path="result_img.png"):
        img_2d = self.pcd_camera_to_2d_img(
            self.pcd_data_to_camera_frame(pcd_origin))
        if save_path != "":
            cv2.imwrite(save_path, img_2d)
