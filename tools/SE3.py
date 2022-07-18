import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import List


class SE3:
    def __init__(self, quaternion_wxyz: List, translation: List):
        # scipy quaternion: xyzw
        # change quaternion format wxyz to xyzw
        quaternion_wxyz = np.array(quaternion_wxyz).reshape(4)
        translation = np.array(translation).reshape(3)
        self.Rotation = R.from_quat([quaternion_wxyz[x] for x in [1, 2, 3, 0]])
        self.rotation_matrix = np.array(self.Rotation.as_matrix())
        self.translation = translation
        self.SE3_matrix = self.get_SE3_matrix()
        self.SE3_inv_matrix = self.get_SE3_inv_matrix()

    def get_SE3_matrix(self):
        SE3_matrix = np.append(self.rotation_matrix,
                               self.translation.reshape(-1, 1), axis=1)
        SE3_matrix = np.append(SE3_matrix,
                               np.array([[0, 0, 0, 1]]), axis=0)
        return SE3_matrix

    def get_SE3_inv_matrix(self):
        SE3_inv_matrix = np.append(self.rotation_matrix.transpose(),
                                   -self.rotation_matrix.transpose().dot(self.translation.reshape(-1, 1)).reshape(-1, 1), axis=1)
        SE3_inv_matrix = np.append(SE3_inv_matrix,
                                   np.array([[0, 0, 0, 1]]), axis=0)
        return SE3_inv_matrix
