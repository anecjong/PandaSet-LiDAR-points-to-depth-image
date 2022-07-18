import open3d as o3d


def pcd_remove_outlier(pcd):
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20,
                                                        std_ratio=2.0)
    inlier_cloud = voxel_down_pcd.select_by_index(ind)
    return inlier_cloud
