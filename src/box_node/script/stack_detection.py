import pcl
from segment import segment_2D
from grasp_selection import grasp_selection
from pose import pose_estimation
import numpy as np

# get the corresponding 3D point cloud, according to the image masks
def crop_point_cloud(pc_array, pc_masks):
    masked_pc_list = []
    for mask in pc_masks:
        masked_pc_list.append(pc_array[mask, :])
    return masked_pc_list

def detect_with_view_DL(pts, gray_img):
    pc_array = pts
    masks = segment_2D(gray_img)
    masked_pc_list = crop_point_cloud(pc_array, masks)
    if(len(masked_pc_list) == 0):
        print("No any plane structures!")
        return False, []
    else:
        masked_pc_filter_list = []
        i = 0
        for masked_pc in masked_pc_list:
            masked_pc = masked_pc[:, :3]
            cloud_in = pcl.PointCloud()
            cloud_in.from_array(masked_pc)
            uni_down_Filter = cloud_in.make_voxel_grid_filter()
            uni_down_Filter.set_leaf_size(0.005, 0.005, 0.005)
            cloud_ds = uni_down_Filter.filter()
            sor_Filter = cloud_ds.make_statistical_outlier_filter()
            sor_Filter.set_mean_k(90)
            sor_Filter.set_std_dev_mul_thresh(1.0)
            cloud_sor = sor_Filter.filter()
            masked_pc_filter = cloud_sor.to_array()
            masked_pc_filter = masked_pc_filter[np.where(np.all(masked_pc_filter!=[0.0, 0.0, 0.0], axis=-1)==True)[0]]
            if(masked_pc_filter.shape[0] == 0):
                continue
            else:
                masked_pc_filter_list.append(masked_pc_filter)
                i = i+1
        if(len(masked_pc_filter_list) == 0):
            print("No any plane structures!")
            return False, []
        else:
            grasp_plane_ids = grasp_selection(masked_pc_filter_list)
            pose_list = []
            if (len(grasp_plane_ids) != 0):
                for i in range(len(grasp_plane_ids)):
                    idx = grasp_plane_ids[i]
                    cluster_planes_i = masked_pc_filter_list[idx] # Pose estimation
                    pose_list.append(pose_estimation(cluster_planes_i))
                return True, pose_list
