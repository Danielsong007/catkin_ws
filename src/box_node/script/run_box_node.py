#!/usr/bin/env python

from __future__ import print_function
import rospy
import numpy as np
import stack_detection
from visualize import *
import realsenseClass
import open3d as o3d

class BoxVisionHelper():
    def __init__(self, ):
        self.rs=realsenseClass.Realsense()
        self.rs.get_frames()
        self.rs.get_pc()
        self.pts_ = self.rs.reshaped_pc
        self.img_ = self.rs.img_color

    def get_object_pose(self):
        success, result_list = stack_detection.detect_with_view_DL(self.pts_, self.img_)
        if success:
            poses_list = []
            for result_tuple in result_list:
                for index in range(7):
                    poses_list.append(result_tuple[2][index])
            scene = o3d.geometry.PointCloud()
            scene.points = o3d.utility.Vector3dVector(self.pts_.reshape(-1, 3))
            R_list = []
            t_list = []
            for result_tuple in result_list:
                R_list.append(result_tuple[0])
                t_list.append(result_tuple[1])                    
            visualize_multi_pose_in_raw_pts(scene, [], R_list, t_list)
        else:
            print(success, result_list)

if __name__ == "__main__":
    try:
        rospy.init_node('box_node_server')
        helper =  BoxVisionHelper()
        helper.get_object_pose()
    except rospy.ROSInterruptException as e:
        print(e)
