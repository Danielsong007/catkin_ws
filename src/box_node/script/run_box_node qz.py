#!/usr/bin/env python

from __future__ import print_function

import time
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2, Image, PointField
from roport.srv import GetObjectPose, GetObjectPoseResponse

# vision dependence
import cv2
import numpy as np
import stack_detection, pcl2_img
import pcl
from sensor_msgs import point_cloud2 as pc2
import ros_numpy 
from visualize import *
import open3d as o3d

DUMMY_FIELD_PREFIX = '__'
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)
# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}

def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1
    
    return np_dtype_list

def pointcloud2_to_array(cloud_msg):
    ''' Converts a rospy PointCloud2 message to a numpy recordarray 

    Reshapes the returned array to have shape (height, width), even if the height is 1.
    The reason for using np.frombuffer rather than struct.unpack is speed... especially
    for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)
    # print("- 1:", time.time())

    # parse the cloud into an array
    cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)
    # print("- 2:", time.time())

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]
    # print("- 3:", time.time())

    cloud_arr = cloud_arr.view('<f4').reshape(len(cloud_arr), -1)
    return cloud_arr[:,:3]

class BoxVisionHelper(object):

    def __init__(self, ):
        super(BoxVisionHelper, self).__init__()

        self.display_result_ = True
        self.is_pose_single_ = False
        self.is_pcl_updated_ = False
        self.is_img_updated_ = False

        self._get_pcl_sub = rospy.Subscriber(
            '/hv1000/point_cloud', PointCloud2, self._get_pcl_handle
        )

        self._get_img_sub = rospy.Subscriber(
            '/hv1000/twod_image', Image, self._get_img_handle
        )

        self._get_object_pose_srv = rospy.Service(
            '/box/get_object_pose', GetObjectPose, self._get_object_pose_handle
        )

    def _get_pcl_handle(self, ros_pointcloud2):
        # convert ros to pcl
        print(ros_pointcloud2.height)
        print(ros_pointcloud2.width)
        pts = pointcloud2_to_array(ros_pointcloud2)
        self.pts_ = pts[:, 0 : 3]
        self.is_pcl_updated_ = True
        rospy.loginfo('Point Cloud has been updated')

    def _get_img_handle(self, ros_image):
        # convert ros to cv2
        image_nparray = ros_numpy.image.image_to_numpy(ros_image)
        self.img_ = image_nparray
        self.is_img_updated_ = True
        rospy.loginfo('Image has been updated')

    def _get_object_pose_handle(self, req):
        resp = GetObjectPoseResponse()
        if self.is_pose_single_:
            # if only need one pose as result, make sure the update processes of pcl and image are completed
            rospy.loginfo('Single pose, Check whether the Point CLoud and Image have been updated')
            while not self.is_pcl_updated_ or not self.is_img_updated_:
                rospy.sleep(0.1)
            rospy.loginfo('Confirm that the Point CLoud and Image have been updated')
            # detect pose of target box
            success, result_list = stack_detection.detect_with_view_DL(self.pts_, self.img_)
            result_tuple = result_list[0]
            if success:
                R, t, pose_detected, rec_wid, rec_len = result_tuple
                ## visualize the detected result
                if self.display_result_:
                    scene = o3d.geometry.PointCloud()
                    scene.points = o3d.utility.Vector3dVector(self.pts_)
                    visualize_pose_in_raw_pts(scene, [], R, t)

                # assign result and respond to client
                pose = Pose()
                pose.position.x = pose_detected[0]
                pose.position.y = pose_detected[1]
                pose.position.z = pose_detected[2]
                pose.orientation.x = pose_detected[3]
                pose.orientation.y = pose_detected[4]
                pose.orientation.z = pose_detected[5]
                pose.orientation.w = pose_detected[6]
                resp.pose = pose
                resp.pose_amount = resp.SINGLE
                resp.result_status = resp.SUCCEEDED
                rospy.logwarn('Get pose %f, %f, %f, %f, %f, %f, %f' % (pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w))
                rospy.loginfo('detect pose successed')
            else:
                resp.result_status = resp.FAILED
                rospy.logerr('detect pose failed')

            self.is_pcl_updated_ = False
            self.is_img_updated_ = False
        else:
            # if need many poses as result, make sure the update processes of pcl is completed
            rospy.loginfo('Multi pose, Check whether the Point CLoud and Image have been updated')
            while not self.is_pcl_updated_ or not self.is_img_updated_:
                rospy.sleep(0.1)
            rospy.loginfo('Confirm that the Point CLoud and Image have been updated')
            # detect pose of target box
            success, result_list = stack_detection.detect_with_view_DL(self.pts_, self.img_)
            if success:
                poses_list = []
                for result_tuple in result_list:
                    for index in range(7):
                        poses_list.append(result_tuple[2][index])
                ## visualize the detected result
                if self.display_result_:
                    scene = o3d.geometry.PointCloud()
                    scene.points = o3d.utility.Vector3dVector(self.pts_)
                    R_list = []
                    t_list = []
                    for result_tuple in result_list:
                        # result, grasp_box, cluster_planes, idx = result_tuple
                        R_list.append(result_tuple[0])
                        t_list.append(result_tuple[1])                    
                    visualize_multi_pose_in_raw_pts(scene, [], R_list, t_list)

                # assign result and respond to client
                resp.poses_list = poses_list
                resp.pose_amount = resp.MULTI
                resp.result_status = resp.SUCCEEDED
                rospy.logwarn('data size is %d'%len(resp.poses_list))
                rospy.loginfo('detect poses successed')
            else:
                resp.poses_list = [0,0,0,0,0,0,1]
                resp.pose_amount = resp.MULTI
                resp.result_status = resp.SUCCEEDED
                rospy.logerr('detect poses failed, return fake pose_list size is %d'%len(resp.poses_list))

            self.is_pcl_updated_ = False
            self.is_img_updated_ = False           
        return resp

if __name__ == "__main__":
    try:
        rospy.init_node('box_node_server')
        helper =  BoxVisionHelper()
        rospy.loginfo("Box Node: BoxVision helper ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
