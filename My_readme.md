# Mimic Example Train MaskRCNN on Own Data
https://zhuanlan.zhihu.com/p/88139849
https://github.com/lucasjinreal/FruitsNutsSeg

# Labelme to COCO is from: 
https://www.jianshu.com/p/ac3159ca8161
https://github.com/fcakyon/labelme2coco

# demo of pytorch
https://zhuanlan.zhihu.com/p/111144134

# Realse
Realse-ros-melodic: https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy
get PointCloud: https://blog.csdn.net/gyxx1998/article/details/121611001

# Bug of send_data_to_box_node
change to: ros_msg.data = b"".join(buffer)

# Moveit+Camera
- Moveit assistant setup + publish the tf between 'baselink' and 'camera'
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link camera_link
