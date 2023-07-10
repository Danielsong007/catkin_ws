import pyrealsense2 as rs
import numpy as np
import cv2
import time
 
class Realsense():
    def __init__(self, ):
        self.pc = rs.pointcloud()
        self.points = rs.points()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        [self.height,self.width]=[480,640]
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)
        self.pipe_profile = self.pipeline.start(self.config) 
        self.depth_sensor = self.pipe_profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        [self.ph,self.pw]=[340,160]
    
    def get_frames(self):
        self.frames = self.pipeline.wait_for_frames()
        self.aligned_frames = self.align.process(self.frames)
        self.aligned_depth_frame = self.aligned_frames.get_depth_frame()
        self.aligned_color_frame = self.aligned_frames.get_color_frame()
        self.img_color = np.asanyarray(self.aligned_color_frame.get_data())
        # cv2.imwrite("/home/mo/catkin_ws/src/box_node/mydata/15.jpg",self.img_color)
        self.img_depth = np.asanyarray(self.aligned_depth_frame.get_data())
        self.depth_intrin = self.aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        self.color_intrin = self.aligned_color_frame.profile.as_video_stream_profile().intrinsics
        self.depth_to_color_extrin = self.aligned_depth_frame.profile.get_extrinsics_to(self.aligned_color_frame.profile)

    def get_pc(self):
        self.pc.map_to(self.aligned_color_frame)
        self.points = self.pc.calculate(self.aligned_depth_frame)
        self.vtx = np.asanyarray(self.points.get_vertices())
        self.vtx = np.reshape(self.vtx,(self.height, self.width, -1)) # XYZ of [x,y] in img_color is self.vtx[y][x][0]
        self.reshaped_pc=np.zeros((self.height,self.width,3))
        for h in range(self.height):
            for w in range(self.width):
                temp_vtx=self.vtx[h][w][0]
                self.reshaped_pc[h][w]=[temp_vtx[0],temp_vtx[1],temp_vtx[2]]
        self.reshaped_pc=np.array(self.reshaped_pc,dtype=np.float32)
        print(self.reshaped_pc[self.ph][self.pw])

    def show_img(self):
        cv2.circle(self.img_color, (self.pw,self.ph), 8, [255,0,255], thickness=-1)
        while True:
            cv2.imshow('RealSence',self.img_color)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break

if __name__ == "__main__":
    try:
        myrs=Realsense()
        myrs.get_frames()
        myrs.get_pc()
        myrs.show_img()
    except rospy.ROSInterruptException as e:
        print(e)

