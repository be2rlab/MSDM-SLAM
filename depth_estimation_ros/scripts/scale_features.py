#!/usr/bin/env python3
import sys
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud, CameraInfo, Image
import cv2
#import re
import numpy as np
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import torch
import message_filters
class ToScale(object):
    def __init__(self, topic_ori_depth, topic_point_cloud,topic_odometry ,topic_scaled_depth):
        #self.fx_real = 645.375183
        #self.cx_real = 631.121765
        #self.fy_real = 645.375183
        #self.cy_real = 361.513153
        self.fx_zed = 531.682373
        self.cx_zed = 639.885132
        self.fy_zed = 531.682373
        self.cy_zed = 356.164581
        #self.fx_zed = 262.316223145
        #self.cx_zed = 318.740905762
        #self.fy_zed = 262.316223145
        #self.cy_zed = 173.938339233
        self.w = 1280
        self.h = 720
        self.cv_bridge = CvBridge()
        self.image_depth = None
        self.image_sparse = np.zeros((self.h, self.w)).astype(np.float32)
        self.image_depth_scaled = None
        self.points = []
        self.pose = ()
        self.q = ()
        self.frame= None
        self.stamp = None
        self.sparses = []
        self.camera_info_msg = None
        self.info = True
        self.idx = 0
    
    
        self.camera_info_in = rospy.get_param('~camera_info_in')
        self.camera_info_out = rospy.get_param('~camera_info_out')
        self.input_image_topic = rospy.get_param('~input_topic')
        self.output_image_topic = rospy.get_param('~image_output_topic')
        # self.camera_info_in = "/zed2/zed_node/left/camera_info"
        # self.camera_info_out = "/camera_out"
        # self.input_image_topic = "/vslam_stereo_inertial/KF_DebugImage"
        # self.output_image_topic = "/image_out"
        

        rospy.loginfo("processing: " + topic_ori_depth + " with point cloud: " + topic_point_cloud + "and odometry:" + topic_odometry +
                      " to make output topic: " + topic_scaled_depth)

        self.pub = rospy.Publisher(topic_scaled_depth, Image, queue_size=5)
        self.pub2 = rospy.Publisher(self.camera_info_out, CameraInfo, queue_size=10)
        self.pub3 = rospy.Publisher(self.output_image_topic, Image, queue_size=5)

        self.sub_depth = message_filters.Subscriber(topic_ori_depth, Image, queue_size=5)
        self.sub_input_image = message_filters.Subscriber(self.input_image_topic, Image, queue_size=5)

        self.sub_pc = message_filters.Subscriber(topic_point_cloud, PointCloud)

        self.sub_odom = message_filters.Subscriber(topic_odometry, Odometry)
        self.sub_info = message_filters.Subscriber(self.camera_info_in, CameraInfo)

        #rospy.loginfo("starting to handel")
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_depth, self.sub_pc,self.sub_odom,self.sub_info,self.sub_input_image], 10, 0.15, allow_headerless=True)
        self.ts.registerCallback(self.handel)
        #rospy.loginfo("handeled")

    def publish(self,scaled_depth,frame_id,stamp_now):
        #header = Header(stamp=msg.header.stamp)
        img_msg = self.cv_bridge.cv2_to_imgmsg(scaled_depth, encoding="16UC1")
        rospy.loginfo("published scaled depth image")
        img_msg.header.frame_id = frame_id
        img_msg.header.stamp = stamp_now
        self.pub.publish(img_msg)
        self.image_input = cv2.cvtColor(self.image_input, cv2.COLOR_GRAY2RGB)
        img_msg2 = self.cv_bridge.cv2_to_imgmsg(self.image_input,encoding="rgb8")
        img_msg2.header.frame_id = frame_id
        img_msg2.header.stamp = stamp_now
        self.pub3.publish(img_msg2)

        header = Header(stamp=stamp_now,frame_id=frame_id)
        self.camera_info_msg.header = header
        self.pub2.publish(self.camera_info_msg)


    def scale(self,sparse,depth):
        rospy.loginfo("Starting scaling process")

        target = np.array(sparse, dtype=np.float16)

        target_numpy = target.copy()

        prediction = np.asarray(depth, dtype=np.float32)

        mask = (target > 0) & (target < 20000)
        #target_numpy_masked = target_numpy * mask
        #prediction_masked = prediction * mask
        #a, b = np.polyfit(prediction_masked, target_numpy_masked, deg=1)

        mask = torch.from_numpy(mask.astype(np.float32))
        target = torch.from_numpy(target_numpy.copy())
        prediction = torch.from_numpy(prediction.copy())

        target_disparity = torch.zeros_like(target)
        target_disparity[mask == 1] = 1.0 / target[mask == 1]

        a_00 = torch.sum(mask * prediction * prediction, (-2, 1))
        a_01 = torch.sum(mask * prediction, (-2, 1))
        a_11 = torch.sum(mask, (-2, 1))

        # right hand side: b = [b_0, b_1]
        b_0 = torch.sum(mask * prediction * target_disparity, (-2, 1))
        b_1 = torch.sum(mask * target_disparity, (-2, 1))

        # solution: x = A^-1 . b = [[a_11, -a_01], [-a_10, a_00]] / (a_00 * a_11 - a_01 * a_10) . b
        scale = torch.zeros_like(b_0)
        shift = torch.zeros_like(b_1)

        det = a_00 * a_11 - a_01 * a_01
        # A needs to be a positive definite matrix.
        valid = det > 0

        scale[valid] = (a_11[valid] * b_0[valid] - a_01[valid] * b_1[valid]) / det[valid]
        shift[valid] = (-a_01[valid] * b_0[valid] + a_00[valid] * b_1[valid]) / det[valid]

        prediction_aligned = scale.view(-1, 1, 1) * prediction + shift.view(-1, 1, 1)
        prediction_aligned[prediction_aligned < 0] = 0

        #prediction_aligned = a * pred + b
        prediction_aligned[prediction_aligned < 0] = 0
        prediciton_depth = 1 / prediction_aligned
        prediciton_depth_mat = np.asarray(prediciton_depth, dtype=np.float32).reshape((self.h, self.w))
        prediciton_depth_mat = np.asarray(prediciton_depth_mat, dtype=np.uint16)
        self.image_depth_scaled = prediciton_depth_mat
        #rospy.loginfo("Finishing scaling process")
        self.image_sparse = np.zeros((self.h, self.w)).astype(np.float32)
        self.publish(self.image_depth_scaled,self.frame,self.stamp)


    def handel(self, msg,msg2,msg3,msg4,msg5):
        #rospy.loginfo("Reading depth sparse,pc and odom")
        if self.info is True:
            camera_info = CameraInfo()
            camera_info = msg4
            self.camera_info_msg = camera_info
            rospy.loginfo("GOT CAMERA INFO")
            print(self.camera_info_msg)
            self.info = False
        x = msg3.pose.pose.position.x
        y = msg3.pose.pose.position.y
        z = msg3.pose.pose.position.z
        self.pose = (x,y,z)
        q1 = msg3.pose.pose.orientation.x
        q2 = msg3.pose.pose.orientation.y
        q3 = msg3.pose.pose.orientation.z
        q4 = msg3.pose.pose.orientation.w
        self.q = (q1, q2, q3, q4)
        self.points = [[p.x, p.y, p.z, 1] for p in msg2.points]
        self.image_depth = self.cv_bridge.imgmsg_to_cv2(msg,"passthrough")
        self.image_input = self.cv_bridge.imgmsg_to_cv2(msg5, "passthrough")
        self.frame= msg.header.frame_id
        self.stamp= msg.header.stamp
        array_P = np.zeros((3, 1)).astype(np.float32)
        array_W = np.zeros((3, 1)).astype(np.float32)
        for p in self.points:
            xw=p[0]
            yw=p[1]
            zw=p[2]
            if xw == 0.0 and yw==0.0 and zw==0.0:
                continue

            r = R.from_quat(list(self.q))
            array_R=r.as_dcm()

            array_P[:]=np.transpose(np.array([list(self.pose)]))

            array_W[:] = np.array([[xw], [yw], [zw]])
            dot = np.dot(array_R, array_W)
            array = dot + (array_P)
            X = array.item(0)
            Y = array.item(1)
            Z = array.item(2)
            u=int(((self.fx_zed*X)/Z)+self.cx_zed)
            v=int(((self.fy_zed*Y)/Z)+self.cy_zed)

            self.sparses.append([Z,u,v])

        for sparse in self.sparses:
            if sparse[0]:
                if sparse[1] > 0 and sparse[1] <= self.w and sparse[2] > 0 and sparse[2] <= self.h:
                    self.image_sparse[sparse[2] - 1, sparse[1] - 1] = sparse[0] * 1000

        #cv2.imwrite("/home/malik/PycharmProjects/pointcloud/Pointcloud/orb/depth_orb_ros/" + "%05i.png" % self.idx,     self.image_sparse.astype(np.uint16))
        rospy.loginfo("saving with idx : " + str(self.idx))
        self.idx = self.idx + 1
        self.points = []
        self.pose = ()
        self.q = ()
        self.sparses = []
        self.scale(self.image_sparse,self.image_depth)


if __name__ == '__main__':
    #argv = rospy.myargv(sys.argv)
    rospy.init_node("Scale_Depth", anonymous=True,log_level=rospy.DEBUG)
    midas = rospy.get_param('~midas_topic')
    map = rospy.get_param('~map_topic')
    pose = rospy.get_param('~pose_topic')
    scaled = rospy.get_param('~output_topic')
    # midas = "/midas_topic"
    # map = "/vslam_stereo_inertial/KF_MapPoints"
    # pose = "/vslam_stereo_inertial/KF_Pose"
    # scaled = "/zed2/zed_node/depth/midas"

    sc = ToScale(midas,map, pose ,scaled)
    rospy.spin()