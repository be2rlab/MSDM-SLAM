#!/usr/bin/env python3
import sys
import rospy
import ros_numpy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud, CameraInfo, Image
import numpy as np
import torch
import message_filters
class ToScale(object):
    def __init__(self, topic_ori_depth, topic_ground_truth,topic_scaled_depth):
        self.image_depth = None
        self.zed = False
        self.image_depth_gt = None
        self.h = 720
        self.w = 1280
        self.image_sparse = np.zeros((self.h, self.w)).astype(np.float32)
        self.image_depth_scaled = None
        self.points = []
        self.pose = ()
        self.q = ()
        self.frame= None
        self.stamp = None
        self.sparses = []
        self.fx_real = 645.375183
        self.cx_real = 631.121765
        self.fy_real = 645.375183
        self.cy_real = 361.513153
        self.camera_info_msg = None

        self.camera_info_in = rospy.get_param('~camera_info_in')
        self.camera_info_out = rospy.get_param('~camera_info_out')

        rospy.loginfo("processing: " + topic_ori_depth + " with Ground Truth: " + topic_ground_truth+
                      " to make output topic: " + topic_scaled_depth)

        self.pub = rospy.Publisher(topic_scaled_depth, Image, queue_size=5)
        self.pub2 = rospy.Publisher(self.camera_info_out, CameraInfo, queue_size=10)

        self.sub_depth = message_filters.Subscriber(topic_ori_depth, Image, queue_size=5)

        self.sub_depth_gr = message_filters.Subscriber(topic_ground_truth, Image,  queue_size=5)

        self.sub_info = message_filters.Subscriber(self.camera_info_in, CameraInfo)

        rospy.loginfo("starting to handel")
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_depth, self.sub_depth_gr, self.sub_info], 10, 0.15, allow_headerless=True)
        self.ts.registerCallback(self.image_cb)
        rospy.loginfo("handeled")
        self.idx=0
        self.info = True



    def publish(self,scaled_depth,frame_id,stamp_now):
        #header.stamp = rospy.Time.now()
        #img_msg = self.cv_bridge.cv2_to_imgmsg(scaled_depth, encoding="16UC1")
        img_msg = ros_numpy.msgify(Image,scaled_depth,encoding='16UC1')
        img_msg.header.frame_id = frame_id
        img_msg.header.stamp = stamp_now
        #img_msg.header.stamp = rospy.Time.now()
        self.idx = self.idx + 1
        self.pub.publish(img_msg)
        header = Header(stamp=stamp_now,frame_id=frame_id)
        self.camera_info_msg.header = header
        self.pub2.publish(self.camera_info_msg)


    def scale(self,sparse,depth):
        #rospy.loginfo("Starting scaling process")

        target = np.array(sparse, dtype=np.float32)

        target_numpy = target.copy()

        prediction = np.asarray(depth, dtype=np.float32)

        mask = (target > 0) & (target < 20000)
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
        prediciton_depth = 1 / prediction_aligned
        prediciton_depth_mat = np.asarray(prediciton_depth, dtype=np.float32).reshape((self.h, self.w))
        prediciton_depth_mat = np.asarray(prediciton_depth_mat,dtype=np.uint16)
        self.image_depth_scaled = prediciton_depth_mat
        #rospy.loginfo("Finishing scaling process")
        self.publish(self.image_depth_scaled,self.frame,self.stamp)



    def image_cb(self, msg1,msg2,msg3):
        rospy.loginfo("Reading depth and depth ground truth")
        if self.info is True:
            camera_info = CameraInfo()
            camera_info = msg3
            self.camera_info_msg = camera_info
            rospy.loginfo("GOT CAMERA INFO")
            print(self.camera_info_msg)
            self.info = False
        #self.image_depth = self.cv_bridge.imgmsg_to_cv2(msg1)
        self.image_depth = ros_numpy.numpify(msg1)
        self.frame= msg1.header.frame_id
        self.stamp= msg1.header.stamp
        #self.image_depth_gt = self.cv_bridge.imgmsg_to_cv2(msg2)
        self.image_depth_gt = ros_numpy.numpify(msg2)

        if self.zed:
            self.image_depth_gt = self.image_depth_gt * 1000.
            self.image_depth_gt[np.isnan(self.image_depth_gt)] = 0


        self.scale(self.image_depth_gt,self.image_depth)


if __name__ == '__main__':
    #argv = rospy.myargv(sys.argv)
    rospy.init_node("Scale_Depth", anonymous=True,log_level=rospy.DEBUG)
    midas = rospy.get_param('~midas_topic')
    gt = rospy.get_param('~gt_topic')
    scaled = rospy.get_param('~output_topic')


    sc = ToScale(midas,gt,scaled)
    rospy.spin()
