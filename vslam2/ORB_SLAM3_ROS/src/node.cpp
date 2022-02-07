//
// Created by vuong on 16/12/2021.
//

#include "node.h"

#include <utility>
node::node(ORB_SLAM3::System::eSensor sensor,
           ros::NodeHandle& node_handle,
           image_transport::ImageTransport& image_transport,
           std::string strOutput)
    : mSensor(sensor),
      nh_(node_handle),
      image_transport_(image_transport),
      strOutputFile(strOutput) {
  mNodeName = ros::this_node::getName();
  mbIMU = (sensor == ORB_SLAM3::System::IMU_MONOCULAR ||
           sensor == ORB_SLAM3::System::IMU_STEREO);
}

node::~node() = default;
void node::Init() {
  // Retrieve static parameters
  nh_.getParam("Vocab_path", strVocFile);
  nh_.getParam("Params", strSettingsFile);
  nh_.getParam("Visualize", mbViewer);
  // Retrieve frame id parameters
  nh_.getParam("world_frame_id", world_frame_id_);  // world_frame
  nh_.getParam("left_camera_frame_id",
               left_cam_frame_id_);  // left_camera_frame
  nh_.getParam("point_cloud_frame_id", point_cloud_frame_id_);  // point cloud

  std::cout << "String config: " << strSettingsFile << std::endl;
  // Init ORB_SLAM3
    mORB_SLAM3 =
        new ORB_SLAM3::System(strVocFile, strSettingsFile, mSensor, mbViewer);
//  mORB_SLAM3(strVocFile,strSettingsFile,mSensor,mbViewer);

  // Retrieve Camera Info
  cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
  GetCamInfo(fSettings);

  // Bring the constructors here to get the state robot easier
  // TODO: Optimize this
  //  mpORB_SLAM3->SetThreads(mpLocalMapping, mpMapDrawer, mpAtlas);
  mpLocalMapping = mORB_SLAM3->mpLocalMapper;
  mpMapDrawer = mORB_SLAM3->mpMapDrawer;
  mpAtlas = mORB_SLAM3->mpAtlas;
  //  mpTracking = mpORB_SLAM3->mpTracker;

  T_ROS_ORB << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;
  T_ROS_ORB.block<3, 3>(0, 0) = Eigen::Quaternionf(T_ROS_ORB.block<3, 3>(0, 0))
                                    .normalized()
                                    .toRotationMatrix();

  spT_ROS_ORB = Sophus::SE3f(T_ROS_ORB);
  std::cout << "Transformation from ORB to ROS: " << std::endl
            << spT_ROS_ORB.matrix() << std::endl;
  spT_ORB_ROS = spT_ROS_ORB.inverse();
  std::cout << "Transformation from ROS to ORB: " << std::endl
            << spT_ORB_ROS.matrix() << std::endl;
  //  bag.open(
  //      "/media/vuong/Samsung_Vuong/Datasets/rector_hall_dataset/dataset1/"
  //      "post.bag",
  //      rosbag::bagmode::Write);
  // Advertise topics
  // Map Point
  mMapPointsPub =
      nh_.advertise<sensor_msgs::PointCloud2>(mNodeName + "/MapPoints", 1);
  // Pose
  mPosePub = nh_.advertise<nav_msgs::Odometry>(mNodeName + "/Pose", 1);

  // Keyframe data(s) For Depth Estimation
  mKFDebugImagePub =
      image_transport_.advertise(mNodeName + "/KF_DebugImage", 1);
  mKFPosePub = nh_.advertise<nav_msgs::Odometry>(mNodeName + "/KF_Pose", 1);
  mMPsObsbyKFPub =
      nh_.advertise<sensor_msgs::PointCloud>(mNodeName + "/KF_MapPoints", 1);
  mKFsFeaturesPub =
      nh_.advertise<sensor_msgs::PointCloud>(mNodeName + "/KF_Features", 1);
  mKFsCamInfoPub =
      nh_.advertise<sensor_msgs::CameraInfo>(mNodeName + "/KF_CamInfo", 1);
}
void node::Update() {
  // Draw current frame

  //  sensor_msgs::ImagePtr img_msg;
  //  sensor_msgs::Image std_img_msg;
  //  string topicDebugFrame = std::string("/DebugFrame");
  //
  //  cv::Mat imDebug = mpTracking->mCurrentFrame.imgLeft.clone();
  //  double timestamp = mpTracking->mCurrentFrame.mTimeStamp;
  //  auto mvCurrentKeys = mpTracking->mCurrentFrame.mvKeys;
  //  auto mvbOutliers = mpTracking->mCurrentFrame.mvbOutlier;
  //  auto mvpMapPoints = mpTracking->mCurrentFrame.mvpMapPoints;
  //
  //  std_img_msg.header.stamp.fromSec(timestamp);
  //  std_img_msg.header.frame_id = left_cam_frame_id_;
  //
  //  cv::cvtColor(imDebug, imDebug, cv::COLOR_GRAY2BGR);
  //
  //  for (int i = 0; i < mvCurrentKeys.size(); i++) {
  //    ORB_SLAM3::MapPoint* pMP = mvpMapPoints[i];
  //    cv::Point2f pt1, pt2;
  //    cv::Point2f point;
  //    if (pMP) {
  //      if (!mvbOutliers[i]) {
  //        point = mvCurrentKeys[i].pt;
  //        pt1.x = mvCurrentKeys[i].pt.x - r;
  //        pt1.y = mvCurrentKeys[i].pt.y - r;
  //        pt2.x = mvCurrentKeys[i].pt.x + r;
  //        pt2.y = mvCurrentKeys[i].pt.y + r;
  //        cv::rectangle(imDebug, pt1, pt2, standardColor);
  //        cv::circle(imDebug, point, 2, standardColor, -1);
  //      }
  //    }
  //  }
  //  img_msg =
  //      cv_bridge::CvImage(std_img_msg.header, "bgr8", imDebug).toImageMsg();
  //  bag.write(topicDebugFrame, Utils::toROSTime(timestamp), img_msg);

  // Only start to public out after initialize when we used IMU
  if (mbIMU && !mpAtlas->GetCurrentMap()->isImuInitialized()) return;

  spTwc = mpMapDrawer->GetCurrentCameraPose();
  mTimestamp = mpMapDrawer->getmTimestamp();

  // we transfer to ROS coord if visual odometry (without IMU)
  if (!mbIMU) spTwc = spT_ROS_ORB * spTwc;

  // Publish current pose (Transformation from camera to target)
  PublishPoseAsTransform(spTwc, mTimestamp);
  //    PublishPoseAsOdometry(spTwc, mTimestamp);
  //    std::vector<ORB_SLAM3::MapPoint*> vpMapPoints =
  //        mpAtlas->GetCurrentMap()->GetAllMapPoints();
  //    PublishMapPointsAsPCL2(vpMapPoints, mTimestamp);
  //}
  if (!mpLocalMapping->mlProcessdKFs.empty()) {
    // TODO: Adding mutex
    ORB_SLAM3::KeyFrame* pKF = mpLocalMapping->mlProcessdKFs.front();
    mpLocalMapping->mlProcessdKFs.pop_front();
    PublishKF(pKF);
  }
}
void node::PublishPoseAsTransform(const Sophus::SE3f& Twc, double timestamp) {
  // You need to transfer to correct coord (ROS) before publish
  // Generate Msg
  geometry_msgs::TransformStamped tfMsg;
  tfMsg.header.stamp = Utils::toROSTime(timestamp);
  tfMsg.header.frame_id = world_frame_id_;  // world -> map
  // zed2_base_link -> zed2_left_camera_optical_frame
  tfMsg.child_frame_id = left_cam_frame_id_;
  Utils::toTransformMsg(Twc, &tfMsg.transform);
  // Broadcast tf
  tf_broadcaster.sendTransform(tfMsg);
}
void node::PublishMapPointsAsPCL2(std::vector<ORB_SLAM3::MapPoint*> vpMapPoints,
                                  double timestamp) {
  //  if (vpMapPoints.empty()) {
  //    ROS_WARN("Empty Map Points");
  //    return;
  //  }
  //  sensor_msgs::PointCloud2 cloud;
  //
  //  const int num_channels = 3;  // x y z
  //
  //  cloud.header.stamp = Utils::toROSTime(timestamp);
  //  cloud.header.frame_id = world_frame_id_;
  //  cloud.height = 1;
  //  cloud.width = vpMapPoints.size();
  //  cloud.is_bigendian = false;
  //  cloud.is_dense = true;
  //  cloud.point_step = num_channels * sizeof(float);
  //  cloud.row_step = cloud.point_step * cloud.width;
  //  cloud.fields.resize(num_channels);
  //
  //  std::string channel_id[] = {"x", "y", "z"};
  //  for (int i = 0; i < num_channels; i++) {
  //    cloud.fields[i].name = channel_id[i];
  //    cloud.fields[i].offset = i * sizeof(float);
  //    cloud.fields[i].count = 1;
  //    cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  //  }
  //  cloud.data.resize(cloud.row_step * cloud.height);
  //
  //  unsigned char* cloud_data_ptr = &(cloud.data[0]);
  //
  //  float data_array[num_channels];
  //  int min_observations_per_point_ = 2;
  //  for (unsigned int i = 0; i < cloud.width; i++) {
  //    // TODO: check this: coord diff between
  //    if (vpMapPoints.at(i)->nObs >= min_observations_per_point_) {
  //      data_array[0] = vpMapPoints.at(i)->GetWorldPos().at<float>(0);  // x.
  //      data_array[1] = vpMapPoints.at(i)->GetWorldPos().at<float>(1);  // y.
  //      data_array[2] = vpMapPoints.at(i)->GetWorldPos().at<float>(2);  // z.
  //      memcpy(cloud_data_ptr + (i * cloud.point_step),
  //             data_array,
  //             num_channels * sizeof(float));
  //    }
  //  }
  //  mMapPointsPub.publish(cloud);
}
void node::PublishKF(ORB_SLAM3::KeyFrame* pKF) {
  Sophus::SE3f Tcw = pKF->GetPose();
  Tcw = Tcw * spT_ORB_ROS;
  // Get MapPoints
  std::vector<ORB_SLAM3::MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

  // Get timestamps
  double timestamp = pKF->mTimeStamp;

  // Start Publish
  // Pose (Tcw)
  nav_msgs::Odometry PoseMsg;
  PoseMsg.header.stamp = Utils::toROSTime(timestamp);
  PoseMsg.header.frame_id = left_cam_frame_id_;

  PoseMsg.pose.pose.orientation.x = Tcw.unit_quaternion().x();
  PoseMsg.pose.pose.orientation.y = Tcw.unit_quaternion().y();
  PoseMsg.pose.pose.orientation.z = Tcw.unit_quaternion().z();
  PoseMsg.pose.pose.orientation.w = Tcw.unit_quaternion().w();

  PoseMsg.pose.pose.position.x = Tcw.translation().x();
  PoseMsg.pose.pose.position.y = Tcw.translation().y();
  PoseMsg.pose.pose.position.z = Tcw.translation().z();

  // Map Points
  sensor_msgs::PointCloud cloud;
  cloud.header.stamp = Utils::toROSTime(timestamp);
  cloud.header.frame_id = world_frame_id_;
  cloud.points.resize(vpMapPoints.size());
  // we'll also add an intensity channel to the cloud
  cloud.channels.resize(1);
  cloud.channels[0].name = "intensities";
  cloud.channels[0].values.resize(vpMapPoints.size());

  // tbb::parallel_for(
  //     tbb::blocked_range<size_t>(0, vpMapPoints.size()),
  //     [&](tbb::blocked_range<size_t> rvpMapPoints) {
  //       for (size_t i = rvpMapPoints.begin(); i < rvpMapPoints.end(); i++) {
  //         ORB_SLAM3::MapPoint* pMP = vpMapPoints[i];
  //         if (pMP && !pMP->isBad()) {
  //           Eigen::Vector3f pW = pMP->GetWorldPos();
  //           if (!mbIMU) pW = spT_ROS_ORB.rotationMatrix() * pW;
  //           cloud.points[i].x = pW(0);
  //           cloud.points[i].y = pW(1);
  //           cloud.points[i].z = pW(2);
  //         }
  //       }
  //     });
  // Camera info
  sensor_msgs::CameraInfo leftInfo;
  leftInfo.header.frame_id = left_cam_frame_id_;
  leftInfo.header.stamp = Utils::toROSTime(timestamp);
  ParseCamInfo(leftInfo);
  // Image
  // Get Image
  cv::Mat imKF = pKF->imgLeft.clone();
  sensor_msgs::ImagePtr img_msg;
  sensor_msgs::Image std_img_msg;
  std_img_msg.header.stamp.fromSec(timestamp);
  std_img_msg.header.frame_id = left_cam_frame_id_;
  cv::cvtColor(imKF, imKF, cv::COLOR_GRAY2BGR);

  for (int i = 0; i < vpMapPoints.size(); i++) {
    ORB_SLAM3::MapPoint* pMP = vpMapPoints[i];
    cv::Point2f pt1, pt2;
    cv::Point2f point;
    if (pMP) {
        point = pKF->mvKeys[i].pt;
        pt1.x = pKF->mvKeys[i].pt.x - r;
        pt1.y = pKF->mvKeys[i].pt.y - r;
        pt2.x = pKF->mvKeys[i].pt.x + r;
        pt2.y = pKF->mvKeys[i].pt.y + r;
        cv::rectangle(imKF, pt1, pt2, standardColor);
        cv::circle(imKF, point, 2, standardColor, -1);
    }
  }
  img_msg = cv_bridge::CvImage(std_img_msg.header, "bgr8", imKF).toImageMsg();
  // Publish
  mKFPosePub.publish(PoseMsg);
  mMPsObsbyKFPub.publish(cloud);
  mKFDebugImagePub.publish(img_msg);
  mKFsCamInfoPub.publish(leftInfo);
}
void node::PublishPoseAsOdometry(const Sophus::SE3d& Twc, double timestamp) {
  nav_msgs::Odometry PoseMsg;
  PoseMsg.header.stamp = Utils::toROSTime(timestamp);
  PoseMsg.header.frame_id = world_frame_id_;
  PoseMsg.child_frame_id = left_cam_frame_id_;

  PoseMsg.pose.pose.orientation.x = Twc.unit_quaternion().x();
  PoseMsg.pose.pose.orientation.y = Twc.unit_quaternion().y();
  PoseMsg.pose.pose.orientation.z = Twc.unit_quaternion().z();
  PoseMsg.pose.pose.orientation.w = Twc.unit_quaternion().w();

  PoseMsg.pose.pose.position.x = Twc.translation().x();
  PoseMsg.pose.pose.position.y = Twc.translation().y();
  PoseMsg.pose.pose.position.z = Twc.translation().z();

  mPosePub.publish(PoseMsg);
}
void node::ParseCamInfo(sensor_msgs::CameraInfo& msg) const {
  // Camera size
  msg.width = camWidth;
  msg.height = camHeight;
  // Distortion model
  msg.distortion_model = "plumb_bob";
  // Intrinsic matrix
  msg.K[0] = fx;
  msg.K[1] = 0.0;
  msg.K[2] = cx;
  msg.K[3] = 0.0;
  msg.K[4] = fy;
  msg.K[5] = cy;
  msg.K[6] = 0.0;
  msg.K[7] = 0.0;
  msg.K[8] = 1.0;
  // Distortion matrix
  msg.D.resize(4);
  msg.D[0] = k1;
  msg.D[1] = k2;
  msg.D[2] = t1;
  msg.D[3] = t2;
  // Rectification matrix
  msg.R[0] = 1.0;
  msg.R[1] = 0.0;
  msg.R[2] = 0.0;
  msg.R[3] = 0.0;
  msg.R[4] = 1.0;
  msg.R[5] = 0.0;
  msg.R[6] = 0.0;
  msg.R[7] = 0.0;
  msg.R[8] = 1.0;
  // Projection matrix
  msg.P[0] = fx;
  msg.P[1] = 0.0;
  msg.P[2] = cx;
  msg.P[3] = 0.0;
  msg.P[4] = 0.0;
  msg.P[5] = fy;
  msg.P[6] = cy;
  msg.P[7] = 0.0;
  msg.P[8] = 0.0;
  msg.P[9] = 0.0;
  msg.P[10] = 1.0;
  msg.P[11] = 0.0;
  // Binning
  msg.binning_x = 0;
  msg.binning_y = 0;
  // ROI
  msg.roi.x_offset = 0;
  msg.roi.y_offset = 0;
  msg.roi.height = 0;
  msg.roi.width = 0;
  msg.roi.do_rectify = false;
}
void node::GetCamInfo(cv::FileStorage& fSettings) {
  bool b_miss_params = false;
  // Camera calibration parameters
  cv::FileNode node = fSettings["Camera1.fx"];
  if (!node.empty() && node.isReal()) {
    fx = node.real();
  } else {
    std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera1.fy"];
  if (!node.empty() && node.isReal()) {
    fy = node.real();
  } else {
    std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera1.cx"];
  if (!node.empty() && node.isReal()) {
    cx = node.real();
  } else {
    std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera1.cy"];
  if (!node.empty() && node.isReal()) {
    cy = node.real();
  } else {
    std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  // Distortion parameters
  node = fSettings["Camera1.k1"];
  if (!node.empty() && node.isReal()) {
    k1 = node.real();
  } else {
    std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera1.k2"];
  if (!node.empty() && node.isReal()) {
    k2 = node.real();
  } else {
    std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera1.p1"];
  if (!node.empty() && node.isReal()) {
    t1 = node.real();
  } else {
    std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera1.p2"];
  if (!node.empty() && node.isReal()) {
    t2 = node.real();
  } else {
    std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }
  camWidth = (int)fSettings["Camera1.width"].real();
  camHeight = (int)fSettings["Camera1.height"].real();
  ROS_WARN_COND(b_miss_params, "MISSING CAMERA PARAMS");
}
void node::SaveTrajectoryBag() {
  //  std::cout << "Saving as Bag file ... " << std::endl;
  //
  //  string topicOdom;
  //  string topicImgDebug;
  //  string topicKFPoseInv, topicKFCloud, topicKFImage, topicKFInfo;
  //
  //  // Frame
  //  topicOdom = std::string("/odom");
  //  //  topicImgDebug = std::string("/DebugImage");
  //  // For KFs
  //  topicKFPoseInv = std::string(mNodeName + "/KF_Pose");
  //  topicKFCloud = std::string(mNodeName + "/KF_MapPoints");
  //  topicKFInfo = std::string(mNodeName + "/KF_CamInfo");
  //  topicKFImage = std::string(mNodeName + "/KF_DebugImage");
  //
  //  std::vector<ORB_SLAM3::KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
  //  sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);
  //
  //  // Transform all keyframes so that the first keyframe is at the origin.
  //  // After a loop closure the first keyframe might not be at the origin.
  //  cv::Mat Two = vpKFs[0]->GetPoseInverse();
  //  Eigen::Matrix4d eTwo;
  //  cv::cv2eigen(Two, eTwo);
  //  eTwo.block<3, 3>(0, 0) = Eigen::Quaterniond(eTwo.block<3, 3>(0, 0))
  //                               .normalized()
  //                               .toRotationMatrix();
  //  Sophus::SE3d spTwo = Sophus::SE3d(eTwo);
  //
  //  // Frame pose is stored relative to its reference keyframe (which is
  //  // optimized by BA and pose graph). We need to get first the keyframe pose
  //  // and then concatenate the relative transformation. Frames not localized
  //  // (tracking failure) are not saved.
  //
  //  // For each frame we have a reference keyframe (lRit), the timestamp (lT)
  //  // and a flag which is true when tracking failed (lbL).
  //
  //  list<ORB_SLAM3::KeyFrame*>::iterator lRit =
  //  mpTracking->mlpReferences.begin(); list<ORB_SLAM3::Frame*>::iterator lFit
  //  = mpTracking->mlpFrames.begin(); list<double>::iterator lT =
  //  mpTracking->mlFrameTimes.begin(); list<bool>::iterator lbL =
  //  mpTracking->mlbLost.begin();
  //
  //  for (list<cv::Mat>::iterator lit =
  //  mpTracking->mlRelativeFramePoses.begin(),
  //                               lend =
  //                               mpTracking->mlRelativeFramePoses.end();
  //       lit != lend;
  //       lit++, lRit++, lFit++, lT++, lbL++) {
  //    if (*lbL) continue;
  //
  //    ORB_SLAM3::KeyFrame* pKF = *lRit;
  //
  //    cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);
  //
  //    // If the reference keyframe was culled, traverse the spanning tree to
  //    get
  //    // a suitable keyframe.
  //    while (pKF->isBad()) {
  //      Trw = Trw * pKF->mTcp;
  //      pKF = pKF->GetParent();
  //    }
  //
  //    // Pose
  //    Trw = Trw * pKF->GetPose() * Two;
  //
  //    cv::Mat Tcw = (*lit) * Trw;
  //    Eigen::Matrix4d itTcw;
  //    cv::cv2eigen(Tcw, itTcw);
  //    itTcw.block<3, 3>(0, 0) = Eigen::Quaterniond(itTcw.block<3, 3>(0, 0))
  //                                  .normalized()
  //                                  .toRotationMatrix();
  //    Sophus::SE3d spitTcw = Sophus::SE3d(itTcw);
  //    spitTcw = spitTcw * spT_ORB_ROS;
  //    Sophus::SE3d spitTwc = spitTcw.inverse();
  //    nav_msgs::Odometry PoseMsg;
  //    PoseMsg.header.stamp = Utils::toROSTime(*lT);
  //
  //    PoseMsg.header.frame_id = world_frame_id_;
  //    PoseMsg.child_frame_id = left_cam_frame_id_;
  //
  //    PoseMsg.pose.pose.orientation.x = spitTwc.unit_quaternion().x();
  //    PoseMsg.pose.pose.orientation.y = spitTwc.unit_quaternion().y();
  //    PoseMsg.pose.pose.orientation.z = spitTwc.unit_quaternion().z();
  //    PoseMsg.pose.pose.orientation.w = spitTwc.unit_quaternion().w();
  //
  //    PoseMsg.pose.pose.position.x = spitTwc.translation().x();
  //    PoseMsg.pose.pose.position.y = spitTwc.translation().y();
  //    PoseMsg.pose.pose.position.z = spitTwc.translation().z();
  //    bag.write(topicOdom, Utils::toROSTime(*lT), PoseMsg);
  //
  //    // Pose as Tf
  //    geometry_msgs::TransformStamped tfMsg;
  //    tfMsg.header.stamp = Utils::toROSTime(*lT);
  //    tfMsg.header.frame_id = world_frame_id_;  // world -> map
  //    tfMsg.child_frame_id =
  //        left_cam_frame_id_;  // zed2_base_link ->
  //                             // zed2_left_camera_optical_frame
  //    Utils::toTransformMsg(spitTwc, &tfMsg.transform);
  //    // Broadcast tf
  //    //    tf_broadcaster.sendTransform(tfMsg);
  //    tf2_msgs::TFMessage tf_msg;
  //    tf_msg.transforms.push_back(tfMsg);
  //    bag.write("/tf", Utils::toROSTime(*lT), tf_msg);
  //    //    // Debug image
  //    //    sensor_msgs::ImagePtr img_msg;
  //    //    sensor_msgs::Image std_img_msg;
  //    //    std_img_msg.header.stamp.fromSec(*lT);
  //    //    std_img_msg.header.frame_id = left_cam_frame_id_;
  //    //    cv::Mat imDebug = pF->imgLeft.clone();
  //    //    //    pF->imgLeft.copyTo(imDebug);
  //    //    cv::cvtColor(imDebug, imDebug, cv::COLOR_GRAY2BGR);
  //    //    // Draw debug image
  //    //    int N = pF->mvKeys.size();
  //    //    //    std::cout << "Frame: " << pF->mnId << std::endl;
  //    //    for (int i = 0; i < N; i++) {
  //    //      ORB_SLAM3::MapPoint* pMP = pF->mvpMapPoints[i];
  //    //      cv::Point2f pt1, pt2;
  //    //      cv::Point2f point;
  //    //      if (pMP) {
  //    //        if (!pF->mvbOutlier[i]) {
  //    //          point = pF->mvKeys[i].pt;
  //    //          pt1.x = pF->mvKeys[i].pt.x - r;
  //    //          pt1.y = pF->mvKeys[i].pt.y - r;
  //    //          pt2.x = pF->mvKeys[i].pt.x + r;
  //    //          pt2.y = pF->mvKeys[i].pt.y + r;
  //    //          cv::rectangle(imDebug, pt1, pt2, standardColor);
  //    //          cv::circle(imDebug, point, 2, standardColor, -1);
  //    //        }
  //    //      }
  //    //    }
  //    //    img_msg =
  //    //        cv_bridge::CvImage(std_img_msg.header, "bgr8",
  //    //        imDebug).toImageMsg();
  //    //    bag.write(topicImgDebug, Utils::toROSTime(*lT), img_msg);
  //  }
  //  // KFs
  //  sensor_msgs::CameraInfo leftInfo;
  //  leftInfo.header.frame_id = "left_camera";
  //  ParseCamInfo(leftInfo);
  //  for (auto pKF : vpKFs) {
  //    if (pKF->isBad()) continue;
  //
  //    // KF Pose
  //    nav_msgs::Odometry PoseMsg;
  //    PoseMsg.header.stamp = Utils::toROSTime(pKF->mTimeStamp);
  //    PoseMsg.header.frame_id = world_frame_id_;
  //    PoseMsg.child_frame_id = left_cam_frame_id_;
  //
  //    cv::Mat Tcw = pKF->GetPose();
  //    Eigen::Matrix4d eiTcw;
  //    cv::cv2eigen(Tcw, eiTcw);
  //    eiTcw.block<3, 3>(0, 0) = Eigen::Quaterniond(eiTcw.block<3, 3>(0, 0))
  //                                  .normalized()
  //                                  .toRotationMatrix();
  //
  //    Sophus::SE3d spTcw = Sophus::SE3d(eiTcw) * spTwo;
  //
  //    spTcw = spTcw * spT_ORB_ROS;
  //
  //    PoseMsg.pose.pose.orientation.x = spTcw.unit_quaternion().x();
  //    PoseMsg.pose.pose.orientation.y = spTcw.unit_quaternion().y();
  //    PoseMsg.pose.pose.orientation.z = spTcw.unit_quaternion().z();
  //    PoseMsg.pose.pose.orientation.w = spTcw.unit_quaternion().w();
  //
  //    PoseMsg.pose.pose.position.x = spTcw.translation().x();
  //    PoseMsg.pose.pose.position.y = spTcw.translation().y();
  //    PoseMsg.pose.pose.position.z = spTcw.translation().z();
  //
  //    bag.write(topicKFPoseInv, Utils::toROSTime(pKF->mTimeStamp), PoseMsg);
  //
  //    // Map Points
  //    //    std::cout << "Map points msg" << std::endl;
  //    sensor_msgs::PointCloud cloud;
  //    cloud.header.stamp = Utils::toROSTime(pKF->mTimeStamp);
  //    cloud.header.frame_id = world_frame_id_;
  //    cloud.points.resize(pKF->mvKeys.size());
  //    // we'll also add an intensity channel to the cloud
  //    cloud.channels.resize(1);
  //    cloud.channels[0].name = "intensities";
  //    cloud.channels[0].values.resize(pKF->mvKeys.size());
  //    int count = 0;
  //    // Get MapPoints
  //    std::vector<ORB_SLAM3::MapPoint*> vpMapPoints =
  //    pKF->GetMapPointMatches(); for (int i = 0; i < pKF->mvKeys.size(); ++i)
  //    {
  //      auto pMP = vpMapPoints[i];
  //      if (pMP && !pMP->isBad()) {
  //        if (!pKF->mvbOutliers[i]) {
  //          Eigen::Vector3d pW;
  //          cv::cv2eigen(pMP->GetWorldPos(), pW);
  //          Eigen::Vector3d pWO =
  //              spTwo.rotationMatrix() * pW + spTwo.translation();
  //          pWO = spT_ROS_ORB.rotationMatrix() * pWO;
  //          cloud.points[count].x = (float)pWO[0];
  //          cloud.points[count].y = (float)pWO[1];
  //          cloud.points[count].z = (float)pWO[2];
  //          count++;
  //        }
  //      }
  //    }
  //    bag.write(topicKFCloud, Utils::toROSTime(pKF->mTimeStamp), cloud);
  //    //    std::cout << "KF img msg" << std::endl;
  //    // KF Image
  //    cv::Mat imKF = pKF->imgLeft.clone();
  //    sensor_msgs::ImagePtr img_msg;
  //    sensor_msgs::Image std_img_msg;
  //    std_img_msg.header.stamp.fromSec(pKF->mTimeStamp);
  //    std_img_msg.header.frame_id = left_cam_frame_id_;
  //    img_msg =
  //        cv_bridge::CvImage(std_img_msg.header, "mono8", imKF).toImageMsg();
  //    bag.write(topicKFImage, Utils::toROSTime(pKF->mTimeStamp), img_msg);
  //
  //    // KF optical info
  //    //    std::cout << "KF Info msg" << std::endl;
  //    leftInfo.header.stamp = Utils::toROSTime(pKF->mTimeStamp);
  //    bag.write(topicKFInfo, Utils::toROSTime(pKF->mTimeStamp), leftInfo);
  //  }
  //  bag.close();
  //  std::cout << "End saving Ros Bag" << std::endl;
}
