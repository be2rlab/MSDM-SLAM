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
      strOutputFile(std::move(strOutput)) {
  mNodeName = ros::this_node::getName();
}

node::~node() {}
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

  // Init ORB_SLAM3
  mpORB_SLAM3 =
      new ORB_SLAM3::System(strVocFile, strSettingsFile, mSensor, mbViewer);

  // Retrieve Camera Info
  cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
  GetCamInfo(fSettings);

  // Bring the constructors here to get the state robot easier
  // TODO: Optimize this
  //  mpORB_SLAM3->SetThreads(mpLocalMapping, mpMapDrawer, mpAtlas);
  mpLocalMapping = mpORB_SLAM3->mpLocalMapper;
  mpMapDrawer = mpORB_SLAM3->mpMapDrawer;
  mpAtlas = mpORB_SLAM3->mpAtlas;

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
  if (!mpAtlas->GetCurrentMap()->isImuInitialized()) return;

  cv::Mat cvTcw;

  mpMapDrawer->GetCurrentCameraPose(cvTcw);
  mTimestamp = mpMapDrawer->GetCurrentCamTimestamp();
  if (!cvTcw.empty()) {
    cv::cv2eigen(cvTcw, eTcw);
    eTcw.block<3, 3>(0, 0) = Eigen::Quaterniond(eTcw.block<3, 3>(0, 0))
                                 .normalized()
                                 .toRotationMatrix();
    Sophus::SE3d spTcw = Sophus::SE3d(eTcw);
    spTwc = spTcw.inverse();
    // Publish current pose (Transformation from camera to target)
    PublishPoseAsTransform(spTwc, mTimestamp);
    PublishPoseAsOdometry(spTwc, mTimestamp);
    std::vector<ORB_SLAM3::MapPoint*> vpMapPoints =
        mpAtlas->GetCurrentMap()->GetAllMapPoints();
    PublishMapPointsAsPCL2(vpMapPoints, mTimestamp);
  }
  if (!mpLocalMapping->mlProcessdKFs.empty()) {
    // TODO: Adding mutex
    ORB_SLAM3::KeyFrame* pKF = mpLocalMapping->mlProcessdKFs.front();
    mpLocalMapping->mlProcessdKFs.pop_front();
    PublishKF(pKF);
  }
}
void node::PublishPoseAsTransform(const Sophus::SE3d& Twc, double timestamp) {
  // Generate Msg
  geometry_msgs::TransformStamped tfMsg;
  tfMsg.header.stamp = Utils::toROSTime(timestamp);
  tfMsg.header.frame_id = world_frame_id_;
  tfMsg.child_frame_id = left_cam_frame_id_;
  Utils::toTransformMsg(Twc, &tfMsg.transform);
  // Broadcast tf
  tf_broadcaster.sendTransform(tfMsg);
}
void node::PublishMapPointsAsPCL2(std::vector<ORB_SLAM3::MapPoint*> vpMapPoints,
                                  double timestamp) {
  if (vpMapPoints.empty()) {
    ROS_WARN("Empty Map Points");
    return;
  }
  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3;  // x y z

  cloud.header.stamp = Utils::toROSTime(timestamp);
  cloud.header.frame_id = world_frame_id_;
  cloud.height = 1;
  cloud.width = vpMapPoints.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = {"x", "y", "z"};
  for (int i = 0; i < num_channels; i++) {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }
  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char* cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  int min_observations_per_point_ = 2;
  for (unsigned int i = 0; i < cloud.width; i++) {
    // TODO: check this: coord diff between
    if (vpMapPoints.at(i)->nObs >= min_observations_per_point_) {
      data_array[0] = vpMapPoints.at(i)->GetWorldPos().at<float>(0);  // x.
      data_array[1] = vpMapPoints.at(i)->GetWorldPos().at<float>(1);  // y.
      data_array[2] = vpMapPoints.at(i)->GetWorldPos().at<float>(2);  // z.
      memcpy(cloud_data_ptr + (i * cloud.point_step),
             data_array,
             num_channels * sizeof(float));
    }
  }
  mMapPointsPub.publish(cloud);
}
void node::PublishKF(ORB_SLAM3::KeyFrame* pKF) {
  Eigen::Matrix4d eTwc;
  cv::cv2eigen(pKF->GetPose(), eTwc);  // POSE Tcw
  eTwc.block<3, 3>(0, 0) = Eigen::Quaterniond(eTwc.block<3, 3>(0, 0))
                               .normalized()
                               .toRotationMatrix();
  Sophus::SE3d Twc(eTwc);

  // Get MapPoints
  std::vector<ORB_SLAM3::MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

  // Get timestamps
  double timestamp = pKF->mTimeStamp;

  // Get Image
  cv::Mat imKF = pKF->imgLeft.clone();
  // Start Publish
  // Pose (Twc)
  nav_msgs::Odometry PoseMsg;
  PoseMsg.header.stamp = Utils::toROSTime(timestamp);
  PoseMsg.header.frame_id = "zed2_left_camera_optical_frame";

  PoseMsg.pose.pose.orientation.x = Twc.unit_quaternion().x();
  PoseMsg.pose.pose.orientation.y = Twc.unit_quaternion().y();
  PoseMsg.pose.pose.orientation.z = Twc.unit_quaternion().z();
  PoseMsg.pose.pose.orientation.w = Twc.unit_quaternion().w();

  PoseMsg.pose.pose.position.x = Twc.translation().x();
  PoseMsg.pose.pose.position.y = Twc.translation().y();
  PoseMsg.pose.pose.position.z = Twc.translation().z();

  mKFPosePub.publish(PoseMsg);

  // Map Points
  sensor_msgs::PointCloud cloud;
  cloud.header.stamp = Utils::toROSTime(timestamp);
  cloud.header.frame_id = "map";
  cloud.points.resize(vpMapPoints.size());
  // we'll also add an intensity channel to the cloud
  cloud.channels.resize(1);
  cloud.channels[0].name = "intensities";
  cloud.channels[0].values.resize(vpMapPoints.size());

  // Corresponding Features
  sensor_msgs::PointCloud cloud_feature;
  cloud_feature.header.stamp = Utils::toROSTime(timestamp);
  cloud_feature.header.frame_id = std::to_string(pKF->mnId);
  cloud_feature.points.resize(vpMapPoints.size());
  // we'll also add an intensity channel to the cloud
  cloud_feature.channels.resize(1);
  cloud_feature.channels[0].name = "intensities";
  cloud_feature.channels[0].values.resize(vpMapPoints.size());

  for (size_t i = 0; i < vpMapPoints.size(); i++) {
    ORB_SLAM3::MapPoint* pMP = vpMapPoints[i];
    if (pMP && !pMP->isBad()) {
      cloud.points[i].x = pMP->GetWorldPos().at<float>(0);
      cloud.points[i].y = pMP->GetWorldPos().at<float>(1);
      cloud.points[i].z = pMP->GetWorldPos().at<float>(2);

      cloud_feature.points[i].x = pKF->mvKeys[i].pt.x;
      cloud_feature.points[i].y = pKF->mvKeys[i].pt.y;
      cloud_feature.points[i].z = 0;
    }
  }
  // Camera info
  sensor_msgs::CameraInfo leftInfo;
  leftInfo.header.frame_id = "left_camera";
  leftInfo.header.stamp = Utils::toROSTime(timestamp);
  ParseCamInfo(leftInfo);
  // Image
  sensor_msgs::ImagePtr img_msg;
  sensor_msgs::Image std_img_msg;
  std_img_msg.header.stamp.fromSec(timestamp);
  std_img_msg.header.frame_id = "zed2_left_camera_optical_frame";
  img_msg = cv_bridge::CvImage(std_img_msg.header, "mono8", imKF).toImageMsg();

  // Publish
  mMPsObsbyKFPub.publish(cloud);
  mKFsFeaturesPub.publish(cloud_feature);
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
void node::ParseCamInfo(sensor_msgs::CameraInfo& msg) {

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
  cv::FileNode node = fSettings["Camera.fx"];
  if (!node.empty() && node.isReal()) {
    fx = node.real();
  } else {
    std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera.fy"];
  if (!node.empty() && node.isReal()) {
    fy = node.real();
  } else {
    std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera.cx"];
  if (!node.empty() && node.isReal()) {
    cx = node.real();
  } else {
    std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera.cy"];
  if (!node.empty() && node.isReal()) {
    cy = node.real();
  } else {
    std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  // Distortion parameters
  node = fSettings["Camera.k1"];
  if (!node.empty() && node.isReal()) {
    k1 = node.real();
  } else {
    std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera.k2"];
  if (!node.empty() && node.isReal()) {
    k2 = node.real();
  } else {
    std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera.p1"];
  if (!node.empty() && node.isReal()) {
    t1 = node.real();
  } else {
    std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera.p2"];
  if (!node.empty() && node.isReal()) {
    t2 = node.real();
  } else {
    std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }
  camWidth = (int)fSettings["Camera.width"].real();
  camHeight = (int)fSettings["Camera.height"].real();
  ROS_WARN_COND(b_miss_params, "MISSING CAMERA PARAMS");
}
