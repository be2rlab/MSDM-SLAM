/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

// FROM ROS
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

// FROM System
#include <chrono>
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <queue>
#include <vector>

// FROM ORB_SLAM3
#include <System.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <node.h>

using namespace std;

class VIO : public node {
 public:
  VIO(ORB_SLAM3::System::eSensor sensor,
      ros::NodeHandle& node_handle,
      image_transport::ImageTransport& image_transport,
      std::string strOutput = std::string());
  ~VIO();
  void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                  const sensor_msgs::ImageConstPtr& msgRight);

  cv::Mat M1l, M2l, M1r, M2r;

 public:
  void setmbClahe(bool mbClahe);
  void setmbRectify(bool mbRectify);
  void setmbResize(bool mbResize);
  void setmnWidth(int mnWidth);
  void setmnHeight(int mnHeight);
  void SavingTrajectory();

 private:
  queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
  std::mutex mBufMutexLeft, mBufMutexRight;

 private:
  // Variables for preprocessing images before passing to ORB_SLAM3
  bool mbResize = false;
  bool mbClahe = false;
  bool mbRectify = true;
  int mnWidth, mnHeight;

 private:
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ORB_SLAM3");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport image_transport(nh);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

  // Some static parameters
  std::string strDataType;
  std::string strDataPath;
  std::string strVocFile;
  std::string strSettingsFile;
  std::string strOutput;
  bool bEqual = false;
  bool bRect = true;
  bool bResize = false;

  // Retrieve parameters
  nh.getParam("data_type", strDataType);
  nh.getParam("data_path", strDataPath);
  nh.getParam("Params", strSettingsFile);
  nh.getParam("Do_Rectify", bRect);
  nh.getParam("Do_Equalize", bEqual);
  nh.getParam("Do_Resize", bResize);
  nh.getParam("Output_name", strOutput);

  VIO mVIO(ORB_SLAM3::System::eSensor::STEREO, nh, image_transport, strOutput);

  // Prepare for Rectify
  if (bRect) {
    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
      ROS_FATAL("ERROR: Wrong path to settings");
      ros::shutdown();
      return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() ||
        R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
        rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
      ROS_FATAL("ERROR: Calibration parameters to rectify stereo are missing!");
      ros::shutdown();
      return -1;
    }

    cv::initUndistortRectifyMap(K_l,
                                D_l,
                                R_l,
                                P_l.rowRange(0, 3).colRange(0, 3),
                                cv::Size(cols_l, rows_l),
                                CV_32F,
                                mVIO.M1l,
                                mVIO.M2l);
    cv::initUndistortRectifyMap(K_r,
                                D_r,
                                R_r,
                                P_r.rowRange(0, 3).colRange(0, 3),
                                cv::Size(cols_r, rows_r),
                                CV_32F,
                                mVIO.M1r,
                                mVIO.M2r);
  }

  // Prepare for resize
  if (bResize) {
    ROS_WARN(
        "Resize input image is enable, Make sure that using correct camera "
        "parameters");
    int new_width, new_height;
    if (nh.hasParam("New_width") && nh.hasParam("New_height")) {
      nh.getParam("New_width", new_width);
      nh.getParam("New_height", new_height);
      mVIO.setmnWidth(new_width);
      mVIO.setmnHeight(new_height);
      mVIO.setmbResize(bResize);
    } else {
      ROS_FATAL("Resize image is enable but missing new W and H");
      return EXIT_FAILURE;
    }
  }

  mVIO.setmbRectify(bRect);
  mVIO.setmbClahe(bEqual);

  mVIO.Init();
  message_filters::Subscriber<sensor_msgs::Image> left_sub(
      nh, "/camera/left/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> right_sub(
      nh, "/camera/right/image_raw", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      sync_pol;
  message_filters::Synchronizer<sync_pol> sync(
      sync_pol(10), left_sub, right_sub);
  sync.registerCallback(boost::bind(&VIO::GrabStereo, &mVIO, _1, _2));

  ros::spin();

  mVIO.SavingTrajectory();
  nh.shutdown();
  return 0;
}

void VIO::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                     const sensor_msgs::ImageConstPtr& msgRight) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try {
    cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try {
    cv_ptrRight = cv_bridge::toCvShare(msgRight);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (mbRectify) {
    cv::Mat imLeft, imRight;
    cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
    mORB_SLAM3->TrackStereo(imLeft, imRight, cv_ptrLeft->header.stamp.toSec());
  } else {
    mORB_SLAM3->TrackStereo(cv_ptrLeft->image,
                             cv_ptrRight->image,
                             cv_ptrLeft->header.stamp.toSec());
  }
  auto t1 = std::chrono::steady_clock::now();
  Update();
  auto t2 = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
  ROS_WARN("Send Messages took %ld microseconds", elapsed.count());
}
VIO::VIO(ORB_SLAM3::System::eSensor sensor,
         ros::NodeHandle& node_handle,
         image_transport::ImageTransport& image_transport,
         std::string strOutput)
    : node(sensor, node_handle, image_transport, strOutput) {}

void VIO::setmbClahe(bool bClahe) { VIO::mbClahe = bClahe; }
void VIO::setmbRectify(bool bRectify) { VIO::mbRectify = bRectify; }
VIO::~VIO() {
  // Release memory
  delete mORB_SLAM3;
  delete mpTracking;
  delete mpLocalMapping;
  //  delete mpLoopClosing;
  delete mpMapDrawer;
  delete mpAtlas;
}
void VIO::SavingTrajectory() {
  // Stop all threads
  mORB_SLAM3->Shutdown();
  ROS_INFO("Saving trajectory...");
  // Save trajectory
  if (mSensor == ORB_SLAM3::System::MONOCULAR ||
      mSensor == ORB_SLAM3::System::IMU_MONOCULAR) {
    mORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  } else {
    if (strOutputFile.empty()) {
      mORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
      mORB_SLAM3->SaveTrajectoryTUM("FrameTrajectory.txt");
    } else {
      //      SaveTrajectoryBag();
      mORB_SLAM3->SaveKeyFrameTrajectoryTUM("kf_" + strOutputFile + ".txt");
      mORB_SLAM3->SaveTrajectoryTUM("f_" + strOutputFile + ".txt");
    }
  }
  ROS_INFO("Saved trajectory!");
}
void VIO::setmbResize(bool bResize) { VIO::mbResize = bResize; }
void VIO::setmnWidth(int nWidth) { VIO::mnWidth = nWidth; }
void VIO::setmnHeight(int nHeight) { VIO::mnHeight = nHeight; }
