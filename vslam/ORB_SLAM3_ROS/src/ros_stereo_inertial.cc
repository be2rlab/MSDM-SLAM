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
#include <sensor_msgs/Imu.h>

// FROM System
#include <chrono>
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <queue>
#include <thread>
#include <vector>

// FROM ORB_SLAM3
#include <ImuTypes.h>
#include <System.h>
#include <node.h>

using namespace std;

class ImuGrabber {
 public:
  ImuGrabber() = default;
  void GrabImu(const sensor_msgs::ImuConstPtr& imu_msg);
  queue<sensor_msgs::ImuConstPtr> imuBuf;
  std::mutex mBufMutex;
};

class VIO : public node {
 public:
  VIO(ORB_SLAM3::System::eSensor sensor,
      ros::NodeHandle& node_handle,
      image_transport::ImageTransport& image_transport,
      std::string strOutput = std::string());
  ~VIO();
  void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
  void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
  static cv::Mat GetImage(const sensor_msgs::ImageConstPtr& img_msg);
  void SyncWithImu();
  cv::Mat M1l, M2l, M1r, M2r;
  ImuGrabber* mpImuGb;

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
  std::string strVocFile;
  std::string strSettingsFile;
  std::string strOutput;
  bool bEqual = false;
  bool bRect = true;
  bool bResize = false;

  // Retrieve parameters
  nh.getParam("Params", strSettingsFile);
  nh.getParam("Do_Rectify", bRect);
  nh.getParam("Do_Equalize", bEqual);
  nh.getParam("Do_Resize", bResize);
  nh.getParam("Output_name", strOutput);

  auto* imugb = new ImuGrabber();

  VIO mVIO(
      ORB_SLAM3::System::eSensor::IMU_STEREO, nh, image_transport, strOutput);

  mVIO.mpImuGb = imugb;

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

  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu =
      nh.subscribe("/imu", 1000, &ImuGrabber::GrabImu, imugb);
  ros::Subscriber sub_img_left =
      nh.subscribe("/camera/left/image_raw", 100, &VIO::GrabImageLeft, &mVIO);
  ros::Subscriber sub_img_right =
      nh.subscribe("/camera/right/image_raw", 100, &VIO::GrabImageRight, &mVIO);

  std::thread sync_thread(&VIO::SyncWithImu, &mVIO);

  ros::spin();

  mVIO.SavingTrajectory();

  return 0;
}

void VIO::GrabImageLeft(const sensor_msgs::ImageConstPtr& img_msg) {
  mBufMutexLeft.lock();
  if (!imgLeftBuf.empty()) imgLeftBuf.pop();
  imgLeftBuf.push(img_msg);
  mBufMutexLeft.unlock();
}

void VIO::GrabImageRight(const sensor_msgs::ImageConstPtr& img_msg) {
  mBufMutexRight.lock();
  if (!imgRightBuf.empty()) imgRightBuf.pop();
  imgRightBuf.push(img_msg);
  mBufMutexRight.unlock();
}

cv::Mat VIO::GetImage(const sensor_msgs::ImageConstPtr& img_msg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  if (cv_ptr->image.type() == 0) {
    return cv_ptr->image.clone();
  } else {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void VIO::SyncWithImu() {
  const double maxTimeDiff = 0.01;
  while (ros::ok()) {
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    if (!imgLeftBuf.empty() && !imgRightBuf.empty() &&
        !mpImuGb->imuBuf.empty()) {
      tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      tImRight = imgRightBuf.front()->header.stamp.toSec();

      this->mBufMutexRight.lock();
      while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1) {
        imgRightBuf.pop();
        tImRight = imgRightBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRight.unlock();

      this->mBufMutexLeft.lock();
      while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1) {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexLeft.unlock();

      if ((tImLeft - tImRight) > maxTimeDiff ||
          (tImRight - tImLeft) > maxTimeDiff) {
        // std::cout << "big time difference" << std::endl;
        continue;
      }
      if (tImLeft > mpImuGb->imuBuf.back()->header.stamp.toSec()) continue;

      this->mBufMutexLeft.lock();
      imLeft = GetImage(imgLeftBuf.front());
      imgLeftBuf.pop();
      this->mBufMutexLeft.unlock();

      this->mBufMutexRight.lock();
      imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
      this->mBufMutexRight.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if (!mpImuGb->imuBuf.empty()) {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while (!mpImuGb->imuBuf.empty() &&
               mpImuGb->imuBuf.front()->header.stamp.toSec() <= tImLeft) {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x,
                          mpImuGb->imuBuf.front()->linear_acceleration.y,
                          mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x,
                          mpImuGb->imuBuf.front()->angular_velocity.y,
                          mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.emplace_back(acc, gyr, t);
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if (mbClahe) {
        mClahe->apply(imLeft, imLeft);
        mClahe->apply(imRight, imRight);
      }
      if (mbResize) {
        cv::resize(imLeft, imLeft, cv::Size(mnWidth, mnHeight));
        cv::resize(imRight, imRight, cv::Size(mnWidth, mnHeight));
      }
      if (mbRectify) {
        cv::remap(imLeft, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRight, M1r, M2r, cv::INTER_LINEAR);
      }

      mpORB_SLAM3->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
      Update();
      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
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
  delete mpORB_SLAM3;
  //  delete mpTracking;
  delete mpLocalMapping;
  //  delete mpLoopClosing;
  delete mpMapDrawer;
  delete mpAtlas;
}
void VIO::SavingTrajectory() {
  // Stop all threads
  mpORB_SLAM3->Shutdown();
  ROS_INFO("Saving trajectory...");
  // Save trajectory
  if (mSensor == ORB_SLAM3::System::MONOCULAR ||
      mSensor == ORB_SLAM3::System::IMU_MONOCULAR) {
    mpORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  } else {
    if (strOutputFile.empty()) {
      mpORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
      mpORB_SLAM3->SaveTrajectoryTUM("FrameTrajectory.txt");
    } else {
      mpORB_SLAM3->SaveKeyFrameTrajectoryTUM("kf_" + strOutputFile + ".txt");
      mpORB_SLAM3->SaveTrajectoryTUM("f_" + strOutputFile + ".txt");
    }
  }
  ROS_INFO("Saved trajectory!");
}
void VIO::setmbResize(bool bResize) { VIO::mbResize = bResize; }
void VIO::setmnWidth(int nWidth) { VIO::mnWidth = nWidth; }
void VIO::setmnHeight(int nHeight) { VIO::mnHeight = nHeight; }

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr& imu_msg) {
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
}
