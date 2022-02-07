//
// Created by vuong on 16/12/2021.
//

#include "utils/Utils.h"
void Utils::toTransformMsg(Sophus::SE3f Twc, geometry_msgs::Transform* tf) {
  // TODO (Vuong): Do we need to change ORB coord to ROS Coordinate
  tf->translation.x = Twc.translation().x();
  tf->translation.y = Twc.translation().y();
  tf->translation.z = Twc.translation().z();
  tf->rotation.w = Twc.unit_quaternion().w();
  tf->rotation.x = Twc.unit_quaternion().x();
  tf->rotation.y = Twc.unit_quaternion().y();
  tf->rotation.z = Twc.unit_quaternion().z();
}
ros::Time Utils::toROSTime(double timestamp) {
  ros::Time output;
  output.fromSec(timestamp);
  return output;
}
