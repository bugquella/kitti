/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :
* Created On   : 2018-12-07
* Copyright    :
* Descriptoin  : read rosbag and write all data according to KITTI dataset format
* References   : http://www.cvlibs.net/datasets/kitti/eval_object.php
======================================================================*/
#ifndef CATKIN_WS_TEST_SRC_ROS_KITTI_WRITER_INCLUDE_ROS_KITTI_WRITER_KITTI_WRITER_H_
#define CATKIN_WS_TEST_SRC_ROS_KITTI_WRITER_INCLUDE_ROS_KITTI_WRITER_KITTI_WRITER_H_

// C++
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
// Boost
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/thread/thread.hpp>

#include <ros_kitti_writer/messages_sync.h>

class KittiWriter {
public:
  // Default constructor
  KittiWriter(ros::NodeHandle nh, ros::NodeHandle private_nh);
  virtual ~KittiWriter();

  void process();
private:
  void createFormatFolders();


  //! Save images and corresponding timestamp
  void saveImage02(const sensor_msgs::Image::ConstPtr & image);
  void saveImage03(const sensor_msgs::Image::ConstPtr & image);

  //! Save lidar and corresponding timestamps
  void saveVelodyne(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  
  void saveOxts(const sensor_msgs::NavSatFix::ConstPtr& gps, const sensor_msgs::Imu::ConstPtr& imu, const geometry_msgs::TwistStamped::ConstPtr& vel);

  /// Image and cloud synchronizer
  sensors_fusion::StereoMessagesSync* stereoCloudSync_; //Stereo image and cloud synchronizer
  sensors_fusion::StereoMessagesSync2* stereoCloudSync2_; //Stereo image and cloud synchronizer
  sensors_fusion::ImageCloudMessagesSync* imageCloudSync_;

  // Multi thread
  boost::thread* processthread_;
  bool processthreadfinished_;

  // Class members
  std::string root_directory_;
  boost::filesystem::path image_02_dir_path_, image_03_dir_path_;
  boost::filesystem::path timestamp_image02_path_, timestamp_image03_path_;
  boost::filesystem::path velo_dir_path_, oxts_dir_path_;
  boost::filesystem::path timestamp_velo_path_, timestamp_oxts_path_;

  // Whole counter
  unsigned long int count_;

  // Only one image topic or not
  int is_one_image_;
};

#endif /* CATKIN_WS_TEST_SRC_ROS_KITTI_WRITER_INCLUDE_ROS_KITTI_WRITER_KITTI_WRITER_H_ */
