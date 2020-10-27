/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年12月7日
 * Copyright    :
 * Descriptoin  :
 * References   :
 ======================================================================*/
#include <ros_kitti_writer/kitti_writer.h>
// C++
#include <chrono>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cstdio>
// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/format.hpp>
// Opencv
#include <opencv2/opencv.hpp>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>

using namespace std;

static string folder_image_02 = "image_02/";
static string folder_image_03 = "image_03/";
static string folder_velodyne_points = "velodyne_points/";
static string folder_oxts = "oxts/";
static string format_image = "%|010|.png";
static string formate_velo = "%|010|.bin";
static string formate_oxts = "%|010|.txt";


/*!
 * @brief From ROS Nano second time to date time
 *        Example:
 *                 uint64_t ros_tt = image->header.stamp.toNSec();
                   string date = toDateTime(ros_tt);// 2011-09-26 13:04:32.351950336
 * @param ros_time
 * @return
 */
static string toDateTime(uint64_t ros_time)
{
  // Convert to chrono nanoseconds type
  chrono::nanoseconds tp(ros_time), tt_nano(ros_time);

  // Get nanoseconds part
  tp -= chrono::duration_cast<chrono::seconds>(tp);

  //to_time_t方法把时间点转化为time_t
  time_t tt =  std::time_t(chrono::duration_cast<chrono::seconds>(chrono::nanoseconds(tt_nano)).count());
  //把time_t转化为tm
  tm *localTm =  localtime(&tt);

  //tm格式化输出
  char buffer[20];
  char format[] = "%Y-%m-%d %H:%M:%S";
  strftime(buffer, sizeof(buffer), format, localTm);

  string nanosec_format = "%|09|";
  stringstream ss;
  ss<<buffer<<"."<<(boost::format(nanosec_format) % tp.count()).str();
  return ss.str();
}

Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;

    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    return euler;
}


 
struct Quaternion {
    double w, x, y, z;
};
 
struct EulerAngles {
    double roll, pitch, yaw;
};
 
EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;
 
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
 
    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);
 
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
 
    return angles;
}


KittiWriter::KittiWriter(ros::NodeHandle nh, ros::NodeHandle private_nh):
    count_(0),
    processthread_(NULL),
    processthreadfinished_(false),
    is_one_image_(0)
{

  // Define lidar parameters
  if (!private_nh.getParam("root_directory", root_directory_)) {
    ROS_ERROR("Have't set $(root_directory)  or $(velo_topic) parameters valid value!");
    ros::shutdown();
  }
  // Define semantic parameters
   string velo_topic(""),
   left_camera_topic(""),
   right_camera_topic(""),
   gps_topic(""),
   imu_topic(""),
   vel_topic(""),
   tf_static_topic(""),
   tf_topic("");
   
  private_nh.param("velo_topic",velo_topic, velo_topic);
  private_nh.param("left_camera_topic", left_camera_topic, left_camera_topic);
  private_nh.param("right_camera_topic", right_camera_topic, right_camera_topic);
  private_nh.param("gps_topic", gps_topic, gps_topic);
  private_nh.param("imu_topic", imu_topic, imu_topic);
  private_nh.param("vel_topic", vel_topic, vel_topic);
  private_nh.param("tf_static_topic", tf_static_topic, tf_static_topic);
  private_nh.param("tf_topic", tf_topic, tf_topic);
  
  // Mush have point cloud topic
  if (velo_topic.empty()) {
    ROS_ERROR("Must set $(velo_topic) parameter valid value!");
    ros::shutdown();
  }

  // Print parameters
  ROS_INFO_STREAM("root_directory: " << root_directory_);
  ROS_INFO_STREAM("velo_topic: " << velo_topic);
  ROS_INFO_STREAM("left_camera_topic: " << left_camera_topic);
  ROS_INFO_STREAM("right_camera_topic: " << right_camera_topic);
  ROS_INFO_STREAM("gps_topic: " << gps_topic);
  ROS_INFO_STREAM("imu_topic: " << imu_topic);
  ROS_INFO_STREAM("vel_topic: " << vel_topic);
  ROS_INFO_STREAM("tf_static_topic: " << tf_static_topic);
  ROS_INFO_STREAM("tf_topic: " << tf_topic);
  
  
  
  /// -----According to input topics to decide sync which messages-----
  if (left_camera_topic.empty() ^ right_camera_topic.empty()) { // if only has one camera image topic
    string image_topic = left_camera_topic.empty() ? right_camera_topic : left_camera_topic;
    imageCloudSync_ = new sensors_fusion::ImageCloudMessagesSync(nh, image_topic, velo_topic);
    is_one_image_ = 0;
    ROS_WARN_STREAM("Only one image topic... ");
  }
  else if (!left_camera_topic.empty() && !right_camera_topic.empty() && gps_topic.empty()) {// has two camera image topic
    is_one_image_ = 1;
    stereoCloudSync_ = new sensors_fusion::StereoMessagesSync(nh, left_camera_topic,right_camera_topic, velo_topic);
  }
  else if (!left_camera_topic.empty() && !right_camera_topic.empty() && !gps_topic.empty() && !imu_topic.empty() && !vel_topic.empty()) {// has two camera image topic
    is_one_image_ = 2;
    stereoCloudSync2_ = new sensors_fusion::StereoMessagesSync2(nh, left_camera_topic,right_camera_topic, velo_topic, gps_topic, imu_topic, vel_topic);
  }
  else {
    ROS_ERROR("Must set one of left_camera_topic or right_camera_topic parameter valid value!");
    ros::shutdown();
  }

  // Create formatted folders
  createFormatFolders();

  processthread_ = new boost::thread(boost::bind(&KittiWriter::process,this));
}

KittiWriter::~KittiWriter() {
  processthreadfinished_ = true;
  processthread_->join();
}

void KittiWriter::process()
{
  // main loop
  while(!processthreadfinished_&&ros::ok()) {
    sensors_fusion::SynchronizedMessages imagePair;
    bool flag = false;
    if (is_one_image_ == 0) {
      flag = imageCloudSync_->is_valid();
      imagePair = imageCloudSync_->getSyncMessages();
    }
    else if(is_one_image_ == 1) {
      flag = stereoCloudSync_->is_valid();
      imagePair =  stereoCloudSync_->getSyncMessages();
    }else if(is_one_image_ == 2) {
      flag = stereoCloudSync2_->is_valid();
      imagePair =  stereoCloudSync2_->getSyncMessages();
    }
    // Get synchronized image with bboxes and cloud data
    if (!flag) {
      ROS_ERROR_THROTTLE(1,"Waiting for image and lidar topics!!!");
      continue;
    }

    ROS_WARN_STREAM("Begin saving data "<<count_);
    // process image
    saveImage02(imagePair.image1_ptr);
    saveImage03(imagePair.image2_ptr);

    // Preprocess point cloud
    saveVelodyne(imagePair.cloud_ptr);
    
    saveOxts(imagePair.gps_ptr, imagePair.imu_ptr, imagePair.vel_ptr);
    ++ count_;
  }// end while
}

void KittiWriter::createFormatFolders()
{
  // Create image 02 and 03 folder
  image_02_dir_path_ = boost::filesystem::path(root_directory_)
                       / folder_image_02
                       / "data";
  if(!boost::filesystem::exists(image_02_dir_path_)) {
    boost::filesystem::create_directories(image_02_dir_path_);
  }
  timestamp_image02_path_ = boost::filesystem::path(root_directory_)
                            / folder_image_02
                            / "timestamps.txt";

  image_03_dir_path_ = boost::filesystem::path(root_directory_)
                       / folder_image_03
                       / "data";
  if(!boost::filesystem::exists(image_03_dir_path_)) {
    boost::filesystem::create_directories(image_03_dir_path_);
  }
  timestamp_image03_path_ = boost::filesystem::path(root_directory_)
                            / folder_image_03
                            / "timestamps.txt";
  // Create velodyne_points folder
  velo_dir_path_ = boost::filesystem::path(root_directory_)
                  / folder_velodyne_points
                  / "data";
  if(!boost::filesystem::exists(velo_dir_path_)) {
    boost::filesystem::create_directories(velo_dir_path_);
  }
  timestamp_velo_path_ = boost::filesystem::path(root_directory_)
                        / folder_velodyne_points
                        / "timestamps.txt";
                        
                        
   oxts_dir_path_ = boost::filesystem::path(root_directory_)
                  / folder_oxts
                  / "data";
  if(!boost::filesystem::exists(oxts_dir_path_)) {
    boost::filesystem::create_directories(oxts_dir_path_);
  }
  timestamp_oxts_path_ = boost::filesystem::path(root_directory_)
                        / folder_oxts
                        / "timestamps.txt";
                        
}

void KittiWriter::saveImage02(const sensor_msgs::Image::ConstPtr & image)
{
  // Convert image detection grid to cv mat
  cv_bridge::CvImagePtr cv_det_grid_ptr;
  try{
    cv_det_grid_ptr = cv_bridge::toCvCopy(image,
        sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat raw_image = cv_det_grid_ptr->image;
  // Get image name
  boost::filesystem::path image_02_file_path = image_02_dir_path_
      /(boost::format(format_image)%count_).str();
  string image_02_file_name = image_02_file_path.string();

  // 1) Save image
  cv::imwrite(image_02_file_name, raw_image);

  // 2) Save timestamps
  fstream filestr;
  filestr.open (timestamp_image02_path_.string().c_str(), fstream::out|fstream::app);
  uint64_t ros_tt = image->header.stamp.toNSec();
  string date = toDateTime(ros_tt);
  filestr<<date<<std::endl;
  filestr.close();
}


void KittiWriter::saveImage03(const sensor_msgs::Image::ConstPtr & image)
{
  // Convert image detection grid to cv mat
  cv_bridge::CvImagePtr cv_det_grid_ptr;
  try{
    cv_det_grid_ptr = cv_bridge::toCvCopy(image,
        sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat raw_image = cv_det_grid_ptr->image;
  // Get image name
  boost::filesystem::path image_03_file_path = image_03_dir_path_
      /(boost::format(format_image)%count_).str();
  string image_03_file_name = image_03_file_path.string();

  // 1) Save image
  cv::imwrite(image_03_file_name, raw_image);

  // 2) Save timestamps
  fstream filestr;
  filestr.open (timestamp_image03_path_.string().c_str(), fstream::out|fstream::app);
  uint64_t ros_tt = image->header.stamp.toNSec();
  string date = toDateTime(ros_tt);
  filestr<<date<<std::endl;
  filestr.close();
}

void KittiWriter::saveVelodyne(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  // Convert to pcl cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2(*cloud));
  cloudMsg->fields[3].name = "intensity";
  pcl::fromROSMsg(*cloudMsg, *pcl_in);

  // cout
  size_t num = pcl_in->size();

  // Get cloud name
  boost::filesystem::path velodyne_file_path = velo_dir_path_
      /(boost::format(formate_velo)%count_).str();
  string velodyne_file_name = velodyne_file_path.string();

  // Begin save data
  FILE* stream = fopen(velodyne_file_name.c_str(), "wb");
  if(stream == NULL) {
    cout<<"error open "<<velodyne_file_name<<endl;
    return ;
  }
  float* data = (float*)malloc(4*num*sizeof(float));
  float* px = data + 0;
  float* py = data + 1;
  float* pz = data + 2;
  float* pI = data + 3;

  for(int i = 0; i < num; ++i) {
    *px = (float)pcl_in->points[i].x;
    *py = (float)pcl_in->points[i].y;
    *pz = (float)pcl_in->points[i].z;
    *pI = (float)pcl_in->points[i].intensity;
    px += 4;
    py += 4;
    pz += 4;
    pI += 4;
  }
  fwrite(data, sizeof(float), 4*num, stream);
  fclose(stream);

  // Save timestamps
  fstream filestr;
  filestr.open (timestamp_velo_path_.string().c_str(), fstream::out|fstream::app);
  uint64_t ros_tt = cloud->header.stamp.toNSec();
  string date = toDateTime(ros_tt);
  filestr<<date<<std::endl;
  filestr.close();
}

void KittiWriter::saveOxts(const sensor_msgs::NavSatFix::ConstPtr& gps, const sensor_msgs::Imu::ConstPtr& imu, const geometry_msgs::TwistStamped::ConstPtr& vel)
{
    // Get cloud name
  boost::filesystem::path oxts_file_path = oxts_dir_path_
      /(boost::format(formate_oxts)%count_).str();
  string oxts_file_name = oxts_file_path.string();

    /*
  // Begin save data
  FILE* stream = fopen(oxts_file_name.c_str(), fstream::out|fstream::app);
  if(stream == NULL) {
    cout<<"error open "<<oxts_file_name<<endl;
    return ;
  }
  */
  
  fstream stream;
  stream.open (oxts_file_name.c_str(), fstream::out|fstream::app);
  
  double lat,lon,alt;
  double roll, pitch,yaw;
  double vn, ve, vf, vl, vu;
  double  ax, ay, af, al,au;
  double wx,wy,wz,wf,wl,wu;
  double pos_accuracy, vel_accuracy;
  int navstat, numsats, posmode, velmode, orimode;
  
  lat = gps->latitude;
  lon = gps->longitude;
  alt = gps->altitude;

  Quaternion q;
  q.x = imu->orientation.x;
  q.y = imu->orientation.y;
  q.z = imu->orientation.z; 
  q.w = imu->orientation.w;
  EulerAngles  angles1 = ToEulerAngles(q);
  roll = angles1.yaw;
  pitch = angles1.pitch;
  yaw = angles1.roll;

  vn = 0;
  ve = 0;
  vf = vel->twist.linear.x;
  vl = vel->twist.linear.y;
  vu = vel->twist.linear.z;
  ax = 0;
  ay = 0;
  af = imu->linear_acceleration.x;
  al = imu->linear_acceleration.y;
  au = imu->linear_acceleration.z;
  wx = 0;
  wy = 0;
  wz = 0;
  wf = vel->twist.angular.x;
  wl = vel->twist.angular.y;
  wu = vel->twist.angular.z;
  pos_accuracy = 0;
  vel_accuracy = 0;
  numsats = 0;
  posmode = 0;
  velmode = 0;
  orimode = 0;
  
  
  stream<<fixed<<setprecision(16)<<lat<<" "<<lon<<" "<<alt<<" ";
  stream<<fixed<<setprecision(16)<<yaw<<" "<<pitch<<" "<<roll<<" ";
  stream<<fixed<<setprecision(16)<<vn<<" "<<vf<<" "<<vl<<" "<<vu<<" "<<ve<<" ";
  stream<<fixed<<setprecision(16)<<ax<<" "<<ay<<" "<<af<<" "<<al<<" "<<au<<" ";
  stream<<fixed<<setprecision(16)<<wx<<" "<<wy<<" "<<wz<<" "<<wf<<" "<<wl<<" "<<wu<<" ";
  stream<<fixed<<setprecision(16)<<pos_accuracy<<" "<<vel_accuracy<<" ";
  stream<<numsats<<" "<<posmode<<" "<<velmode<<" "<<orimode;
  
  stream<<std::endl;
  stream.close();

  // Save timestamps
  fstream filestr;
  filestr.open (timestamp_oxts_path_.string().c_str(), fstream::out|fstream::app);
  uint64_t ros_tt = imu->header.stamp.toNSec();
  string date = toDateTime(ros_tt);
  filestr<<date<<std::endl;
  filestr.close();
  
  
}
