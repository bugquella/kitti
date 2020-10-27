==================
# ROS_KITTI_Writer
 将kitti的rosbag存储为kitti数据集合
## Overview
 kitti数据集合[KITTI Dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php).
 
## 使用
```
$ rosbag play [your_offline_rosbag]
$ roslaunch ros_kitti_writer kitti_writer_standalone.launch
```
## 将kitti数据封装成ROSbag
 [kitti2bag](https://github.com/tomas789/kitti2bag)

