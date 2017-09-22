apriltag_checkout_tag节点
==========================
作者：宋瑾

### 1.功能描述

本节点主要对摄像头采集到的图像和里程计的信息进行处理<br>
- 利用采集到的图像检测apriltag相对于无人车的Pose_aprilToCar
- 采集检测到apriltag时的odom信息
- 将检测到的Pose_aprilToCar和odom结合成一个topic发布

### 2.订阅话题
| 话题名       | 话题类型     | 说明    |
| --------------   | -----------:   |:------------: |
| “/camera/image_raw” | [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)|8位灰度图|
| "/odom_topic" | [nav_msg::Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html )|里程计信息|


### 3.发布信息
| 话题名       |话题类型      | 说明     |发布时间
|--------------   | -----------:   | -----------:   |:------------: |
| “/tagPose_odom” | [custom_msg/PoseStampedArray](/src/custom_msg/msg/PoseStampedArray.msg) |结合Pose和odom|10HZ
### 4.节点通信
- 与相机驱动节点[pointgrey_camera_driver](/src/pointgrey_camera_driver)的通信
> 关系：本节点的上游节点<br>
> 说明：从相机节点订阅“/camera/image_raw”话题来检测tag

- 与里程计驱动节点[odom_localization_driver](src/odom_localization_driver)的通信
> 关系：本节点的上游节点<br>
> 说明：本节点订阅odom_localization_driver发布的"/odom_topic"

- 与[apriltag_mapping](/src/apriltag_mapping)的通信
> 关系：本节点的下游节点<br>
> 说明：apriltag_mapping节点订阅本节点发布的“/tagPose_odom”

- 与[apriltag_localization](/src/apriltag_localization)节点通信
> 关系:本节点的下游节点<br>
> 说明:apriltag_localization节点订阅本节点发布的"/tagPose_odom"
