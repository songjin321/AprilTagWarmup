apriltag_localization节点
========================
作者:宋瑾

### 1.功能描述
本节点利用建好的tag地图map.txt，以及tag相对于无人车的Pose和里程计的信息，
融合得到无人车在世界坐标系下的2DPose

### 2.订阅话题
| 话题名       | 话题类型     | 说明    |
| --------------   | -----------:   |:------------: |
|  “/tagPose_odom” | [custom_msg/PoseStampedArray](/src/custom_msg/msg/PoseStampedArray.msg)|结合Pose和odom

### 3.发布信息
| 话题名       |话题类型      | 说明     |发布时间
|--------------   | -----------:   | -----------:   |:------------: |
| “/carPoseByApriltag” | [re_msgs/Pose2DStamped](http://docs.ros.org/fuerte/api/re_msgs/html/msg/Pose2DStamped.html) |无人车的2DPose|10HZ

### 4.节点通信
- 与[apriltag_checkout_tag](/src/apriltag_checkout_tag)的通信
> 关系:本节点的上游节点<br>
> 说明:从apriltag_checkout_tag节点订阅"/tagPose_odom"

- 与[sensor_fusion_localization](/src/sensor_fusion)的通信
> 关系：本节点的下游节点<br>
> 说明：sensor_fusion_localization节点订阅本节点发布的"/carPoseByApriltag"
