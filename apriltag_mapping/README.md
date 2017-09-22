apriltag_mapping节点
====================
作者:宋瑾

### 1.功能描述
该节点的目的是建立地图，地图包含tag点在世界坐标系下的坐标,生成map.txt文件

### 2.订阅话题
| 话题名       | 话题类型     | 说明    |
| --------------   | -----------:   |:------------: |
|  “/tagPose_odom” | [custom_msg/PoseStampedArray](/src/custom_msg/msg/PoseStampedArray.msg)|结合Pose和odom

### 3.发布信息
不发布话题，运行结束后将地图保存成map.txt文件
                             
### 4.节点通信
- 与[apriltag_checkout_tag](/src/apriltag_checkout_tag)的通信
> 关系:本节点的上游节点<br>
> 说明:从apriltag_checkout_tag节点订阅"/tagPose_odom"