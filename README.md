基于apriltag的无人车定位
Warmup Project  for Freshmen@NRSL about Localization utilizing Apriltag
======================
## 1.系统整体构成
### 物理部分组成：
+ camera:[pointgrey]()</br>
+ turtlebot:</br>
+ apriltag:</br>
### 软件部分组成
一共分为三个包</br>
+ [apriltag_checkout_tag](https://github.com/UnmannedTrackor/ROSSoftwareForHITTractor/tree/develop/src/apriltag_checkout_tag)
+ [apriltag_mapping](https://github.com/UnmannedTrackor/ROSSoftwareForHITTractor/tree/develop/src/apriltag_mapping)
+ [apriltag_localization](https://github.com/UnmannedTrackor/ROSSoftwareForHITTractor/tree/develop/src/apriltag_localization)
## 2.基本流程
天花板上贴有人工标志物apriltag，摄像头朝上安装在无人车上，
## 3.主要目的
学习ros、c++、git以及定位的基本原理，具体需要达到的要求
+ 使用git的基本功能、能在github上进行交流
+ 编独立编写ros的一个节点，学会写msg，cmakefile，launch文件
+ 学习ros下tf的基本使用方法
+ 熟悉c++，了解类、引用、指针以及vector，string的基本用法
+ 概率机器人slam基本原理，包括位姿图优化，机器人运动模型，观测模型，EKF，Monte-Carlo定位
## 4.具体实施
github上建立一个仓库，大家使用git各自维护一个部分，学习基本理论后讲给他人听。
| 人| 负责的部分|
|-----|------|
|巫亚奇|EKF定位|
|朱芬芳|机器人运动、观测模型|
|孙家弼|基于图优化的slam|
|宋瑾|Monte-Carlo定位|
三次小的交流会，包括git、ros和c++，各自有什么不懂的，或者好的资源推荐互相交流。
## 5.要求
本次项目的主要目的在于大家的相互学习和交流上，在这个基础上再提高定位的精度，项目周期暂定为三个月，希望项目结束后每个人对自己负责的部分有深入的了解（数学原理推导，看paper，调研实际情况中遇到的问题，如何改进），对项目整体也有基本了解。
## 6.备注
将学习的资源放在共有仓库的learn文件夹下
> + git 廖学峰 progit
> + ros roswiki
> + c++ prime
> + 图优化 视觉slam十四讲
> + 定位理论 概率机器人

注册一个github的账号
作业：
+ 优化算法
  HW#1:What is the Convex function?(goole;bing;wiki)
+ 线性系统
  HW#1:DDL 9.18
  HW#2:DDL 9.20
+ 计算机视觉
  HW#1:(DDL 9.16 0:00)
+ 数值分析
</br>随时补充