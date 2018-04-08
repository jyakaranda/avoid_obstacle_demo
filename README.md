# avoid_obstacle_demo
An obstacle avoiding demo based on ROS

## Desciption

---

一个简单的基于ROS的避障小车demo，最终需要融合现有的传感器（单线激光雷达、单目摄像头、IMU等）进行避障前进。
目前v0.0.1仅使用了激光雷达进行避障，存在不足有：没有考虑小车的转弯半径、没有充分利用麦克纳姆轮任意方向前进的特点（只能转弯前进）、行进速度较慢、没有进行传感器融合等等。

## Version
- v0.0.1
