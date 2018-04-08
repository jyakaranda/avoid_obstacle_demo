#include "ros/ros.h"
#include "avoid_obstacle_demo/obstacles.h"
#include "avoid_obstacle_demo/obstacle.h"
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <rosgraph_msgs/Clock.h>
#include <string>
#include <vector>

// #include <rosbag/bag.h>
// #include <rosbag/vie.h>
// #include <boost/foreach.hpp>

#define PI 3.1415926
#define DEG2RAD(x) ((x)*PI/180.0)

ros::Time current_time, last_time;
// ros::Duration obstacle_update_interval;
// double param_min_angle, param_max_angle;
// double min_angle, max_angle;
static bool locked = false;


class SensorSource{
    public:
        SensorSource();
        void init();
        void cb_laser_scan(const sensor_msgs::LaserScan::ConstPtr &laserMsg);
        void cb_camera();
        void cb_odom(const nav_msgs::Odometry::ConstPtr odomMsg);

    private:
        ros::NodeHandle n_;
        ros::NodeHandle nh_;
        ros::Subscriber sub_laser_scan;
        ros::Subscriber sub_odom;
        ros::Publisher pub_obstacle;

        ros::Duration obstacle_update_interval;
        // double param_min_angle;
        // double param_max_angle;
        double param_robot_width;
        double param_laser2front;
        bool param_debug;
        double param_min_ob_dist;
        double min_angle;
        double max_angle;
};

SensorSource::SensorSource():nh_("~"){
    init();
}

void SensorSource::init(){

    double tmp;
    nh_.param<double>("robot_width", param_robot_width, 0.0855);
    nh_.param<double>("laser2front", param_laser2front, 0.0656);
    nh_.param<bool>("debug", param_debug, false);
    nh_.param<double>("min_ob_dist", param_min_ob_dist, 0.0);
    nh_.param<double>("obstacle_update_interval", tmp, 0.3);
    obstacle_update_interval.fromSec(tmp);
    // nh_.param<double>("min_angle", param_min_angle, -45.0);
    // nh_.param<double>("max_angle", param_max_angle, 45.0);
    min_angle = PI - atan(param_robot_width/2.0/param_laser2front) - PI;    // 减PI是为了与laserMsg.angle区间一致
    max_angle = PI + atan(param_robot_width/2.0/param_laser2front) - PI;

    sub_laser_scan = n_.subscribe<sensor_msgs::LaserScan>("scan", 100, boost::bind(&SensorSource::cb_laser_scan, this, _1));
    //n_.subscribe("clock", 100);
    sub_odom = n_.subscribe<nav_msgs::Odometry>("odom", 50, boost::bind(&SensorSource::cb_odom, this, _1));
    pub_obstacle = n_.advertise<avoid_obstacle_demo::obstacles>("obstacle_info", 50);
    ros::Publisher pub = n_.advertise<rosgraph_msgs::Clock>("clock", 50);
    pub.publish(rosgraph_msgs::Clock());
}

void SensorSource::cb_laser_scan(const sensor_msgs::LaserScan::ConstPtr &laserMsg){

    ROS_INFO("wtf %d", laserMsg->header.seq);
    //sensor_msgs::LaserScanPtr laserMsg(new sensor_msgs::LaserScan());
    static ros::Time last_scan(0, 0);
    static int count = 0;
    ROS_INFO("cb_laser_scan: %d", count);
    if(laserMsg == 0){
        return;
    }
    // 至少以obstacle_update_interval的时间间隔更新障碍物信息
    if((laserMsg->header.stamp - last_scan) > obstacle_update_interval){
        ROS_INFO("cb_laser_scan: %d", count);
        int li = 0, ri = laserMsg->ranges.size() - 1;
        if((laserMsg->angle_min > max_angle) || (laserMsg->angle_max < min_angle)){
            // 扫描角度不在[min_angle, max_angle]区间，默认前进方向有障碍
            // TODO
            return;
        }
        // 找出扫描点在在[min_angle, max_angle]区间的起始结束下标
        li = (laserMsg->angle_min >= min_angle) ? li : (int)((min_angle-laserMsg->angle_min)/laserMsg->angle_increment);
        ri = (laserMsg->angle_max <= max_angle) ? ri : (laserMsg->ranges.size()-1 - (int)((laserMsg->angle_max-max_angle)/laserMsg->angle_increment));

        avoid_obstacle_demo::obstacles::Ptr obs(new avoid_obstacle_demo::obstacles());
        obs->size = ri - li + 1;
        obs->dist.resize(obs->size);
        obs->angle.resize(obs->size);
        obs->angle_increment = laserMsg->angle_increment;

        for(int i = li; i <= ri; i++){
            obs->dist[i - li] = laserMsg->ranges[i];
            obs->angle[i - li] = laserMsg->angle_min + (i * laserMsg->angle_increment) + PI;  // 加PI是为了统一[-PI， PI]到[0, 2*PI]上
            ROS_INFO("laser obstacle %d - dist: %.2f, angle: %.4f", i, obs->dist[i-li], obs->angle[i-li]);
        }

        count++;
        obs->header.seq = count;
        obs->header.stamp = last_scan;
        // TODO 得到雷达扫描的障碍物数组
        if(!locked){
            locked = true;
            last_scan = laserMsg->header.stamp;
            ROS_INFO("locked !!!");
            pub_obstacle.publish(obs);
            locked = false;
        }
    } else{
        ROS_INFO("interval: %lf", (laserMsg->header.stamp - last_scan).toSec());
    }
}

void SensorSource::cb_camera(){

}

void SensorSource::cb_odom(const nav_msgs::Odometry::ConstPtr odomMsg){

}


int main(int argc, char *argv[]){

    ros::init(argc, argv, "sensor_source");

    // TODO ros::Subscriber sub_camera = n_.subscribe<>();
    // ros::Subscriber sub_odom = n_.subscribe<>

    SensorSource sensors;

    ROS_INFO("sensor_source ready");

    ros::spin();

    return 0;
}