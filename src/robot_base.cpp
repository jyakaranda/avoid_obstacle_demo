#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include "avoid_obstacle_demo/obstacles.h"
#include "avoid_obstacle_demo/obstacle.h"
#include <math.h>
#include <vector>
#include <sys/time.h>

using namespace std;

#define PI 3.1415926
#define DEG2RAD(x) ((x)/180.0*PI)
#define RAD2DIST(x) (0.0855 / (abs(sin(x))) )

double param_robot_width;
double param_laser2front;
bool param_debug;
double param_min_ob_dist;
double param_max_speed;
double param_angle_interval;
// double param_min_angle;
// double param_max_angle;

double theta;
double min_angle, max_angle;

bool is_ok;

ros::Publisher pub_cmd_vel;
ros::Duration duration;
static time_t current_time, last_time;
struct timeval tv;

bool judge(double angle, double dist){
    // 给定扫描点的角度和距离判断是否在碰撞范围内，true为在范围内，false为不在
    if(angle >= (PI-theta) && angle <= (PI+theta)){
        if(dist < param_min_ob_dist){
            return true;
        }
    } else if(angle>=PI+theta && angle<=max_angle || angle<=PI-theta && angle>=min_angle){
        if(dist < RAD2DIST(angle)){
            return true;
        }
    }
    return false;
}

void cb_obstacle_info(avoid_obstacle_demo::obstacles ob){
    static bool locked = false;
    static ros::Rate rate(1);
    bool flag = false;
    vector<avoid_obstacle_demo::obstacle> obs_v;
    gettimeofday(&tv, NULL);
    ROS_INFO("cb_obstacle_info %ld", tv.tv_sec);

    if(locked){
        ROS_INFO("cb_obstacle_info locked");
        return;
    } else {
        locked = true;
        current_time = time(&current_time);
        if((current_time - last_time) < 1){
            ROS_INFO("%ld %ld %lf", current_time, last_time, duration.toSec());
            locked = false;
            return;
        }
        ROS_INFO("%lf %lf %lf", current_time, last_time, duration.toSec());
    }

    for(int i = 0; i < ob.size; i++){
        if(judge(ob.angle[i], ob.dist[i])){
            ROS_INFO("detected obstacle %d in dist: %f m, angle: %f rad; RAD2DIST: %f m", obs_v.size(), ob.dist[i], ob.angle[i], RAD2DIST(ob.angle[i]));
            avoid_obstacle_demo::obstacle tmp;
            tmp.dist = ob.dist[i];  tmp.angle = ob.angle[i];
            obs_v.push_back(tmp);
        }
    }
    
    if(obs_v.size() == 0){
        ROS_INFO("there is no detected obstacle, move forward...");
        geometry_msgs::Twist twist;
        twist.angular.x = 0;  twist.angular.y = 0;  twist.angular.z = 0;
        twist.linear.x = param_max_speed;  twist.linear.y = 0;  twist.linear.z = 0;
        pub_cmd_vel.publish(twist);
        ros::spinOnce();
    } else{
        ROS_INFO("there are %d detected obstacle points", obs_v.size());
        geometry_msgs::Twist twist;
        twist.angular.x = 0;  twist.angular.y = 0;
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
        twist.angular.z = 0;
        pub_cmd_vel.publish(twist);

        // if(obs_v[obs_v.size()/2].angle > PI){
        //     twist.angular.z = -DEG2RAD(param_angle_interval);
        // } else{
            twist.angular.z = DEG2RAD(param_angle_interval);
        // }

        pub_cmd_vel.publish(twist);
        ros::spinOnce();
    }

    last_time = current_time;
    locked = false;
}

void init(){
    is_ok = false;
}

int main(int argc,char *argv[]){

    ros::init(argc, argv, "robot_base_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    duration.fromSec(1.0);

        current_time = time(&current_time);

    last_time=time(&last_time);

    nh.param<double>("robot_width", param_robot_width, 0.0855);
    nh.param<double>("laser2front", param_laser2front, 0.0656);
    nh.param<bool>("debug", param_debug, false);
    nh.param<double>("min_ob_dist", param_min_ob_dist, 0.0);
    nh.param<double>("max_speed", param_max_speed, 0.2);
    // nh.param<double>("min_angle", param_min_angle, 135.0);
    // nh.param<double>("max_angle", param_max_angle, 225.0);
    nh.param<double>("angle_interval", param_angle_interval, 30.0);

    theta = atan(param_robot_width/2.0/(param_min_ob_dist+param_laser2front));
    min_angle = PI - atan(param_robot_width/2.0/param_laser2front);
    max_angle = PI + atan(param_robot_width/2.0/param_laser2front);
    
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 50);
    
    ros::Subscriber sub_obstacle = n.subscribe<avoid_obstacle_demo::obstacles>("obstacle_info", 50, cb_obstacle_info);


    ROS_INFO("robot_base ready");

    ros::spin();

    return 0;

}