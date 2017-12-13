#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"

#include "ros/callback_queue.h"
#include <boost/thread.hpp>

#include <dynamic_reconfigure/server.h>
#include <experiment1/compositeConfig.h>

#include <algorithm>//用于max函数

namespace experiment1
{
class composite
{
public:
    explicit composite(ros::NodeHandle nh);
    void ConfigCallback(experiment1::compositeConfig &config, uint32_t level);
    void ActualCallback(const geometry_msgs::Twist &cmd_vel_temp);
    void TeleopCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel_temp);
    void SensorCallback(const sensor_msgs::LaserScan &sensor_temp);
    void QotCallback(const geometry_msgs::Twist &qot_temp);
    void Calculate();
private:
    dynamic_reconfigure::Server<experiment1::compositeConfig> server;

    ros::Subscriber sensor_sub;
    ros::Subscriber qot_sub;
    ros::Subscriber teleop_sub;
    ros::Subscriber actual_sub;
    ros::Publisher vel_pub;
  
    geometry_msgs::Twist teleop_cmd_vel, actual_cmd_vel, composite_cmd_vel;
    double min_dis, qot, min, delta, delta_s, vel_max, vel_min, ang_max;
    int ind;
};
}