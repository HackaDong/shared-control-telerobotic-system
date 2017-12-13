#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"

#include "ros/callback_queue.h"
#include <boost/thread.hpp>

#include <dynamic_reconfigure/server.h>
#include <experiment1/compositeConfig.h>

geometry_msgs::Twist teleop_cmd_vel;
geometry_msgs::Twist actual_cmd_vel;
geometry_msgs::Twist composite_cmd_vel;
double min_dis, qot=0, min, delta=0.45, vel_max=0.5, ang_max=1.5;
int ind;

void ConfigCallback(experiment1::compositeConfig &config, uint32_t level) {
  delta = config.delta;  
  vel_max = config.vel_max;  
  ang_max = config.ang_max;  
  ROS_INFO("Reconfigure request : [%f] [%f] [%f]",  
           delta,  
           vel_max,  
           ang_max);  
}

void ActualCallback(const geometry_msgs::Twist &cmd_vel_temp)
{
  actual_cmd_vel.linear.x=cmd_vel_temp.linear.x;
  actual_cmd_vel.angular.z=cmd_vel_temp.angular.z;
  //ROS_INFO("NAVIcallback");
}

void TeleopCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel_temp)
{
  teleop_cmd_vel.linear.x=cmd_vel_temp->linear.x;
  teleop_cmd_vel.angular.z=cmd_vel_temp->angular.z;
  //ROS_INFO("teleopcallback");
}

void SensorCallback(const sensor_msgs::LaserScan &sensor_temp)
{
  min = sensor_temp.ranges.at(70);
  ind = 70;
  int num = 556;
  for(int i=70;i<num-70;i++){
    if(sensor_temp.ranges.at(i) < min){
      min = sensor_temp.ranges.at(i);
      ind = i;
    }
  }
  //sensor_temp.ranges.at(ind)
  //ROS_INFO("num is: [%d]",num);
  //ROS_INFO("min_ind is: [%d]",ind);
  //ROS_INFO("min_dis is: [%f]",min);
}

void QotCallback(const geometry_msgs::Twist &qot_temp)
{
  qot = qot_temp.linear.x;
  //ROS_INFO("qotcallback");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "experiment1");
  ros::NodeHandle n_teleop;
  ros::NodeHandle n_actual;
  ros::NodeHandle n_pub;
  ros::NodeHandle n_sensor;
  ros::NodeHandle n_qot;
  ros::Rate loop_rate(10);

  dynamic_reconfigure::Server<experiment1::compositeConfig> server;
  dynamic_reconfigure::Server<experiment1::compositeConfig>::CallbackType f;

  f = boost::bind(&ConfigCallback, _1, _2);
  server.setCallback(f);

  ros::Subscriber sensor_sub = n_sensor.subscribe("scan",10,SensorCallback);
  ros::Subscriber qot_sub = n_qot.subscribe("qot",1,QotCallback);
  ros::Subscriber teleop_sub = n_teleop.subscribe("teleop_velocity_smoother/raw_cmd_vel",1,TeleopCallback);
  ros::Subscriber actual_sub = n_actual.subscribe("mobile_base/commands/velocity", 1,ActualCallback);
  ros::Publisher vel_pub = n_pub.advertise<geometry_msgs::Twist>("sharecontrol",1);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  double obs_cmd=0,
      ind_temp=0,
      k=0;

  while (ros::ok())
    {
    ind_temp = ind;
    if (min<delta){
      if (ind_temp>=70 && ind_temp<=278){
        obs_cmd = ang_max * exp( -(278-ind_temp) / 208 * min / delta );
      }else if (ind_temp>278 && ind_temp<=486){
        obs_cmd = -ang_max *exp( -(ind_temp-278) / 208 * min / delta );
      }else{
        obs_cmd = 0;
      }
      //ROS_INFO("OBS_CMD is: [%f]",obs_cmd);
      k = pow( (min/delta) * (1 - actual_cmd_vel.linear.x/vel_max), 1-qot );
      composite_cmd_vel.angular.z = (1-k)*obs_cmd + k*teleop_cmd_vel.angular.z;
      composite_cmd_vel.linear.x = teleop_cmd_vel.linear.x * k;
      composite_cmd_vel.linear.y = k;
    }else{
      composite_cmd_vel.angular.z = teleop_cmd_vel.angular.z;
      composite_cmd_vel.linear.x = teleop_cmd_vel.linear.x;
      composite_cmd_vel.linear.y = 0;
    }

    if (teleop_cmd_vel.linear.x ==0 && teleop_cmd_vel.angular.z == 0){
      composite_cmd_vel.angular.z = 0;
      //ROS_INFO("no operation");
    }

    vel_pub.publish(composite_cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
    }

  return 0;
}
