#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"

#include "ros/callback_queue.h"
#include <boost/thread.hpp>

geometry_msgs::Twist teleop_cmd_vel;
geometry_msgs::Twist navi_cmd_vel;
geometry_msgs::Twist composite_cmd_vel;
double min_dis;
double qot;

void NaviCallback(const geometry_msgs::Twist &cmd_vel_temp)
{
  navi_cmd_vel.linear.x=cmd_vel_temp.linear.x;
  navi_cmd_vel.angular.x=cmd_vel_temp.angular.x;
  ROS_INFO("NAVIcallback");
}

void TeleopCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel_temp)
{
  teleop_cmd_vel.linear.x=cmd_vel_temp->linear.x;
  teleop_cmd_vel.angular.x=cmd_vel_temp->angular.x;
  ROS_INFO("teleopcallback");
}

void SensorCallback(const sensor_msgs::LaserScan &sensor_temp)
{
  min_dis = sensor_temp.range_min;
  ROS_INFO("SENSORcallback");
}

void QotCallback(const geometry_msgs::Twist &qot_temp)
{
  qot = qot_temp.linear.x;
  ROS_INFO("qotcallback");
}

ros::CallbackQueue teleop_queue;
void callbackThread_teleop()
{
  ROS_INFO_STREAM("Callback_teleop thread id=" << boost::this_thread::get_id());

  ros::NodeHandle n;
  while (n.ok())
  {
    teleop_queue.callAvailable(ros::WallDuration(0.5));
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "experiment1");
  ros::NodeHandle n_teleop;
  ros::NodeHandle n_navi;
  ros::NodeHandle n_pub;
  ros::NodeHandle n_sensor;
  ros::NodeHandle n_qot;
  ros::Rate loop_rate(10);

  ros::Subscriber sensor_sub = n_sensor.subscribe("scan",1,SensorCallback);
  ros::Subscriber qot_sub = n_qot.subscribe("qot",1,&QotCallback);
  ros::Publisher vel_pub = n_pub.advertise<geometry_msgs::Twist>("sharecontrol",1);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
        "teleop_velocity_smoother/raw_cmd_vel",
        1,
        TeleopCallback,
        ros::VoidPtr(),
        &teleop_queue
        );
  ros::Subscriber teleop_vel_sub = n_teleop.subscribe(ops);
  boost::thread teleop_thread(callbackThread_teleop);

  ros::Subscriber navi_vel_sub = n_navi.subscribe("navigation_velocity_smoother/raw_cmd_vel", 1,NaviCallback);
  ros::CallbackQueue navi_queue;
  n_navi.setCallbackQueue(&navi_queue);
  ros::AsyncSpinner navi_spinner(1,&navi_queue);
  navi_spinner.start();

  while (ros::ok())
    {
    navi_queue.callAvailable(ros::WallDuration());
    composite_cmd_vel.linear.x = qot * teleop_cmd_vel.linear.x + navi_cmd_vel.linear.x;
    composite_cmd_vel.angular.x = qot * teleop_cmd_vel.angular.x + navi_cmd_vel.angular.x;
    if(min_dis>1)
    {
      vel_pub.publish(composite_cmd_vel);
    }
    else
    {
      composite_cmd_vel.linear.x = composite_cmd_vel.linear.x * min_dis;
      composite_cmd_vel.angular.x = composite_cmd_vel.angular.x * min_dis;
      vel_pub.publish(composite_cmd_vel);
    }

    ros::spinOnce();
    ROS_INFO("WHILE");
    loop_rate.sleep();
    }
    navi_spinner.stop();
    teleop_thread.join();
    ROS_WARN("STOP");
  return 0;
}
