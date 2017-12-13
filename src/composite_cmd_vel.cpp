#include <composite.h>

namespace experiment1
{
composite::composite(ros::NodeHandle nh): 
   min_dis(0), 
   qot(0), 
   min(0), 
   delta(0.5),
   delta_s(0.2), 
   vel_max(0.3), 
   vel_min(0.1),
   ang_max(1),
   ind(0)
{
  dynamic_reconfigure::Server<experiment1::compositeConfig>::CallbackType f;
  f = boost::bind(&composite::ConfigCallback,this, _1, _2);
  server.setCallback(f);

  sensor_sub = nh.subscribe("scan",10,&composite::SensorCallback,this);
  qot_sub = nh.subscribe("qot",1,&composite::QotCallback,this);
  teleop_sub = nh.subscribe("teleop_velocity_smoother/raw_cmd_vel",1,&composite::TeleopCallback,this);
  actual_sub = nh.subscribe("mobile_base/commands/velocity", 1,&composite::ActualCallback,this);
  vel_pub = nh.advertise<geometry_msgs::Twist>("sharecontrol",1);
  
}

void composite::ConfigCallback(experiment1::compositeConfig &config, uint32_t level) {
  delta = config.delta;  
  vel_max = config.vel_max; 
  vel_min = config.vel_min; 
  ang_max = config.ang_max; 
  delta_s = config.delta_s; 
  ROS_INFO("Reconfigure request : [%f] [%f] [%f] [%f] [%f]",  
           delta, 
           vel_max, 
           vel_min, 
           ang_max,
           delta_s);  
}

void composite::ActualCallback(const geometry_msgs::Twist &cmd_vel_temp)
{
  actual_cmd_vel.linear.x=cmd_vel_temp.linear.x;
  actual_cmd_vel.angular.z=cmd_vel_temp.angular.z;
  //ROS_INFO("NAVIcallback");
}

void composite::TeleopCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel_temp)
{
  teleop_cmd_vel.linear.x=cmd_vel_temp->linear.x;
  teleop_cmd_vel.angular.z=cmd_vel_temp->angular.z;
  //ROS_INFO("teleopcallback");
}

void composite::SensorCallback(const sensor_msgs::LaserScan &sensor_temp)
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

void composite::QotCallback(const geometry_msgs::Twist &qot_temp)
{
  qot = qot_temp.linear.x;
  //ROS_INFO("qotcallback");
}

void composite::Calculate()
{
  double obs_cmd=0,
      ind_temp=0,
      k=0,
      k_last=0;

  ind_temp = ind;//将激光标志位赋值，以免在计算时变化

  if ( ( min>delta_s && min<delta ) || min==delta ){

    //计算vel_obs
    if (ind_temp>=70 && ind_temp<=278){
      obs_cmd = ang_max * exp( -(278-ind_temp) / 208 * ( (min - delta_s) / (delta - delta_s) ) );
    }else if (ind_temp>278 && ind_temp<=486){
      obs_cmd = -ang_max * exp( -(ind_temp-278) / 208 * ( (min - delta_s) / (delta - delta_s) ) );
    }else{
      obs_cmd = 0;
    }
    //ROS_INFO("OBS_CMD is: [%f]",obs_cmd);
    
    k = pow( ( ( min-delta_s) / ( delta-delta_s) ) * ( exp( -abs( actual_cmd_vel.angular.z ) ) ), 1-qot );
    
    if ( abs(k-k_last)>0.05){
      if (k>k_last){
        k = k_last + 0.05;
      }else{
        k = k_last - 0.05;
      }
    }

    composite_cmd_vel.angular.z = (1-k) * obs_cmd + k * teleop_cmd_vel.angular.z;
    composite_cmd_vel.linear.x = teleop_cmd_vel.linear.x * k;
    composite_cmd_vel.linear.y = k;
    composite_cmd_vel.angular.x = obs_cmd;
    k_last = k;
    
  }else if ( min < delta_s || min == delta_s ){

    if (ind_temp>=70 && ind_temp<=278){
      obs_cmd = ang_max;
    }else if (ind_temp>278 && ind_temp<=486){
      obs_cmd = -ang_max;
    }
    
    //当最小距离小于0.2米时，k = 0，不接受手控
    composite_cmd_vel.angular.z = obs_cmd;
    composite_cmd_vel.linear.x = vel_min;
    composite_cmd_vel.linear.y = 0;
    composite_cmd_vel.angular.x = obs_cmd;
    k_last = 0;
  
  }else if( min > delta ){
    //纯手控
    composite_cmd_vel.angular.z = teleop_cmd_vel.angular.z;
    composite_cmd_vel.linear.x = teleop_cmd_vel.linear.x;
    composite_cmd_vel.linear.y = 1;
    composite_cmd_vel.angular.x = 0;
    k_last = 1;
  }

  if (teleop_cmd_vel.linear.x ==0 && teleop_cmd_vel.angular.z == 0){
    composite_cmd_vel.angular.z = 0;
    //ROS_INFO("no operation");
  }

  vel_pub.publish(composite_cmd_vel);
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "experiment1");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  experiment1::composite cp(nh);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  

  while (ros::ok())
    {
    cp.Calculate();
    ros::spinOnce();
    loop_rate.sleep();
    }

  return 0;
}
