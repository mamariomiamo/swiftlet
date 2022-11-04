// #include <plan_env/grid_map.h>
#include <ros/ros.h>
#include <iostream>
int main(int argc, char **argv)
{

  ros::init(argc, argv, "occupancy_mapping");
  ros::NodeHandle nh("~");

  std::cout << "ok here" << std::endl;

  // GridMap::Ptr grid_map_;
  // grid_map_.reset(new GridMap);
  // std::cout << "ok here1" << std::endl;
  // grid_map_->initMap(nh);
  // std::cout << "ok here2" << std::endl;
  // ros::Duration(1.0).sleep();


  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  return 0;
}