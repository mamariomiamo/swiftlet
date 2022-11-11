#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <graph_search.h>

// global variable
Eigen::Vector3d current_pos;
Eigen::Vector3d target_pos;
bool require_planning = false;

void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  current_pos.x() = msg->pose.position.x;
  current_pos.y() = msg->pose.position.y;
  current_pos.z() = msg->pose.position.z;
}

void targetCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  target_pos.x() = msg->pose.position.x;
  target_pos.y() = msg->pose.position.y;
  target_pos.z() = msg->pose.position.z;

  std::cout << "Goal received: " << target_pos.transpose() << std::endl;
  require_planning = true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "occupancy_mapping");
  ros::NodeHandle nh("~");

  // calling on GridMap class to generate inflated occupancy map
  ros::Time time_1 = ros::Time::now();
  GridMap::Ptr grid_map_;
  grid_map_.reset(new GridMap);
  grid_map_->initMap(nh);

  ros::Duration(1.0).sleep();

  ros::Time time_2 = ros::Time::now();

  // initialize gridmap for A* search
  int map_size_;
  nh.getParam("astar_map_size", map_size_);
  std::cout << "map_size is " << map_size_ << std::endl;
  Eigen::Vector3i map_size{map_size_, map_size_, map_size_}; // what is the resolution of the map? 0.1m should be same as

  ros::Time time_3 = ros::Time::now();

  std::cout << "grid map init used: " << (time_2 - time_1).toSec() << std::endl;
  std::cout << "astar GridNodeMap_ init used: " << (time_3 - time_2).toSec() << std::endl;

  double step_size = grid_map_->getResolution();

  std::shared_ptr<GraphSearch::GraphSearch> gs = std::make_shared<GraphSearch::GraphSearch>(map_size_, step_size, false);
  gs->initMap(grid_map_);

  ros::Subscriber pose_nwu_sub_ = nh.subscribe("/drone0/global_nwu", 10, poseCallback);
  ros::Subscriber target_nwu_sub_ = nh.subscribe("/goal", 10, targetCallback);
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("Astar_node", 0);
  // ros::spin();

  ros::Rate rate(20);

  std::vector<Eigen::Vector3d> path_list;
  while (ros::ok())
  {
    if (require_planning)
    {
      GraphSearch::SearchResult ret = gs->search(current_pos, target_pos);

      switch (ret)
      {
      case GraphSearch::SearchResult::SUCCESS_SUB:
      {
        std::cout << "success" << std::endl;
        break;
      }

      default:
        break;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
}
