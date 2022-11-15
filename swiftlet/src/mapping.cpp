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
bool test_jps_;

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

void publishPath(const std::vector<Eigen::Vector3d> &path_list, auto publisher, bool use_jps);
void deletePrePath(const std::vector<Eigen::Vector3d> &path_list, auto publisher);

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
  nh.getParam("test_jps", test_jps_);
  std::cout << "map_size is " << map_size_ << std::endl;
  Eigen::Vector3i map_size{map_size_, map_size_, map_size_}; // what is the resolution of the map? 0.1m should be same as

  ros::Time time_3 = ros::Time::now();

  std::cout << "grid map init used: " << (time_2 - time_1).toSec() << std::endl;
  std::cout << "astar GridNodeMap_ init used: " << (time_3 - time_2).toSec() << std::endl;

  double step_size = grid_map_->getResolution();
  std::cout << "step size is " << step_size << std::endl;

  // instantiate graphsearch object
  std::shared_ptr<GraphSearch::GraphSearch> gs = std::make_shared<GraphSearch::GraphSearch>(map_size_, step_size, false);
  gs->initMap(grid_map_);

  std::shared_ptr<GraphSearch::JPS> jps = std::make_shared<GraphSearch::JPS>(map_size_, step_size, true);
  jps->initMap(grid_map_);

  ros::Subscriber pose_nwu_sub_ = nh.subscribe("/drone0/global_nwu", 10, poseCallback);
  ros::Subscriber target_nwu_sub_ = nh.subscribe("/goal", 10, targetCallback);
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("Astar_node", 0);
  // ros::spin();

  ros::Rate rate(20);

  std::vector<Eigen::Vector3d> path_list;
  std::vector<Eigen::Vector3d> prev_path;
  std::vector<Eigen::Vector3d> path_list_jps;
  std::vector<Eigen::Vector3d> prev_path_jps;
  while (ros::ok())
  {
    if (require_planning)
    {
      GraphSearch::SearchResult ret1 = gs->search(current_pos, target_pos);
      std::cout << "A star finished" << std::endl;
      // GraphSearch::SearchResult ret1 = jps->search(current_pos, target_pos);

      // switch (ret)
      // {
      // case GraphSearch::SearchResult::SUCCESS_SUB:
      // {
      //   std::cout << "success" << std::endl;
      //   path_list.clear();

      //   gs->getPath(path_list);
      //   // std::cout << " size of path list " << path_list.size() << std::endl;
      //   deletePrePath(prev_path, &vis_pub);
      //   publishPath(path_list, &vis_pub, gs->useJPS());
      //   prev_path.swap(path_list);
      //   require_planning = false;
      //   break;
      // }

      // default:
      //   break;
      // }

      switch (ret1)
      {
      case GraphSearch::SearchResult::SUCCESS_SUB:
      {
        std::cout << "JPS success" << std::endl;
        path_list_jps.clear();

        gs->getPath(path_list_jps);
        // std::cout << " size of path list " << path_list.size() << std::endl;
        deletePrePath(prev_path_jps, &vis_pub);
        publishPath(path_list_jps, &vis_pub, gs->useJPS());
        prev_path_jps.swap(path_list_jps);
        require_planning = false;
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

void publishPath(const std::vector<Eigen::Vector3d> &path_list, auto publisher, bool use_jps)
{
  std::cout << "publishing path" << std::endl;
  visualization_msgs::MarkerArray astar_nodes;
  int array_size = path_list.size();
  astar_nodes.markers.clear();
  // astar_nodes.markers.resize(array_size);
  // path_list.clear();
  // path_list.emplace_back(current_pos);
  // path_list.emplace_back(target_pos);

  for (int i = 0; i < array_size; i++)
  {
    visualization_msgs::Marker node;
    node.header.frame_id = "map";
    node.header.stamp = ros::Time::now();
    node.id = i;
    node.type = visualization_msgs::Marker::SPHERE;
    node.action = visualization_msgs::Marker::ADD;
    node.pose.position.x = path_list[i].x();
    node.pose.position.y = path_list[i].y();
    node.pose.position.z = path_list[i].z();
    node.pose.orientation.x = 0.0;
    node.pose.orientation.y = 0.0;
    node.pose.orientation.z = 0.0;
    node.pose.orientation.w = 1.0;
    node.scale.x = 0.1;
    node.scale.y = 0.1;
    node.scale.z = 0.1;
    node.color.a = 1.0; // Don't forget to set the alpha!

    if (use_jps)
    {
      node.color.r = 1.0;
      node.color.g = 0.0;
      node.color.b = 0.0;
    }
    else
    {
      node.color.r = 0.0;
      node.color.g = 1.0;
      node.color.b = 0.0;
    }

    astar_nodes.markers.emplace_back(node);
  }
  publisher->publish(astar_nodes);
}

void deletePrePath(const std::vector<Eigen::Vector3d> &path_list, auto publisher)
{
  std::cout << "publishing path" << std::endl;
  visualization_msgs::MarkerArray astar_nodes;
  int array_size = path_list.size();
  astar_nodes.markers.clear();
  // astar_nodes.markers.resize(array_size);
  // path_list.clear();
  // path_list.emplace_back(current_pos);
  // path_list.emplace_back(target_pos);

  for (int i = 0; i < array_size; i++)
  {
    visualization_msgs::Marker node;
    node.header.frame_id = "map";
    node.header.stamp = ros::Time::now();
    node.id = i;
    node.type = visualization_msgs::Marker::SPHERE;
    node.action = visualization_msgs::Marker::DELETE;
    node.pose.position.x = path_list[i].x();
    node.pose.position.y = path_list[i].y();
    node.pose.position.z = path_list[i].z();
    node.pose.orientation.x = 0.0;
    node.pose.orientation.y = 0.0;
    node.pose.orientation.z = 0.0;
    node.pose.orientation.w = 1.0;
    node.scale.x = 0.1;
    node.scale.y = 0.1;
    node.scale.z = 0.1;
    node.color.a = 1.0; // Don't forget to set the alpha!
    node.color.r = 0.0;
    node.color.g = 1.0;
    node.color.b = 0.0;
    astar_nodes.markers.emplace_back(node);
  }
  publisher->publish(astar_nodes);
}