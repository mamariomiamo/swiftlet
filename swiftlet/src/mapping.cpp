#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <graph_search.h>
#include <corridor_gen.h>
#include <pcl/conversions.h>
#include <random>
using CorridorGen::Corridor;
// global variable
Eigen::Vector3d current_pos;
Eigen::Vector3d target_pos;
bool require_planning = false;
bool test_jps_;

double resolution = 0.1;
double clearance = 0.2; // radius of drone
int max_sample = 50;
double ceiling = 3.0;
double floor_limit = 1.0;
double goal_pt_margin = 0.2;
std::shared_ptr<CorridorGen::CorridorGenerator> corridor_generator = std::make_shared<CorridorGen::CorridorGenerator>(resolution, clearance, max_sample, ceiling, floor_limit, goal_pt_margin);
pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::random_device dev;
std::mt19937 generator(dev());
std::uniform_real_distribution<double> dis(0.0, 1.0);

geometry_msgs::Point vect2Point(const Eigen::Vector3d &vect)
{
  geometry_msgs::Point point;
  point.x = vect.x();
  point.y = vect.y();
  point.z = vect.z();

  return point;
}

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

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_in, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *local_cloud);
  corridor_generator->updatePointCloud(local_cloud);
}

void publishPointLists(const std::vector<Eigen::Vector3d> &path_list, ros::Publisher *publisher, int color);
void publishPath(const std::vector<Eigen::Vector3d> &path_list, ros::Publisher *publisher);
void deletePrePath(const std::vector<Eigen::Vector3d> &path_list, ros::Publisher *publisher);
void deletePrePointLists(const std::vector<Eigen::Vector3d> &path_list, ros::Publisher *publisher);
void visualizeCorridor(const Corridor &corridor, ros::Publisher *publisher);
void visualizeCorridors(const std::vector<Corridor> &corridors, ros::Publisher *publisher);

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
  ros::Subscriber local_cloud_sub_ = nh.subscribe("/laser_simulator/local_cloud_world", 10, cloudCallback);
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("Astar_node", 100);
  ros::Publisher corridor_center_pub = nh.advertise<visualization_msgs::MarkerArray>("corridor_center", 100);
  ros::Publisher waypt_pub = nh.advertise<visualization_msgs::MarkerArray>("waypts", 100);
  ros::Publisher path_segment_pub = nh.advertise<visualization_msgs::Marker>("initial_path", 100);
  ros::Publisher corridor_vis_pub = nh.advertise<visualization_msgs::Marker>("corridor", 100);
  ros::Publisher corridors_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("corridors", 100);
  // ros::spin();

  ros::Rate rate(20);

  std::vector<Eigen::Vector3d> path_list;
  std::vector<Eigen::Vector3d> prev_path;

  std::vector<Corridor> corridor_list;
  std::vector<Eigen::Vector3d> waypt_list;
  std::vector<Eigen::Vector3d> pre_waypt_list;
  std::vector<Eigen::Vector3d> corridor_center;

  while (ros::ok())
  {
    if (require_planning)
    {
      GraphSearch::SearchResult ret = gs->search(current_pos, target_pos);
      std::cout << "A star finished" << std::endl;

      switch (ret)
      {
      case GraphSearch::SearchResult::SUCCESS_SUB:
      {
        std::cout << "success" << std::endl;
        path_list.clear();
        prev_path.clear();
        waypt_list.clear();
        pre_waypt_list.clear();

        gs->getPath(path_list);
        ros::Time before_gen = ros::Time::now();
        // Corridor test_corridor = corridor_generator->GenerateOneSphere(path_list.front());
        corridor_generator->generateCorridorAlongPath(path_list);
        corridor_list = corridor_generator->getCorridor();
        waypt_list = corridor_generator->getWaypointList();
        corridor_center.clear();
        for (auto corridor : corridor_list)
        {
          corridor_center.push_back(corridor.first);
        }
        // std::for_each(waypt_list.begin(), waypt_list.end(), [](auto waypt)
        //               { std::cout << waypt.transpose() << std::endl; });
        ros::Time after_gen = ros::Time::now();
        std::cout << "gen used: " << (after_gen - before_gen).toSec() << std::endl;
        // std::cout << "position: " << test_corridor.first.transpose() << " radius: " << test_corridor.second << std::endl;
        // std::cout << " size of path list " << path_list.size() << std::endl;
        deletePrePointLists(prev_path, &vis_pub);
        publishPointLists(path_list, &vis_pub, gs->useJPS());
        deletePrePointLists(pre_waypt_list, &waypt_pub);
        publishPointLists(waypt_list, &waypt_pub, true);
        deletePrePath(pre_waypt_list, &path_segment_pub);
        publishPath(waypt_list, &path_segment_pub);
        publishPointLists(corridor_center, &corridor_center_pub, 2);
        // visualizeCorridor(test_corridor, &corridor_vis_pub);
        visualizeCorridors(corridor_list, &corridors_vis_pub);
        prev_path.swap(path_list);
        pre_waypt_list.swap(waypt_list);
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

void visualizeCorridor(const Corridor &corridor, ros::Publisher *publisher)
{
  auto [position, radius] = corridor;
  visualization_msgs::Marker corridor_sphere;
  corridor_sphere.header.frame_id = "map";
  corridor_sphere.header.stamp = ros::Time::now();
  corridor_sphere.id = 0;
  corridor_sphere.type = visualization_msgs::Marker::SPHERE;
  corridor_sphere.action = visualization_msgs::Marker::ADD;
  corridor_sphere.pose.position.x = corridor.first.x();
  corridor_sphere.pose.position.y = corridor.first.y();
  corridor_sphere.pose.position.z = corridor.first.z();
  corridor_sphere.pose.orientation.x = 0.0;
  corridor_sphere.pose.orientation.y = 0.0;
  corridor_sphere.pose.orientation.z = 0.0;
  corridor_sphere.pose.orientation.w = 1.0;
  corridor_sphere.scale.x = radius * 2;
  corridor_sphere.scale.y = radius * 2;
  corridor_sphere.scale.z = radius * 2;
  corridor_sphere.color.a = 0.5; // Don't forget to set the alpha!

  publisher->publish(corridor_sphere);
}

void visualizeCorridors(const std::vector<Corridor> &corridors, ros::Publisher *publisher)
{
  visualization_msgs::MarkerArray corridors_array;
  std::cout << corridors.size() << std::endl;
  corridors_array.markers.reserve(corridors.size());
  int index = 0;
  for (auto corridor : corridors)
  {
    auto [position, radius] = corridor;
    visualization_msgs::Marker corridor_sphere;
    corridor_sphere.header.frame_id = "map";
    corridor_sphere.header.stamp = ros::Time::now();
    corridor_sphere.id = index;
    corridor_sphere.type = visualization_msgs::Marker::SPHERE;
    corridor_sphere.action = visualization_msgs::Marker::ADD;
    corridor_sphere.pose.position.x = position.x();
    corridor_sphere.pose.position.y = position.y();
    corridor_sphere.pose.position.z = position.z();
    corridor_sphere.pose.orientation.x = 0.0;
    corridor_sphere.pose.orientation.y = 0.0;
    corridor_sphere.pose.orientation.z = 0.0;
    corridor_sphere.pose.orientation.w = 1.0;
    corridor_sphere.scale.x = radius * 2;
    corridor_sphere.scale.y = radius * 2;
    corridor_sphere.scale.z = radius * 2;
    corridor_sphere.color.a = 0.2; // Don't forget to set the alpha!
    corridor_sphere.color.r = dis(generator);
    corridor_sphere.color.g = dis(generator);
    corridor_sphere.color.b = dis(generator);

    corridors_array.markers.emplace_back(corridor_sphere);

    // std::cout << "position is: " << position.transpose() << " radius is: " << radius << std::endl;
    index++;
  }

  std::cout << "corridor marker array size is: " << corridors_array.markers.size() << std::endl;

  publisher->publish(corridors_array);
}

void publishPointLists(const std::vector<Eigen::Vector3d> &path_list, ros::Publisher *publisher, int color)
{
  // std::cout << "publishing path" << std::endl;
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
    node.scale.x = 0.05;
    node.scale.y = 0.05;
    node.scale.z = 0.05;
    node.color.a = 1.0; // Don't forget to set the alpha!

    if (color == 1)
    {
      node.color.r = 1.0;
      node.color.g = 0.0;
      node.color.b = 0.0;
    }
    else if (color == 0)
    {
      node.color.r = 0.0;
      node.color.g = 1.0;
      node.color.b = 0.0;
    }
    else
    {
      node.color.r = 1.0;
      node.color.g = 1.0;
      node.color.b = 1.0;
    }

    astar_nodes.markers.emplace_back(node);
  }
  publisher->publish(astar_nodes);
}

void deletePrePointLists(const std::vector<Eigen::Vector3d> &path_list, ros::Publisher *publisher)
{
  // std::cout << "publishing path" << std::endl;
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

void publishPath(const std::vector<Eigen::Vector3d> &path_list, ros::Publisher *publisher)
{
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.scale.x = 0.05;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  line_strip.points.clear();

  for (int i = 0; i < path_list.size(); i++)
  {
    geometry_msgs::Point p;
    p = vect2Point(path_list[i]);
    line_strip.points.push_back(p);
  }

  publisher->publish(line_strip);
}

void deletePrePath(const std::vector<Eigen::Vector3d> &path_list, ros::Publisher *publisher)
{
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::Marker::DELETE;
  line_strip.scale.x = 0.5;
  line_strip.color.r = 1.0;
  line_strip.color.a = 1.0;

  line_strip.points.clear();

  for (int i = 0; i < path_list.size(); i++)
  {
    line_strip.points.push_back(vect2Point(path_list[i]));
  }

  publisher->publish(line_strip);
}