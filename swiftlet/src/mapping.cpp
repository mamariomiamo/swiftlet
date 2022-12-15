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
#include <thread>
#include <mutex>

using CorridorGen::Corridor;
// global variable
Eigen::Vector3d current_pos;
Eigen::Vector3d target_pos;
bool require_planning = false;
bool test_jps_;
bool replanning_trigger = false;

Eigen::Vector3d previous_replan_start_pt;

ros::Publisher position_setpoint_nwu_pub;

std::mutex global_waypoint_list_mutex;

double resolution = 0.1;
double clearance = 0.2; // radius of drone
int max_sample = 100;
double ceiling = 3.0;
double floor_limit = 1.0;
double goal_pt_margin = 0.2;
std::shared_ptr<CorridorGen::CorridorGenerator> corridor_generator = std::make_shared<CorridorGen::CorridorGenerator>(resolution, clearance, max_sample, ceiling, floor_limit, goal_pt_margin);
pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::random_device dev;
std::mt19937 generator(dev());
std::uniform_real_distribution<double> dis(0.0, 1.0);
std::vector<Eigen::Vector3d> global_waypt_list; // this is to store waypts for all iteration, will be updated with pop_back after passing waypts
Eigen::Vector3d current_waypt_command, previous_waypt_command;
bool first_time_send = true;
bool first_plan_ = true;
bool init_ = false;
bool navigation_started_ = false;
bool receive_new_goal_ = false;

geometry_msgs::Point vect2Point(const Eigen::Vector3d &vect)
{
  geometry_msgs::Point point;
  point.x = vect.x();
  point.y = vect.y();
  point.z = vect.z();

  return point;
}

void waypt_commander_sender_callback(const ros::TimerEvent &)
{
  std::lock_guard<std::mutex> guard(global_waypoint_list_mutex);
  // std::cout << "waypoint send\n";

  // if(!navigation_started_)
  if (global_waypt_list.empty())
  {
    return;
  }

  else
  {
    navigation_started_ = true;

    current_waypt_command = global_waypt_list.back();

    double distance_to_waypoint = (current_pos - current_waypt_command).norm();
    if (distance_to_waypoint < 0.3) // we are reaching the waypoint
    {
      // std::cout << "global_waypt_list size " << global_waypt_list.size() << "\n";
      if (global_waypt_list.size() > 1)
      {
        global_waypt_list.pop_back();
        current_waypt_command = global_waypt_list.back();
      }
    }

    geometry_msgs::Point waypoint = vect2Point(current_waypt_command);
    geometry_msgs::PoseStamped pos_sp;
    pos_sp.header.stamp = ros::Time::now();
    pos_sp.header.frame_id = "map";
    pos_sp.pose.position = waypoint;
    pos_sp.pose.orientation.w = 1;
    pos_sp.pose.orientation.x = 0;
    pos_sp.pose.orientation.y = 0;
    pos_sp.pose.orientation.z = 0;
    position_setpoint_nwu_pub.publish(pos_sp);
  }
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  current_pos.x() = msg->pose.position.x;
  current_pos.y() = msg->pose.position.y;
  current_pos.z() = msg->pose.position.z;

  if (!init_)
  {
    previous_replan_start_pt = current_pos;
    target_pos = current_pos;
    init_ = true;
  }

  // two scenarios to trigger replanning
  // (1) if the global_waypt_list size is 1 and the last one is far from the target
  // (2) if we are some distance away from last replan start point
  // OR if the path we plan is in collision with obstacles

  if (navigation_started_) // navigation_started_ will be set when UAV first plan a path and send target
  {
    if (global_waypt_list.size() < 2 && (global_waypt_list.back() - target_pos).norm() > 1)
    {
      std::cout << "left one waypt to go in global_waypt_list and it is far from target_pos, replan trigger\n";
      std::cout << "target_pos is " << target_pos.transpose() << std::endl;
      replanning_trigger = true;
      return;
    }

    else if ((previous_replan_start_pt - current_pos).norm() > 3) // we are some distance away from last replan point
    {

      // std::cout << "global_waypt_list.front() " << global_waypt_list.front().transpose() << std::endl;
      // std::cout << "target_pos " << target_pos.transpose() << std::endl;

      // we will not trigger replan if the target_pos is already in the global_waypt_list to be sent
      if ((current_pos - target_pos).norm() < 0.2)
      {
        replanning_trigger = false;
        // std::cout << "target_pos is already in the global_waypt_list" << std::endl;
      }

      // target_pos is far away and we will need to trigger replan
      else
      {
        std::cout << "far from last replan point, replan triggered\n";
        replanning_trigger = true;
        return;
      }
    }

    else // we have enought waypoints in global_waypt_list and we are not far away from last replan point
         // i.e. half way executing the waypoint list
    {
      // std::cout << "far from last replan point, replan triggered\n";
      replanning_trigger = false;
    }
  }

  else
  {
    replanning_trigger = false;
    return;
  }

  // else
  // {
  //   require_planning = false;
  // }
}

void targetCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  receive_new_goal_ = true;
  target_pos.x() = msg->pose.position.x;
  target_pos.y() = msg->pose.position.y;
  target_pos.z() = msg->pose.position.z;
  if(target_pos.z() > ceiling)
  {
    target_pos.z() = ceiling/2;
  }

  std::cout << "Goal received: " << target_pos.transpose() << std::endl;

  if (first_plan_)
  {
    std::cout << "first target received\n";
    require_planning = true;
    first_plan_ = false;
  }

  // navigation_started_ = true;
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
void visualizeSampleDirection(const auto sample_direction, ros::Publisher *publisher);

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
  nh.getParam("ceiling", ceiling);
  std::cout << "map_size is " << map_size_ << std::endl;
  Eigen::Vector3i map_size{map_size_, map_size_, map_size_}; // what is the resolution of the map? 0.1m should be same as

  ros::Time time_3 = ros::Time::now();

  std::cout << "grid map init used: " << (time_2 - time_1).toSec() << std::endl;
  std::cout << "astar GridNodeMap_ init used: " << (time_3 - time_2).toSec() << std::endl;

  double step_size = grid_map_->getResolution();
  std::cout << "step size is " << step_size << std::endl;

  // instantiate graphsearch object
  std::shared_ptr<GraphSearch::GraphSearch> gs = std::make_shared<GraphSearch::GraphSearch>(map_size_, step_size, false, true);
  gs->setCeiling(ceiling);
  gs->initMap(grid_map_);

  std::shared_ptr<GraphSearch::JPS> jps = std::make_shared<GraphSearch::JPS>(map_size_, step_size, true);
  jps->initMap(grid_map_);

  ros::Subscriber pose_nwu_sub_ = nh.subscribe("/drone0/global_nwu", 10, poseCallback);
  ros::Subscriber target_nwu_sub_ = nh.subscribe("/goal", 10, targetCallback);
  ros::Subscriber local_cloud_sub_ = nh.subscribe("/local_pointcloud", 10, cloudCallback);
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("Astar_node", 100);
  ros::Publisher corridor_center_pub = nh.advertise<visualization_msgs::MarkerArray>("corridor_center", 100);
  ros::Publisher waypt_pub = nh.advertise<visualization_msgs::MarkerArray>("waypts", 100);
  ros::Publisher path_segment_pub = nh.advertise<visualization_msgs::Marker>("initial_path", 100);
  ros::Publisher corridor_vis_pub = nh.advertise<visualization_msgs::Marker>("corridor", 100);
  ros::Publisher corridors_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("corridors", 100);
  ros::Publisher sample_direction_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("sample_direction", 100);

  position_setpoint_nwu_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav/ref_pose/nwu", 100);

  // boost::function<void(ros::Timer & timer_event, ros::Publisher * publisher)> timer_callback = [&position_setpoint_nwu_pub](ros::Timer &timer_event, ros::Publisher *publisher)
  // {
  //   waypt_commander_sender_callback(timer_event, publisher);
  // };

  ros::Timer waypt_command_sender = nh.createTimer(ros::Duration(0.01), waypt_commander_sender_callback);
  // ros::spin();

  ros::Rate rate(20);

  std::vector<Eigen::Vector3d> path_list;
  std::vector<Eigen::Vector3d> prev_path;

  std::vector<Corridor> corridor_list;
  std::vector<Eigen::Vector3d> waypt_list;         // this is to store waypts found for one iteration
  std::vector<Eigen::Vector3d> waypt_list_reverse; // waypt_list in reverse order
  std::vector<Eigen::Vector3d> pre_waypt_list;
  std::vector<Eigen::Vector3d> corridor_center;

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> sample_direction;

  Eigen::Vector3d replanning_start;

  GraphSearch::SearchResult ret;

  int waypoint_completion_counter = 0;

  bool first_plan = true;
  while (ros::ok())
  {

    if (receive_new_goal_ || replanning_trigger)
    {
      if (receive_new_goal_)
      {
        std::lock_guard<std::mutex> guard(global_waypoint_list_mutex);
        // Get replanning start point
        if (global_waypt_list.empty()) // first time plan
        {
          replanning_start = previous_replan_start_pt; // current hovering position
          std::cout << "first time plan: replanning_start " << replanning_start.transpose() << "\n";
        }

        else
        {
          replanning_start = global_waypt_list.back();
          previous_replan_start_pt = replanning_start;
        }

        ret = gs->search(replanning_start, target_pos);

        receive_new_goal_ = false;
      }

      else if (replanning_trigger)
      {
        // replanning is triggered
        {
          std::lock_guard<std::mutex> guard(global_waypoint_list_mutex);
          // Get replanning start point
          replanning_start = global_waypt_list.back();
          previous_replan_start_pt = replanning_start;
          ret = gs->search(replanning_start, target_pos);

          replanning_trigger = false;
        }
      }
      // GraphSearch::SearchResult ret = gs->search(current_pos, target_pos);
      // std::cout << "A star returned" << std::endl;

      switch (ret)
      {

      case GraphSearch::SearchResult::SUCCESS_SUB:
      {
        std::cout << "Search path to sub-goal is successful" << std::endl;
        path_list.clear();
        prev_path.clear();
        waypt_list.clear();
        pre_waypt_list.clear();

        gs->getPath(path_list);
        ros::Time before_gen = ros::Time::now();
        // Corridor test_corridor = corridor_generator->GenerateOneSphere(path_list.front());
        bool generate_success = corridor_generator->generateCorridorAlongPath(path_list);
        corridor_list = corridor_generator->getCorridor();
        waypt_list = corridor_generator->getWaypointList();
        waypt_list_reverse = waypt_list;
        std::reverse(waypt_list_reverse.begin(), waypt_list_reverse.end());
        global_waypoint_list_mutex.lock();
        if (global_waypt_list.empty()) // first iteration
        {
          // global_waypt_list.reserve(waypt_list_reverse.size());
          std::cout << "first iteration: global_waypt_list is empty " << std::endl;
          global_waypt_list.insert(global_waypt_list.end(), waypt_list_reverse.begin(), waypt_list_reverse.end());
        }

        else
        {
          // global_waypt_list.reserve(waypt_list_reverse.size() + global_waypt_list.size());
          // global_waypt_list.insert(global_waypt_list.end(), waypt_list_reverse.begin(), waypt_list_reverse.end());
          global_waypt_list.clear();
          global_waypt_list = waypt_list_reverse;
        }

        // std::for_each(waypt_list.begin(), waypt_list.end(), [](auto val)
        //               { std::cout << val.transpose() << "\n"; });
        global_waypoint_list_mutex.unlock();

        sample_direction = corridor_generator->getSampleDirection();
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
        visualizeSampleDirection(sample_direction, &sample_direction_vis_pub);
        // visualizeCorridor(test_corridor, &corridor_vis_pub);
        visualizeCorridors(corridor_list, &corridors_vis_pub);
        prev_path.swap(path_list);
        pre_waypt_list.swap(waypt_list);
        // require_planning = false;
        ret = GraphSearch::SearchResult::SUCCESS;
        break;
      }

      case GraphSearch::SearchResult::SUCCESS:
      {

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
  // std::cout << corridors.size() << std::endl;
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

  // std::cout << "corridor marker array size is: " << corridors_array.markers.size() << std::endl;

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

void visualizeSampleDirection(const auto sample_direction, ros::Publisher *publisher)
{
  // std::cout << "publishing path" << std::endl;
  visualization_msgs::MarkerArray direction_list;
  int array_size = sample_direction.size();
  direction_list.markers.clear();
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
    node.type = visualization_msgs::Marker::ARROW;
    node.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point from, to;
    from = vect2Point(sample_direction[i].first);
    to = vect2Point(sample_direction[i].second);

    node.points.push_back(from);
    node.points.push_back(to);
    node.scale.x = 0.03;
    node.scale.y = 0.08;
    node.scale.z = 0.05;
    node.color.a = 1.0; // Don't forget to set the alpha!

    node.color.r = 1.0;
    node.color.g = 0.0;
    node.color.b = 0.0;

    direction_list.markers.emplace_back(node);
  }
  publisher->publish(direction_list);
}