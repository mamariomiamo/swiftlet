#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
enum ASTAR_RET
{
  SUCCESS,
  INIT_ERR,
  SEARCH_ERR
};

struct GridNode;
constexpr double inf = 1 >> 20;
typedef GridNode *GridNodePtr;
struct GridNode
{
  Eigen::Vector3i index; // grid map index
  GridNode *parent{nullptr};
  enum node_state
  {
    OPENLIST = 1,
    CLOSELIST = 2,
    UNEXPLORED = 3,
  };

  node_state state{UNEXPLORED};

  double rounds;

  double cost_so_far{inf};          // running cost
  double total_cost_estimated{inf}; // estimate cost to go, must be admissible
                                    // i.e. means it should be less than actual cost to goal
};

class NodeComparator // such that node with lower cost (closer to goal) will be infront
{
public:
  bool operator()(GridNodePtr node1, GridNodePtr node2)
  {
    return node1->total_cost_estimated > node2->total_cost_estimated;
  }
};

// global variable
Eigen::Vector3d current_pos;
Eigen::Vector3d target_pos;
GridNodePtr ***GridNodeMap_;
Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
GridMap::Ptr astar_grid_map_;
double step_size_;
double inv_step_size_;
Eigen::Vector3d center_; // center point of start and end positions
bool require_planning = false;
std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;
const double tie_breaker_ = 1.0 + 1.0 / 10000;
int rounds_{0};

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

void initGridMap(GridMap::Ptr grid_map, const Eigen::Vector3i &map_size);

ASTAR_RET AStarSearch(const double step_size, const Eigen::Vector3d &start, const Eigen::Vector3d &goal);

bool ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx);

Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index);

int checkOccupancy(const Eigen::Vector3d &pos);

double getDiagHeu(GridNodePtr node1, GridNodePtr node2);

double getHeu(GridNodePtr node1, GridNodePtr node2);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "occupancy_mapping");
  ros::NodeHandle nh("~");

  // calling on GridMap class to generate inflated occupancy map
  GridMap::Ptr grid_map_;
  grid_map_.reset(new GridMap);
  grid_map_->initMap(nh);
  ros::Duration(1.0).sleep();

  // initialize gridmap for A* search
  Eigen::Vector3i map_size{300, 300, 300}; // what is the resolution of the map?
  initGridMap(grid_map_, map_size);

  double step_size = grid_map_->getResolution();

  ros::Subscriber pose_nwu_sub_ = nh.subscribe("/drone0/global_nwu", 10, poseCallback);
  ros::Subscriber target_nwu_sub_ = nh.subscribe("/goal", 10, targetCallback);
  // ros::spin();

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();
    status = ros::ok();
    if (require_planning)
    {
      ros::Time time_1 = ros::Time::now();
      ASTAR_RET ret = AStarSearch(step_size, current_pos, target_pos);
      ros::Time time_2 = ros::Time::now();

      require_planning = false;

      if (ret == ASTAR_RET::SUCCESS)
      {
        std::cout << " Search Success " << std::endl;
        std::cout << "Used: " << (time_2 - time_1).toSec() << " seconds" << std::endl;
      }

      else if (ret == ASTAR_RET::SEARCH_ERR)
      {
        std::cout << " Search Error " << std::endl;
      }
    }
    rate.sleep();
  }

  return 0;
}

void initGridMap(GridMap::Ptr grid_map, const Eigen::Vector3i &map_size)
{
  POOL_SIZE_ = map_size;
  CENTER_IDX_ = map_size / 2;

  GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
  for (int i = 0; i < POOL_SIZE_(0); i++)
  {
    GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
    for (int j = 0; j < POOL_SIZE_(1); j++)
    {
      GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
      for (int k = 0; k < POOL_SIZE_(2); k++)
      {
        GridNodeMap_[i][j][k] = new GridNode;
      }
    }
  }

  astar_grid_map_ = grid_map;
}

ASTAR_RET AStarSearch(const double step_size, const Eigen::Vector3d &start, const Eigen::Vector3d &goal)
{
  ros::Time time_1 = ros::Time::now();
  std::cout << "Conducting search" << std::endl;
  rounds_ = rounds_ + 1;
  step_size_ = step_size;
  inv_step_size_ = 1 / step_size;
  center_ = (start + goal) / 2;

  Eigen::Vector3i start_index, goal_index;
  Eigen::Vector3i start_index_temp, goal_index_temp;
  Coord2Index(start, start_index_temp);
  Coord2Index(goal, goal_index_temp);

  std::cout << "start_index_temp " << start_index_temp.transpose() << std::endl;
  std::cout << "goal_index_temp " << goal_index_temp.transpose() << std::endl;

  if (!ConvertToIndexAndAdjustStartEndPoints(start, goal, start_index, goal_index))
  {
    ROS_ERROR("Unable to handle the initial or end point, force return!");
    return ASTAR_RET::INIT_ERR;
  }

  GridNodePtr startPtr = GridNodeMap_[start_index(0)][start_index(1)][start_index(2)];
  GridNodePtr endPtr = GridNodeMap_[goal_index(0)][goal_index(1)][goal_index(2)];

  std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
  openSet_.swap(empty);
  GridNodePtr neighborPtr = nullptr;
  GridNodePtr current = nullptr;
  endPtr->index = goal_index;

  startPtr->index = start_index;
  startPtr->cost_so_far = 0;
  startPtr->rounds = rounds_;
  startPtr->total_cost_estimated = getHeu(startPtr, endPtr);
  startPtr->state = GridNode::OPENLIST;
  startPtr->parent = nullptr;
  openSet_.push(startPtr);

  double cost_so_far_temp;
  int num_iter = 0;

  while (!openSet_.empty())
  {
    num_iter++;
    current = openSet_.top();
    openSet_.pop();
    if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
    {
      // ros::Time time_2 = ros::Time::now();
      // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
      // if((time_2 - time_1).toSec() > 0.1)
      //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
      // gridPath_ = retrievePath(current);
      return ASTAR_RET::SUCCESS;
    }

    current->state = GridNode::CLOSELIST;

    for (int dx = -1; dx <= 1; dx++)
      for (int dy = -1; dy <= 1; dy++)
        for (int dz = -1; dz <= 1; dz++)
        {
          if (dx == 0 && dy == 0 && dz == 0)
            continue;

          Eigen::Vector3i neighborIdx;
          neighborIdx = current->index + Eigen::Vector3i(dx, dy, dz);

          if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
          {
            continue;
          }

          neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
          neighborPtr->index = neighborIdx;

          bool explored = (neighborPtr->rounds == rounds_);
          if (explored && neighborPtr->state == GridNode::CLOSELIST)
          {
            continue; // we have explored this neighbor and it is also been expanded
          }

          neighborPtr->rounds == rounds_;

          // check if neighborPtr is occupied
          if (checkOccupancy(Index2Coord(neighborPtr->index)))
          {
            continue;
          }

          double edge_cost = sqrt(dx * dx + dy * dy + dz * dz);
          cost_so_far_temp = current->cost_so_far + edge_cost;

          if (!explored)
          {
            // this is a new node, we need to:
            // 1. update its cost_so_far
            // 2. add to openlist
            // 3. update its parent node
            // 4. push to priority queue
            //
            neighborPtr->cost_so_far = cost_so_far_temp;
            neighborPtr->total_cost_estimated = cost_so_far_temp + getHeu(neighborPtr, endPtr);
            neighborPtr->parent = current;
            neighborPtr->state = GridNode::OPENLIST;
            openSet_.push(neighborPtr);
          }

          else if (cost_so_far_temp < neighborPtr->cost_so_far)
          {
            // update the node with a shorter parent
            neighborPtr->parent = current;
            neighborPtr->cost_so_far = cost_so_far_temp;
            neighborPtr->total_cost_estimated = cost_so_far_temp + getHeu(neighborPtr, endPtr);
          }
        }
    ros::Time time_2 = ros::Time::now();
    // if ((time_2 - time_1).toSec() > 0.2)
    // {
    //   ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
    //   return ASTAR_RET::SEARCH_ERR;
    // }
  }

  ros::Time time_2 = ros::Time::now();

  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

  return ASTAR_RET::SEARCH_ERR;
}

bool ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx)
{
  if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
    return false;

  int occ;
  if (checkOccupancy(Index2Coord(start_idx)))
  {
    ROS_WARN("Start point is insdide an obstacle.");
    do
    {
      start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
      // cout << "start_pt=" << start_pt.transpose() << endl;
      if (!Coord2Index(start_pt, start_idx))
      {
        return false;
      }

      occ = checkOccupancy(Index2Coord(start_idx));
      if (occ == -1)
      {
        ROS_WARN("[Astar] Start point outside the map region.");
        return false;
      }
    } while (occ);
  }

  if (checkOccupancy(Index2Coord(end_idx)))
  {
    ROS_WARN("End point is insdide an obstacle.");
    do
    {
      end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
      // cout << "end_pt=" << end_pt.transpose() << endl;
      if (!Coord2Index(end_pt, end_idx))
      {
        return false;
      }

      occ = checkOccupancy(Index2Coord(start_idx));
      if (occ == -1)
      {
        ROS_WARN("[Astar] End point outside the map region.");
        return false;
      }
    } while (checkOccupancy(Index2Coord(end_idx)));
  }

  return true;
}

bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx)
{
  idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

  if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
  {
    ROS_ERROR("center in dex is =%d %d %d", CENTER_IDX_(0), CENTER_IDX_(1), CENTER_IDX_(2));
    ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
    ROS_ERROR("Position %d %d %d", pt(0), pt(1), pt(2));
    return false;
  }

  return true;
}

Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index)
{
  return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
}

int checkOccupancy(const Eigen::Vector3d &pos) { return astar_grid_map_->getInflateOccupancy(pos); }

double getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
  double dx = abs(node1->index(0) - node2->index(0));
  double dy = abs(node1->index(1) - node2->index(1));
  double dz = abs(node1->index(2) - node2->index(2));

  double h = 0.0;
  int diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx == 0)
  {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy == 0)
  {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz == 0)
  {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return h;
}

double getHeu(GridNodePtr node1, GridNodePtr node2)
{
  return tie_breaker_ * getDiagHeu(node1, node2);
}