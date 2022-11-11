#include <graph_search.h>
namespace GraphSearch
{

    GraphSearch::GraphSearch(int map_size, int map_resolution, bool use_jps)
        : map_resolution_(map_resolution), use_jps_(use_jps)
    {
        map_resolution_inverse_ = 1 / map_resolution_;
        map_size_ << map_size, map_size, map_size;
        pool_size_ = map_size_;
        center_index_ = pool_size_ / 2;
        current_search_round_ = 0;
        initGridMap();
    }

    GraphSearch::~GraphSearch()
    {
        for (auto grid : GridMap_)
        {
            delete grid;
        }
    }

    // void GraphSearch::initMap(const auto &map_obj)
    void GraphSearch::initMap(const GridMap::Ptr &map_obj)
    {
        occupancy_map_ = map_obj;
    }

    void GraphSearch::initGridMap()
    {
        int capacity = pool_size_.x() * pool_size_.y() * pool_size_.z();
        GridMap_.reserve(capacity);
        for (int i = 0; i < capacity; i++)
        {
            GridMap_.emplace_back(new GridNode);
        }
    }

    void GraphSearch::getPath(std::vector<Eigen::Vector3d> &path_list)
    {
        GridNodePtr temp = ret_node_;
        path_list.emplace_back(Index2Coord(temp->index));

        while (temp->parent != nullptr)
        {
            temp = temp->parent;
            path_list.emplace_back(Index2Coord(temp->index));
        }
        reverse(path_list.begin(), path_list.end());
    }

    SearchResult GraphSearch::search(const Eigen::Vector3d &start, const Eigen::Vector3d &goal)
    {
        ++current_search_round_;

        // Check if goal is out of sensing range
        if ((goal - start).norm() > 10)
        {
            ROS_WARN("Goal is outside sensing range, will need to adjust");
            sub_goal_ = start + (goal - start) / (goal - start).norm() * 9.5;
        }

        else
        {
            sub_goal_ = goal;
        }

        center_ = (sub_goal_ - goal) / 2;

        Eigen::Vector3i start_index, goal_index;
        if (!ConvertToIndexAndAdjustStartEndPoints(start, sub_goal_, start_index, goal_index))
        {
            ROS_ERROR("Unable to handle the initial or end point, force return!");
            return SearchResult::INIT_ERR;
        }

        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, GridNodeComparator> empty;
        pq.swap(empty);

        GridNodePtr start_node = GridMap_[voxelIndex2vectIndex(start_index)];
        GridNodePtr goal_node = GridMap_[voxelIndex2vectIndex(goal_index)];
        start_node->index = start_index;
        goal_node->index = goal_index;
        start_node->g_cost = 0;
        start_node->h_cost = getHeuristic(start_node, goal_node);
        start_node->f_cost = start_node->h_cost;
        start_node->parent = nullptr;
        start_node->state = NodeState::OPENLIST;
        pq.push(start_node);

        GridNodePtr current_node = nullptr;
        ros::Time time_1 = ros::Time::now();
        while (!pq.empty())
        {
            current_node = pq.top();
            pq.pop();
            if (current_node->index.x() == goal_node->index.x() &&
                current_node->index.y() == goal_node->index.y() &&
                current_node->index.z() == goal_node->index.z())
            {
                ros::Time time_2 = ros::Time::now();
                double time_lapse = (time_2 - time_1).toSec();
                std::cout << "path found in: " << time_lapse << " seconds." << std::endl;
                ret_node_ = goal_node;
                return SearchResult::SUCCESS_SUB;
            }

            current_node->state = NodeState::CLOSELIST;
            getSuccessorNode(current_node, goal_node);

            ros::Time time_2 = ros::Time::now();
            double time_lapse = (time_2 - time_1).toSec();
            if (time_lapse > 0.5)
            {
                std::cout << "A star search used too long: " << time_lapse << std::endl;
                return SearchResult::FAILURE;
            }
        }
    }

        bool GraphSearch::ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx)
    {
        std::cout << "start point: " << start_pt.transpose() << std::endl;
        std::cout << "end_pt: " << end_pt.transpose() << std::endl;
        if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
            return false;

        int occ;
        if (checkOccupancy(Index2Coord(start_idx)))
        {
            ROS_WARN("Start point is insdide an obstacle.");
            do
            {
                start_pt = (start_pt - end_pt).normalized() * map_resolution_ + start_pt; // vector from end to start
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
                end_pt = (end_pt - start_pt).normalized() * map_resolution_ + end_pt; // vector from start to end
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

    void GraphSearch::getSuccessorNode(const GridNodePtr &current_node, const GridNodePtr &goal_node)
    {
        GridNodePtr neighbor_node = nullptr;
        Eigen::Vector3i neighbor_index;
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0) // ego node
                    {
                        continue;
                    }

                    neighbor_index(0) = (current_node->index)(0) + dx;
                    neighbor_index(1) = (current_node->index)(1) + dy;
                    neighbor_index(2) = (current_node->index)(2) + dz;

                    if (neighbor_index(0) < 1 || neighbor_index(0) >= pool_size_(0) - 1 || neighbor_index(1) < 1 || neighbor_index(1) >= pool_size_(1) - 1 || neighbor_index(2) < 1 || neighbor_index(2) >= pool_size_(2) - 1)
                    {
                        continue;
                    }

                    neighbor_node = GridMap_[voxelIndex2vectIndex(neighbor_index)];
                    neighbor_node->index = neighbor_index;

                    // check if the node has been queried in this round
                    bool explored = neighbor_node->query_rounds == current_search_round_;

                    // if the node has been queried AND node is in close list, we skip it
                    if (explored && neighbor_node->state == NodeState::CLOSELIST)
                    {
                        continue;
                    }

                    // update node's query round
                    neighbor_node->query_rounds = current_search_round_;

                    // check if node is occupied
                    if (checkOccupancy(Index2Coord(neighbor_node->index)))
                    {
                        continue;
                    }

                    double edge_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    double neighbor_g_temp = current_node->g_cost + edge_cost;

                    if (!explored)
                    {
                        neighbor_node->parent = current_node;
                        neighbor_node->h_cost = getHeuristic(neighbor_node, goal_node);
                        neighbor_node->g_cost = neighbor_g_temp;
                        neighbor_node->f_cost = neighbor_g_temp + neighbor_node->h_cost;
                        neighbor_node->state = NodeState::OPENLIST;
                        pq.push(neighbor_node);
                    }

                    else if (neighbor_node->g_cost > neighbor_g_temp)
                    {
                        // check if the g_cost will be lower if we choose current_node as parent of neighbor_node
                        neighbor_node->parent = current_node;
                        neighbor_node->g_cost = neighbor_g_temp;
                        neighbor_node->f_cost = neighbor_g_temp + neighbor_node->h_cost;
                    }
                }
    }

}; // namespace GraphSearch