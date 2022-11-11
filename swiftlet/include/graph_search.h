#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <queue>
#include <plan_env/grid_map.h>

namespace GraphSearch
{
    struct GridNode;
    typedef GridNode *GridNodePtr;
    constexpr double inf = 1 >> 20;

    enum NodeState
    {
        OPENLIST = 1,
        CLOSELIST,
        UNDEFINED
    };

    enum SearchResult
    {
        SUCCESS = 1,
        SUCCESS_SUB,
        SEARCH_ERR,
        INIT_ERR,
        FAILURE,
        TIMEOUT
    };

    struct GridNode
    {
        Eigen::Vector3i index; // node index

        double g_cost{inf}; // cost from start
        double h_cost{inf}; // heuristic cost
        double f_cost{inf}; // total cost i.e. f_cost = g_cost + h_cost

        GridNodePtr parent{nullptr};

        NodeState state{UNDEFINED};
        int query_rounds{0}; // previous round of query of this node
                             // to be compared with GraphSearch::rounds_
                             // Every call of GraphSearch(start, end, ...)
    };

    // custom comparator for GridNodePtr priority_queue
    class GridNodeComparator
    {
    public:
        bool operator()(GridNodePtr node1, GridNodePtr node2)
        {
            return node1->f_cost > node2->f_cost;
        }
    };

    // GraphSearch class
    class GraphSearch
    {
    public:
        GraphSearch(int map_size, int map_resolution, bool use_jps);
        ~GraphSearch();

        SearchResult search(const Eigen::Vector3d &start, const Eigen::Vector3d &goal);

        void initMap(const GridMap::Ptr &map_obj);
        // void initMap(const auto &map_obj);

        void getPath(std::vector<Eigen::Vector3d> &path_list);

    private:
        // GridNodePtr ***GridMap_; // 3 dimensional array to store the GridMapPtr
        std::vector<GridNodePtr> GridMap_;

        GridMap::Ptr occupancy_map_;

        GridNodePtr ret_node_;

        int current_search_round_;
        double map_resolution_, map_resolution_inverse_;
        const double tie_breaker_ = 1.0 + 1.0 / 10000;
        bool use_jps_;

        Eigen::Vector3i map_size_, pool_size_, center_index_;
        Eigen::Vector3d start_, goal_, center_, sub_goal_;

        void initGridMap();

        void getSuccessorNode(const GridNodePtr &current_node, const GridNodePtr &goal_node);

        bool ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

        inline int checkOccupancy(Eigen::Vector3d node_pos)
        {
            return occupancy_map_->getInflateOccupancy(node_pos);
        }

        inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index)
        {
            return (index - center_index_).cast<double>() * map_resolution_ + center_;
        }

        inline bool Coord2Index(const Eigen::Vector3d &pos, Eigen::Vector3i &index)
        {
            index = ((pos - center_) * map_resolution_inverse_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + center_index_;
            if (index(0) < 0)
            {
                index(0) = 1;
            }

            if (index(1) < 0)
            {
                index(1) = 1;
            }

            if (index(2) < 0)
            {
                index(2) = 1;
            }

            if (index(0) >= pool_size_.x())
            {
                index(0) = pool_size_.x() - 1;
            }

            if (index(1) >= pool_size_.y())
            {
                index(0) = pool_size_.y() - 1;
            }

            if (index(2) >= pool_size_.z())
            {
                index(0) = pool_size_.z() - 1;
            }
            return true;
        }

        inline int voxelIndex2vectIndex(const Eigen::Vector3i &index)
        {
            return index.x() + index.y() * pool_size_.x() + index.z() * pool_size_.x() * pool_size_.y();
        }

        inline double getDiagonalHeuristic(const GridNodePtr &node1, const GridNodePtr &node2)
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

        inline double getHeuristic(const GridNodePtr &node1, const GridNodePtr &node2)
        {
            return tie_breaker_ * getDiagonalHeuristic(node1, node2);
        }

        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, GridNodeComparator> pq; // pq stores openlist nodes, pq.top() will return the GridNodePtr with the smallest estimated total cost (highest priority)

    }; // class GraphSearch

}; // namespace GraphSearch
