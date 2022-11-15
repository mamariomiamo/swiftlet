#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <queue>
#include <plan_env/grid_map.h>

//JPS reference
// https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search.html
// https://zhuanlan.zhihu.com/p/25093275
// https://github.com/KumarRobotics/jps3d
// https://ieeexplore-ieee-org.libproxy1.nus.edu.sg/document/7839930
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
        Eigen::Vector3i dir;   // direction of expanding used for JPS

        double g_cost{inf}; // cost from start
        double h_cost{inf}; // heuristic cost
        double f_cost{inf}; // total cost i.e. f_cost = g_cost + h_cost

        GridNodePtr parent{nullptr};

        NodeState state{UNDEFINED};
        int query_rounds{0}; // previous round of query of this node
                             // to be compared with GraphSearch::rounds_
                             // Every call of GraphSearch(start, end, ...)
    };

    struct JPS3DNeib
    {
        // for each (dx,dy,dz) these contain:
        //    ns: neighbors that are always added
        //    f1: forced neighbors to check
        //    f2: neighbors to add if f1 is forced
        int ns[27][3][26];
        int f1[27][3][12];
        int f2[27][3][12];
        // nsz contains the number of neighbors for the four different types of moves:
        // no move (norm 0):        26 neighbors always added
        //                          0 forced neighbors to check (never happens)
        //                          0 neighbors to add if forced (never happens)
        // straight (norm 1):       1 neighbor always added
        //                          8 forced neighbors to check
        //                          8 neighbors to add if forced
        // diagonal (norm sqrt(2)): 3 neighbors always added
        //                          8 forced neighbors to check
        //                          12 neighbors to add if forced
        // diagonal (norm sqrt(3)): 7 neighbors always added
        //                          6 forced neighbors to check
        //                          12 neighbors to add if forced
        static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}}; // declaration and initialization
        JPS3DNeib()
        {
            int id = 0;
            for (int dz = -1; dz <= 1; ++dz)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        int norm1 = abs(dx) + abs(dy) + abs(dz);

                        for (int dev = 0; dev < nsz[norm1][0]; ++dev)
                            Neib(dx, dy, dz, norm1, dev, ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);

                        for (int dev = 0; dev < nsz[norm1][1]; ++dev)
                        {
                            FNeib(dx, dy, dz, norm1, dev,
                                  f1[id][0][dev], f1[id][1][dev], f1[id][2][dev],
                                  f2[id][0][dev], f2[id][1][dev], f2[id][2][dev]);
                        }

                        id++;
                    }
                }
            }
        }

    private:
        void Neib(int dx, int dy, int dz, int norm1, int dev, int &tx, int &ty, int &tz)
        {
            switch (norm1)
            {
            case 0:
                switch (dev)
                {
                case 0:
                    tx = 1;
                    ty = 0;
                    tz = 0;
                    return;
                case 1:
                    tx = -1;
                    ty = 0;
                    tz = 0;
                    return;
                case 2:
                    tx = 0;
                    ty = 1;
                    tz = 0;
                    return;
                case 3:
                    tx = 1;
                    ty = 1;
                    tz = 0;
                    return;
                case 4:
                    tx = -1;
                    ty = 1;
                    tz = 0;
                    return;
                case 5:
                    tx = 0;
                    ty = -1;
                    tz = 0;
                    return;
                case 6:
                    tx = 1;
                    ty = -1;
                    tz = 0;
                    return;
                case 7:
                    tx = -1;
                    ty = -1;
                    tz = 0;
                    return;
                case 8:
                    tx = 0;
                    ty = 0;
                    tz = 1;
                    return;
                case 9:
                    tx = 1;
                    ty = 0;
                    tz = 1;
                    return;
                case 10:
                    tx = -1;
                    ty = 0;
                    tz = 1;
                    return;
                case 11:
                    tx = 0;
                    ty = 1;
                    tz = 1;
                    return;
                case 12:
                    tx = 1;
                    ty = 1;
                    tz = 1;
                    return;
                case 13:
                    tx = -1;
                    ty = 1;
                    tz = 1;
                    return;
                case 14:
                    tx = 0;
                    ty = -1;
                    tz = 1;
                    return;
                case 15:
                    tx = 1;
                    ty = -1;
                    tz = 1;
                    return;
                case 16:
                    tx = -1;
                    ty = -1;
                    tz = 1;
                    return;
                case 17:
                    tx = 0;
                    ty = 0;
                    tz = -1;
                    return;
                case 18:
                    tx = 1;
                    ty = 0;
                    tz = -1;
                    return;
                case 19:
                    tx = -1;
                    ty = 0;
                    tz = -1;
                    return;
                case 20:
                    tx = 0;
                    ty = 1;
                    tz = -1;
                    return;
                case 21:
                    tx = 1;
                    ty = 1;
                    tz = -1;
                    return;
                case 22:
                    tx = -1;
                    ty = 1;
                    tz = -1;
                    return;
                case 23:
                    tx = 0;
                    ty = -1;
                    tz = -1;
                    return;
                case 24:
                    tx = 1;
                    ty = -1;
                    tz = -1;
                    return;
                case 25:
                    tx = -1;
                    ty = -1;
                    tz = -1;
                    return;
                }
            case 1:
                tx = dx;
                ty = dy;
                tz = dz;
                return;
            case 2:
                switch (dev)
                {
                case 0:
                    if (dz == 0)
                    {
                        tx = 0;
                        ty = dy;
                        tz = 0;
                        return;
                    }
                    else
                    {
                        tx = 0;
                        ty = 0;
                        tz = dz;
                        return;
                    }
                case 1:
                    if (dx == 0)
                    {
                        tx = 0;
                        ty = dy;
                        tz = 0;
                        return;
                    }
                    else
                    {
                        tx = dx;
                        ty = 0;
                        tz = 0;
                        return;
                    }
                case 2:
                    tx = dx;
                    ty = dy;
                    tz = dz;
                    return;
                }
            case 3:
                switch (dev)
                {
                case 0:
                    tx = dx;
                    ty = 0;
                    tz = 0;
                    return;
                case 1:
                    tx = 0;
                    ty = dy;
                    tz = 0;
                    return;
                case 2:
                    tx = 0;
                    ty = 0;
                    tz = dz;
                    return;
                case 3:
                    tx = dx;
                    ty = dy;
                    tz = 0;
                    return;
                case 4:
                    tx = dx;
                    ty = 0;
                    tz = dz;
                    return;
                case 5:
                    tx = 0;
                    ty = dy;
                    tz = dz;
                    return;
                case 6:
                    tx = dx;
                    ty = dy;
                    tz = dz;
                    return;
                }
            }
        }
        void FNeib(int dx, int dy, int dz, int norm1, int dev,
                   int &fx, int &fy, int &fz,
                   int &nx, int &ny, int &nz)
        {
            switch (norm1)
            {
            case 1:
                switch (dev)
                {
                case 0:
                    fx = 0;
                    fy = 1;
                    fz = 0;
                    break;
                case 1:
                    fx = 0;
                    fy = -1;
                    fz = 0;
                    break;
                case 2:
                    fx = 1;
                    fy = 0;
                    fz = 0;
                    break;
                case 3:
                    fx = 1;
                    fy = 1;
                    fz = 0;
                    break;
                case 4:
                    fx = 1;
                    fy = -1;
                    fz = 0;
                    break;
                case 5:
                    fx = -1;
                    fy = 0;
                    fz = 0;
                    break;
                case 6:
                    fx = -1;
                    fy = 1;
                    fz = 0;
                    break;
                case 7:
                    fx = -1;
                    fy = -1;
                    fz = 0;
                    break;
                }
                nx = fx;
                ny = fy;
                nz = dz;
                // switch order if different direction
                if (dx != 0)
                {
                    fz = fx;
                    fx = 0;
                    nz = fz;
                    nx = dx;
                }

                if (dy != 0)
                {
                    fz = fy;
                    fy = 0;
                    nz = fz;
                    ny = dy;
                }
                return;
            case 2:
                if (dx == 0)
                {
                    switch (dev)
                    {
                    case 0:
                        fx = 0;
                        fy = 0;
                        fz = -dz;
                        nx = 0;
                        ny = dy;
                        nz = -dz;
                        return;
                    case 1:
                        fx = 0;
                        fy = -dy;
                        fz = 0;
                        nx = 0;
                        ny = -dy;
                        nz = dz;
                        return;
                    case 2:
                        fx = 1;
                        fy = 0;
                        fz = 0;
                        nx = 1;
                        ny = dy;
                        nz = dz;
                        return;
                    case 3:
                        fx = -1;
                        fy = 0;
                        fz = 0;
                        nx = -1;
                        ny = dy;
                        nz = dz;
                        return;
                    case 4:
                        fx = 1;
                        fy = 0;
                        fz = -dz;
                        nx = 1;
                        ny = dy;
                        nz = -dz;
                        return;
                    case 5:
                        fx = 1;
                        fy = -dy;
                        fz = 0;
                        nx = 1;
                        ny = -dy;
                        nz = dz;
                        return;
                    case 6:
                        fx = -1;
                        fy = 0;
                        fz = -dz;
                        nx = -1;
                        ny = dy;
                        nz = -dz;
                        return;
                    case 7:
                        fx = -1;
                        fy = -dy;
                        fz = 0;
                        nx = -1;
                        ny = -dy;
                        nz = dz;
                        return;
                    // Extras
                    case 8:
                        fx = 1;
                        fy = 0;
                        fz = 0;
                        nx = 1;
                        ny = dy;
                        nz = 0;
                        return;
                    case 9:
                        fx = 1;
                        fy = 0;
                        fz = 0;
                        nx = 1;
                        ny = 0;
                        nz = dz;
                        return;
                    case 10:
                        fx = -1;
                        fy = 0;
                        fz = 0;
                        nx = -1;
                        ny = dy;
                        nz = 0;
                        return;
                    case 11:
                        fx = -1;
                        fy = 0;
                        fz = 0;
                        nx = -1;
                        ny = 0;
                        nz = dz;
                        return;
                    }
                }
                else if (dy == 0)
                {
                    switch (dev)
                    {
                    case 0:
                        fx = 0;
                        fy = 0;
                        fz = -dz;
                        nx = dx;
                        ny = 0;
                        nz = -dz;
                        return;
                    case 1:
                        fx = -dx;
                        fy = 0;
                        fz = 0;
                        nx = -dx;
                        ny = 0;
                        nz = dz;
                        return;
                    case 2:
                        fx = 0;
                        fy = 1;
                        fz = 0;
                        nx = dx;
                        ny = 1;
                        nz = dz;
                        return;
                    case 3:
                        fx = 0;
                        fy = -1;
                        fz = 0;
                        nx = dx;
                        ny = -1;
                        nz = dz;
                        return;
                    case 4:
                        fx = 0;
                        fy = 1;
                        fz = -dz;
                        nx = dx;
                        ny = 1;
                        nz = -dz;
                        return;
                    case 5:
                        fx = -dx;
                        fy = 1;
                        fz = 0;
                        nx = -dx;
                        ny = 1;
                        nz = dz;
                        return;
                    case 6:
                        fx = 0;
                        fy = -1;
                        fz = -dz;
                        nx = dx;
                        ny = -1;
                        nz = -dz;
                        return;
                    case 7:
                        fx = -dx;
                        fy = -1;
                        fz = 0;
                        nx = -dx;
                        ny = -1;
                        nz = dz;
                        return;
                    // Extras
                    case 8:
                        fx = 0;
                        fy = 1;
                        fz = 0;
                        nx = dx;
                        ny = 1;
                        nz = 0;
                        return;
                    case 9:
                        fx = 0;
                        fy = 1;
                        fz = 0;
                        nx = 0;
                        ny = 1;
                        nz = dz;
                        return;
                    case 10:
                        fx = 0;
                        fy = -1;
                        fz = 0;
                        nx = dx;
                        ny = -1;
                        nz = 0;
                        return;
                    case 11:
                        fx = 0;
                        fy = -1;
                        fz = 0;
                        nx = 0;
                        ny = -1;
                        nz = dz;
                        return;
                    }
                }
                else
                { // dz==0
                    switch (dev)
                    {
                    case 0:
                        fx = 0;
                        fy = -dy;
                        fz = 0;
                        nx = dx;
                        ny = -dy;
                        nz = 0;
                        return;
                    case 1:
                        fx = -dx;
                        fy = 0;
                        fz = 0;
                        nx = -dx;
                        ny = dy;
                        nz = 0;
                        return;
                    case 2:
                        fx = 0;
                        fy = 0;
                        fz = 1;
                        nx = dx;
                        ny = dy;
                        nz = 1;
                        return;
                    case 3:
                        fx = 0;
                        fy = 0;
                        fz = -1;
                        nx = dx;
                        ny = dy;
                        nz = -1;
                        return;
                    case 4:
                        fx = 0;
                        fy = -dy;
                        fz = 1;
                        nx = dx;
                        ny = -dy;
                        nz = 1;
                        return;
                    case 5:
                        fx = -dx;
                        fy = 0;
                        fz = 1;
                        nx = -dx;
                        ny = dy;
                        nz = 1;
                        return;
                    case 6:
                        fx = 0;
                        fy = -dy;
                        fz = -1;
                        nx = dx;
                        ny = -dy;
                        nz = -1;
                        return;
                    case 7:
                        fx = -dx;
                        fy = 0;
                        fz = -1;
                        nx = -dx;
                        ny = dy;
                        nz = -1;
                        return;
                    // Extras
                    case 8:
                        fx = 0;
                        fy = 0;
                        fz = 1;
                        nx = dx;
                        ny = 0;
                        nz = 1;
                        return;
                    case 9:
                        fx = 0;
                        fy = 0;
                        fz = 1;
                        nx = 0;
                        ny = dy;
                        nz = 1;
                        return;
                    case 10:
                        fx = 0;
                        fy = 0;
                        fz = -1;
                        nx = dx;
                        ny = 0;
                        nz = -1;
                        return;
                    case 11:
                        fx = 0;
                        fy = 0;
                        fz = -1;
                        nx = 0;
                        ny = dy;
                        nz = -1;
                        return;
                    }
                }
            case 3:
                switch (dev)
                {
                case 0:
                    fx = -dx;
                    fy = 0;
                    fz = 0;
                    nx = -dx;
                    ny = dy;
                    nz = dz;
                    return;
                case 1:
                    fx = 0;
                    fy = -dy;
                    fz = 0;
                    nx = dx;
                    ny = -dy;
                    nz = dz;
                    return;
                case 2:
                    fx = 0;
                    fy = 0;
                    fz = -dz;
                    nx = dx;
                    ny = dy;
                    nz = -dz;
                    return;
                // Need to check up to here for forced!
                case 3:
                    fx = 0;
                    fy = -dy;
                    fz = -dz;
                    nx = dx;
                    ny = -dy;
                    nz = -dz;
                    return;
                case 4:
                    fx = -dx;
                    fy = 0;
                    fz = -dz;
                    nx = -dx;
                    ny = dy;
                    nz = -dz;
                    return;
                case 5:
                    fx = -dx;
                    fy = -dy;
                    fz = 0;
                    nx = -dx;
                    ny = -dy;
                    nz = dz;
                    return;
                // Extras
                case 6:
                    fx = -dx;
                    fy = 0;
                    fz = 0;
                    nx = -dx;
                    ny = 0;
                    nz = dz;
                    return;
                case 7:
                    fx = -dx;
                    fy = 0;
                    fz = 0;
                    nx = -dx;
                    ny = dy;
                    nz = 0;
                    return;
                case 8:
                    fx = 0;
                    fy = -dy;
                    fz = 0;
                    nx = 0;
                    ny = -dy;
                    nz = dz;
                    return;
                case 9:
                    fx = 0;
                    fy = -dy;
                    fz = 0;
                    nx = dx;
                    ny = -dy;
                    nz = 0;
                    return;
                case 10:
                    fx = 0;
                    fy = 0;
                    fz = -dz;
                    nx = 0;
                    ny = dy;
                    nz = -dz;
                    return;
                case 11:
                    fx = 0;
                    fy = 0;
                    fz = -dz;
                    nx = dx;
                    ny = 0;
                    nz = -dz;
                    return;
                }
            }
        }
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
        GraphSearch(int map_size, double map_resolution, bool use_jps);
        ~GraphSearch();

        SearchResult search(const Eigen::Vector3d &start, const Eigen::Vector3d &goal);

        void initMap(const GridMap::Ptr &map_obj);
        // void initMap(const auto &map_obj);

        void getPath(std::vector<Eigen::Vector3d> &path_list);

        bool useJPS() { return this->use_jps_; };

    protected:
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

        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, GridNodeComparator> pq; // pq stores openlist nodes, pq.top() will return the GridNodePtr with the smallest estimated total cost (highest priority)

        void initGridMap();

        virtual void getSuccessorNode(const GridNodePtr &current_node, const GridNodePtr &goal_node);

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
            std::cout << "coord2index: " << index.transpose() << std::endl;
            std::cout << "pos: " << pos.transpose() << std::endl;
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
            // return index.x() + index.y() * pool_size_.x() + index.z() * pool_size_.x() * pool_size_.y();
            return index.z() + index.y() * pool_size_.z() + index.x() * pool_size_.z() * pool_size_.y();
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

    }; // class GraphSearch

    class JPS : public GraphSearch
    {
    public:
        JPS3DNeib *jn3d;
        Eigen::Vector3i goalIdx;

        JPS(int map_size, double map_resolution, bool use_jps) : GraphSearch(map_size, map_resolution, use_jps)
        {
            jn3d = new JPS3DNeib();
        };

        ~JPS()
        {
            delete jn3d;
        };

        void getSuccessorNode(const GridNodePtr &current_node, const GridNodePtr &goal_node) override;
        void JPSGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets, vector<double> &edgeCostSets,  Eigen::Vector3i &goalIdx);
        bool hasForced(const Eigen::Vector3i &idx, const Eigen::Vector3i &dir);
        bool jump(const Eigen::Vector3i &curIdx, const Eigen::Vector3i &expDir, Eigen::Vector3i &neiIdx);
        // bool jump(const Eigen::Vector3i &curIdx, const Eigen::Vector3i &expDir, Eigen::Vector3i &neiIdx, Eigen::Vector3i &goalIdx);
    };

}; // namespace GraphSearch
