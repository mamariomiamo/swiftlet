#include <iostream>
#include <Eigen/Dense>
#include <queue>
#include <algorithm>
#include <pcl/octree/octree_search.h>

namespace CorridorGen
{
    class CorridorGenerator
    {
    public:
        typedef std::pair<Eigen::Vector3d, double> Corridor;

    public: // public member variable
        // flight_corridor_[i].first is the position of i^th spherical corridor centre
        // flight_corridor_[i].second is the radius of i^th spherical corridor
        std::vector<Corridor> flight_corridor_;

    private: // private member variable
        Eigen::Vector3d initial_position_, goal_position_, local_guide_point_;

        // global guide path
        std::vector<Eigen::Vector3d> input_path_;

        // store the local point cloud
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_ =
            decltype(octree_)(0.1);

    public: // public member function
        CorridorGenerator();
        ~CorridorGenerator();

        void updatePointCloud();

        // Generate a sphere SFC given a position
        auto GenerateOneSphere(const Eigen::Vector3d &pos);

        void updateGlobalPath();

    private: // private member function
        Eigen::Vector3d getGuidePoint(const std::vector<Eigen::Vector3d> &input_path_, const Corridor& input_corridor);
        Corridor batchSample(const Eigen::Vector3d& guide_point, const Corridor& input_corridor);
        void generateCorridorAlongPath();

        // more functinos to implement batchSample(...)
    };

}; // namespace CorridorGenerator
