#include <corridor_gen.h>

using namespace CorridorGen;
CorridorGenerator::CorridorGenerator(double resolution, double clearance)
    : resolution_(resolution), clearance_(clearance)
{
    octree_.deleteTree();
    octree_.setResolution(resolution_);
}

void CorridorGenerator::updatePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &new_cloud)
{
    octree_.deleteTree();
    octree_.setInputCloud(new_cloud);
    octree_.addPointsFromInputCloud();
}

void CorridorGenerator::updateGlobalPath(const std::vector<Eigen::Vector3d> &path)
{
    guide_path_ = path;
    initial_position_ = path.front(); // front() and back() return read-only reference
    goal_position_ = path.back();

    generateCorridorAlongPath();
}

void CorridorGenerator::generateCorridorAlongPath()
{
    Corridor current_corridor = GenerateOneSphere(initial_position_);
    flight_corridor_.emplace_back(current_corridor);

    // todo: implement time check to break from loop
    while (true)
    {
        local_guide_point_ = getGuidePoint(guide_path_, current_corridor);
        current_corridor = batchSample(local_guide_point_, current_corridor);
        flight_corridor_.emplace_back(current_corridor);

        if (pointInCorridor(goal_position_, current_corridor))
        {
            std::cout << "Corridor Generated" << std::endl;
            break;
        }
    }
}

Corridor CorridorGenerator::GenerateOneSphere(const Eigen::Vector3d &pos)
{
    // lock guard to prevent cloud from changing?
    int K = 1;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    pcl::PointXYZ searchPoint;
    searchPoint.x = pos.x();
    searchPoint.y = pos.y();
    searchPoint.z = pos.z();

    // consider using radiusSearch?
    if (octree_.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        std::cout << "Found neighbor" << std::endl;
        return Corridor(pos, sqrt(double(pointNKNSquaredDistance[0])) - clearance_);
    }
    else
    {
        std::cout << "cloud empty?" << std::endl;
        return Corridor(pos, 10.0);
    }
}

Eigen::Vector3d CorridorGenerator::getGuidePoint(const std::vector<Eigen::Vector3d> &guide_path, const Corridor &input_corridor)
{
}

bool CorridorGenerator::pointInCorridor(const Eigen::Vector3d &point, const Corridor &corridor)
{
}

Corridor CorridorGenerator::batchSample(const Eigen::Vector3d &guide_point, const Corridor &input_corridor)
{
}
