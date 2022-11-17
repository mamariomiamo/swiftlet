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
    guide_path_.clear();
    guide_path_ = path;               // copy assignment
    initial_position_ = path.front(); // front() and back() return read-only reference
    goal_position_ = path.back();
    std::reverse(guide_path_.begin(), guide_path_.end());
}

void CorridorGenerator::generateCorridorAlongPath(const std::vector<Eigen::Vector3d> &path)
{
    flight_corridor_.clear();
    updateGlobalPath(path);

    Corridor current_corridor = GenerateOneSphere(initial_position_);
    flight_corridor_.emplace_back(current_corridor);
    int max_iter = guide_path_.size();
    int iter_count = 0;

    // todo: implement time check to break from loop
    while (true)
    {
        iter_count++;
        if (iter_count > max_iter)
        {
            std::cout << "fail to generate corridor" << std::endl;
            break;
        }
        local_guide_point_ = getGuidePoint(guide_path_, current_corridor);
        // current_corridor = batchSample(local_guide_point_, current_corridor);
        current_corridor = GenerateOneSphere(local_guide_point_);
        flight_corridor_.emplace_back(current_corridor);

        if (pointInCorridor(goal_position_, current_corridor))
        {
            std::cout << "Corridors Generated" << std::endl;
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

Eigen::Vector3d CorridorGenerator::getGuidePoint(std::vector<Eigen::Vector3d> &guide_path, const Corridor &input_corridor)
{
    auto [center, radius] = input_corridor;
    while ((center - guide_path.back()).norm() < radius)
    {
        guide_path.pop_back();
    }

    return guide_path.back();
}

bool CorridorGenerator::pointInCorridor(const Eigen::Vector3d &point, const Corridor &corridor)
{
    auto [center, radius] = corridor;
    return ((point - center).norm()) < radius;
}

Corridor CorridorGenerator::batchSample(const Eigen::Vector3d &guide_point, const Corridor &input_corridor)
{
}


// about return a vector efficiently 
// https://stackoverflow.com/questions/28760475/how-to-return-a-class-member-vector-in-c11
const std::vector<Corridor> &CorridorGenerator::getCorridor() const
{
    return flight_corridor_;
}
