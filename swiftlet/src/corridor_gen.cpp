#include <corridor_gen.h>

using namespace CorridorGen;
CorridorGenerator::CorridorGenerator(double resolution, double clearance, int max_sample, double ceiling, double floor, double goal_pt_margin)
    : resolution_(resolution), clearance_(clearance), max_sample_(max_sample), ceiling_(ceiling), floor_(floor), goal_pt_margin_(goal_pt_margin)
{
    octree_.deleteTree();
    octree_.setResolution(resolution_);
    std::random_device rd;
    gen_ = std::mt19937_64(rd());
    one_third_ = 1.0 / 3.0;
    // normal_rand_ = std::normal_distribution<double>(0.0, 1.0);
    // normal_rand_(gen_);
}

void CorridorGenerator::updatePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &new_cloud)
{
    cloud_ = new_cloud;
    octree_.deleteTree();
    octree_.setInputCloud(new_cloud);
    octree_.addPointsFromInputCloud();
}

void CorridorGenerator::updateGlobalPath(const std::vector<Eigen::Vector3d> &path)
{
    guide_path_.clear();
    // std::cout << "original path " << std::endl;
    // for (auto point : path)
    // {
    //     std::cout << "point " << point.transpose() << std::endl;
    // }
    guide_path_ = path;               // copy assignment
    initial_position_ = path.front(); // front() and back() return read-only reference
    goal_position_ = path.back();
    std::reverse(guide_path_.begin(), guide_path_.end());
}

void CorridorGenerator::generateCorridorAlongPath(const std::vector<Eigen::Vector3d> &path)
{
    flight_corridor_.clear(); // clear flight corridor generated from previous iteration
    waypoint_list_.clear();   // clear waypoints initialized from previous iteration
    updateGlobalPath(path);
    waypoint_list_.push_back(initial_position_);
    Corridor current_corridor = GenerateOneSphere(initial_position_);
    Corridor previous_corridor = current_corridor;
    current_corridor.second = 1;
    flight_corridor_.emplace_back(current_corridor);
    int max_iter = guide_path_.size();
    int iter_count = 0;
    std::cout << "before while true" << std::endl;
    std::cout << "initial_position_ " << initial_position_.transpose() << std::endl;
    // todo: implement time check to break from loop
    while (true)
    {
        iter_count++;
        if (iter_count > 100)
        {
            std::cout << "fail to generate corridor" << std::endl;
            break;
        }
        local_guide_point_ = getGuidePoint(guide_path_, current_corridor);

        // std::cout << "local_guide_point_ " << local_guide_point_.transpose() << std::endl;
        // current_corridor = batchSample(local_guide_point_, current_corridor);
        current_corridor = uniformBatchSample(local_guide_point_, current_corridor);

        // current_corridor = directionalSample(local_guide_point_, current_corridor);
        // current_corridor = GenerateOneSphere(local_guide_point_);
        Eigen::Vector3d intermediate_waypoint = getMidPointBetweenCorridors(previous_corridor, current_corridor);
        previous_corridor = current_corridor;

        // std::cout << "current_corridor centre " << current_corridor.first.transpose() << " radius " << current_corridor.second << std::endl;

        flight_corridor_.emplace_back(current_corridor);
        waypoint_list_.emplace_back(intermediate_waypoint);

        // if (pointInCorridor(goal_position_, current_corridor))
        if (pointNearCorridor(goal_position_, current_corridor))
        {
            std::cout << "Corridors Generated" << std::endl;
            waypoint_list_.emplace_back(goal_position_);
            std::cout << "waypt size is " << waypoint_list_.size() << std::endl;
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
        // std::cout << "Found neighbor" << std::endl;
        double radius = sqrt(double(pointNKNSquaredDistance[0])) - clearance_;
        Eigen::Vector3d nearestPt;

        nearestPt.x() = (*cloud_)[pointIdxNKNSearch[0]].x;
        nearestPt.y() = (*cloud_)[pointIdxNKNSearch[0]].y;
        nearestPt.z() = (*cloud_)[pointIdxNKNSearch[0]].z;

        // std::cout << "nearestPt is " << nearestPt.transpose() << std::endl;

        if (radius > searchPoint.z)
        {
            radius = abs(searchPoint.z);
        }

        return Corridor(pos, radius);
    }
    else
    {
        std::cout << "cloud empty?" << std::endl;
        return Corridor(pos, 10.0);
    }
}

auto CorridorGenerator::GenerateOneSphereVerbose(const Eigen::Vector3d &pos)
{
    // lock guard to prevent cloud from changing?
    int K = 1;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    pcl::PointXYZ searchPoint;
    searchPoint.x = pos.x();
    searchPoint.y = pos.y();
    searchPoint.z = pos.z();
    Eigen::Vector3d nearestPt;
    // consider using radiusSearch?
    if (octree_.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        // std::cout << "Found neighbor" << std::endl;
        double radius = sqrt(double(pointNKNSquaredDistance[0])) - clearance_;

        nearestPt.x() = (*cloud_)[pointIdxNKNSearch[0]].x;
        nearestPt.y() = (*cloud_)[pointIdxNKNSearch[0]].y;
        nearestPt.z() = (*cloud_)[pointIdxNKNSearch[0]].z;

        // std::cout << "nearestPt is " << nearestPt.transpose() << std::endl;

        if (radius > searchPoint.z)
        {
            radius = abs(searchPoint.z);
        }

        // return Corridor(pos, radius);
        return std::make_pair(Corridor(pos, radius), nearestPt);
    }
    else
    {
        nearestPt << 50, 50, 50;
        std::cout << "cloud empty?" << std::endl;
        // return Corridor(pos, 10.0);
        return std::make_pair(Corridor(pos, 10.0), nearestPt);
    }
}

Eigen::Vector3d CorridorGenerator::getGuidePoint(std::vector<Eigen::Vector3d> &guide_path, const Corridor &input_corridor)
{
    auto [center, radius] = input_corridor;

    // std::cout << "before popping " << std::endl;

    // for (auto point : guide_path)
    // {
    //     std::cout << "point " << point.transpose() << std::endl;
    // }
    while ((center - guide_path.back()).norm() < radius)
    {
        guide_path.pop_back();
        // std::cout << "popped " << std::endl;
    }

    // std::cout << "after popping " << std::endl;

    // for (auto point : guide_path)
    // {
    //     std::cout << "point " << point.transpose() << std::endl;
    // }
    return guide_path.back();
}

bool CorridorGenerator::pointInCorridor(const Eigen::Vector3d &point, const Corridor &corridor)
{
    auto [center, radius] = corridor;
    return ((point - center).norm()) < radius;
}

bool CorridorGenerator::pointNearCorridor(const Eigen::Vector3d &point, const Corridor &corridor)
{
    auto [center, radius] = corridor;
    return ((point - center).norm() - goal_pt_margin_) < radius;
}

Corridor CorridorGenerator::batchSample(const Eigen::Vector3d &guide_point, const Corridor &input_corridor)
{
    auto [previous_center, radius] = input_corridor;
    Eigen::Vector3d sample_x_vector = previous_center - guide_point;
    std::cout << "previous_center " << previous_center.transpose() << std::endl;
    std::cout << "guide_point " << guide_point.transpose() << std::endl;
    Eigen::Vector3d sample_x_direction = sample_x_vector / sample_x_vector.norm();
    Eigen::Vector3d global_x_direction(1, 0, 0);
    double angle_between = acos(sample_x_direction.dot(global_x_direction));
    Eigen::Vector3d rotation_axis;
    Eigen::Matrix3d rotation_matrix;

    double x_var = one_third_ * sample_x_vector.norm() / 5;
    double y_var = 2 * x_var;
    double z_var = y_var;
    double x_sd = sqrt(x_var);
    double y_sd = sqrt(y_var);
    double z_sd = y_sd;
    int sample_num = 0;

    if ((0 <= angle_between && angle_between < 0.017) || (0 >= angle_between && angle_between > -0.017))
    {
        rotation_matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    }

    else if ((3.124 <= angle_between && angle_between < 3.159) || (-3.124 >= angle_between && angle_between > -3.159))
    {
        rotation_matrix << -1, 0, 0, 0, -1, 0, 0, 0, 1;
    }

    else
    {
        rotation_axis = global_x_direction.cross(sample_x_direction);
        rotation_axis = rotation_axis / rotation_axis.norm();
        rotation_matrix = Eigen::AngleAxisd(angle_between, rotation_axis);
    }

    // rotation_matrix = Eigen::AngleAxisd(angle_between, rotation_axis);

    // x_normal_rand_ = std::normal_distribution<double>(guide_point.x(), x_sd);
    // y_normal_rand_ = std::normal_distribution<double>(guide_point.y(), y_sd);
    // z_normal_rand_ = std::normal_distribution<double>(guide_point.y(), z_sd);

    x_normal_rand_ = std::normal_distribution<double>(0, x_sd);
    y_normal_rand_ = std::normal_distribution<double>(0, y_sd);
    z_normal_rand_ = std::normal_distribution<double>(0, z_sd);
    std::cout << "one_third_ " << one_third_ << std::endl;
    std::cout << "sample_x_vector.norm() " << sample_x_vector.norm() << std::endl;
    std::cout << "x_var " << x_var << std::endl;
    std::cout << "y_var " << y_var << std::endl;
    std::cout << "z_var " << z_var << std::endl;
    std::cout << "x_sd " << x_sd << std::endl;
    std::cout << "y_sd " << y_sd << std::endl;
    std::cout << "z_sd " << z_sd << std::endl;

    std::priority_queue<CorridorPtr, std::vector<CorridorPtr>, CorridorComparator> empty_pool;
    corridor_pool_.swap(empty_pool);

    std::cout << "before batch sample while" << std::endl;

    while (sample_num < max_sample_)
    {
        sample_num++;
        double x_coord = x_normal_rand_(gen_);
        double y_coord = y_normal_rand_(gen_);
        double z_coord = z_normal_rand_(gen_);

        // Eigen::Vector3d candidate_pt(0.0, y_coord, z_coord);
        Eigen::Vector3d candidate_pt(x_coord, y_coord, z_coord);
        // needs to transform the candidate_pt to align the axes
        // std::cout << "before transformation " << candidate_pt.transpose() << std::endl;
        // candidate_pt = rotation_matrix.transpose() * candidate_pt + guide_point;
        candidate_pt = rotation_matrix * candidate_pt + guide_point;
        // std::cout << "candidate_pt " << candidate_pt.transpose() << std::endl;

        // if (double(candidate_pt.z()) > ceiling_ || double(candidate_pt.z()) < floor_)
        // {
        //     std::cout << "skipped" << std::endl;
        //     continue;
        // }

        CorridorPtr candidiate_corridor = std::make_shared<CorridorSample>();
        candidiate_corridor->geometry = GenerateOneSphere(candidate_pt);
        // std::cout << "candidiate_corridor " << candidiate_corridor->geometry.first.transpose() << " radius " << candidiate_corridor->geometry.second << std::endl;
        candidiate_corridor->updateScore(input_corridor, Eigen::Vector2d(1.0, 1.0));
        corridor_pool_.push(candidiate_corridor);
    }

    auto ret = corridor_pool_.top();
    return ret->geometry;
}

Corridor CorridorGenerator::directionalSample(const Eigen::Vector3d &guide_point, const Corridor &input_corridor)
{
    Corridor new_corridor;
    auto [pre_center, pre_radius] = input_corridor;
    auto ret = GenerateOneSphereVerbose(guide_point);
    Corridor guide_pt_corridor = ret.first;
    Eigen::Vector3d obstacle = ret.second;
    auto [center, radius] = guide_pt_corridor;
    double desired_radius = 1;
    double resolution = 0.1;
    std::vector<Eigen::Vector3d> sample_point_bucket;
    if (radius >= desired_radius)
    {
        return guide_pt_corridor;
    }

    else
    {
        std::cout << " desired_radius " << desired_radius << std::endl;
        std::cout << " radius " << radius << std::endl;
        if (desired_radius - radius < resolution)
        {
            return guide_pt_corridor;
        }
        Eigen::Vector3d pre2guide = guide_point - pre_center;
        pre2guide = pre2guide / pre2guide.norm();

        Eigen::Vector3d obs2guide = guide_point - obstacle;
        obs2guide = obs2guide / obs2guide.norm();

        Eigen::Vector3d sample_direction = pre2guide + obs2guide;
        sample_direction = sample_direction / sample_direction.norm();

        Eigen::Vector3d global_x_direction(1, 0, 0);
        double angle_between = acos(sample_direction.dot(global_x_direction));
        Eigen::Vector3d rotation_axis;
        Eigen::Matrix3d rotation_matrix;

        std::priority_queue<CorridorPtr, std::vector<CorridorPtr>, CorridorComparator> empty_pool;
        corridor_pool_.swap(empty_pool);

        if ((0 <= angle_between && angle_between < 0.017) || (0 >= angle_between && angle_between > -0.017))
        {
            rotation_matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        }

        else if ((3.124 <= angle_between && angle_between < 3.159) || (-3.124 >= angle_between && angle_between > -3.159))
        {
            rotation_matrix << -1, 0, 0, 0, -1, 0, 0, 0, 1;
        }

        else
        {
            rotation_axis = global_x_direction.cross(sample_direction);
            rotation_axis = rotation_axis / rotation_axis.norm();
            rotation_matrix = Eigen::AngleAxisd(angle_between, rotation_axis);
        }
        int count = 0;
        for (double i = resolution; i < (desired_radius - radius); i += resolution)
        // for (double i = resolution; i < 1; i += resolution)
        {
            count++;
            Eigen::Vector3d candidate_point;
            candidate_point << i, 0.0, 0.0;
            std::cout << " candidate_point " << candidate_point.transpose() << std::endl;
            candidate_point = rotation_matrix * candidate_point + guide_point;

            CorridorPtr candidiate_corridor = std::make_shared<CorridorSample>();
            candidiate_corridor->geometry = GenerateOneSphere(candidate_point);
            // std::cout << "candidiate_corridor " << candidiate_corridor->geometry.first.transpose() << " radius " << candidiate_corridor->geometry.second << std::endl;
            candidiate_corridor->updateScore(input_corridor, Eigen::Vector2d(1.0, 5.0));
            corridor_pool_.push(candidiate_corridor);
            // sample_point_bucket.push_back(candidate_point);
        }

        std::cout << "walked steps: " << count << std::endl;

        auto ret = corridor_pool_.top();
        return ret->geometry;
    }
}

Corridor CorridorGenerator::uniformBatchSample(const Eigen::Vector3d &guide_point, const Corridor &input_corridor)
{
    auto [previous_center, pre_radius] = input_corridor;
    // auto [pre_center, pre_radius] = input_corridor;
    auto ret = GenerateOneSphereVerbose(guide_point);
    Corridor guide_pt_corridor = ret.first;
    Eigen::Vector3d obstacle = ret.second;
    auto [center, radius] = guide_pt_corridor;
    double desired_radius = 1;
    double resolution = 0.1;
    std::vector<Eigen::Vector3d> sample_point_bucket;
    if (radius >= desired_radius)
    {
        return guide_pt_corridor;
    }

    else
    {
        if (desired_radius - radius < resolution)
        {
            return guide_pt_corridor;
        }

        Eigen::Vector3d sample_x_vector = previous_center - guide_point;
        // std::cout << "previous_center " << previous_center.transpose() << std::endl;
        // std::cout << "guide_point " << guide_point.transpose() << std::endl;
        Eigen::Vector3d sample_x_direction = sample_x_vector / sample_x_vector.norm();
        Eigen::Vector3d global_x_direction(1, 0, 0);
        double angle_between = acos(sample_x_direction.dot(global_x_direction));
        Eigen::Vector3d rotation_axis;
        Eigen::Matrix3d rotation_matrix;

        double x_var = one_third_ * sample_x_vector.norm() / 5;
        double y_var = 2 * x_var;
        double z_var = y_var;
        double x_sd = sqrt(x_var);
        double y_sd = sqrt(y_var);
        double z_sd = y_sd;
        int sample_num = 0;

        if ((0 <= angle_between && angle_between < 0.017) || (0 >= angle_between && angle_between > -0.017))
        {
            rotation_matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        }

        else if ((3.124 <= angle_between && angle_between < 3.159) || (-3.124 >= angle_between && angle_between > -3.159))
        {
            rotation_matrix << -1, 0, 0, 0, -1, 0, 0, 0, 1;
        }

        else
        {
            rotation_axis = global_x_direction.cross(sample_x_direction);
            rotation_axis = rotation_axis / rotation_axis.norm();
            rotation_matrix = Eigen::AngleAxisd(angle_between, rotation_axis);
        }

        // rotation_matrix = Eigen::AngleAxisd(angle_between, rotation_axis);

        // x_normal_rand_ = std::normal_distribution<double>(guide_point.x(), x_sd);
        // y_normal_rand_ = std::normal_distribution<double>(guide_point.y(), y_sd);
        // z_normal_rand_ = std::normal_distribution<double>(guide_point.y(), z_sd);

        x_normal_rand_ = std::normal_distribution<double>(0, x_sd);
        y_normal_rand_ = std::normal_distribution<double>(0, y_sd);
        z_normal_rand_ = std::normal_distribution<double>(0, z_sd);
        // std::cout << "one_third_ " << one_third_ << std::endl;
        // std::cout << "sample_x_vector.norm() " << sample_x_vector.norm() << std::endl;
        // std::cout << "x_var " << x_var << std::endl;
        // std::cout << "y_var " << y_var << std::endl;
        // std::cout << "z_var " << z_var << std::endl;
        // std::cout << "x_sd " << x_sd << std::endl;
        // std::cout << "y_sd " << y_sd << std::endl;
        // std::cout << "z_sd " << z_sd << std::endl;

        std::priority_queue<CorridorPtr, std::vector<CorridorPtr>, CorridorComparator> empty_pool;
        corridor_pool_.swap(empty_pool);

        std::cout << "before batch sample while" << std::endl;

        while (sample_num < max_sample_)
        {
            sample_num++;
            double x_coord = x_normal_rand_(gen_);
            double y_coord = y_normal_rand_(gen_);
            double z_coord = z_normal_rand_(gen_);

            // Eigen::Vector3d candidate_pt(0.0, y_coord, z_coord);
            Eigen::Vector3d candidate_pt(x_coord, y_coord, z_coord);
            // needs to transform the candidate_pt to align the axes
            // std::cout << "before transformation " << candidate_pt.transpose() << std::endl;
            // candidate_pt = rotation_matrix.transpose() * candidate_pt + guide_point;
            candidate_pt = rotation_matrix * candidate_pt + guide_point;
            // std::cout << "candidate_pt " << candidate_pt.transpose() << std::endl;

            // if (double(candidate_pt.z()) > ceiling_ || double(candidate_pt.z()) < floor_)
            // {
            //     std::cout << "skipped" << std::endl;
            //     continue;
            // }

            CorridorPtr candidiate_corridor = std::make_shared<CorridorSample>();
            candidiate_corridor->geometry = GenerateOneSphere(candidate_pt);
            // std::cout << "candidiate_corridor " << candidiate_corridor->geometry.first.transpose() << " radius " << candidiate_corridor->geometry.second << std::endl;
            candidiate_corridor->updateScore(input_corridor, Eigen::Vector2d(2.0, 1.0));
            corridor_pool_.push(candidiate_corridor);
        }

        // auto ret = corridor_pool_.top();
        // return ret->geometry;
        return corridor_pool_.top()->geometry;
    }
}

// about return a vector efficiently
// https://stackoverflow.com/questions/28760475/how-to-return-a-class-member-vector-in-c11
const std::vector<Corridor> &CorridorGenerator::getCorridor() const
{
    return flight_corridor_;
}

const std::vector<Eigen::Vector3d> &CorridorGenerator::getWaypointList() const
{
    return waypoint_list_;
}

Eigen::Vector3d CorridorGenerator::getMidPointBetweenCorridors(const Corridor &previous_corridor, const Corridor &current_corridor)
{
    auto [pre_center, pre_radius] = previous_corridor;
    auto [curr_center, curr_radius] = current_corridor;
    std::cout << "pre_center " << pre_center.transpose() << " pre_radius " << pre_radius << std::endl;
    std::cout << "curr_center " << curr_center.transpose() << " curr_radius " << curr_radius << std::endl;
    Eigen::Vector3d previous2current;
    previous2current = curr_center - pre_center;
    double distance_between = previous2current.norm();
    previous2current = previous2current / distance_between;
    double distance_to_waypt = pre_radius - (pre_radius + curr_radius - distance_between) * 0.5;
    Eigen::Vector3d waypt = previous2current * distance_to_waypt + pre_center;
    std::cout << "waypt " << waypt.transpose() << std::endl;
    return waypt;
}