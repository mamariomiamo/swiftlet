#pragma once
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace px4fsm_helper
{
    Eigen::Vector3d Point2Vect(const geometry_msgs::Point &point)
    {
        Eigen::Vector3d vect;
        vect << point.x, point.y, point.z;
        return vect;
    }

    geometry_msgs::Point Vect2Point(const Eigen::Vector3d &vect)
    {
        geometry_msgs::Point point;
        point.x = vect.x();
        point.y = vect.y();
        point.z = vect.z();

        return point;
    }

    Eigen::Vector3d Quat2Euler(const geometry_msgs::Quaternion &quat)
    {
        tf2::Quaternion q(
            quat.x,
            quat.y,
            quat.z,
            quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        Eigen::Vector3d euler;
        euler << roll, pitch, yaw;

        return euler;
    }

    geometry_msgs::Point GeoVect2Point(const geometry_msgs::Vector3 &vect)
    {
        geometry_msgs::Point ret;
        ret.x = vect.x;
        ret.y = vect.y;
        ret.z = vect.z;
    }

    double GetYawFromQuat(const geometry_msgs::Quaternion &quat)
    {
        Eigen::Vector3d euler;
        euler = Quat2Euler(quat);
        double yaw = euler(2);
        return yaw;
    }
}; // namespace px4fsm_helper