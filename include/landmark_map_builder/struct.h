#ifndef LANDMARK_STRUCT_H_
#define LANDMARK_STRUCT_H_

#include <string>
#include <Eigen/Dense>

struct Pose
{
    double x, y, z;
};

struct Landmark
{
    std::string name;
    std::vector<Pose> pose;
    // Eigen::Matrix<double, Eigen::Dynamic, 2> pose;
};

#endif