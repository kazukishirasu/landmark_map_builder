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
};

struct Data_Points
{
    std::string name;
    Eigen::MatrixX2d pose;
    Eigen::VectorXd cluster;
};

#endif