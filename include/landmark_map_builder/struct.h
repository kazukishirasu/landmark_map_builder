#ifndef LANDMARK_STRUCT_H_
#define LANDMARK_STRUCT_H_

#include <string>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>

struct Landmark
{
    std::string name;
    std::vector<geometry_msgs::Pose> pose;
    std::vector<Eigen::Matrix2d> cov;
};

struct Data_Points
{
    std::string name;
    Eigen::MatrixX2d pose;
    Eigen::VectorXd cluster;
};

#endif