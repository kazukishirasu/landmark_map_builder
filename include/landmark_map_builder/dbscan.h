#ifndef DBSCAN_H_
#define DBSCAN_H_

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <Eigen/Dense>
#include "landmark_map_builder/Landmarks.h"

namespace landmark_map_builder {

class dbscan
{
public:
    struct Data_Points{
        std::string name;
        Eigen::MatrixX2d pose;
        Eigen::VectorXd cluster;
    };
    dbscan(std::string, std::string, std::string);
    ~dbscan();
    void load_yaml();
    void main();
    void clustering(Data_Points&);
    void save_yaml();
private:
    std::string param_file_, landmark_file_, save_file_;
    std::vector<Data_Points> dp_list_;
    float eps_ = 0.5;
    unsigned int minpts_ = 100;
};

}
#endif