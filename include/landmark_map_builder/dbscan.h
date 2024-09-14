#ifndef DBSCAN_H_
#define DBSCAN_H_

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "landmark_map_builder/struct.h"

namespace landmark_map_builder {

class dbscan
{
public:
    dbscan(std::string, std::string);
    ~dbscan();
    void load_yaml();
    void main();
    void clustering(Data_Points&);
    void save_yaml();
private:
    std::string landmark_file_, save_file_;
    std::vector<Data_Points> dp_list_;
    float eps = 0.5;
    unsigned int minpts = 100;
};

}
#endif