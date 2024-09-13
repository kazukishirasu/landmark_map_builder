#ifndef DBSCAN_H_
#define DBSCAN_H_

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include "landmark_map_builder/struct.h"

namespace landmark_map_builder {

class dbscan
{
public:
    dbscan(std::string, std::string);
    ~dbscan();
    void load_yaml();
private:
    std::string landmark_file_, map_file_;
    std::vector<Data_Points> dp_list_;
};

}
#endif