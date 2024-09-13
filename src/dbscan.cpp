#include "landmark_map_builder/dbscan.h"

namespace landmark_map_builder
{

dbscan::dbscan(std::string landmark_file, std::string map_file)
{
    std::cout << "Start DBSCAN clustering" << std::endl;
    landmark_file_ = landmark_file;
    map_file_ = map_file;
    load_yaml();
}

dbscan::~dbscan()
{
}

void dbscan::load_yaml()
{
    try
    {
        YAML::Node node = YAML::LoadFile(landmark_file_);
        YAML::Node landmark = node["landmark"];
        for (YAML::const_iterator it=landmark.begin(); it!=landmark.end(); ++it)
        {
            Data_Points dp;
            dp.name = it->first.as<std::string>();
            YAML::Node config = landmark[dp.name];
            dp.pose.resize(config.size(), 2);
            dp.id = Eigen::VectorXd::Constant(config.size(), 1, -1);
            int i = 0;
            for (YAML::const_iterator it=config.begin(); it!=config.end(); ++it)
            {
                std::string id = it->first.as<std::string>();
                dp.pose(i, 0) = it->second["pose"][0].as<float>();
                dp.pose(i, 1) = it->second["pose"][1].as<float>();
                i++;
            }
            dp_list_.push_back(dp);
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    // for (const auto& dp:dp_list_)
    // {
    //     std::cout << "--------------------" << std::endl;
    //     std::cout << "name : " << dp.name.c_str() << std::endl;
    //     for (size_t i = 0; i < dp.pose.rows(); i++)
    //     {
    //         std::cout << "x:" << dp.pose(i, 0) << " y:" << dp.pose(i, 1) << std::endl;
    //     }
    //     for (size_t i = 0; i < dp.id.rows(); i++)
    //     {
    //         std::cout << "id:" << dp.id(i, 0) << std::endl;
    //     }
    // }
}
}

int main(int argc, char *argv[])
{
    landmark_map_builder::dbscan dbscan(argv[1], argv[2]);
    return 0;
}