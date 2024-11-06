#ifndef VISUALIZE_LANDMARK_H_
#define VISUALIZE_LANDMARK_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include "landmark_map_builder/Landmarks.h"

namespace landmark_map_builder {

class visualize_landmark
{
public:
    visualize_landmark();
    ~visualize_landmark();
    void cb_landmarks(const Landmarks::ConstPtr &msg);
    bool cb_save_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void load_yaml();
    bool save_yaml();
    void visualization();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber landmark_sub_;
    ros::ServiceServer save_srv_;
    ros::Publisher sphere_pub_;
    ros::Publisher text_pub_;
    std::string landmark_file_;
    Landmarks landmark_list_;
};

}
#endif