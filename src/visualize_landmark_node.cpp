#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include "landmark_map_builder/struct.h"

class visualize_landmark_node
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher sphere_pub_;
    ros::Publisher text_pub_;
    std::string landmark_file_;
    std::vector<Landmark> landmark_list_;
public:
    visualize_landmark_node();
    ~visualize_landmark_node();
    void loop();
    void load_yaml();
    void visualize_landmark(std::vector<Landmark>&);
};

visualize_landmark_node::visualize_landmark_node() : pnh_("~")
{
    ROS_INFO("Start visualize_landmark_node");
    load_yaml();
    sphere_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_sphere", 1);
    text_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_text", 1);
}

visualize_landmark_node::~visualize_landmark_node()
{
}

void visualize_landmark_node::loop()
{
    visualize_landmark(landmark_list_);
}

void visualize_landmark_node::load_yaml()
{
    try
    {
        pnh_.param("landmark_file", landmark_file_, std::string(ros::package::getPath("landmark_map_builder") += "/landmark/landmark_ver0.yaml"));
        ROS_INFO("Load %s", landmark_file_.c_str());
        Landmark lm;
        YAML::Node node = YAML::LoadFile(landmark_file_);
        YAML::Node landmark = node["landmark"];
        for (YAML::const_iterator it=landmark.begin(); it!=landmark.end(); ++it)
        {
            lm.name = it->first.as<std::string>();
            YAML::Node config = landmark[lm.name];
            for (YAML::const_iterator it=config.begin(); it!=config.end(); ++it)
            {
                Pose pos;
                std::string id = it->first.as<std::string>();
                pos.x = it->second["pose"][0].as<float>();
                pos.y = it->second["pose"][1].as<float>();
                pos.z = it->second["pose"][2].as<float>();
                lm.pose.push_back(pos);
            }
            landmark_list_.push_back(lm);
        }
    }
    catch(const std::exception& e)
    {
        ROS_WARN("%s", e.what());
    }
}

void visualize_landmark_node::visualize_landmark(std::vector<Landmark>& lm_list)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "sphere";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    for (const auto &lm:lm_list)
    {
        std::string name = lm.name;
        for (const auto &pos:lm.pose)
        {
            point.x = pos.x;
            point.y = pos.y;
            point.z = pos.z;
            marker.points.push_back(point);
            if (name == "Elevator"){
                color.r = 0.0;
                color.g = 0.0;
                color.b = 1.0;
            }else if (name == "Door"){
                color.r = 0.90;
                color.g = 0.71;
                color.b = 0.13;
            }else{
                color.r = 1.0;
                color.g = 0.0;
                color.b = 0.0;
            }
            color.a = 1.0;
            marker.colors.push_back(color);
        }
    }
    sphere_pub_.publish(marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "register_landmark_node");
    visualize_landmark_node vl;
    while (ros::ok())
    {
        vl.loop();
    }
    return 0;
}