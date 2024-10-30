#ifndef REGISTER_LANDMARK_H_
#define REGISTER_LANDMARK_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <yolov5_pytorch_ros/BoundingBoxes.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include "landmark_map_builder/struct.h"

namespace landmark_map_builder {

class record_landmark
{
public:
    record_landmark();
    ~record_landmark();
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cb_yolo(const yolov5_pytorch_ros::BoundingBoxes::ConstPtr& msg);
    bool cb_save_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool save_yaml(std::vector<Landmark>&);
    void load_yaml();
    void init_list();
    void loop();
    bool get_transform(geometry_msgs::TransformStamped&, double&);
    void get_yaw(double&, double&);
    void get_pose(geometry_msgs::TransformStamped&, double&, const int&);
    void visualize_landmark(std::vector<Landmark>&);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber scan_sub_;
    ros::Subscriber yolo_sub_;
    ros::ServiceServer save_srv_;
    ros::Publisher sphere_pub_;
    ros::Publisher text_pub_;
    tf2_ros::Buffer tfbuffer_;
    tf2_ros::TransformListener tflistener_;
    std::string landmark_name_file_, landmark_record_file_;
    std::vector<std::string> landmark_name_;
    std::vector<Landmark> landmark_list_;
    sensor_msgs::LaserScan::ConstPtr scan_;
    yolov5_pytorch_ros::BoundingBoxes::ConstPtr bb_;
    int w_img_ = 1280;
    double prob_threshold_ = 0.6;
    int min_obj_size_ = 30;
};

}
#endif