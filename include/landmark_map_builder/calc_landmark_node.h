#ifndef CALC_LANDMARK_H_
#define CALC_LANDMARK_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <yolov5_pytorch_ros/BoundingBoxes.h>
#include "landmark_map_builder/Landmarks.h"

namespace landmark_map_builder {

class calc_landmark
{
public:
    calc_landmark();
    ~calc_landmark();
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cb_yolo(const yolov5_pytorch_ros::BoundingBoxes::ConstPtr& msg);
    void load_yaml();
    void loop();
    bool get_transform(geometry_msgs::TransformStamped&, double&);
    void get_pose(geometry_msgs::TransformStamped&, double&, double&, const int&);
    void get_yaw(double&, double&);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber scan_sub_;
    ros::Subscriber yolo_sub_;
    ros::Publisher landmark_pub_;
    sensor_msgs::LaserScan::ConstPtr scan_;
    yolov5_pytorch_ros::BoundingBoxes::ConstPtr bb_;
    tf2_ros::Buffer tfbuffer_;
    tf2_ros::TransformListener tflistener_;
    std::string landmark_class_file_;
    Landmarks landmark_list_;
    int img_width_;
    double cutoff_min_ang_, cutoff_max_ang_;
    double prob_threshold_ = 0.8;
    int min_obj_size_ = 0;
};

}
#endif