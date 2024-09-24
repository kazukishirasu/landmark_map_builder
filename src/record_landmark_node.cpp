#include "landmark_map_builder/record_landmark_node.h"

namespace landmark_map_builder
{

record_landmark::record_landmark() : pnh_("~")
{
    ROS_INFO("Start record_landmark_node");
    load_yaml();
    scan_sub_ = nh_.subscribe("/scan", 30, &record_landmark::cb_scan, this);
    yolo_sub_ = nh_.subscribe("/detected_objects_in_image", 30, &record_landmark::cb_yolo, this);
    save_srv_ = nh_.advertiseService("/save_landmark", &record_landmark::cb_save_srv, this);
    sphere_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_sphere", 1);
    text_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_text", 1);
}

record_landmark::~record_landmark()
{
}

void record_landmark::cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!listener_.waitForTransform(msg->header.frame_id, "map", msg->header.stamp + ros::Duration().fromNSec(msg->ranges.size()*msg->time_increment), ros::Duration(1.0)))
    {
        return;
    }
    projector_.transformLaserScanToPointCloud("map", *msg, cloud_, listener_);
}

void record_landmark::cb_yolo(const yolov5_pytorch_ros::BoundingBoxes& msg)
{
    if (cloud_.points.empty())
    {
        return;
    }

    if (std::abs(msg.header.stamp.toSec() - cloud_.header.stamp.toSec()) < 0.01)
    {
        return;
    }

    for (const auto &bb:msg.bounding_boxes)
    {
        auto itr = std::find(landmark_name_.begin(), landmark_name_.end(), bb.Class);
        const int index = std::distance(landmark_name_.begin(), itr);
        if (bb.probability > prob_threshold_ && bb.xmax - bb.xmin > min_obj_size_ && itr != landmark_name_.end())
        {
            get_pose(index, bb.xmax, bb.xmin);
        }
    }
}

bool record_landmark::cb_save_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if (save_yaml(landmark_list_))
    {
        ROS_INFO("Landmark saved successfully");
        return true;
    }else{
        return false;
    }
}

void record_landmark::load_yaml()
{
    pnh_.param("landmark_name_file", landmark_name_file_, std::string(ros::package::getPath("landmark_map_builder") += "/landmark/name.yaml"));
    pnh_.param("landmark_record_file", landmark_record_file_, std::string(ros::package::getPath("landmark_map_builder") += "/landmark/landmark_ver0.yaml"));
    ROS_INFO("Load name_file : %s", landmark_name_file_.c_str());
    ROS_INFO("Load record_file : %s", landmark_record_file_.c_str());

    try
    {
        YAML::Node node = YAML::LoadFile(landmark_name_file_);
        for (const auto &name:node["name"])
        {
            landmark_name_.push_back(name.as<std::string>());
        }
    }
    catch(const std::exception& e)
    {
        ROS_WARN("%s", e.what());
    }

    init_list();
}

void record_landmark::init_list()
{
    Landmark lm;
    for (const auto &name:landmark_name_)
    {
        lm.name = name;
        landmark_list_.push_back(lm);
    }
}

void record_landmark::get_pose(int index, int xmax, int xmin)
{
    Pose pos;
    auto yaw = -((((xmin + xmax) / 2) - (w_img_/2)) * M_PI) / (w_img_/2);
    if (yaw < 0) {yaw += (M_PI * 2);}
    int i = (yaw * cloud_.points.size()) / (M_PI * 2);
    pos.x = cloud_.points[i].x;
    pos.y = cloud_.points[i].y;
    pos.z = 0.0;
    landmark_list_[index].pose.push_back(pos);
}

bool record_landmark::save_yaml(std::vector<Landmark>& lm_list)
{
    try
    {
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "landmark";
        out << YAML::BeginMap;
        for (size_t index = 0; const auto &lm:lm_list)
        {
            std::string name = lm.name;
            out << YAML::Key << name;
            out << YAML::BeginMap;
            for (size_t id_n = 0; const auto &pos:lm.pose)
            {
                std::string id = "id";
                id += std::to_string(id_n);
                out << YAML::Key << id;
                out << YAML::BeginMap;
                out << YAML::Key << "pose" << YAML::Value << YAML::Flow << YAML::BeginSeq << pos.x << pos.y << pos.z << YAML::EndSeq;
                out << YAML::Key << "enable" << YAML::Value << true;
                out << YAML::Key << "option" << YAML::Value << YAML::Null;
                out << YAML::EndMap;
                id_n++;
            }
            out << YAML::EndMap;
            index++;
        }
        out << YAML::EndMap;
        out << YAML::EndMap;
        std::ofstream fout(landmark_record_file_);
        fout << "# prob_threshold = " + std::to_string(prob_threshold_);
        fout << ", min_obj_size = " + std::to_string(min_obj_size_) + "\n";
        fout << out.c_str();
        return true;
    }
    catch(const std::exception& e)
    {
        ROS_WARN("%s", e.what());
        return false;
    }
}

void record_landmark::visualize_landmark(std::vector<Landmark>& lm_list)
{
    //----------new----------
    // visualization_msgs::Marker sphere, text;
    // sphere.header.frame_id = "map";
    // sphere.header.stamp = ros::Time::now();
    // sphere.ns = "sphere";
    // sphere.type = visualization_msgs::Marker::SPHERE;
    // sphere.action = visualization_msgs::Marker::ADD;
    // sphere.pose.orientation.x = 0.0;
    // sphere.pose.orientation.y = 0.0;
    // sphere.pose.orientation.z = 0.0;
    // sphere.pose.orientation.w = 1.0;
    // sphere.scale.x = 0.5;
    // sphere.scale.y = 0.5;
    // sphere.scale.z = 0.5;
    // sphere.color.r = 0.0;
    // sphere.color.g = 0.0;
    // sphere.color.b = 1.0;
    // sphere.color.a = 0.8;

    // text.header.frame_id = "map";
    // text.header.stamp = ros::Time::now();
    // text.ns = "text";
    // text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // text.action = visualization_msgs::Marker::ADD;
    // text.pose.orientation.x = 0.0;
    // text.pose.orientation.y = 0.0;
    // text.pose.orientation.z = 0.0;
    // text.pose.orientation.w = 1.0;
    // text.scale.x = 0.5;
    // text.scale.y = 0.5;
    // text.scale.z = 0.5;
    // text.color.r = 1.0;
    // text.color.g = 1.0;
    // text.color.b = 1.0;
    // text.color.a = 1.0;

    // for (size_t id = 0; const auto &lm:lm_list)
    // {
    //     text.text = lm.name;
    //     for (const auto &pos:lm.pose)
    //     {
    //         sphere.pose.position.x = pos.x;
    //         sphere.pose.position.y = pos.y;
    //         sphere.pose.position.z = pos.z;
    //         text.pose.position.x = pos.x;
    //         text.pose.position.y = pos.y + 0.4;
    //         text.pose.position.z = pos.z;
    //         sphere.id = id;
    //         text.id = id;
    //         sphere_pub_.publish(sphere);
    //         text_pub_.publish(text);
    //         id++;
    //     }
    // }

    //----------old----------
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
        if (lm.name == "Elevator"){
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
        }else if (lm.name == "Door"){
            color.r = 0.90;
            color.g = 0.71;
            color.b = 0.13;
        }else if (lm.name == "Vending machine"){
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
        }else{
            color.r = 0.5;
            color.g = 0.5;
            color.b = 0.5;
        }
        color.a = 1.0;
        for (const auto &pos:lm.pose)
        {
            point.x = pos.x;
            point.y = pos.y;
            point.z = pos.z;
            marker.points.push_back(point);
            marker.colors.push_back(color);
        }
    }
    sphere_pub_.publish(marker);
}

void record_landmark::loop()
{
    visualize_landmark(landmark_list_);
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "record_landmark_node");
    landmark_map_builder::record_landmark rl;
    ros::Rate rate(1.0);
    while (ros::ok())
    {
        ros::spinOnce();
        rl.loop();
        rate.sleep();
    }
    return 0;
}