#include "landmark_map_builder/visualize_landmark_node.h"

namespace landmark_map_builder {
visualize_landmark::visualize_landmark() : pnh_("~")
{
    ROS_INFO("Start visualize_landmark_node");
    load_yaml();
    landmark_sub_ = nh_.subscribe("/landmarks", 1, &visualize_landmark::cb_landmarks, this);
    save_srv_ = nh_.advertiseService("/save_landmark", &visualize_landmark::cb_save_srv, this);
    sphere_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_sphere", 1);
    text_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_text", 1);
}

visualize_landmark::~visualize_landmark()
{
}

void visualize_landmark::cb_landmarks(const Landmarks::ConstPtr& msg)
{
    for (const auto& message:msg->landmarks){
        bool find = false;
        for (auto& landmark:landmark_list_.landmarks){
            if (message.Class == landmark.Class){
                for (const auto& p:message.poses)
                    landmark.poses.push_back(p);
                find = true;
            }
        }
        if (!find){
            Landmark landmark;
            landmark.Class = message.Class;
            for (const auto& p:message.poses){
                landmark.poses.push_back(p);
            }
            landmark_list_.landmarks.push_back(landmark);
        }
    }
}

bool visualize_landmark::cb_save_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if (save_yaml()){
        ROS_INFO("Landmark saved successfully");
        return true;
    }else{
        ROS_WARN("Failed to save landmark");
        return false;
    }
}

void visualize_landmark::load_yaml()
{
    pnh_.param("landmark_file", landmark_file_, std::string(ros::package::getPath("landmark_map_builder") += "/landmark/landmark_ver0.yaml"));
    ROS_INFO("Load %s", landmark_file_.c_str());
    try{
        std::ifstream file(landmark_file_);
        YAML::Node node = YAML::LoadFile(landmark_file_);
        for (YAML::const_iterator it=node["landmark"].begin(); it!=node["landmark"].end(); ++it){
            std::string Class = it->first.as<std::string>();
            Landmark landmark;
            landmark.Class = Class;
            for (YAML::const_iterator it=node["landmark"][Class].begin(); it!=node["landmark"][Class].end(); ++it){
                LandmarkPose p;
                p.pose_cov.pose.position.x = it->second["pose"][0].as<float>();
                p.pose_cov.pose.position.y = it->second["pose"][1].as<float>();
                p.pose_cov.pose.position.z = it->second["pose"][2].as<float>();
                landmark.poses.push_back(p);
            }
            landmark_list_.landmarks.push_back(landmark);
        }
    }
    catch(const std::exception& e){
        ROS_WARN("%s", e.what());
    }
}

bool visualize_landmark::save_yaml()
{
    try{
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "landmark";
        out << YAML::BeginMap;
        for (const auto& landmark:landmark_list_.landmarks){
            out << YAML::Key << landmark.Class;
            out << YAML::BeginMap;
            int id_n = 0;
            for (const auto& p:landmark.poses){
                std::string id = "id";
                id += std::to_string(id_n);
                out << YAML::Key << id;
                out << YAML::BeginMap;
                out << YAML::Key << "pose" << YAML::Value << YAML::Flow << YAML::BeginSeq
                << p.pose_cov.pose.position.x
                << p.pose_cov.pose.position.y
                << p.pose_cov.pose.position.z << YAML::EndSeq;
                out << YAML::Key << "enable" << YAML::Value << true;
                out << YAML::Key << "option" << YAML::Value << YAML::Null;
                out << YAML::EndMap;
                id_n++;
            }
            out << YAML::EndMap;
        }
        out << YAML::EndMap;
        out << YAML::EndMap;
        std::ofstream fout(landmark_file_);
        fout << out.c_str();
        return true;
    }
    catch(const std::exception& e)
    {
        ROS_WARN("%s", e.what());
        return false;
    }
}

void visualize_landmark::visualization()
{
    // ----------new----------
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

    // for (const auto& landmark:landmark_list_.landmarks){
    //     text.text = landmark.Class;
    //     int id = 0;
    //     for (const auto& p:landmark.poses){
    //         sphere.pose.position.x = p.pose_cov.pose.position.x;
    //         sphere.pose.position.y = p.pose_cov.pose.position.y;
    //         sphere.pose.position.z = p.pose_cov.pose.position.z;
    //         text.pose.position.x = p.pose_cov.pose.position.x;
    //         text.pose.position.y = p.pose_cov.pose.position.y + 0.4;
    //         text.pose.position.z = p.pose_cov.pose.position.z;
    //         sphere.id = id;
    //         text.id = id;
    //         sphere_pub_.publish(sphere);
    //         text_pub_.publish(text);
    //         id++;
    //     }
    // }

    // ----------old----------
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

    for (const auto& landmark:landmark_list_.landmarks){
        if (landmark.Class == "Elevator"){
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
        }else if (landmark.Class == "Door"){
            color.r = 0.90;
            color.g = 0.71;
            color.b = 0.13;
        }else if (landmark.Class == "Vending machine"){
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
        }else if (landmark.Class == "Fire extinguisher"){
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
        }else{
            color.r = 0.5;
            color.g = 0.5;
            color.b = 0.5;
        }
        color.a = 1.0;
        for (const auto& p:landmark.poses){
            point.x = p.pose_cov.pose.position.x;
            point.y = p.pose_cov.pose.position.y;
            point.z = p.pose_cov.pose.position.z;
            marker.points.push_back(point);
            marker.colors.push_back(color);
        }
    }
    sphere_pub_.publish(marker);
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualize_landmark_node");
    landmark_map_builder::visualize_landmark vl;
    ros::Rate rate(30.0);
    while (ros::ok()){
        ros::spinOnce();
        vl.visualization();
        rate.sleep();
    }
    return 0;
}