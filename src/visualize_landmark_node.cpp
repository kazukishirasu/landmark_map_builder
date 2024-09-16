#include "landmark_map_builder/visualize_landmark_node.h"

namespace landmark_map_builder {
visualize_landmark::visualize_landmark() : pnh_("~"), server_(new interactive_markers::InteractiveMarkerServer("visualize_landmark"))
{
    ROS_INFO("Start visualize_landmark_node");
    load_yaml();
}

visualize_landmark::~visualize_landmark()
{
}

void visualize_landmark::load_yaml()
{
    pnh_.param("landmark_file", landmark_file_, std::string(ros::package::getPath("landmark_map_builder") += "/map/map_ver0.yaml"));
    ROS_INFO("Load %s", landmark_file_.c_str());
    try
    {
        YAML::Node node = YAML::LoadFile(landmark_file_);
        YAML::Node landmark = node["landmark"];
        for (YAML::const_iterator it=landmark.begin(); it!=landmark.end(); ++it)
        {
            Landmark lm;
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

void visualize_landmark::main()
{
    for (const auto& lm:landmark_list_)
    {
        for (size_t id = 0; const auto& pos:lm.pose)
        {
            geometry_msgs::Point position = setPosition(pos);
            server_->insert(makeInteractiveMarker(lm.name, position, id), boost::bind(&visualize_landmark::cb_feedback, this, _1));
            id++;
        }
    }
    server_->applyChanges();
    ros::spin();
}

geometry_msgs::Point visualize_landmark::setPosition(const Pose& pos)
{
    ROS_INFO("Set position");
    geometry_msgs::Point position;
    position.x = pos.x;
    position.y = pos.y;
    position.z = pos.z;

    return position;
}

visualization_msgs::InteractiveMarker visualize_landmark::makeInteractiveMarker(const std::string& name, const geometry_msgs::Point& position, size_t id)
{
    ROS_INFO("Make InteractiveMarker");
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.stamp = ros::Time::now();
    int_marker.header.frame_id = "map";
    int_marker.name = name + std::to_string(id);
    int_marker.description = name + std::to_string(id);

    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    control.markers.push_back(makeSphereMarker(name, position));
    int_marker.controls.push_back(control);

    return int_marker;
}

visualization_msgs::Marker visualize_landmark::makeSphereMarker(const std::string& name, const geometry_msgs::Point& position)
{
    ROS_INFO("Make SphereMarker");
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    if (name == "Elevator"){
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }else if (name == "Door"){
        marker.color.r = 0.90;
        marker.color.g = 0.71;
        marker.color.b = 0.13;
    }else if (name == "Vending machine"){
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }else{
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
    }
    marker.color.a = 1.0;
    marker.pose.position = position;
    
    return marker;
}

void visualize_landmark::cb_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    ROS_INFO("Recived feedback");
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualize_landmark_node");
    landmark_map_builder::visualize_landmark vl;
    vl.main();
    return 0;
}