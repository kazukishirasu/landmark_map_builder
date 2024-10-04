#include "landmark_map_builder/visualize_landmark_node.h"

namespace landmark_map_builder {
visualize_landmark::visualize_landmark() : pnh_("~"), server_(new interactive_markers::InteractiveMarkerServer("visualize_landmark"))
{
    ROS_INFO("Start visualize_landmark_node");
    pnh_.param("debug", debug_, false);
    load_yaml();
    save_srv_ = nh_.advertiseService("/save_landmark", &visualize_landmark::cb_save_srv, this);
}

visualize_landmark::~visualize_landmark()
{
}

void visualize_landmark::cb_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    ROS_INFO("%s : x:%lf, y:%lf", feedback->marker_name.c_str(), feedback->pose.position.x, feedback->pose.position.y);
    for (auto& lm:landmark_list_)
    {
        if (lm.name == feedback->marker_name)
        {
            lm.pose = feedback->pose;
        }
    }
}

void visualize_landmark::cb_add(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    geometry_msgs::Point position = feedback->pose.position;
    position.x = position.x + 0.5;
    std::string marker_name;
    std::string name = feedback->control_name;
    int id = 0;
    while (1)
    {
        marker_name = name + std::to_string(id);
        bool find = false;
        for (const auto& lm:landmark_list_)
        {
            if (lm.name == marker_name)
            {
                find = true;
                break;
            }
        }
        if (find)
        {
            id++;
        }else if (!find)
        {
            break;
        }
    }
    for (const auto& cov:cov_list_)
    {
        if (cov.first == feedback->marker_name)
        {
            cov_list_.push_back(std::make_pair(marker_name, cov.second));
            break;
        }
    }
    visualization_msgs::InteractiveMarker int_marker = makeInteractiveMarker(name, position, id);
    landmark_list_.push_back(int_marker);
    server_->insert(int_marker, boost::bind(&visualize_landmark::cb_feedback, this, _1));
    menu_handler_.apply(*server_, marker_name);
    server_->applyChanges();
    ROS_INFO("add %s", marker_name.c_str());
}

void visualize_landmark::cb_delete(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    for (auto itr = landmark_list_.begin(); itr != landmark_list_.end(); ++itr)
    {
        if (itr->name == feedback->marker_name)
        {
            landmark_list_.erase(itr);
            break;
        }
    }
    for (auto itr = cov_list_.begin(); itr != cov_list_.end(); ++itr)
    {
        if (itr->first == feedback->marker_name)
        {
            cov_list_.erase(itr);
            break;
        }
    }
    server_->erase(feedback->marker_name);
    server_->applyChanges();
    ROS_INFO("delete %s", feedback->marker_name.c_str());
}

bool visualize_landmark::cb_save_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if (save_yaml())
    {
        ROS_INFO("Landmark saved successfully");
        return true;
    }else{
        ROS_WARN("Failed to save landmark");
        return false;
    }
}

void visualize_landmark::load_yaml()
{
    pnh_.param("landmark_file", landmark_file_, std::string(ros::package::getPath("landmark_map_builder") += "/map/map_ver0.yaml"));
    ROS_INFO("Load %s", landmark_file_.c_str());
    try
    {
        std::ifstream file(landmark_file_);
        std::getline(file, first_line_);
        YAML::Node node = YAML::LoadFile(landmark_file_);
        YAML::Node landmark = node["landmark"];
        for (YAML::const_iterator it=landmark.begin(); it!=landmark.end(); ++it)
        {
            std::string name = it->first.as<std::string>();
            YAML::Node config = landmark[name];
            size_t id = 0;
            for (YAML::const_iterator it=config.begin(); it!=config.end(); ++it)
            {
                geometry_msgs::Point position;
                position.x = it->second["pose"][0].as<float>();
                position.y = it->second["pose"][1].as<float>();
                position.z = it->second["pose"][2].as<float>();
                landmark_list_.push_back(makeInteractiveMarker(name, position, id));

                std::string name_id = name + std::to_string(id);
                Eigen::Matrix2d cov;
                cov(0, 0) = it->second["cov"][0][0].as<double>();
                cov(0, 1) = it->second["cov"][0][1].as<double>();
                cov(1, 0) = it->second["cov"][1][0].as<double>();
                cov(1, 1) = it->second["cov"][1][1].as<double>();
                cov_list_.push_back(std::make_pair(name_id, cov));
                id++;
            }
        }
    }
    catch(const std::exception& e)
    {
        ROS_WARN("%s", e.what());
    }
}

bool visualize_landmark::save_yaml()
{
    try
    {
        std::vector<std::string> name_list;
        for (const auto& lm:landmark_list_)
        {
            std::string name = lm.controls[0].markers[0].ns;
            auto itr = std::find(name_list.begin(), name_list.end(), name);
            if (itr == name_list.end())
            {
                name_list.push_back(name);
            }
        }
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "landmark";
        out << YAML::BeginMap;
        for (const auto& name:name_list)
        {
            out << YAML::Key << name;
            out << YAML::BeginMap;
            for (size_t id_n = 0; const auto& lm:landmark_list_)
            {
                if (name == lm.controls[0].markers[0].ns)
                {
                    geometry_msgs::Point pos = lm.pose.position;
                    std::string id = "id";
                    id += std::to_string(id_n);
                    out << YAML::Key << id;
                    out << YAML::BeginMap;
                    out << YAML::Key << "pose" << YAML::Value << YAML::Flow << YAML::BeginSeq << pos.x << pos.y << pos.z << YAML::EndSeq;
                    out << YAML::Key << "cov" << YAML::BeginSeq;
                    for (const auto& cov:cov_list_)
                    {
                        if (lm.name == cov.first)
                        {
                            for (size_t j = 0; j < cov.second.rows(); j++)
                            {
                                out << YAML::Flow << YAML::BeginSeq;
                                for (size_t k = 0; k < cov.second.cols(); k++)
                                {
                                    out << cov.second(j, k);
                                }
                                out << YAML::EndSeq;
                            }
                            break;
                        }
                    }
                    out << YAML::EndSeq;
                    out << YAML::Key << "enable" << YAML::Value << true;
                    out << YAML::Key << "option" << YAML::Value << YAML::Null;
                    out << YAML::EndMap;
                    id_n++;
                }
            }
            out << YAML::EndMap;
        }
        out << YAML::EndMap;
        out << YAML::EndMap;
        std::ofstream fout(landmark_file_);
        if (debug_)
        {
            fout << first_line_ << "\n";
        }
        fout << out.c_str();
        return true;
    }
    catch(const std::exception& e)
    {
        ROS_WARN("%s", e.what());
        return false;
    }
}

void visualize_landmark::initMenu()
{
    menu_handler_.insert("add", boost::bind(&visualize_landmark::cb_add, this, _1));
    menu_handler_.insert("delete", boost::bind(&visualize_landmark::cb_delete, this, _1));
}

void visualize_landmark::main()
{
    initMenu();
    for (const auto& lm:landmark_list_)
    {
        server_->insert(lm, boost::bind(&visualize_landmark::cb_feedback, this, _1));
        menu_handler_.apply(*server_, lm.name);
    }
    server_->applyChanges();
    ros::spin();
}

visualization_msgs::InteractiveMarker visualize_landmark::makeInteractiveMarker(const std::string& name, const geometry_msgs::Point& position, size_t id)
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.stamp = ros::Time::now();
    int_marker.header.frame_id = "map";
    int_marker.name = name + std::to_string(id);
    int_marker.pose.position = position;

    visualization_msgs::InteractiveMarkerControl marker_control, menu_control;
    marker_control = makeMarkerControl(name, id);
    menu_control = makeMenuControl();
    int_marker.controls.push_back(marker_control);
    int_marker.controls.push_back(menu_control);

    return int_marker;
}

visualization_msgs::InteractiveMarkerControl visualize_landmark::makeMarkerControl(const std::string& name, size_t id)
{
    visualization_msgs::InteractiveMarkerControl control;
    control.name = name;
    control.always_visible = true;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.ns = name;
    marker.id = id;
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
    control.markers.push_back(marker);

    return control;
}

visualization_msgs::InteractiveMarkerControl visualize_landmark::makeMenuControl()
{
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    visualization_msgs::InteractiveMarkerControl::BUTTON;

    return control;
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualize_landmark_node");
    landmark_map_builder::visualize_landmark vl;
    vl.main();
    return 0;
}