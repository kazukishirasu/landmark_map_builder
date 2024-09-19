#ifndef VISUALIZE_LANDMARK_H_
#define VISUALIZE_LANDMARK_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <std_srvs/Empty.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "landmark_map_builder/struct.h"

namespace landmark_map_builder {

class visualize_landmark
{
public:
    visualize_landmark();
    ~visualize_landmark();
    void cb_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
    void cb_add(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
    void cb_delete(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
    bool cb_save_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void load_yaml();
    bool save_yaml();
    void initMenu();
    void main();
    visualization_msgs::InteractiveMarker makeInteractiveMarker(const std::string&, const geometry_msgs::Point&, size_t);
    visualization_msgs::InteractiveMarkerControl makeMarkerControl(const std::string&, size_t);
    visualization_msgs::InteractiveMarkerControl makeMenuControl();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::ServiceServer save_srv_;
    std::string landmark_file_;
    std::vector<visualization_msgs::InteractiveMarker> landmark_list_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    interactive_markers::MenuHandler menu_handler_;
};

}
#endif