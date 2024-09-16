#ifndef VISUALIZE_LANDMARK_H_
#define VISUALIZE_LANDMARK_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "landmark_map_builder/struct.h"

namespace landmark_map_builder {

class visualize_landmark
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher sphere_pub_;
    ros::Publisher text_pub_;
    std::string landmark_file_;
    std::vector<Landmark> landmark_list_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
public:
    visualize_landmark();
    ~visualize_landmark();
    void load_yaml();
    void main();
    visualization_msgs::InteractiveMarker makeInteractiveMarker(const std::string&, const geometry_msgs::Point&, size_t);
    visualization_msgs::Marker makeSphereMarker(const std::string&, const geometry_msgs::Point&);
    void cb_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
    geometry_msgs::Point setPosition(const Pose&);
};

}
#endif