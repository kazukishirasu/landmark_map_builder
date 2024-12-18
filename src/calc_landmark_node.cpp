#include "landmark_map_builder/calc_landmark_node.h"

namespace landmark_map_builder
{

calc_landmark::calc_landmark() : pnh_("~"), tflistener_(tfbuffer_)
{
    ROS_INFO("Start calc_landmark_node");
    pnh_.param("image_width", img_width_, 1280);
    pnh_.param("cutoff_min_angle", cutoff_min_ang_, 0.0);
    pnh_.param("cutoff_max_angle", cutoff_max_ang_, 0.0);
    load_yaml();
    scan_sub_ = nh_.subscribe("/scan", 1, &calc_landmark::cb_scan, this);
    yolo_sub_ = nh_.subscribe("/detected_objects_in_image", 1, &calc_landmark::cb_yolo, this);
    landmark_pub_ = nh_.advertise<Landmarks>("/landmarks", 10);
}

calc_landmark::~calc_landmark()
{
}

void calc_landmark::cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_ = msg;
}

void calc_landmark::cb_yolo(const yolov5_pytorch_ros::BoundingBoxes::ConstPtr& msg)
{
    bb_ = msg;
}

void calc_landmark::load_yaml()
{
    pnh_.param("landmark_class_file", landmark_class_file_, std::string(ros::package::getPath("landmark_map_builder") += "/param/class.yaml"));
    ROS_INFO("Load class file : %s", landmark_class_file_.c_str());
    try{
        YAML::Node node = YAML::LoadFile(landmark_class_file_);
        for (const auto &Class:node["class"]){
            Landmark landmark;
            landmark.Class = Class.as<std::string>();
            landmark_list_.landmarks.push_back(landmark);
        }
    }
    catch(const std::exception& e){
        ROS_WARN("%s", e.what());
    }
}

void calc_landmark::loop()
{
    if (!scan_){
        ROS_WARN("Waiting for scan data...");
        return;
    }

    if (!bb_){
        ROS_WARN("Waiting for yolo data...");
        return;
    }

    geometry_msgs::TransformStamped transformstamped;
    double lidar_yaw;
    if (!get_transform(transformstamped, lidar_yaw))
        return;

    std::cout << "-----" << std::endl;
    for (const auto &b:bb_->bounding_boxes){
        bool find = false;
        int index = 0;
        for (const auto& landmark:landmark_list_.landmarks){
            if (b.Class == landmark.Class){
                find = true;
                break;
            }
            index++;
        }
        if (b.probability > prob_threshold_ && (b.xmax - b.xmin) > min_obj_size_ && find){
            std::cout << b.Class << "=>";
            double yaw = -((((b.xmin + b.xmax) / 2) - (img_width_ / 2)) * M_PI) / (img_width_ / 2);
            get_pose(transformstamped, yaw, lidar_yaw, index);
        }
    }
    landmark_pub_.publish(landmark_list_);
    for (auto& landmark:landmark_list_.landmarks){
        landmark.poses.clear();
    }
}

bool calc_landmark::get_transform(geometry_msgs::TransformStamped& transformstamped, double& lidar_yaw)
{
    try{
        transformstamped = tfbuffer_.lookupTransform("base_link", scan_->header.frame_id, ros::Time(0));
        tf2::Quaternion q;
        tf2::fromMsg(transformstamped.transform.rotation, q);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, lidar_yaw);
    }
    catch(const tf2::TransformException &e){
        ROS_WARN("%s", e.what());
        return false;
    }

    try{
        transformstamped = tfbuffer_.lookupTransform("map", scan_->header.frame_id, ros::Time(0));
    }
    catch(const tf2::TransformException &e){
        ROS_WARN("%s", e.what());
        return false;
    }
    return true;
}

void calc_landmark::get_pose(geometry_msgs::TransformStamped& transformstamped, double& yaw, double& lidar_yaw, const int& index)
{
    get_yaw(yaw, lidar_yaw);
    geometry_msgs::PointStamped point_in, point_out;
    int i = (yaw * scan_->ranges.size()) / (M_PI * 2);
    double a = (scan_->angle_increment * i) - std::abs(scan_->angle_min);
    if (a < cutoff_min_ang_ || a > cutoff_max_ang_){
        std::cout << "cutoff" << std::endl;
        return;
    }
    a -= lidar_yaw;
    point_in.point.x = scan_->ranges[i] * std::cos(a);
    point_in.point.y = scan_->ranges[i] * std::sin(a);
    point_in.point.z = 0;
    try{
        tf2::doTransform(point_in, point_out, transformstamped);
        LandmarkPose p;
        p.pose_cov.pose.position.x = point_out.point.x;
        p.pose_cov.pose.position.y = point_out.point.y;
        p.pose_cov.pose.position.z = 0.0;
        if (std::isnan(p.pose_cov.pose.position.x) || std::isnan(p.pose_cov.pose.position.y) || std::isinf(p.pose_cov.pose.position.x) || std::isinf(p.pose_cov.pose.position.y))
            return;
        std::cout << "x:" << p.pose_cov.pose.position.x << " y:" << p.pose_cov.pose.position.y << std::endl;
        landmark_list_.landmarks[index].poses.push_back(p);
    }
    catch(const tf2::TransformException &e){
        ROS_WARN("%s", e.what());
        return;
    }
}

void calc_landmark::get_yaw(double& yaw, double& lidar_yaw)
{
    // 0~2π
    if (yaw < 0)
        yaw += M_PI * 2;
    // LiDARの検出開始角度を考慮
    yaw -= scan_->angle_min;
    if (yaw > M_PI * 2)
        yaw -= M_PI * 2;
    // LiDARの角度を考慮
    yaw -= lidar_yaw;
    if (yaw > M_PI * 2){
        yaw -= M_PI * 2;
    }else if (yaw < 0){
        yaw += M_PI * 2;
    }
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calc_landmark_node");
    landmark_map_builder::calc_landmark cl;
    ros::Rate rate(30.0);
    while (ros::ok()){
        ros::spinOnce();
        cl.loop();
        rate.sleep();
    }
    return 0;
}