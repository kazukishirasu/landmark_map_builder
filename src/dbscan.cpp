#include "landmark_map_builder/dbscan.h"

namespace landmark_map_builder
{

dbscan::dbscan(std::string param_file, std::string landmark_file, std::string save_file)
{
    std::cout << "Start DBSCAN clustering" << std::endl;
    param_file_ = param_file;
    landmark_file_ = landmark_file;
    save_file_ = save_file;
    load_yaml();
}

dbscan::~dbscan()
{
}

void dbscan::load_yaml()
{
    try{
        std::cout << "Load: " << landmark_file_.c_str() << std::endl;
        YAML::Node node = YAML::LoadFile(landmark_file_);
        YAML::Node landmark = node["landmark"];
        for (YAML::const_iterator it=landmark.begin(); it!=landmark.end(); ++it){
            Data_Points dp;
            dp.name = it->first.as<std::string>();
            YAML::Node config = landmark[dp.name];
            dp.pose.resize(config.size(), 2);
            dp.cluster = Eigen::VectorXd::Constant(config.size(), 1, -1);
            int i = 0;
            for (YAML::const_iterator it=config.begin(); it!=config.end(); ++it){
                std::string id = it->first.as<std::string>();
                dp.pose(i, 0) = it->second["pose"][0].as<float>();
                dp.pose(i, 1) = it->second["pose"][1].as<float>();
                i++;
            }
            dp_list_.push_back(dp);
        }
    }
    catch(const std::exception& e){
        std::cerr << e.what() << std::endl;
    }
    try{
        YAML::Node node = YAML::LoadFile(param_file_);
        eps_ = node["eps"].as<float>();
        minpts_ = node["minpts"].as<unsigned int>();
    }
    catch(const std::exception& e){
        std::cerr << e.what() << std::endl;
    }
}

void dbscan::main()
{
    std::cout << "eps = " << eps_ << ", minpts = " << minpts_ << std::endl;
    for (auto& dp:dp_list_){
        clustering(dp);
    }
    std::cout << "--------------------" << std::endl;
    save_yaml();
}

void dbscan::clustering(Data_Points& dp)
{
    std::cout << "--------------------" << std::endl << dp.name.c_str() << std::endl;
    unsigned int cluster_number = 1;
    for (size_t i = 0; i < dp.pose.rows(); i++){
        // cluster = -1：クラスタ未割り当て
        // cluster = 0 ：ノイズ点
        // cluster = 1~：クラスタ番号
        Eigen::VectorXd tmp = dp.cluster;
        if (dp.cluster(i, 0) == -1){
            // 各点までの距離を計算
            for (size_t j = 0; j < dp.pose.rows(); j++){
                double distance = (dp.pose.row(i) - dp.pose.row(j)).norm();
                if (distance < eps_){
                    tmp(j, 0) = cluster_number;
                }
            }
            // クラスタの確定
            if ((tmp.array() == cluster_number).count() >= minpts_){
                dp.cluster = tmp;
                cluster_number++;
            }else{
                dp.cluster(i, 0) = 0;
            }
        }
    }
    std::cout << "cluster sum:" << cluster_number - 1 << std::endl;
}

void dbscan::save_yaml()
{
    Landmarks landmark_list;
    for (const auto& dp:dp_list_){
        Landmark landmark;
        landmark.Class = dp.name;
        // クラスタの数だけ繰り返し
        for (size_t i = 1; i <= dp.cluster.maxCoeff(); i++){
            int j = 0;
            int size = (dp.cluster.array() == i).count();
            Eigen::MatrixX2d pose(size, 2);
            for (size_t k = 0; k < dp.cluster.rows(); k++){
                if (dp.cluster(k, 0) == i){
                    pose.row(j) = dp.pose.row(k);
                    j++;
                }
            }
            Eigen::RowVector2d mean = pose.colwise().mean();
            Eigen::MatrixXd centered = pose.rowwise() - mean;
            Eigen::Matrix2d cov = (centered.transpose() * centered) / double(pose.rows());
            LandmarkPose p;
            p.pose_cov.pose.position.x = mean[0];
            p.pose_cov.pose.position.y = mean[1];
            p.pose_cov.covariance[0] = cov(0, 0);
            p.pose_cov.covariance[1] = cov(0, 1);
            p.pose_cov.covariance[6] = cov(1, 0);
            p.pose_cov.covariance[7] = cov(1, 1);
            landmark.poses.push_back(p);
        }
        landmark_list.landmarks.push_back(landmark);
    }
    try{
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "landmark";
        out << YAML::BeginMap;
        for (const auto& landmark:landmark_list.landmarks){
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
                out << YAML::Key << "cov" << YAML::BeginSeq;
                out << YAML::Flow << YAML::BeginSeq;
                out << p.pose_cov.covariance[0] << p.pose_cov.covariance[1] << YAML::EndSeq;
                out << YAML::Flow << YAML::BeginSeq;
                out << p.pose_cov.covariance[6] << p.pose_cov.covariance[7] << YAML::EndSeq;
                out << YAML::EndSeq;
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
        fout << "# eps = " + std::to_string(eps_);
        fout << ", minpts = " + std::to_string(minpts_) + "\n";
        fout << out.c_str();
        std::cout << "Save to : " << save_file_.c_str() << std::endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
}
}

int main(int argc, char *argv[])
{
    landmark_map_builder::dbscan dbscan(argv[1], argv[2], argv[3]);
    dbscan.main();
    return 0;
}