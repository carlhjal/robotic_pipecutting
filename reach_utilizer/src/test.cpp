#include <iostream>
#include <fstream>
#include <vector>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/nvp.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.hpp>  // Corrected header
#include <tinyxml2.h>

struct PoseData {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    double score = 0.0;

    geometry_msgs::msg::Pose toROSPose() const {
        geometry_msgs::msg::Pose pose;
        pose.position.x = matrix(0, 3);
        pose.position.y = matrix(1, 3);
        pose.position.z = matrix(2, 3);
        
        Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0);
        Eigen::Quaterniond q(rot);

        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        return pose;
    }
};

struct PoseDatabase {
    std::vector<PoseData> poses;
};

PoseDatabase loadXML(const std::string& filename) {
    PoseDatabase db;
    tinyxml2::XMLDocument doc;

    if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Error: Could not load XML file " << filename << std::endl;
        return db;
    }

    tinyxml2::XMLElement* root = doc.FirstChildElement("PoseDatabase");
    if (!root) {
        std::cerr << "Error: Missing <PoseDatabase> root element" << std::endl;
        return db;
    }

    for (tinyxml2::XMLElement* poseElem = root->FirstChildElement("Pose"); poseElem; poseElem = poseElem->NextSiblingElement("Pose")) {
        PoseData pose;
        tinyxml2::XMLElement* matrixElem = poseElem->FirstChildElement("Matrix");
        tinyxml2::XMLElement* scoreElem = poseElem->FirstChildElement("Score");

        if (matrixElem && scoreElem) {
            std::vector<double> values;
            const char* matrixText = matrixElem->GetText();
            std::istringstream iss(matrixText);
            double val;
            while (iss >> val) {
                values.push_back(val);
            }

            if (values.size() == 16) {
                for (int i = 0; i < 16; ++i) {
                    pose.matrix(i / 4, i % 4) = values[i];
                }
            }

            pose.score = std::stod(scoreElem->GetText());
            db.poses.push_back(pose);
        }
    }

    return db;
}

int main() {
    std::string filename = "/home/carl/thesis/thesis_ws/src/robotic_pipecutting/reach_utilizer/results/reach.db.xml";
    PoseDatabase db = loadXML(filename);

    for (size_t i = 0; i < db.poses.size(); ++i) {
        geometry_msgs::msg::Pose rosPose = db.poses[i].toROSPose();
        std::cout << "Pose " << i + 1 << ":\n";
        std::cout << "Position: (" << rosPose.position.x << ", " 
                  << rosPose.position.y << ", " << rosPose.position.z << ")\n";
        std::cout << "Orientation (quaternion): (" 
                  << rosPose.orientation.x << ", " 
                  << rosPose.orientation.y << ", " 
                  << rosPose.orientation.z << ", " 
                  << rosPose.orientation.w << ")\n";
        std::cout << "Score: " << db.poses[i].score << "\n\n";
    }

    return 0;
}
