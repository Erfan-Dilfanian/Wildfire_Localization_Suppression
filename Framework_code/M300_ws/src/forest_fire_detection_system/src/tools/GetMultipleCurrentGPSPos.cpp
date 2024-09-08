/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: GetCurrentGPSPos.cpp
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-01-13
 *
 *   @Description:
 *
 *******************************************************************************/


#include <sensor_msgs/NavSatFix.h>
#include <tools/PositionHelper.hpp>
#include <tools/PrintControl/FileWritter.hpp>
#include <tools/SystemLib.hpp>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdlib>  // For getenv

int main(int argc, char** argv) {
    ros::init(argc, argv, "get_multiple_current_gps_pos_node");
    int average_times = 10;
    FFDS::TOOLS::PositionHelper posHelper;

    YAML::Node root;  // Root node
    root["nodes"] = YAML::Node(YAML::NodeType::Sequence);  // Initialize 'nodes' as a sequence

    int point_id = 1;
    char user_input;

    do {
        sensor_msgs::NavSatFix gps = posHelper.getAverageGPS(average_times);
        PRINT_INFO(
            "current GPS position under %d average times is lon: %.9f, lat: %.9f, alt: %.9f",
            average_times, gps.longitude, gps.latitude, gps.altitude);
        
        YAML::Node new_node;
        new_node["id"] = point_id++;
        new_node["x"] = gps.latitude;
        new_node["y"] = gps.longitude;
        new_node["z"] = gps.altitude;

        root["nodes"].push_back(new_node);

        std::cout << "Move the drone and type 'y' to get new GPS position or any other key to exit: ";
        std::cin >> user_input;
	/*
        if (user_input == 'y' || user_input == 'Y') {
            moveDrone(); // Call the function to move the drone
        }*/
        
    } while (user_input == 'y' || user_input == 'Y');

    // Get the user's home directory from the environment variable
    const char* homeDir = getenv("HOME");
    if (homeDir == nullptr) {
        std::cerr << "Failed to get the HOME environment variable." << std::endl;
        return 1;
    }

    // Construct the directory path
    std::string directory = std::string(homeDir) + "/M300_ws/src/dji_osdk_ros_cv4/config/";
    std::string filepath = directory + "nodes.yaml";
    std::cout<<"filepath: "<<filepath<<std::endl;

    // Create the directory if it doesn't exist
    struct stat info;
    if (stat(directory.c_str(), &info) != 0) {
        if (mkdir(directory.c_str(), 0777) != 0) {
            std::cerr << "Failed to create directory: " << directory << std::endl;
            return 1;
        }
    }

    // Write the YAML nodes to a file
    std::ofstream fout(filepath);
    fout << root;
    fout.close();

    return 0;
}
