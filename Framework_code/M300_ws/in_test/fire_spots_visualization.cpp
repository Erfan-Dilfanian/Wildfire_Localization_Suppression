//
// Created by qin on 25/10/23.
//

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "vision_msgs/Detection2DArray.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <std_msgs/Float32.h> // Include Float32 message type for PixelError percentage

// Define a global vector to store centers
std::vector<cv::Point> centers;

// Global publisher for PixelError
ros::Publisher pixel_error_pub;

using namespace std;

void ImageBoxesCallback(const sensor_msgs::ImageConstPtr &msg,
                        const vision_msgs::Detection2DArrayConstPtr &msg_fire_spot) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Get the size of the received image
    int image_width = cv_ptr->image.cols;
    int image_height = cv_ptr->image.rows;
    ROS_INFO("Received image size: %d x %d", image_width, image_height);

    // convert dectection2DArray to vector<cv::Rect>
    vector<cv::Rect> fire_spots;
    LOG(INFO) << "fire spots number: " << msg_fire_spot->detections.size();
    for (auto &box: msg_fire_spot->detections) {

        // Calculate the center of the bounding box
        int center_x = box.bbox.center.x;
        int center_y = box.bbox.center.y;
        ROS_INFO("Center of bounding box: (%d, %d)", center_x, center_y);

        // Store the center in the global vector
        centers.emplace_back(center_x, center_y);

        // bounding boxes coordinates
        fire_spots.emplace_back(box.bbox.center.x - box.bbox.size_x / 2,
                                box.bbox.center.y - box.bbox.size_y / 2,
                                box.bbox.size_x,
                                box.bbox.size_y);
    }

    // Draw the bounding box
    for (auto &box: fire_spots) {
        cv::rectangle(cv_ptr->image, box, cv::Scalar(0, 0, 255), 2);
    }
   
    // Calculate the average center
    cv::Point average_center(0, 0);
    for (const auto &center : centers) {
        average_center.x += center.x;
        average_center.y += center.y;
    }
    if (!centers.empty()) {
        average_center.x /= centers.size();
        average_center.y /= centers.size();
        ROS_INFO("Average Center: (%d, %d)", average_center.x, average_center.y);

        // Draw the averaged center as a red line
        cv::line(cv_ptr->image, cv::Point(average_center.x - 10, average_center.y),
                 cv::Point(average_center.x + 10, average_center.y), cv::Scalar(0, 0, 255), 2);
        cv::line(cv_ptr->image, cv::Point(average_center.x, average_center.y - 10),
                 cv::Point(average_center.x, average_center.y + 10), cv::Scalar(0, 0, 255), 2);

        // Calculate PixelError as a percentage
        float pixel_error_percent = static_cast<float>((average_center.x - (image_width / 2)) / (image_width / 2)) * 100.0;
        ROS_INFO("PixelError Percentage: %.2f%%", pixel_error_percent);

        // Publish PixelError Percentage
        std_msgs::Float32 pixel_error_msg;
        pixel_error_msg.data = pixel_error_percent;
        pixel_error_pub.publish(pixel_error_msg);
    }
         cv::imshow("view", cv_ptr->image);
    cv::waitKey(1);


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fire_spots_visualization");
    ros::start();

    ros::NodeHandle nodeHandler;


// Initialize the publisher for PixelError
    pixel_error_pub = nodeHandler.advertise<std_msgs::Float32>("/pixel_error_percentage", 1);

    // message filter for images
//    message_filters::Subscriber<sensor_msgs::Image> sub_image;
//    sub_image.subscribe(nodeHandler, "/dji_osdk_ros/main_camera_images", 1);
    image_transport::ImageTransport it(nodeHandler);
    image_transport::SubscriberFilter sub_image(it, "/dji_osdk_ros/main_wide_RGB", 1);
    message_filters::Subscriber<vision_msgs::Detection2DArray> sub_fire_spot;
    sub_fire_spot.subscribe(nodeHandler, "/bounding_boxes/fire_spots", 1);

    // Sync the subscribed data
    message_filters::TimeSynchronizer<sensor_msgs::Image, vision_msgs::Detection2DArray>
            sync(sub_image, sub_fire_spot, 10);
    sync.registerCallback(boost::bind(&ImageBoxesCallback, _1, _2));

    while (nodeHandler.ok()) {
        ros::spinOnce();
    }

    ros::shutdown();

    return 0;
}
