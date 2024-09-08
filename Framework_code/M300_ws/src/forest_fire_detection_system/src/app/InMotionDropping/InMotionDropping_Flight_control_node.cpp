/** @file advanced_sensing_node.cpp
authors: Erfan Dilfanian, Huajun Dong
 */

//INCLUDE
#include <ros/ros.h>
#include <ros/package.h>
#include <dji_osdk_ros/common_type.h>
#include <iostream>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

#include <vector>
#include <random>
#include <cmath>


#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/SetGoHomeAltitude.h>
#include <dji_osdk_ros/GetGoHomeAltitude.h>
#include <dji_osdk_ros/SetCurrentAircraftLocAsHomePoint.h>
#include <dji_osdk_ros/SetAvoidEnable.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/EmergencyBrake.h>
#include <dji_osdk_ros/GetAvoidEnable.h>

#include <cstdlib>  // from uses: for getenv

#include<dji_osdk_ros/SetJoystickMode.h>
#include<dji_osdk_ros/JoystickAction.h>

#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/GimbalAction.h>
#include <dji_osdk_ros/CameraEV.h>
#include <dji_osdk_ros/CameraShutterSpeed.h>
#include <dji_osdk_ros/CameraAperture.h>
#include <dji_osdk_ros/CameraISO.h>
#include <dji_osdk_ros/CameraFocusPoint.h>
#include <dji_osdk_ros/CameraTapZoomPoint.h>
#include <dji_osdk_ros/CameraSetZoomPara.h>
#include <dji_osdk_ros/CameraZoomCtrl.h>
#include <dji_osdk_ros/CameraStartShootBurstPhoto.h>
#include <dji_osdk_ros/CameraStartShootAEBPhoto.h>
#include <dji_osdk_ros/CameraStartShootSinglePhoto.h>
#include <dji_osdk_ros/CameraStartShootIntervalPhoto.h>
#include <dji_osdk_ros/CameraStopShootPhoto.h>
#include <dji_osdk_ros/CameraRecordVideoAction.h>

#include <fstream>

# include <thread> // we need this header for multi thread programming

// #include <app/single_fire_point_task/SingleFirePointTaskManager.hpp>
#include <sensor_msgs/NavSatFix.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <vector>
#include <sensor_msgs/ChannelFloat32.h>
#include <glog/logging.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/QuaternionStamped.h>


#include <tools/MathLib.hpp>
// #include <tools/MathLib_1.hpp>

// #include <dji_sdk_ros/SetLocalPosRef.h>

#include "std_msgs/UInt16.h"

#include <fl/Headers.h>

#include "vision_msgs/Detection2DArray.h" //as I need it for fuzzy control

#include <std_msgs/Float32.h> // need it for pixel error percentage

#include <geometry_msgs/Point.h>

#include <atomic> // to define atomic variable

#include <filesystem> // for directory creation (C++17 and later)

#include <sys/stat.h> // For mkdir function

//CODE

using namespace dji_osdk_ros;
using namespace std;

/*
FFDS::APP::SingleFirePointTaskManager::SingleFirePointTaskManager() {



    task_control_client =
            nh.serviceClient<dji_osdk_ros::FlightTaskControl>("/flight_task_control");


    gpsPositionSub =
            nh.subscribe("dji_osdk_ros/gps_position", 10,
                         &SingleFirePointTaskManager::gpsPositionSubCallback, this);
    attitudeSub =
            nh.subscribe("dji_osdk_ros/attitude", 10,
                         &SingleFirePointTaskManager::attitudeSubCallback, this);*/



// gimbal_control_client = nh.serviceClient<dji_osdk_ros::GimbalAction>("gimbal_task_control");





// obtain the authorization when really needed... Now :)
/* obtainCtrlAuthority.request.enable_obtain = true;
 obtain_ctrl_authority_client.call(obtainCtrlAuthority);
 if (obtainCtrlAuthority.response.result) {
     PRINT_INFO("get control authority!");
 } else {
     PRINT_ERROR("can NOT get control authority!");
     return;
 }

 ros::Duration(3.0).sleep();
 //PRINT_INFO("initializing Done");
}
*/

/*FFDS::APP::SingleFirePointTaskManager::~SingleFirePointTaskManager() {
   /* obtainCtrlAuthority.request.enable_obtain = false;
    obtain_ctrl_authority_client.call(obtainCtrlAuthority);
    if (obtainCtrlAuthority.response.result) {
        PRINT_INFO("release control authority!");
    } else {
        PRINT_ERROR("can NOT release control authority!");
    }*/
//}

ros::Publisher servoPub;

ros::ServiceClient task_control_client;
ros::ServiceClient set_joystick_mode_client;
ros::ServiceClient joystick_action_client;

bool moveByPosOffset(FlightTaskControl &task, const JoystickCommand &offsetDesired,
                     float posThresholdInM,
                     float yawThresholdInDeg);

bool GeoPositioningFlag;

void CircularDivisionPlanner(const JoystickCommand &offsetDesired, uint32_t timeMs);


sensor_msgs::NavSatFix gps_position_;

geometry_msgs::PointStamped local_position_;

float euler[3];


void gpsPositionSubCallback2(
        const sensor_msgs::NavSatFix::ConstPtr &gpsPosition) {
    gps_position_ = *gpsPosition;
    // ROS_INFO("latitude is [%f]",gps_position_.latitude);
    // ROS_INFO("longitude is [%f]",gps_position_.longitude);

}

void QuaternionSubCallback(const geometry_msgs::QuaternionStamped msg) {


    float quat[4];
    quat[0] = msg.quaternion.x;
    quat[1] = msg.quaternion.y;
    quat[2] = msg.quaternion.z;
    quat[3] = msg.quaternion.w;

    euler[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                     1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    euler[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    euler[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]),
                     -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
    euler[0] = euler[0] * 180 / M_PI; // radian to degree
    euler[1] = euler[1] * 180 / M_PI; // radian to degree
    euler[2] = euler[2] * 180 / M_PI; // radian to degree

    // calibration. making frames equal. when drone pointing north, yaw be 0.
    // beware of changes between NED  and ENU
    euler[0] = 90 - euler[0];


    // euler[0] is yaw
}

void LocalPositionSubCallback(
        const geometry_msgs::PointStamped::ConstPtr &LocalPosition) {
    local_position_ = *LocalPosition;

}


/*sensor_msgs::NavSatFix
FFDS::APP::SingleFirePointTaskManager::getHomeGPosAverage(int times) {
    FFDS::TOOLS::PositionHelper posHelper;
    return posHelper.getAverageGPS(times);
}*/

sensor_msgs::NavSatFix getAverageGPS(const int);


float cosd(float angleDegrees) {
    double angleRadians = angleDegrees * (M_PI / 180.0);
    return cos(angleRadians);
}

float sind(float angleDegrees) {
    double angleRadians = angleDegrees * (M_PI / 180.0);
    return sin(angleRadians);
}

double Rad2Deg(double Rad) { return Rad * (180 / M_PI); }

double Deg2Rad(double Deg) { return Deg * (M_PI / 180); }

sensor_msgs::NavSatFix fire_gps;

static int N;//Fire spots number

static int M;

float INF = 1e100; // infinity

//Define destination class
class node {
public:
    double x, y; // coordinate of the node
    double z; //height of the node
    int id; // id of the node
};

class Velocity {
public:
    float Vx;
    float Vy;

    // Constructor
    Velocity(double vx, double vy) : Vx(vx), Vy(vy) {}

    void displayVelocity() const {
        cout << "Velocity (Vx, Vy): (" << Vx << ", " << Vy << ")" << endl;
    }
};


class CircularPathParams {
public:
    float radius;                         // Radius of the circle
    double theta_dot;                     // circular path angular velocity
    float theta_step_degrees;                     // theta step in degrees
    Velocity CircularVelocity;                        // Velocity components in x and y axes while traversing on the circular path
    float theta_step_radians;
    float total_time;
    float number_of_divisions;
    float time_step;
    float yawRate;
    // float time_step = theta_step_radians / theta_dot;

    void CalculateParams() {
        theta_step_radians = Deg2Rad(theta_step_degrees);
        total_time = (2 * M_PI) / theta_dot;         // total time drone traverse in the circular path
        number_of_divisions = 360 / theta_step_degrees;     // number of division the velocity function would be called
        time_step = total_time / number_of_divisions;
        yawRate = Rad2Deg(theta_dot);

    } // semicolon is not necessary here

    // Constructor with initialization list
    CircularPathParams(float r, double theta_dot_, float theta_step_degrees_, double vx = 0, double vy = 0)
            : radius(r), theta_dot(theta_dot_), theta_step_degrees(theta_step_degrees_), CircularVelocity(vx, vy) {
        CalculateParams();  // Call CalculateParams method to compute additional parameters
    }

    // note: it's important to initialize member variables in the initialization list in the same order as they are
    // declared in the class definition to avoid potential issues with member initialization order.

    // float time_step = theta_step_radians / theta_dot;

};

//Define vector to store all the nodes
std::vector <node> nodes_vec;

//Define the function to calculate the distance of nodes
double calculateDis(const node &a, const node &b) {
    return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
};

//A function defination for TSP
bool isVisited(bool visited[]) {
    for (int i = 1; i < N; i++) {
        if (visited[i] == false) {
            return false;
        }
    }
    return true;
}

void FireCallback(const geometry_msgs::PoseArrayConstPtr &fire_spots_GPS) {
    // print number of fire spots
    LOG(INFO) << "The number of fire spots: " << fire_spots_GPS->poses.size() << ".";
    // print the average GPS positions of fire spots
    double latitude = 0;
    double longitude = 0;
    double altitude = 0;
    for (const geometry_msgs::Pose &fire_spot: fire_spots_GPS->poses) {
        latitude += fire_spot.position.x;
        longitude += fire_spot.position.y;
        altitude += fire_spot.position.z;
    }
    latitude /= fire_spots_GPS->poses.size();
    longitude /= fire_spots_GPS->poses.size();
    altitude /= fire_spots_GPS->poses.size();
    LOG(INFO) << "The average GPS positions of fire spots: " << latitude << ", " << longitude << ", " << altitude
              << ".";

    fire_gps.latitude = latitude;
    fire_gps.longitude = longitude;
    fire_gps.altitude = altitude;

}

void
velocityAndYawRateControl(const JoystickCommand &offsetDesired, uint32_t timeMs, float abs_vel, float d, float height,
                          float delay);

void controlServo(int angle) {
    std_msgs::UInt16 msg;
    msg.data = angle;
    servoPub.publish(msg);
}


void FireCallback2(const geometry_msgs::PoseArrayConstPtr &fire_spots_GPS) {
    // Print number of fire spots
    LOG(INFO) << "The number of fire spots: " << fire_spots_GPS->poses.size() << ".";

    //Put subscribed fire spots number to N
    N = fire_spots_GPS->poses.size();

    //Define array to store all the nodes
    node nodes[N];


    //Put fire spots GPS into “nodes”， array of node class
    int i = 1;

    for (const geometry_msgs::Pose &fire_spot: fire_spots_GPS->poses) {
        nodes[i].id = i;
        nodes[i].x = fire_spot.position.x;
        nodes[i].y = fire_spot.position.y;
        nodes[i].z = fire_spot.position.z;
        i++;
    }


    //    homeGPos = getAverageGPS(50);
    ros::spinOnce();

    //Put home GPS into “nodes”， array of node class
    nodes[0].id = 0;
    nodes[0].x = gps_position_.latitude;
    nodes[0].y = gps_position_.longitude;
    nodes[0].z = gps_position_.altitude;

    //Get home GPS
    float homeGPS_posArray[3];
    homeGPS_posArray[0] = gps_position_.latitude;
    homeGPS_posArray[1] = gps_position_.longitude;
    homeGPS_posArray[2] = gps_position_.altitude;

    //Define a 2D array to store the distance
    double dis[N][N];

    //Calculate the distance of nodes
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            dis[i][j] = calculateDis(nodes[i], nodes[j]);
        }
    }

    //The following codes are for dynamic programming
    M = 1 << (N - 1);
    double dp[N][M];
    // store the path
    std::vector<int> path;

    //TSP Algorithm
    // ��ʼ��dp[i][0]
    for (int i = 0; i < N; i++) {
        dp[i][0] = dis[i][0];
    }
    // ���dp[i][j],�ȸ������ٸ�����
    for (int j = 1; j < M; j++) {
        for (int i = 0; i < N; i++) {
            dp[i][j] = INF;
            // �������j(��״̬j)�а������i,�򲻷��������˳�
            if (((j >> (i - 1)) & 1) == 1) {
                continue;
            }
            for (int k = 1; k < N; k++) {
                if (((j >> (k - 1)) & 1) == 0) {
                    continue;
                }
                if (dp[i][j] > dis[i][k] + dp[k][j ^ (1 << (k - 1))]) {
                    dp[i][j] = dis[i][k] + dp[k][j ^ (1 << (k - 1))];
                }
            }
        }
    }

    // ��Ƿ�������
    bool visited[N] = {false};
    // ǰ���ڵ���
    int pioneer = 0, min = INF, S = M - 1, temp;
    // ��������ż�������
    path.push_back(0);

    while (!isVisited(visited)) {
        for (int i = 1; i < N; i++) {
            if (visited[i] == false && (S & (1 << (i - 1))) != 0) {
                if (min > dis[pioneer][i] + dp[i][(S ^ (1 << (i - 1)))]) {
                    min = dis[pioneer][i] + dp[i][(S ^ (1 << (i - 1)))];
                    temp = i;
                }
            }
        }
        pioneer = temp;
        path.push_back(pioneer);
        visited[pioneer] = true;
        S = S ^ (1 << (pioneer - 1));
        min = INF;
    }
    //Till here we get the path vector, which stores the travelling sequence for the nodes

    // Put the sequenced nodes in array into predefined globalvector
    for (int i = 1; i < N; i++) {
        nodes_vec.push_back(nodes[path[i]]);
    }
}

char in_or_out;  // this variable tells whether you are doing indoor experiment or outdoor experiment

void ZigZagDivisionPlanner(const JoystickCommand &offsetDesired, uint32_t timeMs);

void LineOfFireCallback(const geometry_msgs::PoseArrayConstPtr &fire_spots_GPS) {

    cout << "LineOfFireCallback called";

    // Print number of fire spots
    LOG(INFO) << "The number of fire spots: " << fire_spots_GPS->poses.size() << ".";

    //Put subscribed fire spots number to N
    N = fire_spots_GPS->poses.size();

    // Define array to store all the nodes
    std::vector<node> nodes(fire_spots_GPS->poses.size());

    // Put fire spots GPS into “nodes”， array of node class
    int i = 0;
    for (const geometry_msgs::Pose &fire_spot: fire_spots_GPS->poses) {
        nodes[i].id = i;                   // fire index
        nodes[i].x = fire_spot.position.x; // fire latitude
        nodes[i].y = fire_spot.position.y; // fire longitude
        nodes[i].z = fire_spot.position.z; // fire altitude
        i++;
    }

    // Copy nodes to nodes_vec
    nodes_vec = nodes;
    /*
    for (int j = 0; j<nodes_vec.size(); j++){
    cout<<"nodes_vec: x:"<<nodes_vec[j].x<<"nodes_vec: y:"<<nodes_vec[j].y<<"nodes_vec: z:"<<nodes_vec[j].z<<endl;
    }
    cout<<"next attemp";
    */
}


// Define a structure to represent a 2D point
struct Point {
    float x, y;
};

// Structure to hold the parameters of a line
struct Line {
    double slope;
    double intercept;
    int num_inliers;
    std::vector<Point> inlier_points; // Store inliers
};

// Function to calculate the distance between two points
double distance(const Point &p1, const Point &p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

// Function to fit a line to a set of 2D points using RANSAC
Line fitLineRANSAC(const std::vector<Point> &points, int num_iterations, double threshold) {
    // Initialize variables to store the best-fitting line parameters
    Line best_line = {0.0, 0.0, 0, {}};

    // Random number generator for sampling points
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = 0; i < num_iterations; ++i) {
        // Randomly sample two points
        std::uniform_int_distribution<int> dist(0, points.size() - 1);
        int idx1 = dist(gen);
        int idx2 = dist(gen);

        // Fit a line to the sampled points (simple linear regression)
        float x1 = points[idx1].x, y1 = points[idx1].y;
        float x2 = points[idx2].x, y2 = points[idx2].y;
        double slope = (y2 - y1) / (x2 - x1);
        double intercept = y1 - slope * x1;

        // Count the number of inliers and store them
        int inliers = 0;
        std::vector<Point> current_inliers;
        for (const Point &p : points) {
            double d = std::abs(p.y - (slope * p.x + intercept));
            if (d < threshold) {
                inliers++;
                current_inliers.push_back(p);
            }
        }

        // Update the best-fitting line if the current line has more inliers
        if (inliers > best_line.num_inliers) {
            best_line.slope = slope;
            best_line.intercept = intercept;
            best_line.num_inliers = inliers;
            best_line.inlier_points = current_inliers;
        }
    }

    return best_line;
}

// Function to process the array and call RANSAC
// Line processArrayAndFitLine(const float fire_gps_local_pos[][3], int size) {

Line processArrayAndFitLine(const double fire_gps_local_pos[][3], int size, float threshold) {


    cout << "We are in ProcessArrayAndFitLine function";
    // Convert the array to vector of points
    std::vector <Point> points;
    for (int i = 0; i < size; ++i) {
        Point p;
        p.x = fire_gps_local_pos[i][0];
        p.y = fire_gps_local_pos[i][1];
        points.push_back(p);

        // Print the x and y values
        std::cout << "Point " << i + 1 << ": x = " << p.x << ", y = " << p.y << std::endl;


    }

    // Fit a line using RANSAC
    int num_iterations = 100000; // Adjust as needed

    return fitLineRANSAC(points, num_iterations, threshold);
}

// Function to locate the point on the fitted line closest to the first sample
/*
Point closestPointOnLine(const Line& line, const Point& first_sample) {
    // Calculate the point on the line closest to the first sample
    Point closest_point;
    closest_point.x = (first_sample.y - line.intercept + line.slope * first_sample.x) / (1 + std::pow(line.slope, 2));
    closest_point.y = line.slope * closest_point.x + line.intercept;
    return closest_point;
}
*/
// Function to calculate the point on the line closest to the given sample point
Point closestPointOnLine(const Line &line, const Point &sample_point) {
    Point closest_point;
    double slope_inverse = -1 / line.slope; // Slope of the line perpendicular to the given line

    // Calculate x-coordinate of the closest point using perpendicular distance formula
    closest_point.x = (sample_point.x + slope_inverse * sample_point.y - line.intercept + line.slope * line.intercept) /
                      (line.slope + slope_inverse);

    // Calculate y-coordinate of the closest point using the equation of the given line
    closest_point.y = line.slope * closest_point.x + line.intercept;

    return closest_point;
}

// Function to find the intersection point of two lines
Point intersectionPoint(const Line &best_line, const Point &first_sample) {
    Point intersection;

    // Calculate the slope of the line perpendicular to the best-fitting line
    double perpendicular_slope = -1.0 / best_line.slope;

    // Calculate the y-intercept of the line passing through the first sample
    double perpendicular_intercept = first_sample.y - perpendicular_slope * first_sample.x;

    // Calculate the x-coordinate of the intersection point
    intersection.x = (perpendicular_intercept - best_line.intercept) / (best_line.slope - perpendicular_slope);

    // Calculate the y-coordinate of the intersection point
    intersection.y = best_line.slope * intersection.x + best_line.intercept;

    return intersection;
}

// Function to traverse on the best-fitting line from a given point for a certain distance
Point traverseOnLine(const Line &best_line, const Point &starting_point, double distance) {
    Point new_point;

    // Calculate the change in x and y based on the slope of the best-fitting line
    double delta_x = distance / std::sqrt(1 + std::pow(best_line.slope, 2));
    double delta_y = best_line.slope * delta_x;

    // Determine the direction of traversal based on the slope of the line
    if (best_line.slope >= 0) {
        new_point.x = starting_point.x + delta_x;
        new_point.y = starting_point.y + delta_y;
    } else {
        new_point.x = starting_point.x - delta_x;
        new_point.y = starting_point.y - delta_y;
    }

    return new_point;
}

class ZigZagParams {  // A class for ZigZag pattern parameters
public:
    float length; //zigzag length
    float width;  //zigzag width
    int number;   //number of zigzag
    // int split;  //division in each branch of the zigzag
    float orientation; //orientation of the zigzag
    double velocity;
/*
    ZigZagParams(float l, float w, float o, int n, int s) {
        length = l;
        width = w;
        number = n;
        split = s;
        orientation = o;
    }
*/

};

void ZigZagPlanner(FlightTaskControl &task, ZigZagParams zz_params);

Point GPS2Coordinates(sensor_msgs::NavSatFix homeGPos, sensor_msgs::NavSatFix GPS){
    Point coordinates;
    double homeGPS_posArray[3] = {homeGPos.latitude, homeGPos.longitude, homeGPos.altitude};

    double GPS_posArray[3] = {GPS.latitude, GPS.longitude, GPS.altitude};

    ROS_INFO("homegpos latitude is [%f]", homeGPS_posArray[0]);
    ROS_INFO("homegpos longitude is [%f]", homeGPS_posArray[1]);
    ROS_INFO("homegpos attitude is [%f]", homeGPS_posArray[2]);

    ROS_INFO("currentpos latitude is [%f]", GPS_posArray[0]);
    ROS_INFO("currentgpos longitude is [%f]", GPS_posArray[1]);
    ROS_INFO("currentgpos attitude is [%f]", GPS_posArray[2]);

    // ros::Duration(2).sleep();

    double m[3];


    FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, GPS_posArray, m);

    ROS_INFO("x is [%f]", m[0]);
    ROS_INFO("y is [%f]", m[1]);

    coordinates.x = m[0];
    coordinates.y = m[1];

    return coordinates;

}


void doRANSAC(std::vector <node> nodes_vec, double fire_coordinates[][3], Line& best_Line, Point& starting_point, float threshold, double run_up_distance);

bool stopSLAM = false;
bool FakeFireSpotCounterMode;

void FireSpotCounter()
        {
cout<< "FireSpotCounter thread is running";
            const std::string package_path =
                    ros::package::getPath("dji_osdk_ros");
            const std::string config_path = package_path + "/config/geopositioning_params.yaml";
            PRINT_INFO("Load parameters from:%s", config_path.c_str());
            YAML::Node GeoPosConfig = YAML::LoadFile(config_path);

            // Accessing the integer value from the YAML file
            int number_of_fire_spots_criterion = GeoPosConfig["number_of_fire_spots_criterion"].as<int>(); // after this number zigzag would cut off
   /*
    char ManualSLAMstopper;
            while(stopSLAM == false){
        ros::spinOnce();
        cout<<"checking the number of Fire spots identified"<<endl;
        if (nodes_vec.size()>number_of_fire_spots_criterion){
            cout<<"cutting ZigZag to reduce SLAM error";
            stopSLAM = true;
            return;
        }
        cout<<"if YOu want the SLAM to stop, please enter y:"<<endl;
        cin>>ManualSLAMstopper;
        if(ManualSLAMstopper == 'y') {stopSLAM == true;};

cout<<"the FireSPotCounter thread finished working";
    }
*/
   int counter;
   // for debug:
   if(FakeFireSpotCounterMode==true) {

       while (counter < number_of_fire_spots_criterion) {
           counter++;
           std::this_thread::sleep_for(std::chrono::milliseconds(5));
           if(counter%1000 == 0){
           cout << "counter is:" << counter << endl;
           // show every 1000 times
           }


       }
   }
   else {
       while (nodes_vec.size() <= number_of_fire_spots_criterion) {

           cout << "number of found fire spots is:" << nodes_vec.size() << endl;
           ros::spinOnce(); // without this line you won't see the number of firespots regularly
       }
   }

   stopSLAM = true;
        };

// Function to create and configure the fuzzy logic engine
fl::Engine* createFuzzyEngine(double velocitymax);

// Function to perform fuzzy inference
double fuzzyInference(fl::Engine* engine, double inputError);

float Pixel_Error_Percentage;
void pixelErrorCallback(const std_msgs::Float32::ConstPtr& msg) {
    // Process the received pixel error percentage message
    Pixel_Error_Percentage = msg->data;
    ROS_INFO("Received PixelError Percentage: %.2f%%", Pixel_Error_Percentage);
    // Add your processing logic here
}

// Global variable to store centers of bounding boxes
std::vector<geometry_msgs::Point> fire_bbx_centers;

// Callback function to process the incoming Detection2DArray messages
void boundingBoxCallback(const vision_msgs::Detection2DArray::ConstPtr& msg)
{
    // Clear the previous centers
    fire_bbx_centers.clear();

    // Iterate through the detections in the message
    for (const auto& detection : msg->detections)
    {
        geometry_msgs::Point center;
        center.x = detection.bbox.center.x;
        center.y = detection.bbox.center.y;
        center.z = 0; // Assuming a 2D scenario, z can be set to 0

        // Store the center point
        fire_bbx_centers.push_back(center);
    }
}


// Function to calculate and print the average of the centers
// Function to calculate and print the average of the centers
geometry_msgs::Point calculateAndPrintAverageCenter() {
    geometry_msgs::Point avg_center;
/*
    if (fire_bbx_centers.empty()) {
        std::cout << "No bounding boxes to calculate average." << std::endl;
        avg_center.x = 0;
        avg_center.y = 0;
        avg_center.z = 0;
        return avg_center;
    }
*/
    double sum_x = 0.0;
    double sum_y = 0.0;

    for (const auto& center : fire_bbx_centers) {
        sum_x += center.x;
        sum_y += center.y;
    }

    avg_center.x = sum_x / fire_bbx_centers.size();
    avg_center.y = sum_y / fire_bbx_centers.size();
    avg_center.z = 0;

    std::cout << "Average Center: x = " << avg_center.x << ", y = " << avg_center.y << std::endl;

    return avg_center;
}

void FuzzyVelocityTraversal(const JoystickCommand &offsetDesired, uint32_t timeMs);

// Define a global atomic boolean variable
std::atomic<bool> stopFuzzyControl(false);

// Function for the first thread to ask the user to enter 'y'
void askUserToStop() {
    while (!stopFuzzyControl) {
        std::cout << "Enter 'y' to stop fuzzy control: ";
        char input;
        std::cin >> input;
        if (input == 'y') {
            stopFuzzyControl = true;
        }
    }
}

void setupPlot(const std::vector<float>& x, const std::vector<float>& y) {
    plt::plot(y, x, "bo"); // Switched x and y
    plt::xlabel("Y"); // Y-axis now represents X-coordinate
    plt::ylabel("X"); // X-axis now represents Y-coordinate
    plt::title("Scatter Plot of Points");
    plt::plot(std::vector<float>{0.0}, std::vector<float>{0.0}, "ro");
    plt::annotate("H", 0.0, 0.0);
    plt::grid(true);
}

std::string getHomeDirectory() {
    const char* homedir;
    if ((homedir = getenv("HOME")) == nullptr) {
        homedir = getenv("USERPROFILE"); // Windows
    }
    return std::string(homedir);
}

// Added function to create a directory
void createDirectory(const std::string& path) {
    int status = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status == 0) {
        std::cout << "Directory created: " << path << std::endl;
    } else {
        std::cerr << "Failed to create directory: " << path << std::endl;
    }
}


void writeVectorsToCSV(const std::vector<float>& x, const std::vector<float>& y, const std::string& path); // write fire spots x and y to a file

void writeLineToCSV(const Line& line, const std::string& path);

void writePointToCSV(const Point& point, const std::string& path);

std::string SaveAllPath; // to record stuff. Made it global to be recognizable by functions

std::string record_index;

int main(int argc, char **argv) {

    /*FFDS::MODULES::GimbalCameraOperator gcOperator;

      /*reset the camera and gimbal */
    // if (gcOperator.resetCameraZoom() && gcOperator.resetGimbal()) {
    //    PRINT_INFO("reset camera and gimbal successfully!")
    //} else {
    //   PRINT_WARN("reset camera and gimbal failed!")
    //}

/*
    std::string filename;
    std::cout << "Enter the name of the file: ";
    std::cin >> filename;

    // Open the file for writing
    std::ofstream outputFile(filename);

    // Check if the file is opened successfully
    if (!outputFile.is_open()) {
        std::cerr << "Error opening the file!" << std::endl;
        return 1; // Exit with error
    }

    // Write header
    outputFile
            << "M300_position: (s)\tM300.lat \tM300.long \tM300.alt \t fire.lat \t fire.long\t fire.alt \t fire_gps_expected.lat \t fire_gps_expected.long \tfire_gps_expected.alt \n  ";
*/
    ros::init(argc, argv, "flight_control_node");
    ros::NodeHandle nh;
    task_control_client = nh.serviceClient<FlightTaskControl>("/flight_task_control");
    auto set_go_home_altitude_client = nh.serviceClient<SetGoHomeAltitude>("/set_go_home_altitude");
    auto get_go_home_altitude_client = nh.serviceClient<GetGoHomeAltitude>("get_go_home_altitude");
    auto set_current_point_as_home_client = nh.serviceClient<SetCurrentAircraftLocAsHomePoint>(
            "/set_current_aircraft_point_as_home");
    auto enable_horizon_avoid_client = nh.serviceClient<SetAvoidEnable>("/set_horizon_avoid_enable");
    auto enable_upward_avoid_client = nh.serviceClient<SetAvoidEnable>("/set_upwards_avoid_enable");
    auto get_avoid_enable_client = nh.serviceClient<GetAvoidEnable>("get_avoid_enable_status");
    auto obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>(
            "obtain_release_control_authority");
    auto emergency_brake_client = nh.serviceClient<dji_osdk_ros::EmergencyBrake>("emergency_brake");

    set_joystick_mode_client = nh.serviceClient<SetJoystickMode>("set_joystick_mode");
    joystick_action_client = nh.serviceClient<JoystickAction>("joystick_action");

    // Here, you can add the code to set the home position using the /dji_sdk/set_local_pos_ref service
    /*ros::ServiceClient client = nh.serviceClient<dji_osdk_ros::SetLocalPosRef>("/set_local_pos_reference");

   // Wait for the service to become available
   if (!setLocalPosRefClient.waitForExistence(ros::Duration(5.0)))
   {
       ROS_ERROR("Service '/dji_sdk/set_local_pos_ref' not available.");
       return 1;
   }

   // Create the service request
   dji_sdk::SetLocalPosRef srv;

   // Call the service to set the home position
   if (setLocalPosRefClient.call(srv))
   {
       if (srv.response.result)
       {
           ROS_INFO("Home position set successfully.");
       }
       else
       {
           ROS_ERROR("Failed to set home position.");
       }
   }
   else
   {
       ROS_ERROR("Service call failed.");
       return 1;
   }

*/
// scenarios
    cout << "please select case (all cases are for single fire point): " << std::endl
         << "[a] In-motion-dropping without geo-positioning" << std::endl
         << "[b] Hovering dropping after geo-positioning" << std::endl << "[c] In-motion-dropping after geo-positioning"
         << std::endl << "[d] guided-IN-motion-dropping after geopositioning"
         << std::endl << "[e] Multiple fire spots hovering mode"
         << std::endl << "[f] Line of fire" << std::endl;
    char scenario;
    cin >> scenario;

    cout << "indoor test or outdoor test?" << endl << "[a] indoor" << endl << "[b] outdoor" << endl;
    cin >> in_or_out;

    cout<<"please enter record index";
    cin>>record_index;

    /*
    cout << "please enter camera pitch angle in degree (no f at end please)" << endl;
    float camera_pitch;
    cin >> camera_pitch;



    cout << "please enter valve delay in miliseconds" << endl;
    cin >> release_delay;

    float height;
    cout << "please enter altitude to reach" << endl;
    cin >> height;
     */

    // load yaml parameters
    const std::string package_path =
            ros::package::getPath("dji_osdk_ros");
    const std::string config_path = package_path + "/config/general_params.yaml";
    PRINT_INFO("Load parameters from:%s", config_path.c_str());
    YAML::Node GeneralConfig = YAML::LoadFile(config_path);

    float camera_pitch = GeneralConfig["general_params"]["camera_pitch"].as<float>();
    int release_delay = GeneralConfig["general_params"]["release_delay"].as<int>();
    float height = GeneralConfig["general_params"]["height"].as<float>();
    float lateral_adjustment = GeneralConfig["general_params"]["lateral_adjustment"].as<float>();
    float gimbal_yaw_adjustment = GeneralConfig["general_params"]["gimbal_yaw_adjustment"].as<float>();
    float threshold = GeneralConfig["general_params"]["threshold"].as<float>();
    double run_up_distance = GeneralConfig["general_params"]["run_up_distance"].as<float>();
    bool apply_fuzzy_control = GeneralConfig["general_params"]["apply_fuzzy_control"].as<bool>();
    double VelocityMax = GeneralConfig["general_params"]["velocity_max"].as<double>(); // max velocity for fuzzy controller
    double inputLateralAdjustment = GeneralConfig["general_params"]["user_input_lateral_adjustment"].as<bool>();
    FakeFireSpotCounterMode = GeneralConfig["general_params"]["fake_fire_spot_counter_mode"].as<bool>();
    double yaw_rate_adjustment = GeneralConfig["general_params"]["yaw_rate_adjustmnet_for_circular_path"].as<double>();
    float abs_vel = GeneralConfig["general_params"]["approach_absolute_velocity"].as<double>(); // absolute velocity to approach line of fire
    double time_of_approach = GeneralConfig["general_params"]["time_of_approach"].as<double>(); // time of in motion dropping in miliseconds
    bool inputExtraYawAdjustment = GeneralConfig["general_params"]["extra_yaw_adjustment"].as<bool>();



    std::cout << "Camera Pitch: " << camera_pitch << std::endl;
    std::cout << "Release Delay: " << release_delay << std::endl;
    std::cout << "Height: " << height << std::endl;
    std::cout << "Lateral Adjustment: " << lateral_adjustment << std::endl;
    std::cout << "Gimbal Yaw Adjustment: " << gimbal_yaw_adjustment << std::endl;
    std::cout << "Threshold: " << threshold << std::endl;
    std::cout << "run up distance: " << run_up_distance << std::endl;
    std::cout << "apply_fuzzy_control: " << apply_fuzzy_control << std::endl;


    auto gimbal_control_client = nh.serviceClient<GimbalAction>("gimbal_task_control");
    auto camera_set_EV_client = nh.serviceClient<CameraEV>("camera_task_set_EV");
    auto camera_set_shutter_speed_client = nh.serviceClient<CameraShutterSpeed>("camera_task_set_shutter_speed");
    auto camera_set_aperture_client = nh.serviceClient<CameraAperture>("camera_task_set_aperture");
    auto camera_set_iso_client = nh.serviceClient<CameraISO>("camera_task_set_ISO");
    auto camera_set_focus_point_client = nh.serviceClient<CameraFocusPoint>("camera_task_set_focus_point");
    auto camera_set_tap_zoom_point_client = nh.serviceClient<CameraTapZoomPoint>("camera_task_tap_zoom_point");
    auto camera_set_zoom_para_client = nh.serviceClient<CameraSetZoomPara>("camera_task_set_zoom_para");
    auto camera_task_zoom_ctrl_client = nh.serviceClient<CameraZoomCtrl>("camera_task_zoom_ctrl");
    auto camera_start_shoot_single_photo_client = nh.serviceClient<CameraStartShootSinglePhoto>(
            "camera_start_shoot_single_photo");
    auto camera_start_shoot_aeb_photo_client = nh.serviceClient<CameraStartShootAEBPhoto>(
            "camera_start_shoot_aeb_photo");
    auto camera_start_shoot_burst_photo_client = nh.serviceClient<CameraStartShootBurstPhoto>(
            "camera_start_shoot_burst_photo");
    auto camera_start_shoot_interval_photo_client = nh.serviceClient<CameraStartShootIntervalPhoto>(
            "camera_start_shoot_interval_photo");
    auto camera_stop_shoot_photo_client = nh.serviceClient<CameraStopShootPhoto>("camera_stop_shoot_photo");
    auto camera_record_video_action_client = nh.serviceClient<CameraRecordVideoAction>("camera_record_video_action");



/*

  std::cout << "Please select the estimated orientation of the fire: ";

    std::cout
            << "| Available commands:                                            |"
            << std::endl;
    std::cout
            << "| [a] North                                |"
            << std::endl;
    std::cout
            << "| [b] North-west             |"
            << std::endl;
    std::cout << "| [c] West  |"
              << std::endl;
    std::cout << "| [d] South-West |"
              << std::endl;
    std::cout << "| [e] South |"
              << std::endl;
    std::cout << "| [f] South-East |"
              << std::endl;
    std::cout << "| [g] East |"
              << std::endl;
    std::cout << "| [h] North-East |"
              << std::endl;
*/



/*
    char inputChar;
    std::cin >> inputChar;

    switch (inputChar) {
        case 'a':
        {
            break;
        }

        
    }
  */

    EmergencyBrake emergency_brake;
    FlightTaskControl control_task;
    ObtainControlAuthority obtainCtrlAuthority;

    obtainCtrlAuthority.request.enable_obtain = true;
    obtain_ctrl_authority_client.call(obtainCtrlAuthority);

    ros::Subscriber gpsPositionSub;
    gpsPositionSub =
            nh.subscribe("dji_osdk_ros/gps_position", 10,
                         gpsPositionSubCallback2);


    ros::Subscriber LocalPositionSub;
    LocalPositionSub =
            nh.subscribe("dji_osdk_ros/local_position", 10,
                         LocalPositionSubCallback);

    ros::Subscriber QuaternionSub;
    QuaternionSub =
            nh.subscribe("dji_osdk_ros/attitude", 10,
                         QuaternionSubCallback);


    servoPub = nh.advertise<std_msgs::UInt16>("servo", 10); // Initialize servoPub

    // subscribe to the fire GPS
    ros::Subscriber fire_spots_GPS_sub = nh.subscribe("/position/fire_spots_GPS", 1, FireCallback);

    // Subscribe to the bounding boxes topic
    ros::Subscriber bbx_sub = nh.subscribe("/bounding_boxes/fire_spots", 10, boundingBoxCallback);

    sensor_msgs::NavSatFix homeGPos = getAverageGPS(50);
    double homeGPS_posArray[2]; // note that double holds more digits comapred to float
    homeGPS_posArray[0] = homeGPos.latitude;
    homeGPS_posArray[1] = homeGPos.longitude;

    // FFDS::TOOLS::T a_pos[2];


    Point recent_drone_coord; // recent drone coordinates as a Point class



    if (scenario == 'a') {


        float lat;
        float lon;
        float alt;
        cout << "please enter fire's latitude" << endl;
        cin >> lat;
        cout << endl;
        cout << "please enter fire's longitude" << endl;
        cin >> lon;
        cout << endl;
        cout << "please enter lateral adjustment" << endl;
        float lateral_adjustment;
        cout << endl;
        cin >> lateral_adjustment;
        // cout<<"please enter fire's alt"<<endl;
        //cin>>alt;

        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if (control_task.response.result == false) {
            ROS_ERROR_STREAM("Takeoff task failed");
        }

        if (control_task.response.result == true) {
            ROS_INFO_STREAM("Takeoff task successful");
            // ros::Duration(2.0).sleep();




            fire_gps.latitude = lat;
            fire_gps.longitude = lon;
            fire_gps.altitude = alt;

/*
            fire_gps.latitude = 45.45842238198102;
            fire_gps.longitude = -73.93238311980387;
            fire_gps.altitude = 111.356392;
*/

            double fire_GPS_posArray[2]; // posArray :  Position Array

            fire_GPS_posArray[0] = fire_gps.latitude;
            fire_GPS_posArray[1] = fire_gps.longitude;

            ros::spinOnce();


            ROS_INFO("fire_GPS_posArray[0] Is [%f]", fire_GPS_posArray[0]);
            ROS_INFO("fire_GPS_posArray[1] Is [%f]", fire_GPS_posArray[1]);

            double fire_gps_local_pos[2];

            FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, fire_GPS_posArray, fire_gps_local_pos);

            ROS_INFO("fire_gps_local_pos[0] Is [%f]", fire_gps_local_pos[0]);
            ROS_INFO("fire_gps_local_pos[1] Is [%f]", fire_gps_local_pos[1]);
            ROS_INFO("fire_gps_local_pos[2] Is [%f]", fire_gps_local_pos[2]);

            ros::spinOnce();

            moveByPosOffset(control_task, {0, 0, height - 1, 0}, 1, 3);

            double mission_start_pos[3] = {fire_gps_local_pos[0] - 10, fire_gps_local_pos[1] + 8,
                                           9}; // it also can be current x y z

            ROS_INFO("homegpos latitude is [%f]", homeGPS_posArray[0]);
            ROS_INFO("homegpos longitude is [%f]", homeGPS_posArray[1]);
            ROS_INFO("homegpos attitude is [%f]", homeGPS_posArray[2]);


            float yaw_const = 0;

            moveByPosOffset(control_task,
                            {mission_start_pos[0], mission_start_pos[1], 0, yaw_const}, 1, 3);

            ros::spinOnce();

            double current_GPS_posArray[3];
            current_GPS_posArray[0] = gps_position_.latitude;
            current_GPS_posArray[1] = gps_position_.longitude;
            current_GPS_posArray[2] = gps_position_.altitude;

            FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, current_GPS_posArray, mission_start_pos);


            float yaw_adjustment; // yaw adjustment before approach
            float deltaX = fire_gps_local_pos[0] - mission_start_pos[0];
            float deltaY = fire_gps_local_pos[1] - mission_start_pos[1];

            ROS_INFO("deltaX is [%f]", deltaX);
            ROS_INFO("deltaY is [%f]", deltaY);


            yaw_adjustment = Rad2Deg(atan2(deltaY, deltaX)); // note that tan2 output is in radian
            // Also I added 90 as we want the yaw angle from x axis which is in Y direction

            // fl::Engine* engine = new fl::Engine;

            moveByPosOffset(control_task,
                            {-lateral_adjustment * sind(yaw_adjustment), lateral_adjustment * cosd(yaw_adjustment), 0,
                             0}, 1,
                            3);

            ROS_INFO("yaw_adjustment_angle is [%f]", yaw_adjustment);
            moveByPosOffset(control_task, {0, 0, 0, yaw_adjustment}, 1,
                            3);  // note that x y z goes into this function

            // velocity mission

            float d = sqrt(
                    pow(fire_gps_local_pos[0] - mission_start_pos[0], 2) +
                    pow(fire_gps_local_pos[1] - mission_start_pos[1], 2));

            float abs_vel = 5; // absolute velocity that needs to be projected






            velocityAndYawRateControl({abs_vel * cosd(yaw_adjustment), abs_vel * sind(yaw_adjustment), 0}, time_of_approach,
                                      abs_vel, d, height, release_delay);


            ROS_INFO_STREAM("Step 1 over!EmergencyBrake for 2s\n");
            emergency_brake_client.call(emergency_brake);
            ros::Duration(2).sleep();
        }

    }
    if (scenario == 'b' || scenario == 'c' || scenario == 'd') {


        double fire_GPS_posArray[3]; // posArray :  Position Array


        sensor_msgs::NavSatFix fire_gps_expected;
        float epsilon;

        float diff_latitude;
        float diff_longitude;
        float diff_altitude;

        float zz_l; //zigzag_length
        float zz_w; //zigzag_width

        cout << "please enter zigzag length (like 8 meter)" << endl;
        cin >> zz_l;
        cout << "please enter zigzag width (like 4 meter)" << endl;
        cin >> zz_w;

        float split; // split value
        cout << "please enter the split value, like 12" << endl;
        cin >> split;


        std::cout << "Please enter the approximate expected GPS position of fire: " << std::endl;
        std::cout << "Latitude: ";
        std::cin >> fire_gps_expected.latitude;
        std::cout << "Longitude: ";
        std::cin >> fire_gps_expected.longitude;
        std::cout << "Altitude: ";
        std::cin >> fire_gps_expected.altitude;

        cout << "please enter the epsilon, the allowed geolocalizing error, like 0.001" << endl;
        cin >> epsilon;

        //note that inputs shoudl be before take off!
        double m[3];

        double current_GPS_posArray[3];

        float yaw_const;
        std::cout << " please enter initial yaw angle in degree-Z axes downward" << std::endl;
        std::cin >> yaw_const;

        float gimbal_yaw_adjustment;
        cout << "please enter gimbal yaw adjustment" << endl;
        cin >> gimbal_yaw_adjustment;


        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if (control_task.response.result == false) {
            ROS_ERROR_STREAM("Takeoff task failed");
        }

        if (control_task.response.result == true) {
            ROS_INFO_STREAM("Takeoff task successful");
            // ros::Duration(2.0).sleep();

            moveByPosOffset(control_task, {0, 0, 0, yaw_const}, 1, 3);

            ros::spinOnce();
            ROS_INFO("euler1 is [%f]", euler[0]);
            ROS_INFO("euler2 is [%f]", euler[1]);
            ROS_INFO("euler3 is [%f]", euler[2]);

            ROS_INFO("yaw_const is [%f]", yaw_const);


            GimbalAction gimbalAction;
            gimbalAction.request.is_reset = false;
            gimbalAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
            gimbalAction.request.rotationMode = 0;
            gimbalAction.request.pitch = camera_pitch;
            gimbalAction.request.roll = 0.0f;
            // gimbalAction.request.yaw = -yaw_const+90;
            // gimbalAction.request.yaw = 180.0f + gimbal_yaw_adjustment;
            // gimbalAction.request.yaw = -180.0f+gimbal_yaw_adjustment;
            gimbalAction.request.yaw = gimbal_yaw_adjustment;
            gimbalAction.request.time = 0.5;
            gimbal_control_client.call(gimbalAction);

            cout << "camera angle changed!" << endl;


            ROS_INFO_STREAM("Move by position offset request sending ...");
            moveByPosOffset(control_task, {0, 0, height - 1, yaw_const}, 1, 3);

            cout << "M300 rotated";


            if (in_or_out == 'b') {
                bool SLAM_flag = 0;
                while (SLAM_flag == 0) {
                    cout << "please rosrun detection and SLAM nodes. Then press 1" << endl;
                    int detect_index;//detection_starter_indicator
                    cin >> detect_index;
                    if (detect_index == 1) { SLAM_flag = 1; }
                    else { cout << "Please press 1 if SLAM is initiated"; }
                }



                // Print the entered GPS coordinates
                std::cout << "Entered GPS position: " << std::endl;
                std::cout << "Latitude: " << fire_gps_expected.latitude << std::endl;
                std::cout << "Longitude: " << fire_gps_expected.longitude << std::endl;
                std::cout << "Altitude: " << fire_gps_expected.altitude << std::endl;


                while (true) {


                    ros::spinOnce();


                    double frequency = 30; // 30 Hz
                    ros::Rate rate(frequency);


                    ROS_INFO("destination y is [%f] and x is [%f]: ", zz_l * sind(yaw_const), zz_l * cosd(yaw_const));

                    float zzl1 = -zz_l * sind(yaw_const) / split;
                    float zzl2 = zz_l * cosd(yaw_const) / split;

                    for (int i = 0; i < split; i++) {

                        moveByPosOffset(control_task, {zzl1, zzl2, 0, yaw_const}, 1, 3);

                        ros::spinOnce();
                        rate.sleep();
                        cout << "ROS spinned" << endl;
                        /* outputFile << std::setprecision(10) << gps_position_.latitude << "\t" << std::setprecision(10)
                                    << gps_position_.longitude << "\t" << std::setprecision(10) << gps_position_.altitude
                                    << "\t" << std::setprecision(10) << fire_gps.latitude << "\t"
                                    << std::setprecision(10)
                                    << fire_gps.longitude << "\t" << std::setprecision(10) << fire_gps.altitude << "\t"
                                    << std::setprecision(10) << fire_gps_expected.latitude << "\t"
                                    << std::setprecision(10)
                                    << fire_gps_expected.longitude << "\t" << std::setprecision(10)
                                    << fire_gps_expected.altitude << "\n";
 */
                    }


                    current_GPS_posArray[0] = gps_position_.latitude;
                    current_GPS_posArray[1] = gps_position_.longitude;
                    current_GPS_posArray[2] = gps_position_.altitude;

                    ROS_INFO("homegpos latitude is [%f]", homeGPS_posArray[0]);
                    ROS_INFO("homegpos longitude is [%f]", homeGPS_posArray[1]);
                    ROS_INFO("homegpos attitude is [%f]", homeGPS_posArray[2]);

                    ROS_INFO("currentpos latitude is [%f]", current_GPS_posArray[0]);
                    ROS_INFO("currentgpos longitude is [%f]", current_GPS_posArray[1]);
                    ROS_INFO("currentgpos attitude is [%f]", current_GPS_posArray[2]);

                    // ros::Duration(2).sleep();


                    FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, current_GPS_posArray, m);

                    ROS_INFO("x is [%f]", m[0]);
                    ROS_INFO("y is [%f]", m[1]);





                    /* ROS_INFO("x is [%f]",local_position_.point.x);
                     ROS_INFO("y is [%f]",local_position_.point.y);
                     ROS_INFO("z is [%f]",local_position_.point.z);*/
                    //ROS_INFO("latitude is [%f]",gps_position_.latitude);
                    //ROS_INFO("longitude is [%f]",gps_position_.longitude);
                    //ros::spin(); //here is good?
                    ROS_INFO_STREAM("first zigzag line completed!");

                    diff_latitude = std::abs(fire_gps_expected.latitude - fire_gps.latitude);
                    diff_longitude = std::abs(fire_gps_expected.longitude - fire_gps.longitude);
                    diff_altitude = std::abs(fire_gps_expected.altitude - fire_gps.altitude);

                    cout << "diff_latitude is" << diff_latitude << endl;
                    cout << "diff_longitude is" << diff_longitude << endl;
                    cout << "diff_latitude is" << diff_altitude << endl;

                    // Check if the difference exceeds the epsilon value
                    if (diff_latitude < epsilon || diff_longitude < epsilon) {
                        cout << "desirable difference" << endl;
                        break;

                        // altitude difference does not matter
                    }

                    float zzw1 = zz_w * cosd(yaw_const) / split;
                    float zzw2 = zz_w * sind(yaw_const) / split;

                    for (int i = 0; i < split; i++) {

                        moveByPosOffset(control_task, {zzw1, zzw2, 0, yaw_const}, 1, 3);

                        ros::spinOnce();
                        rate.sleep();
                        cout << "ROS spinned" << endl;
                        /* outputFile << std::setprecision(10) << gps_position_.latitude << "\t" << std::setprecision(10)
                                    << gps_position_.longitude << "\t" << std::setprecision(10) << gps_position_.altitude
                                    << "\t" << std::setprecision(10) << fire_gps.latitude << "\t"
                                    << std::setprecision(10)
                                    << fire_gps.longitude << "\t" << std::setprecision(10) << fire_gps.altitude << "\t"
                                    << std::setprecision(10) << fire_gps_expected.latitude << "\t"
                                    << std::setprecision(10)
                                    << fire_gps_expected.longitude << "\t" << std::setprecision(10)
                                    << fire_gps_expected.altitude << "\n";
 */

                    }

                    ros::spinOnce();

                    diff_latitude = std::abs(fire_gps_expected.latitude - fire_gps.latitude);
                    diff_longitude = std::abs(fire_gps_expected.longitude - fire_gps.longitude);
                    diff_altitude = std::abs(fire_gps_expected.altitude - fire_gps.altitude);

                    cout << "diff_latitude is" << diff_latitude << endl;
                    cout << "diff_longitude is" << diff_longitude << endl;
                    cout << "diff_latitude is" << diff_altitude << endl;


                    if (diff_latitude < epsilon && diff_longitude < epsilon) {
                        cout << "desirable difference" << endl;
                        break;
                    }


                    current_GPS_posArray[0] = gps_position_.latitude;
                    current_GPS_posArray[1] = gps_position_.longitude;
                    current_GPS_posArray[2] = gps_position_.altitude;

                    FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, current_GPS_posArray, m);

                    ROS_INFO("x is [%f]", m[0]);
                    ROS_INFO("y is [%f]", m[1]);


                    ROS_INFO_STREAM("Second zigzag line completed!");

                    float zzl3 = zz_l * sind(yaw_const) / split;
                    float zzl4 = -zz_l * cosd(yaw_const) / split;

                    for (int i = 0; i < split; i++) {

                        moveByPosOffset(control_task, {zzl3, zzl4, 0.0, yaw_const}, 0.8, 3);

                        ros::spinOnce();
                        rate.sleep();
                        cout << "ROS spinned" << endl;
                        /*  outputFile << std::setprecision(10) << gps_position_.latitude << "\t" << std::setprecision(10)
                                     << gps_position_.longitude << "\t" << std::setprecision(10) << gps_position_.altitude
                                     << "\t" << std::setprecision(10) << fire_gps.latitude << "\t"
                                     << std::setprecision(10)
                                     << fire_gps.longitude << "\t" << std::setprecision(10) << fire_gps.altitude << "\t"
                                     << std::setprecision(10) << fire_gps_expected.latitude << "\t"
                                     << std::setprecision(10)
                                     << fire_gps_expected.longitude << "\t" << std::setprecision(10)
                                     << fire_gps_expected.altitude << "\n";
  */
                    }

                    ros::spinOnce();

                    diff_latitude = std::abs(fire_gps_expected.latitude - fire_gps.latitude);
                    diff_longitude = std::abs(fire_gps_expected.longitude - fire_gps.longitude);
                    diff_altitude = std::abs(fire_gps_expected.altitude - fire_gps.altitude);

                    cout << "diff_latitude is" << diff_latitude << endl;
                    cout << "diff_longitude is" << diff_longitude << endl;
                    cout << "diff_latitude is" << diff_altitude << endl;


                    if (diff_latitude < epsilon && diff_longitude < epsilon) {
                        cout << "desirable difference" << endl;
                        break;
                    }


                    ROS_INFO_STREAM("Third zigzag line completed!");


                    float zzw3 = zz_w * cosd(yaw_const) / split;
                    float zzw4 = zz_w * sind(yaw_const) / split;

                    for (int i = 0; i < split; i++) {

                        moveByPosOffset(control_task, {zzw3, zzw4, 0.0, yaw_const}, 1, 3);

                        ros::spinOnce();
                        rate.sleep();
                        cout << "ROS spinned" << endl;
                        /*     outputFile << std::setprecision(10) << gps_position_.latitude << "\t" << std::setprecision(10)
                                        << gps_position_.longitude << "\t" << std::setprecision(10) << gps_position_.altitude
                                        << "\t" << std::setprecision(10) << fire_gps.latitude << "\t"
                                        << std::setprecision(10)
                                        << fire_gps.longitude << "\t" << std::setprecision(10) << fire_gps.altitude << "\t"
                                        << std::setprecision(10) << fire_gps_expected.latitude << "\t"
                                        << std::setprecision(10)
                                        << fire_gps_expected.longitude << "\t" << std::setprecision(10)
                                        << fire_gps_expected.altitude << "\n";
     */
                    }

                    ros::spinOnce();

                    diff_latitude = std::abs(fire_gps_expected.latitude - fire_gps.latitude);
                    diff_longitude = std::abs(fire_gps_expected.longitude - fire_gps.longitude);
                    diff_altitude = std::abs(fire_gps_expected.altitude - fire_gps.altitude);

                    cout << "diff_latitude is" << diff_latitude << endl;
                    cout << "diff_longitude is" << diff_longitude << endl;
                    cout << "diff_latitude is" << diff_altitude << endl;


                    if (diff_latitude < epsilon && diff_longitude < epsilon) {
                        cout << "desirable difference" << endl;
                        break;
                    }


                    ROS_INFO_STREAM("Fourth zigzag line completed!");


                    float zzl5 = -zz_l * sind(yaw_const) / split;
                    float zzl6 = zz_l * cosd(yaw_const) / split;

                    for (int i = 0; i < split; i++) {

                        moveByPosOffset(control_task, {zzl5, zzl6, 0.0, yaw_const}, 1, 3);

                        ros::spinOnce();
                        rate.sleep();
                        cout << "ROS spinned" << endl;
                        /* outputFile << std::setprecision(10) << gps_position_.latitude << "\t" << std::setprecision(10)
                                    << gps_position_.longitude << "\t" << std::setprecision(10) << gps_position_.altitude
                                    << "\t" << std::setprecision(10) << fire_gps.latitude << "\t"
                                    << std::setprecision(10)
                                    << fire_gps.longitude << "\t" << std::setprecision(10) << fire_gps.altitude << "\t"
                                    << std::setprecision(10) << fire_gps_expected.latitude << "\t"
                                    << std::setprecision(10)
                                    << fire_gps_expected.longitude << "\t" << std::setprecision(10)
                                    << fire_gps_expected.altitude << "\n";
 */
                    }

                    ros::spinOnce();

                    diff_latitude = std::abs(fire_gps_expected.latitude - fire_gps.latitude);
                    diff_longitude = std::abs(fire_gps_expected.longitude - fire_gps.longitude);
                    diff_altitude = std::abs(fire_gps_expected.altitude - fire_gps.altitude);

                    cout << "diff_latitude is" << diff_latitude << endl;
                    cout << "diff_longitude is" << diff_longitude << endl;
                    cout << "diff_latitude is" << diff_altitude << endl;

                    ROS_INFO_STREAM("fifth zigzag line completed!");

                    ros::spinOnce();

                    current_GPS_posArray[0] = gps_position_.latitude;
                    current_GPS_posArray[1] = gps_position_.longitude;
                    current_GPS_posArray[2] = gps_position_.altitude;

                    FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, current_GPS_posArray, m);

                    ROS_INFO("x is [%f]", m[0]);
                    ROS_INFO("y is [%f]", m[1]);

                    ros::spinOnce();

                    if (diff_latitude < epsilon && diff_longitude < epsilon) {
                        cout << "desirable difference" << endl;
                        break;
                    } else { break; }






                    // moveByPosOffset(control_task, {zz_w*cosd(yaw_const), zz_w*sind(yaw_const), 0.0, yaw_const}, 1, 3);
                    // moveByPosOffset(control_task, {-3*sind(yaw_const), static_cast<DJI::OSDK::float32_t>(-6.5*cosd(yaw_const)), 0.0, yaw_const}, 1, 3);

// the more generous you are in threshold, the more agile your drone would be




                }


                if (diff_latitude < epsilon && diff_longitude < epsilon) {

                    fire_GPS_posArray[0] = fire_gps.latitude;
                    fire_GPS_posArray[1] = fire_gps.longitude;
                    fire_GPS_posArray[2] = fire_gps.altitude;
                } else {
                    fire_GPS_posArray[0] = fire_gps_expected.latitude;
                    fire_GPS_posArray[1] = fire_gps_expected.longitude;
                    fire_GPS_posArray[2] = fire_gps_expected.altitude;
                }


            }


            if (in_or_out == 'a') {

                float zz_l = 8;  //zigzag_length
                float zz_w = 4;   //zigzag_width

                moveByPosOffset(control_task, {-zz_l * sind(yaw_const), zz_l * cosd(yaw_const), 0, yaw_const}, 1,
                                3);

                moveByPosOffset(control_task, {zz_w * cosd(yaw_const), zz_w * sind(yaw_const), 0, yaw_const}, 1, 3);

                moveByPosOffset(control_task, {zz_l * sind(yaw_const), -zz_l * cosd(yaw_const), 0.0, yaw_const},
                                0.8,
                                3);

                moveByPosOffset(control_task, {zz_w * cosd(yaw_const), zz_w * sind(yaw_const), 0.0, yaw_const}, 1,
                                3);


                fire_gps.latitude = 45.45936158153436;
                fire_gps.longitude = -73.91910071573902;
                fire_gps.altitude = 111.356392;


                fire_GPS_posArray[0] = fire_gps.latitude;
                fire_GPS_posArray[1] = fire_gps.longitude;
                fire_GPS_posArray[2] = fire_gps.altitude;

            }


            ros::spinOnce();

            current_GPS_posArray[0] = gps_position_.latitude;
            current_GPS_posArray[1] = gps_position_.longitude;
            current_GPS_posArray[2] = gps_position_.altitude;  // these assignments should be before using LatLong2Meter function

            ROS_INFO("fire_GPS_posArray[0] Is [%f]", fire_GPS_posArray[0]);
            ROS_INFO("fire_GPS_posArray[0] Is [%f]", fire_GPS_posArray[1]);
            ROS_INFO("fire_GPS_posArray[0] Is [%f]", fire_GPS_posArray[2]);


            FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, current_GPS_posArray, m);
            ROS_INFO("current position's x is [%f]", m[0]);
            ROS_INFO("current position's y is [%f]", m[1]);
            ROS_INFO("current position's z is [%f]", m[2]); //m[2] is incorrect

            double fire_gps_local_pos[3];

            FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, fire_GPS_posArray, fire_gps_local_pos);


            ROS_INFO("fire's x is [%f]", fire_gps_local_pos[0]);
            ROS_INFO("fire's y is [%f]", fire_gps_local_pos[1]);
            ROS_INFO("fire's z is [%f]", fire_gps_local_pos[2]);


            if (scenario == 'b') {

                int angle = 100;

                moveByPosOffset(control_task,
                                {fire_gps_local_pos[0] - m[0], fire_gps_local_pos[1] - m[1], 0.0, yaw_const}, 0.1,
                                3); //less threshold



                ros::spinOnce();

                current_GPS_posArray[0] = gps_position_.latitude;
                current_GPS_posArray[1] = gps_position_.longitude;
                current_GPS_posArray[2] = gps_position_.altitude;


                FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, current_GPS_posArray, m);
                ROS_INFO("current position's x is [%f]", m[0]);
                ROS_INFO("current position's y is [%f]", m[1]);

                ROS_INFO("current position's lat is [%f]", current_GPS_posArray[0]);
                ROS_INFO("current position's long is [%f]", current_GPS_posArray[1]);


                std::string DropWaterCommand = "rosrun arduino_actuator servo_pub.py";
                FILE *pp = popen(DropWaterCommand.c_str(), "r");
                if (pp != NULL) {
                    PRINT_INFO("drop water successfully!");
                } else {
                    PRINT_INFO("fail to drop water!");


                }

                ros::Duration(4).sleep();

/*

                std::string DropWaterCommand = "rosrun arduino_actuator servo_pub.py";
                int result2 = system(DropWaterCommand.c_str());

                if (result2 == 0) {
                    ROS_INFO("drop water successfully!");
                } else {
                    ROS_INFO("fail to drop water!");
                }

                for (int i=1; i<100;i++) {
                    controlServo(angle);
                    ros::spinOnce();
                }
*/
            }
            if (scenario == 'c') {

                // set mission start position. I set it at the south east of the fire point
                float mission_start_pos[3] = {fire_gps_local_pos[0] - 7, fire_gps_local_pos[1] + 4,
                                              9}; // it also can be current x y z

                ROS_INFO("moving to the start mission position");




                // go to mission start position
                moveByPosOffset(control_task,
                                {mission_start_pos[0] - m[0], mission_start_pos[1] - m[1], 0, yaw_const},
                                1, 3);


                // adjust initial yaw angle
                float yaw_adjustment; // yaw adjustment before approach
                float deltaX = fire_gps_local_pos[0] - mission_start_pos[0];
                float deltaY = fire_gps_local_pos[1] - mission_start_pos[1];

                ROS_INFO("deltaX is [%f]", deltaX);
                ROS_INFO("deltaY is [%f]", deltaY);


                yaw_adjustment = Rad2Deg(atan2(deltaY, deltaX)); // note that tan2 output is in radian
                // Also I added 90 as we want the yaw angle from x axis which is in Y direction

                ROS_INFO("yaw_adjustment_angle is [%f]", yaw_adjustment);
                moveByPosOffset(control_task, {0, 0, 0, yaw_adjustment}, 1, 3);

                // velocity mission

                float d = sqrt(
                        pow(fire_gps_local_pos[0] - mission_start_pos[0], 2) +
                        pow(fire_gps_local_pos[1] - mission_start_pos[1], 2));

                float abs_vel = 5; // absolute velocity that needs to be projected



                velocityAndYawRateControl({abs_vel * cosd(yaw_adjustment), abs_vel * sind(yaw_adjustment), 0}, 5000,
                                          abs_vel, d, height, release_delay);


                ROS_INFO_STREAM("Step 1 over!EmergencyBrake for 2s\n");
                emergency_brake_client.call(emergency_brake);
                ros::Duration(2).sleep();


            }
        }
    }

    if (scenario == 'e') {
        // float homeGPS_posArray[3];


        //Get fire GPS position and use callback function to put all the deteced fire spots GPS info and sequence to nodes_vec, a global vector
        ros::Subscriber fire_spots_GPS_sub = nh.subscribe("/position/fire_spots_GPS", 1, FireCallback2);


        // Some copied codes from Erfan's about M300 functions (including some new codes)
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if (control_task.response.result == false) {
            ROS_ERROR_STREAM("Takeoff task failed");
        }
        if (control_task.response.result == true) {
            ROS_INFO_STREAM("Takeoff task successful");

            float yaw_const;
            std::cout << " please enter initial yaw angle in degree-Z axes downward" << std::endl;
            std::cin >> yaw_const;


            float zz_l = 8;  //zigzag_length
            float zz_w = 4;   //zigzag_width

            moveByPosOffset(control_task, {-zz_l * sind(yaw_const), zz_l * cosd(yaw_const), 0, yaw_const}, 1,
                            3);

            moveByPosOffset(control_task, {zz_w * cosd(yaw_const), zz_w * sind(yaw_const), 0, yaw_const}, 1, 3);

            moveByPosOffset(control_task, {zz_l * sind(yaw_const), -zz_l * cosd(yaw_const), 0.0, yaw_const},
                            0.8,
                            3);

            moveByPosOffset(control_task, {zz_w * cosd(yaw_const), zz_w * sind(yaw_const), 0.0, yaw_const}, 1,
                            3);


            ros::spinOnce();

            double current_GPS_posArray[3];


            for (int i = 0; i < nodes_vec.size(); i++) {
                double fire_GPS_posArray[nodes_vec.size()][3];
                double fire_gps_local_pos[nodes_vec.size() - 1][3];
                if (i = 0) {
                    //Define fire_GPS_posArray[i][3] for relative distance transformation
                    fire_GPS_posArray[i][0] = nodes_vec[1].x;
                    fire_GPS_posArray[i][1] = nodes_vec[1].y;
                    fire_GPS_posArray[i][2] = nodes_vec[1].z;

                    FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, fire_GPS_posArray[i], fire_gps_local_pos[i]);

                    ros::spinOnce();

                    //Increase some height
                    moveByPosOffset(control_task, {0, 0, 7, 0}, 1, 3);

                    //Define mission start position
                    double mission_start_pos[3] = {fire_gps_local_pos[i][0] - 7, fire_gps_local_pos[i][1] + 4,
                                                   0}; // it also can be current x y z

                    //Fly to the mission start position with fixed yaw angle
                    moveByPosOffset(control_task, {mission_start_pos[0], mission_start_pos[1], 0, yaw_const}, 1,
                                    3);

                    ros::spinOnce();

                    current_GPS_posArray[0] = gps_position_.latitude;
                    current_GPS_posArray[1] = gps_position_.longitude;
                    current_GPS_posArray[2] = gps_position_.altitude;

                    FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, current_GPS_posArray, mission_start_pos);

                    float yaw_adjustment; // yaw adjustment before approach
                    float deltaX = fire_gps_local_pos[i][0] - mission_start_pos[0];
                    float deltaY = fire_gps_local_pos[i][1] - mission_start_pos[1];

                    ROS_INFO("deltaX is [%f]", deltaX);
                    ROS_INFO("deltaY is [%f]", deltaY);

                    yaw_adjustment = Rad2Deg(atan2(deltaY, deltaX));
                    // note that tan2 output is in radian
                    // Also I added 90 as we want the yaw angle from x axis which is in Y direction
                    ROS_INFO("yaw_adjustment_angle is [%f]", yaw_adjustment);

                    moveByPosOffset(control_task, {0, 0, 0, yaw_adjustment}, 1,
                                    3);  // note that x y z goes into this funciton

                    // velocity mission

                    float d = sqrt(
                            pow(fire_gps_local_pos[i][0] - mission_start_pos[0], 2) +
                            pow(fire_gps_local_pos[i][1] - mission_start_pos[1], 2));

                    float abs_vel = 5; // absolute velocity that needs to be projected

                    float height = 10;
                    velocityAndYawRateControl(
                            {abs_vel * cosd(yaw_adjustment), abs_vel * sind(yaw_adjustment), 0}, 5000,
                            abs_vel, d, height, 0);
                    ROS_INFO_STREAM("Step 1 over!EmergencyBrake for 2s\n");
                    emergency_brake_client.call(emergency_brake);
                    ros::Duration(2).sleep();
                } else {
                    //Define fire_GPS_posArray[i][3] for relative distance transformation
                    fire_GPS_posArray[i][0] = nodes_vec[i + 1].x;
                    fire_GPS_posArray[i][1] = nodes_vec[i + 1].y;
                    fire_GPS_posArray[i][2] = nodes_vec[i + 1].z;

                    FFDS::TOOLS::LatLong2Meter(fire_GPS_posArray[i - 1], fire_GPS_posArray[i],
                                               fire_gps_local_pos[i]);

                    ros::spinOnce();

                    current_GPS_posArray[0] = gps_position_.latitude;
                    current_GPS_posArray[1] = gps_position_.longitude;
                    current_GPS_posArray[2] = gps_position_.altitude;

                    double recent_local_pos[3];

                    FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, current_GPS_posArray, recent_local_pos);

                    float yaw_adjustment; // yaw adjustment before approach
                    float deltaX = fire_gps_local_pos[i][0] - recent_local_pos[0];
                    float deltaY = fire_gps_local_pos[i][1] - recent_local_pos[1];

                    ROS_INFO("deltaX is [%f]", deltaX);
                    ROS_INFO("deltaY is [%f]", deltaY);

                    yaw_adjustment = Rad2Deg(atan2(deltaY, deltaX));
                    // note that tan2 output is in radian
                    // Also I added 90 as we want the yaw angle from x axis which is in Y direction
                    ROS_INFO("yaw_adjustment_angle is [%f]", yaw_adjustment);


                    moveByPosOffset(control_task, {0, 0, 0, yaw_adjustment}, 1,
                                    3);  // note that x y z goes into this funciton
                    float d = sqrt(
                            pow(fire_gps_local_pos[i][0] - recent_local_pos[0], 2) +
                            pow(fire_gps_local_pos[i][1] - recent_local_pos[1], 2));

                    float abs_vel = 5; // absolute velocity that needs to be projected

                    float height = 10;
                    velocityAndYawRateControl(
                            {abs_vel * cosd(yaw_adjustment), abs_vel * sind(yaw_adjustment), 0}, 5000,
                            abs_vel, d, height, 0);
                    ROS_INFO_STREAM("Step 1 over!EmergencyBrake for 2s\n");
                    emergency_brake_client.call(emergency_brake);
                    ros::Duration(2).sleep();
                }
            }

        }
    }

    if (scenario == 'f') {
        // float homeGPS_posArray[3];

        // reset gimbal
        GimbalAction gimbalAction;
        gimbalAction.request.is_reset = true;
        gimbalAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        gimbal_control_client.call(gimbalAction);

        cout << "we are inside scenario's f code" << endl;  // for debug
        //Get fire GPS position and use callback function to put all the detected fire spots GPS info and sequence to nodes_vec, a global vector
        ros::Subscriber line_of_fire_sub = nh.subscribe("/position/fire_spots_GPS", 1, LineOfFireCallback);


        /*float gimbal_yaw_adjustment;
        cout << "please enter gimbal yaw adjustment" << endl;
        cin >> gimbal_yaw_adjustment;*/
        double yaw_const;
        std::cout << " please enter initial yaw angle in degree-Z axes downward" << std::endl;
        std::cin >> yaw_const;

        /*float threshold;
        cout << "please enter threshold for RANSAC"<<endl;
        cin >> threshold;*/
        // Some copied codes from Erfan's about M300 functions (including some new codes)
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if (control_task.response.result == false) {
            ROS_ERROR_STREAM("Takeoff task failed");
        }
        if (control_task.response.result == true) {
            ROS_INFO_STREAM("Takeoff task successful");


            moveByPosOffset(control_task, {0, 0, height - 1, 0}, 1, 3);
            moveByPosOffset(control_task, {0, 0, 0, 90}, 1,
                            3); // note that north is x axis, east is y, and down axis is the z

            GimbalAction gimbalAction;
            gimbalAction.request.rotationMode = 0;
            gimbalAction.request.pitch = camera_pitch;
            gimbalAction.request.roll = 0;
            // gimbalAction.request.yaw = -yaw_const+90;
            // gimbalAction.request.yaw = 180.0f + gimbal_yaw_adjustment;
            gimbalAction.request.yaw = gimbal_yaw_adjustment;
            gimbalAction.request.time = 0;
            gimbal_control_client.call(gimbalAction);

            CircularPathParams circular_params(7, 0.1, 1);
/*
            circular_params.theta_dot = 0.1;
            circular_params.radius = 7;
            circular_params.theta_step_degrees = 1;*/

            circular_params.CalculateParams();

            for (float theta = 0; theta < 360; theta = theta + circular_params.theta_step_degrees) {
                // time_step = (M_PI/theta_dot)/theta_step;
                circular_params.CircularVelocity.Vx =
                        circular_params.radius * circular_params.theta_dot * cosd(theta); // this is not instantaneous
                circular_params.CircularVelocity.Vy = circular_params.radius * circular_params.theta_dot * sind(theta);
                cout << "Vx is:" << circular_params.CircularVelocity.Vx << " Vy is:"
                     << circular_params.CircularVelocity.Vy << "time step in ms is:" << circular_params.time_step * 1000
                     << ", theta:"<<theta<<endl;

                if (theta == 45 || theta == 90 || theta == 135 || theta == 180 || theta == 225 || theta == 270 ||
                    theta == 315 || theta == 360) {
                    cout<<endl<<"stop to sweep pitch angle for H20T camera";
                    gimbalAction.request.rotationMode = 1; // this mode is absolute mode
                    gimbalAction.request.roll = 0;
                    gimbalAction.request.yaw = gimbal_yaw_adjustment;
                    gimbalAction.request.pitch = -70.0f;
                    gimbalAction.request.time = 1; // Dont knwo th efunction exactly. make pitch movement smoother?
                    gimbal_control_client.call(gimbalAction);
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    gimbalAction.request.roll = 0;
                    gimbalAction.request.pitch = -20.0f;
                    gimbalAction.request.yaw = gimbal_yaw_adjustment;
                    gimbalAction.request.time = 2.5; // Dont knwo th efunction exactly. make pitch movement smoother?
                    gimbal_control_client.call(gimbalAction);
                    std::this_thread::sleep_for(std::chrono::milliseconds(800));
                    gimbalAction.request.roll = 0;
                    gimbalAction.request.yaw = gimbal_yaw_adjustment;
                    gimbalAction.request.pitch = camera_pitch;
                    gimbalAction.request.time = 2.5; // Dont knwo th efunction exactly. make pitch movement smoother?
                    gimbal_control_client.call(gimbalAction);
                    std::this_thread::sleep_for(std::chrono::milliseconds(700));


                    /*
                    if (theta == 45 || theta == 90 || theta == 135 || theta == 180 || theta == 225 || theta == 270 ||
                        theta == 315 || theta == 360) {
                        float initial_pitch = -50.0f;
                        float final_pitch = -15.0f;
                        gimbalAction.request.rotationMode = 0;
                        gimbalAction.request.roll = 0;
                        gimbalAction.request.pitch = initial_pitch;
                        gimbalAction.request.time = 1; // Dont knwo th efunction exactly. make pitch movement smoother?
                        gimbal_control_client.call(gimbalAction);
                        gimbalAction.request.roll = 0;
                        gimbalAction.request.pitch = final_pitch;
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                        gimbalAction.request.time = 2.5; // Dont knwo th efunction exactly. make pitch movement smoother?
                        gimbal_control_client.call(gimbalAction);
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                        gimbalAction.request.roll = 0;
                        gimbalAction.request.pitch = camera_pitch;
                        gimbalAction.request.time = 1.5; // Dont know th efunction exactly. make pitch movement smoother?
                        gimbal_control_client.call(gimbalAction);
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    */
                    /*
                    for (float pitch = initial_pitch; pitch < final_pitch; pitch+=10) {
                        gimbalAction.request.pitch = pitch;
                        cout<<"pitch: "<<pitch<<endl;
                        // gimbalAction.request.yaw = -yaw_const+90;
                        // gimbalAction.request.yaw = 180.0f + gimbal_yaw_adjustment;
                        // gimbalAction.request.yaw = -180.0f+gimbal_yaw_adjustment;
                        gimbalAction.request.time = 0.3; // Dont knwo th efunction exactly. make pitch movement smoother?
                        gimbal_control_client.call(gimbalAction);
                    }
                    gimbalAction.request.pitch = camera_pitch;
                    gimbal_control_client.call(gimbalAction);
                     */


                }



                CircularDivisionPlanner({circular_params.CircularVelocity.Vx, circular_params.CircularVelocity.Vy, 0,
                                         circular_params.yawRate-yaw_rate_adjustment}, circular_params.time_step * 1000);


            }


            ZigZagParams zigzag_params;
            /* zigzag_params.length = 12;
            zigzag_params.width = 6;
            zigzag_params.number = 1;
            zigzag_params.split = split;*/

            const std::string package_path =
                    ros::package::getPath("dji_osdk_ros");
            const std::string config_path = package_path + "/config/zigzag_params.yaml";
            PRINT_INFO("Load parameters from:%s", config_path.c_str());
            YAML::Node ZigZagconfig = YAML::LoadFile(config_path);


            zigzag_params.length = ZigZagconfig["zigzag_params"]["length"].as<float>();
            zigzag_params.width = ZigZagconfig["zigzag_params"]["width"].as<float>();
            zigzag_params.number = ZigZagconfig["zigzag_params"]["number"].as<int>();
            zigzag_params.velocity = ZigZagconfig["zigzag_params"]["velocity"].as<float>();
            zigzag_params.orientation = yaw_const; // next time read it from a yaml file

            std::cout << "ZigZag Parameters:" << std::endl;
            std::cout << "Length: " << zigzag_params.length << std::endl;
            std::cout << "Width: " << zigzag_params.width << std::endl;
            std::cout << "Number: " << zigzag_params.number << std::endl;

            // Clear the vector if needed
            nodes_vec.clear();


            moveByPosOffset(control_task, {0, 0, 0, yaw_const}, 1, 3);
            GeoPositioningFlag = 0;

            int detect_index;//detection_starter_indicator
            if (in_or_out == 'b') {
                bool SLAM_flag = 0;
                while (SLAM_flag == 0) {
                    cout << "please rosrun detection and SLAM nodes. Then press 1" << endl;
                    cin >> detect_index;
                    if (detect_index == 1) { SLAM_flag = 1; }
                    else { cout << "Please press 1 if SLAM is initiated"<<endl; }
                }
            }

            std::thread FireSpotCounter_thread(FireSpotCounter);
            ZigZagPlanner(control_task, zigzag_params);
            FireSpotCounter_thread.join();

            ros::spinOnce();


            if (in_or_out == 'a') {

                // Load YAML file
                const std::string package_path =
                        ros::package::getPath("dji_osdk_ros");
                const std::string config_path = package_path + "/config/nodes.yaml";
                PRINT_INFO("Load parameters from:%s", config_path.c_str());
                YAML::Node config = YAML::LoadFile(config_path);


                // Check if "nodes" key exists
                if (config["nodes"]) {
                    for (const auto &n: config["nodes"]) {
                        node temp;
                        temp.id = n["id"].as<int>();
                        temp.x = n["x"].as<float>();
                        temp.y = n["y"].as<float>();
                        temp.z = n["z"].as<float>();
                        nodes_vec.push_back(temp);
                    }
                }

                // Print loaded nodes
                for (const auto &n: nodes_vec) {
                    std::cout << "Node ID: " << n.id << ", X: " << n.x << ", Y: " << n.y << ", Z: " << n.z << std::endl;
                }
            }

            /*
            nodes_vec.push_back(n5);
            nodes_vec.push_back(n6);
            nodes_vec.push_back(n7);
            nodes_vec.push_back(n8);
            nodes_vec.push_back(n9);
            nodes_vec.push_back(n10);
            nodes_vec.push_back(n11);
            nodes_vec.push_back(n12);
*/
            // Save the final plot in a specific path within the user's home directory
            std::string homeDirectory = getHomeDirectory(); // Get home directory
            SaveAllPath = homeDirectory + "/M300_ws/records/"+record_index+"/";

            // Create directory if it doesn't exist
            createDirectory(SaveAllPath); // Create directory

            double current_GPS_posArray[3];

            double fire_gps_local_pos[nodes_vec.size()][3]; // coordinates of fire spots (x,y,z)

            double fire_GPS_posArray[nodes_vec.size()][3];  // GPS of fire spots

            cout << "number of fire spots are: " << nodes_vec.size() << std::endl;

            cout << "Home GPS position: latitude  " << homeGPS_posArray[0] << "longitude  " << homeGPS_posArray[1]
                 << endl;

            for (int i = 0; i < nodes_vec.size(); ++i) {

                fire_GPS_posArray[i][0] = nodes_vec[i].x;
                fire_GPS_posArray[i][1] = nodes_vec[i].y;
                fire_GPS_posArray[i][2] = nodes_vec[i].z;

                FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, fire_GPS_posArray[i], fire_gps_local_pos[i]);
                std::cout << "Node ID: " << nodes_vec[i].id << ", latitude: " << nodes_vec[i].x << ", longitude: "
                          << nodes_vec[i].y << ", z: " << nodes_vec[i].z << std::endl;
                std::cout << "fire's x position " << fire_gps_local_pos[i][0] << ", fire's y position "
                          << fire_gps_local_pos[i][1] << std::endl;


            }

            // Extract x and y coordinates into vectors
            std::vector<float> x, y;
            for (size_t i = 0; i < nodes_vec.size(); ++i) {
                x.push_back(fire_gps_local_pos[i][0]);
                y.push_back(fire_gps_local_pos[i][1]);
            }

            // Call the function to write vectors to a CSV file
            // Provide the full path to the CSV file
            writeVectorsToCSV(x, y, SaveAllPath+"fire_spots_x_and_y_"+record_index+".csv");

            // Print x and y before plotting
            std::cout << "X coordinates: ";
            for (auto val: x) {
                std::cout << val << " ";
            }
            std::cout << std::endl;

            std::cout << "Y coordinates: ";
            for (auto val: y) {
                std::cout << val << " ";
            }
            std::cout << std::endl;

            // Call setupPlot to set up initial plot
            setupPlot(x, y); // Setup initial plot


            cout << "now we have gps position of fire spots, we go ahead and find optimum line for approach";

            Line best_line;
            Point starting_point;
            bool flag = 1;
            char approach_confirm;
            char ground_truth_gps_aprch_cmnd;

            while (flag == 1) {
                doRANSAC(nodes_vec, fire_gps_local_pos, best_line, starting_point, threshold, run_up_distance);
                cout << "confirm approach? [y/n]";
                cin >> approach_confirm;
                if (approach_confirm == 'y') {
                    flag = 0;
                } else {
                    cout << "approach ground truth gps? [y/n]";
                    cin >> ground_truth_gps_aprch_cmnd;
                    if (ground_truth_gps_aprch_cmnd == 'y') {
                        nodes_vec.clear();
                        // Load YAML file
                        const std::string package_path =
                                ros::package::getPath("dji_osdk_ros");
                        const std::string config_path = package_path + "/config/nodes.yaml";
                        PRINT_INFO("Load parameters from:%s", config_path.c_str());
                        YAML::Node config = YAML::LoadFile(config_path);

                        // Check if "nodes" key exists
                        if (config["nodes"]) {
                            for (const auto &n: config["nodes"]) {
                                node temp;
                                temp.id = n["id"].as<int>();
                                temp.x = n["x"].as<float>();
                                temp.y = n["y"].as<float>();
                                temp.z = n["z"].as<float>();
                                nodes_vec.push_back(temp);

                                double current_GPS_posArray[3];

                                double fire_gps_local_pos[nodes_vec.size()][3]; // coordinates of fire spots (x,y,z)

                                double fire_GPS_posArray[nodes_vec.size()][3];  // GPS of fire spots

                                cout << "number of fire spots are: " << nodes_vec.size() << std::endl;

                                cout << "Home GPS position: latitude  " << homeGPS_posArray[0] << "longitude  "
                                     << homeGPS_posArray[1];

                                for (int i = 0; i < nodes_vec.size(); ++i) {

                                    fire_GPS_posArray[i][0] = nodes_vec[i].x;
                                    fire_GPS_posArray[i][1] = nodes_vec[i].y;
                                    fire_GPS_posArray[i][2] = nodes_vec[i].z;

                                    FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, fire_GPS_posArray[i],
                                                               fire_gps_local_pos[i]);
                                    std::cout << "Node ID: " << nodes_vec[i].id << ", latitude: " << nodes_vec[i].x
                                              << ", longitude: "
                                              << nodes_vec[i].y << ", z: " << nodes_vec[i].z << std::endl;
                                    std::cout << "fire's x position " << fire_gps_local_pos[i][0]
                                              << ", fire's y position "
                                              << fire_gps_local_pos[i][1] << std::endl;


                                }

                                // Extract x and y coordinates into vectors
                                // Update x and y vectors
                                x.clear();
                                y.clear();
                                for (size_t i = 0; i < nodes_vec.size(); ++i) {
                                    x.push_back(fire_gps_local_pos[i][0]);
                                    y.push_back(fire_gps_local_pos[i][1]);
                                }


                                // Print x and y before plotting
                                std::cout << "X coordinates: ";
                                for (auto val: x) {
                                    std::cout << val << " ";
                                }
                                std::cout << std::endl;

                                std::cout << "Y coordinates: ";
                                for (auto val: y) {
                                    std::cout << val << " ";
                                }
                                std::cout << std::endl;

                                // Update plot with new data
                                setupPlot(x, y); // Update plot with new data

                            }
                        }

                    } else {
                        cout << "calculating RANSAC again";

                    }

                }


            }





            ros::spinOnce();


/*
                current_GPS_posArray[0] = gps_position_.latitude;
                current_GPS_posArray[1] = gps_position_.longitude;
                current_GPS_posArray[2] = gps_position_.altitude;

                float recent_local_pos[3];
*/
            recent_drone_coord = GPS2Coordinates(homeGPos, gps_position_);

            // FFDS::TOOLS::LatLong2Meter(homeGPS_posArray, current_GPS_posArray, recent_local_pos);


            float yaw_adjustment; // yaw adjustment before approach

            cout << "starting point x :" << starting_point.x << "recent y :" << starting_point.y
                 << endl;

            cout << "recent x :" << recent_drone_coord.x << "recent y :" << recent_drone_coord.y
                 << endl;
            // yaw_adjustment = Rad2Deg(atan2(deltaY, deltaX));
            // note that tan2 output is in radian
            // Also I added 90 as we want the yaw angle from x axis which is in Y direction


            moveByPosOffset(control_task,
                            {starting_point.x - recent_drone_coord.x, starting_point.y - recent_drone_coord.y, 0, 0},
                            1, 0.01);  // note that x y z goes into this function
            cout << "moved to the starting point" << endl;
            yaw_adjustment = Rad2Deg(atan(best_line.slope));
            cout << "yaw_adjustment is" << yaw_adjustment << endl;

            moveByPosOffset(control_task, {0, 0, 0, yaw_adjustment}, 1,
                            3);  // note that x y z goes into this funciton

            cout << "rotated and ready for approaching fire!" << endl;

            // preset auto lateral adjustment form yaml file
            moveByPosOffset(control_task,
                            {-lateral_adjustment * sind(yaw_adjustment), lateral_adjustment * cosd(yaw_adjustment), 0,
                             yaw_adjustment}, 1, 3);

            // user lateral adjustment
            // Negative lateral adjustment in yaml file is toward left

            if(inputLateralAdjustment == true){
                while (true) {
                    cout << "Please enter lateral adjustment value: ";
                    cin >> lateral_adjustment;

                    moveByPosOffset(control_task,
                                    {-lateral_adjustment * sind(yaw_adjustment),
                                     lateral_adjustment * cosd(yaw_adjustment), 0,
                                     yaw_adjustment}, 1, 3);

                    char continue_adjustment;
                    bool valid_input = false;
                    while (!valid_input) {
                        cout << "Do you want to continue lateral adjusting? (y/n): ";
                        cin >> continue_adjustment;
                        if (continue_adjustment == 'y' || continue_adjustment == 'Y') {
                            valid_input = true;
                        } else if (continue_adjustment == 'n' || continue_adjustment == 'N') {
                            valid_input = true;
                            break;
                        } else {
                            cout << "Invalid input. Please enter 'y' or 'n'." << endl;
                        }
                    }

                    if (continue_adjustment == 'n' || continue_adjustment == 'N') {
                        break;
                    }
                }
            }

            /*
            if(inputExtraYawAdjustment == true){
                double extra_yaw_adjustment;
                while (true) {
                    cout << "Please enter extra yaw adjustment value (neagtive is CCW): ";
                    cin >> extra_yaw_adjustment;

                    moveByPosOffset(control_task, {0, 0, 0, extra_yaw_adjustment}, 1,0.01);  // note that x y z goes into this funciton

                    char continue_adjustment;
                    bool valid_input = false;
                    while (!valid_input) {
                        cout << "Do you want to continue extra yaw adjusting? (y/n): ";
                        cin >> continue_adjustment;
                        if (continue_adjustment == 'y' || continue_adjustment == 'Y') {
                            valid_input = true;
                        } else if (continue_adjustment == 'n' || continue_adjustment == 'N') {
                            valid_input = true;
                            break;
                        } else {
                            cout << "Invalid input. Please enter 'y' or 'n'." << endl;
                        }
                    }

                    if (continue_adjustment == 'n' || continue_adjustment == 'N') {
                        break;
                    }
                }
            }
*/
            // fuzzy control
            if (apply_fuzzy_control == true) {

                // Create the first thread for manual stoppping of the fuzzy controller
                std::thread inputThread(askUserToStop);

                geometry_msgs::Point avg_center;

                double PixelErrorPercentage = 200;  // something to make sure that it doesnt satisfy the if statement if no bounding boxes has been found

                // start fuzzy controlling

                float Wide_image_width = 640/2;

                // Create the fuzzy engine
                fl::Engine *engine = createFuzzyEngine(VelocityMax);
                if (!engine) {
                    return 1;
                }

                while (stopFuzzyControl == false) {

                    ros::spinOnce();

                    if (!fire_bbx_centers.empty()) {

                        // Access the centers
                        for (const auto &center: fire_bbx_centers) {
                            ROS_INFO("Center: x = %f, y = %f", center.x, center.y);
                        }

                        avg_center = calculateAndPrintAverageCenter();
                        std::cout << "Returned Average Center: x = " << avg_center.x << ", y = " << avg_center.y
                                  << std::endl;


                        PixelErrorPercentage = ((avg_center.x - Wide_image_width) / Wide_image_width) * 100;

                        if (PixelErrorPercentage < -100.0 || PixelErrorPercentage > 100.0) {
                            std::cout << "Error value out of range" << std::endl;

                        }

                        // criterion to stop fuzzy
                        if (abs(PixelErrorPercentage) < 0.1) {
                            stopFuzzyControl = true;
                            break; }

                        // Perform fuzzy inference and print the output value
                        double AdjustingVelocity = fuzzyInference(engine, PixelErrorPercentage);
                        std::cout << "Pixel Error Percentage: " << PixelErrorPercentage << ", Velocity: "
                                  << AdjustingVelocity
                                  << std::endl;

                        // Applying adjusting velocity
                        FuzzyVelocityTraversal(
                                {-AdjustingVelocity * sind(yaw_adjustment), AdjustingVelocity * cosd(yaw_adjustment),
                                 0},
                                20);
                    }
                    if (abs(PixelErrorPercentage) < 0.1) {
                        stopFuzzyControl = true;
                        break; }
                }

                // Join the threads to the main thread
                if (inputThread.joinable()) {
                    inputThread.join();
                }
                }

                // In Motion Dropping mission


                velocityAndYawRateControl({abs_vel * cosd(yaw_adjustment), abs_vel * sind(yaw_adjustment), 0}, time_of_approach,
                                          abs_vel, run_up_distance, height, release_delay);   // make sure the approach time is long enough especiallly when the approach speed is low
                // emergency_brake_client.call(emergency_brake);
                // ros::Duration(4).sleep();

                ros::spinOnce();
                recent_drone_coord = GPS2Coordinates(homeGPos, gps_position_);
                cout << "nodes_vec[0].x:" << fire_GPS_posArray[0][0] << " and nodes_vec[0]" << fire_GPS_posArray[0][1]
                     << endl;
                cout << "recent drone coordinates: x:" << recent_drone_coord.x << "y :" << recent_drone_coord.y << endl;
                // go above the first fire point
                moveByPosOffset(control_task, {fire_gps_local_pos[0][0] - recent_drone_coord.x,
                                               fire_gps_local_pos[0][1] - recent_drone_coord.y, 0, 0}, 1, 3);
                gimbalAction.request.rotationMode = 0;
                gimbalAction.request.pitch = -camera_pitch-90.0f;
                gimbalAction.request.roll = 0.0f;
                // gimbalAction.request.yaw = -yaw_const+90;
                gimbalAction.request.time = 0.5;
                gimbal_control_client.call(gimbalAction);

                moveByPosOffset(control_task, {0, 0, 15, 0}, 1,
                                3);
            }
        }


        PRINT_INFO("going home now");
        control_task.request.task =
                dji_osdk_ros::FlightTaskControl::Request::TASK_GOHOME;
        task_control_client.call(control_task);
        if (control_task.response.result == true) {
            PRINT_INFO("go home successful");
        } else {
            PRINT_WARN("go home failed.");
        }

        control_task.request.task =
                dji_osdk_ros::FlightTaskControl::Request::TASK_LAND;
        PRINT_INFO(
                "Landing request sending ... need your confirmation on the remoter!");
        task_control_client.call(control_task);
        if (control_task.response.result == true) {
            PRINT_INFO("land task successful");
        } else {
            PRINT_ERROR("land task failed.");
        }


        ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");
        // outputFile.close();
        ros::spin();
        return 0;
    }


bool moveByPosOffset(FlightTaskControl &task, const JoystickCommand &offsetDesired,
                     float posThresholdInM,
                     float yawThresholdInDeg) {
    task.request.task = FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
    task.request.joystickCommand.x = offsetDesired.x;
    task.request.joystickCommand.y = offsetDesired.y;
    task.request.joystickCommand.z = offsetDesired.z;
    task.request.joystickCommand.yaw = offsetDesired.yaw;
    task.request.posThresholdInM = posThresholdInM;
    task.request.yawThresholdInDeg = yawThresholdInDeg;

    task_control_client.call(task);
    return task.response.result;
}


void
velocityAndYawRateControl(const JoystickCommand &offsetDesired, uint32_t timeMs, float abs_vel, float d, float height,
                          float delay) {


    double originTime = 0;
    double currentTime = 0;
    // uint64_t elapsedTimeInMs = 0;
    float elapsedTimeInMs = 0;

    SetJoystickMode joystickMode;
    JoystickAction joystickAction;

    joystickMode.request.horizontal_mode = joystickMode.request.HORIZONTAL_VELOCITY;
    joystickMode.request.vertical_mode = joystickMode.request.VERTICAL_VELOCITY;
    joystickMode.request.yaw_mode = joystickMode.request.YAW_RATE;
    joystickMode.request.horizontal_coordinate = joystickMode.request.HORIZONTAL_GROUND;
    joystickMode.request.stable_mode = joystickMode.request.STABLE_ENABLE;
    set_joystick_mode_client.call(joystickMode);

    joystickAction.request.joystickCommand.x = offsetDesired.x;
    joystickAction.request.joystickCommand.y = offsetDesired.y;
    joystickAction.request.joystickCommand.z = offsetDesired.z;
    // joystickAction.request.joystickCommand.yaw = offsetDesired.yaw;  // This is for yaw rate, which we dont want
    int angle = 100;  // Set the desired angle here
    originTime = ros::Time::now().toSec();
    currentTime = originTime;
    elapsedTimeInMs = (currentTime - originTime) * 1000;
    bool flag = 0;
    float g = 9.81;

    double release_time = (((d / abs_vel) - sqrt((2 * height) / g)) * 1000) + delay; // release time in Ms
    cout<<"release_time is:"<<release_time<<endl;
    while (elapsedTimeInMs <= timeMs) {
        currentTime = ros::Time::now().toSec();
        elapsedTimeInMs = (currentTime - originTime) * 1000;
        // ROS_INFO("timeinMs [%f]",elapsedTimeInMs);



        if (elapsedTimeInMs > release_time || release_time<=0) {
            // controlServo(angle);


            ros::spinOnce();
            if (flag == 0) {

                std::string DropWaterCommand = "rosrun arduino_actuator servo_pub.py";
                FILE *pp = popen(DropWaterCommand.c_str(), "r");
                if (pp != NULL) {
                    PRINT_INFO("drop water successfully!");
                } else {
                    PRINT_INFO("fail to drop water!");


                }

/*
                std::string DropWaterCommand = "rosrun arduino_actuator servo_pub.py";
                int result3 = system(DropWaterCommand.c_str());

                if (result3 == 0) {
                    ROS_INFO("drop water successfully!");
                } else {
                    ROS_INFO("fail to drop water!");
                }
                */

                ROS_INFO("released valve at [%f]", elapsedTimeInMs);
            }
            joystick_action_client.call(joystickAction);
            flag = 1;
        } else {
            joystick_action_client.call(joystickAction);

        }


    }
}

void ZigZagDivisionPlanner(const JoystickCommand &offsetDesired, uint32_t timeMs) {

    double originTime = 0;
    double currentTime = 0;
    uint64_t elapsedTimeInMs = 0;

    SetJoystickMode joystickMode;
    JoystickAction joystickAction;

    joystickMode.request.horizontal_mode = joystickMode.request.HORIZONTAL_VELOCITY;
    joystickMode.request.vertical_mode = joystickMode.request.VERTICAL_VELOCITY;
    joystickMode.request.yaw_mode = joystickMode.request.YAW_RATE;
    joystickMode.request.horizontal_coordinate = joystickMode.request.HORIZONTAL_GROUND;
    joystickMode.request.stable_mode = joystickMode.request.STABLE_ENABLE;
    set_joystick_mode_client.call(joystickMode);

    joystickAction.request.joystickCommand.x = offsetDesired.x;
    joystickAction.request.joystickCommand.y = offsetDesired.y;
    joystickAction.request.joystickCommand.z = offsetDesired.z;
    joystickAction.request.joystickCommand.yaw = offsetDesired.yaw;

    originTime = ros::Time::now().toSec();
    currentTime = originTime;
    elapsedTimeInMs = (currentTime - originTime) * 1000;
cout<<"stopSLAM is"<<stopSLAM<<endl;
    while (elapsedTimeInMs <= timeMs && stopSLAM == false) {
        currentTime = ros::Time::now().toSec();
        elapsedTimeInMs = (currentTime - originTime) * 1000;
        joystick_action_client.call(joystickAction);
    }
}

void CircularDivisionPlanner(const JoystickCommand &offsetDesired, uint32_t timeMs) {
    double originTime = 0;
    double currentTime = 0;
    uint64_t elapsedTimeInMs = 0;

    SetJoystickMode joystickMode;
    JoystickAction joystickAction;

    joystickMode.request.horizontal_mode = joystickMode.request.HORIZONTAL_VELOCITY;
    joystickMode.request.vertical_mode = joystickMode.request.VERTICAL_VELOCITY;
    joystickMode.request.yaw_mode = joystickMode.request.YAW_RATE;
    joystickMode.request.horizontal_coordinate = joystickMode.request.HORIZONTAL_GROUND;
    joystickMode.request.stable_mode = joystickMode.request.STABLE_ENABLE;
    set_joystick_mode_client.call(joystickMode);

    joystickAction.request.joystickCommand.x = offsetDesired.x;
    joystickAction.request.joystickCommand.y = offsetDesired.y;
    joystickAction.request.joystickCommand.z = offsetDesired.z;
    joystickAction.request.joystickCommand.yaw = offsetDesired.yaw;

    originTime = ros::Time::now().toSec();
    currentTime = originTime;
    elapsedTimeInMs = (currentTime - originTime) * 1000;

    while (elapsedTimeInMs <= timeMs) {
        currentTime = ros::Time::now().toSec();
        elapsedTimeInMs = (currentTime - originTime) * 1000;
        joystick_action_client.call(joystickAction);
    }
}

static bool isEqual(const double a, const double b) {
    return (fabs(a - b) <= 1e-15);
}

sensor_msgs::NavSatFix getAverageGPS(
        const int average_times) {
    sensor_msgs::NavSatFix homeGPos;
    while (isEqual(0.0, gps_position_.latitude) ||
           isEqual(0.0, gps_position_.longitude) ||
           isEqual(0.0, gps_position_.altitude)) {
        ros::spinOnce();

    }
    // PRINT_WARN("zero in gps_position, waiting for normal gps position!");

    for (int i = 0; (i < average_times) && ros::ok(); i++) {
        ros::spinOnce();

        homeGPos.latitude += gps_position_.latitude;
        homeGPos.longitude += gps_position_.longitude;
        homeGPos.altitude += gps_position_.altitude;

        ros::Rate(10).sleep();
    }
    homeGPos.latitude = homeGPos.latitude / average_times;
    homeGPos.longitude = homeGPos.longitude / average_times;
    homeGPos.altitude = homeGPos.altitude / average_times;

    return homeGPos;
}

void ZigZagPlanner(FlightTaskControl &task, ZigZagParams zz_params) {
    cout<<"Starting zigzag"<<endl;
    ros::spinOnce();

    float velocities[4][2] = {
            {-zz_params.velocity * sind(zz_params.orientation),
                                                                                zz_params.velocity *
                                                                                cosd(zz_params.orientation)},
            {zz_params.velocity * cosd(zz_params.orientation),   zz_params.velocity *
                                                                                sind(zz_params.orientation)},
            {zz_params.velocity * sind(zz_params.orientation),  -zz_params.velocity *
                                                                                cosd(zz_params.orientation)},
            {zz_params.velocity * cosd(zz_params.orientation),   zz_params.velocity *
                                                                                sind(zz_params.orientation)}};
    /*
     * double frequency = 30; // 30 Hz
    ros::Rate rate(frequency);
*/
    cout<<"velocities[0][0] is"<<velocities[0][1]<<endl;
    cout<<"time in division"<<zz_params.length/zz_params.velocity<<endl;
    float length_time = (zz_params.length/zz_params.velocity)*1000; // time it takes to traverse the length of ZigZag
    float width_time = (zz_params.width/zz_params.velocity)*1000; // time it takes to traverse the width of ZigZag;

    for (int n = 0; n < zz_params.number; n++) { // loop for the number of zigzags
        ZigZagDivisionPlanner({velocities[0][0], velocities[0][1], 0}, length_time); // note that time should be in Ms
        if(stopSLAM == true) {return;}
        ros::spinOnce();
        ZigZagDivisionPlanner({velocities[1][0], velocities[1][1], 0}, width_time); // note that indexing starts form 0 in C++
        if(stopSLAM == true) {return;}
        ros::spinOnce();
        ZigZagDivisionPlanner({velocities[2][0], velocities[2][1], 0}, length_time);
        if(stopSLAM == true) {return;}
        ros::spinOnce();
        ZigZagDivisionPlanner({velocities[3][0], velocities[3][1], 0}, width_time);
        if(stopSLAM == true) {return;}

/*
        for (int i = 0; i < zz_params.split; i++) {
            moveByPosOffset(task, {lengths_steps[0][0], lengths_steps[0][1], 0, zz_params.orientation}, 1, 3);

            ros::spinOnce();
            rate.sleep();
            cout << "ROS spinned" << endl;
            cout<<"number of found fire spots are:"<<nodes_vec.size();
            if (nodes_vec.size()>number_of_fire_spots_criterion){
            cout<<"cutting ZigZag to reduce SLAM error";
                return;
            }
        }
*/
        //ROS_INFO("x is [%f]",local_position_.point.x);
        // ROS_INFO("y is [%f]",local_position_.point.y);
        // ROS_INFO("z is [%f]",local_position_.point.z);*/
        //ROS_INFO("latitude is [%f]",gps_position_.latitude);
        //ROS_INFO("longitude is [%f]",gps_position_.longitude);
        //ros::spin(); //here is good?
        //ROS_INFO_STREAM("first zigzag line completed!");


            
/*



        ros::spinOnce();




        for (int i = 0; i < zz_params.split; i++) {
            moveByPosOffset(task, {lengths_steps[2][0], lengths_steps[2][1], 0, zz_params.orientation}, 1, 3);

            ros::spinOnce();
            rate.sleep();
            cout << "ROS spinned" << endl;
                        cout<<"number of found fire spots are:"<<nodes_vec.size();
            if (nodes_vec.size()>number_of_fire_spots_criterion){
            cout<<"cutting ZigZag to reduce SLAM error";
                return;
            }
        }

        ros::spinOnce();

        ROS_INFO_STREAM("Third zigzag line completed!");




        ros::spinOnce();


        ROS_INFO_STREAM("Fourth zigzag line completed!");



        ros::spinOnce();
        */
    }

    // moveByPosOffset(control_task, {zz_w*cosd(yaw_const), zz_w*sind(yaw_const), 0.0, yaw_const}, 1, 3);
    // moveByPosOffset(control_task, {-3*sind(yaw_const), static_cast<DJI::OSDK::float32_t>(-6.5*cosd(yaw_const)), 0.0, yaw_const}, 1, 3);

// the more generous you are in threshold, the more agile your drone would be

}

void doRANSAC(std::vector <node> nodes_vector, double fire_coordinates[][3], Line& best_line, Point& starting_point, float threshold, double run_up_distance){
    int size = nodes_vector.size(); // # number of rows in fire_gps_local

    // Process the array and fit the line
    // Line best_line = processArrayAndFitLine(fire_gps_local_pos, size);
    best_line = processArrayAndFitLine(fire_coordinates, size, threshold);


    writeLineToCSV(best_line, SaveAllPath+record_index+"_line_data.csv");

    // Print the parameters of the best-fitting line
    std::cout << "Best-fitting line: y = " << best_line.slope << "x + " << best_line.intercept << std::endl;
    std::cout << "Number of inliers: " << best_line.num_inliers << std::endl;

    // Calculate the point on the fitted line closest to the first "real" inlier (not false alarm)
    Point first_inlier = best_line.inlier_points[0];
    std::cout << "First inlier: x = " << first_inlier.x << ", y = " << first_inlier.y << std::endl;
    Point closest_point = closestPointOnLine(best_line, first_inlier);

    Point intersecPoint = intersectionPoint(best_line, first_inlier);
    cout << "intersection_point_x is:" << intersecPoint.x << "and its y is:" << intersecPoint.y << endl;;
    // Print the coordinates of the closest point
    std::cout << "Closest point on the line to the first sample: (" << closest_point.x << ", "
              << closest_point.y << ")" << std::endl;

    // Plot the line
    std::vector<float> line_x, line_y;
    for (float x_val = -10; x_val <= 30; x_val += 0.1) { // Adjust the range as needed
        float y_val = best_line.slope * x_val + best_line.intercept;
        line_x.push_back(y_val);  // Note: Switched x and y
        line_y.push_back(x_val);  // Note: Switched x and y
    }

    plt::plot(line_x, line_y, "r"); // Plot the line in blue

    starting_point = traverseOnLine(best_line, intersecPoint, run_up_distance);
    std::cout << "starting point (x,y): (" << starting_point.x << ", " << starting_point.y << ")"
              << std::endl;

    // Call the function to write the point to a CSV file
    writePointToCSV(starting_point, SaveAllPath+record_index+"_starting_point.csv");

    plt::plot({starting_point.y}, {starting_point.x}, "go"); // DONT FORGET THE BRACKET


    //saving plot
    cout<<"saving the figure";

    std::string saveFigurePath = SaveAllPath + record_index + ".png"; // Adjust the relative path as needed
    plt::save(saveFigurePath); // Save the plot
    std::cout << "Plot saved to: " << saveFigurePath << std::endl;

    // Show plot
    plt::show();
}

// Function to create and configure the fuzzy logic engine
fl::Engine* createFuzzyEngine(double velocityMax) {
    // Create a fuzzy logic engine
    fl::Engine* engine = new fl::Engine;
    engine->setName("VelocityControl");

    // Define the input variable 'Error'
    fl::InputVariable* error = new fl::InputVariable;
    error->setName("Error");
    error->setRange(-100.0, 100.0);
    error->addTerm(new fl::Triangle("VeryNegative", -100.0, -100.0, -50.0));
    error->addTerm(new fl::Triangle("Negative", -100.0, -50.0, 0.0));
    error->addTerm(new fl::Triangle("Zero", -25.0, 0.0, 25.0));
    error->addTerm(new fl::Triangle("Positive", 0.0, 50.0, 100.0));
    error->addTerm(new fl::Triangle("VeryPositive", 50.0, 100.0, 100.0));
    engine->addInputVariable(error);

    // Define the output variable 'Velocity' with dynamic range
    fl::OutputVariable* velocity = new fl::OutputVariable;
    velocity->setName("Velocity");
    velocity->setRange(-velocityMax, velocityMax);
    velocity->setDefaultValue(fl::nan);
    velocity->addTerm(new fl::Triangle("VeryNegative", -velocityMax, -velocityMax, -velocityMax / 2));
    velocity->addTerm(new fl::Triangle("Negative", -velocityMax, -velocityMax / 2, 0.0));
    velocity->addTerm(new fl::Triangle("Zero", -velocityMax / 10, 0.0, velocityMax / 10));
    velocity->addTerm(new fl::Triangle("Positive", 0.0, velocityMax / 2, velocityMax));
    velocity->addTerm(new fl::Triangle("VeryPositive", velocityMax / 2, velocityMax, velocityMax));
    velocity->setAggregation(new fl::Maximum());  // Set aggregation operator to Maximum
    velocity->setDefuzzifier(new fl::Centroid());
    engine->addOutputVariable(velocity);

    // Define the rule block
    fl::RuleBlock* ruleBlock = new fl::RuleBlock;
    ruleBlock->setName("RuleBlock1");
    ruleBlock->setConjunction(new fl::AlgebraicProduct());
    ruleBlock->setDisjunction(new fl::Maximum());
    ruleBlock->setActivation(new fl::General());

    // Define each rule individually
    ruleBlock->addRule(fl::Rule::parse("if Error is VeryNegative then Velocity is VeryNegative", engine));
    ruleBlock->addRule(fl::Rule::parse("if Error is Negative then Velocity is Negative", engine));
    ruleBlock->addRule(fl::Rule::parse("if Error is Zero then Velocity is Zero", engine));
    ruleBlock->addRule(fl::Rule::parse("if Error is Positive then Velocity is Positive", engine));
    ruleBlock->addRule(fl::Rule::parse("if Error is VeryPositive then Velocity is VeryPositive", engine));

    engine->addRuleBlock(ruleBlock);

    // Set implication operator explicitly
    ruleBlock->setImplication(new fl::AlgebraicProduct());

    // Check if the engine is valid
    std::string status;
    if (!engine->isReady(&status)) {
        std::cout << "Engine is not ready: " << status << std::endl;
        delete engine;
        return nullptr;
    }

    return engine;
}

// Function to perform fuzzy inference
double fuzzyInference(fl::Engine* engine, double inputError) {
    if (!engine) {
        std::cerr << "Engine is not initialized." << std::endl;
        return fl::nan;
    }

    fl::InputVariable* error = engine->getInputVariable("Error");
    fl::OutputVariable* velocity = engine->getOutputVariable("Velocity");

    // Set the input value and process the engine
    error->setValue(inputError);
    engine->process();

    // Get and return the output value
    return velocity->getValue();
}


void FuzzyVelocityTraversal(const JoystickCommand &offsetDesired, uint32_t timeMs)
{
    double originTime  = 0;
    double currentTime = 0;
    uint64_t elapsedTimeInMs = 0;

    SetJoystickMode joystickMode;
    JoystickAction joystickAction;

    joystickMode.request.horizontal_mode = joystickMode.request.HORIZONTAL_VELOCITY;
    joystickMode.request.vertical_mode = joystickMode.request.VERTICAL_VELOCITY;
    joystickMode.request.yaw_mode = joystickMode.request.YAW_RATE;
    joystickMode.request.horizontal_coordinate = joystickMode.request.HORIZONTAL_GROUND;
    joystickMode.request.stable_mode = joystickMode.request.STABLE_ENABLE;
    set_joystick_mode_client.call(joystickMode);

    joystickAction.request.joystickCommand.x = offsetDesired.x;
    joystickAction.request.joystickCommand.y = offsetDesired.y;
    joystickAction.request.joystickCommand.z = offsetDesired.z;
    joystickAction.request.joystickCommand.yaw = offsetDesired.yaw;

    originTime  = ros::Time::now().toSec();
    currentTime = originTime;
    elapsedTimeInMs = (currentTime - originTime)*1000;

    while(elapsedTimeInMs <= timeMs)
    {
        currentTime = ros::Time::now().toSec();
        elapsedTimeInMs = (currentTime - originTime) * 1000;
        joystick_action_client.call(joystickAction);
    }
}

void writeVectorsToCSV(const std::vector<float>& x, const std::vector<float>& y, const std::string& path) {
    // Open a file in write mode
    std::ofstream outFile(path);

    // Check if the file is opened successfully
    if (outFile.is_open()) {
        // Write the header
        outFile << "x,y\n";

        // Write the contents of the vectors to the file
        for (size_t i = 0; i < x.size(); ++i) {
            outFile << x[i] << "," << y[i] << "\n";
        }

        // Close the file
        outFile.close();
        std::cout << "Data successfully written to " << path << std::endl;
    } else {
        std::cerr << "Failed to open the file: " << path << std::endl;
    }
}


void writeLineToCSV(const Line& line, const std::string& path) {
    // Open a file in write mode
    std::ofstream outFile(path);

    // Check if the file is opened successfully
    if (outFile.is_open()) {
        // Write the header
        outFile << "slope,intercept,num_inliers,inlier_x,inlier_y\n";

        // Write the line data
        outFile << line.slope << ","
                << line.intercept << ","
                << line.num_inliers << ","
                << "\""; // Start of inlier points

        // Write the inlier points
        for (size_t i = 0; i < line.inlier_points.size(); ++i) {
            outFile << line.inlier_points[i].x << "," << line.inlier_points[i].y;
            if (i < line.inlier_points.size() - 1) {
                outFile << ";"; // Separate points with a semicolon
            }
        }
        outFile << "\"\n"; // End of inlier points

        // Close the file
        outFile.close();
        std::cout << "Data successfully written to " << path << std::endl;
    } else {
        std::cerr << "Failed to open the file: " << path << std::endl;
    }
}

void writePointToCSV(const Point& point, const std::string& path) {
    // Open a file in write mode
    std::ofstream outFile(path);

    // Check if the file is opened successfully
    if (outFile.is_open()) {
        // Write the header
        outFile << "x,y\n";

        // Write the point data
        outFile << point.x << "," << point.y << "\n";

        // Close the file
        outFile.close();
        std::cout << "Data successfully written to " << path << std::endl;
    } else {
        std::cerr << "Failed to open the file: " << path << std::endl;
    }
}
