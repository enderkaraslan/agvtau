#pragma once

//*************** ROS LIBRARIES ******************************/

#include "rclcpp/rclcpp.hpp"                    //< ros library
#include "sensor_msgs/msg/image.hpp"            //< camera subscribe library
#include "std_msgs/msg/string.hpp"              //< state publish library
#include "geometry_msgs/msg/twist.hpp"          //< speed publish library
#include "nav_msgs/msg/odometry.hpp"            //< Odometry library
#include "geometry_msgs/msg/pose_stamped.hpp"   //< PoseStamped library

//*************** OTHER LIBRARIES ******************************/

#include <opencv2/opencv.hpp>                   //< opencv2 library
#include "cv_bridge/cv_bridge.h"                //< transform ros image to opencv image

//*************** INCLUDES FOR OTHER CLASSES AND FUNCTIONS ******/

#include "Constants.h"                          //< my constants library
#include "ImageProcessor.h"                     //< static library to detect line from image
#include "Route.h"                              //< route information library
#include "Timer.h"                              //< timer information library

namespace AGV
{

class CameraNode : public rclcpp::Node
{
public:
    CameraNode();
    ~CameraNode();
    
private:

    //*************** SUBSCRIBERS ******************************/

    // Camera Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;
    rclcpp::CallbackGroup::SharedPtr camera_callback_group_;

    // State Subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscriber_;
    rclcpp::CallbackGroup::SharedPtr state_callback_group_;

    // Subscribe to odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::CallbackGroup::SharedPtr odometry_callback_group_;

    //*************** PUBLISHERS *******************************/

    // State Publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;    

    // Speed Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_publisher_;

    // Publish 2D goal pose for navigation
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;

    //*************** CALLBACKS ********************************/

    // Callback function for camera data
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr& msg);

    // Callback function for odometry
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr& msg);
    
    // Callback function for state data
    void stateCallback(const std_msgs::msg::String::SharedPtr& msg);
    
    //*************** VARIABLES ********************************/

    std::string state_{"LineFollow"};       //< default state to publish

    bool is_turn_in_progress_{true};        //< flag to detect a turn only once
    int8_t turning_direction_;              //< is turning to left or right
    double q_x = 0.0;                       //< orientation x
    double q_y = 0.0;                       //< orientation y
    double q_z = 0.0;                       //< orientation z
    double w = 0.0;                         //< orientation w
    double x_{0.0};                         //< position x
    double y_{0.0};                         //< position y
    double yaw_z_{0.0};                     //< angle of the vehicle 
    double initial_yaw_{0.0};               //< angle of the vehicle for the turn algorithm
    double location_x_;                     //< instant x position for turning algorithm
    double location_y_;                     //< instant y position for turning algorithm

    bool navigation_complated_{false};      //< flag to disable navigation

    //*************** MUTEXS ***********************************/

    std::mutex state_mutex_;                //< mutex for state
    std::mutex odom_mutex_;                 //< mutex for odometry messages

    //*************** CLASSES **********************************/

    Route my_route_;                        //< route class
    Timer my_timer_;                        //< timer class to keep object detection time

    //**************** FUNCTIONS *******************************/

    void publishSpeed(const double linear_speed, double angular_speed);
    void publishState(const std::string& state);
    double eulerFromQuaternion(double x, double y, double z, double w);
    double getDistanceTwoPoints(double x_1, double y_1, double x_2, double y_2);

};   // class CameraNode

}   // namespace AGV