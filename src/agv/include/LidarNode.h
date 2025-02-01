#pragma once

#include "rclcpp/rclcpp.hpp"                    //< ros library
#include "sensor_msgs/msg/laser_scan.hpp"       //< lidar subscribe library
#include "std_msgs/msg/string.hpp"              //< state publish library
#include "geometry_msgs/msg/twist.hpp"          //< speed publish library

#include "Constants.h"

namespace AGV
{
class LidarNode : public rclcpp::Node
{
public:
    LidarNode();
    ~LidarNode();

private:
    //*************** SUBSCRIBERS ******************************/

    // Lidar Subscriber 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;

    // State Subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscriber_;
    rclcpp::CallbackGroup::SharedPtr state_callback_group_;

    //*************** PUBLISHERS *******************************/

    // Speed Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_publisher_;

    // State Publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;    

    //*************** CALLBACKS ********************************/

    // Lidar callback function
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr& msg);

    // State callback function
    void stateCallback(const std_msgs::msg::String::SharedPtr& msg);

    //*************** VARIABLES *******************************/

    std::string state_;                             //< current state handler
    std::string previous_state_;                    //< previous state handler
    
    //*************** MUTEXS ***********************************/

    std::mutex state_mutex_;                        //< mutex for state

    //**************** FUNCTIONS *******************************/

    void publishState(const std::string& state);
    void publishSpeed(const double linear_speed, double angular_speed);

};  // class LidarNode

}   // namespace AGV
