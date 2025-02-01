#include "CameraNode.h"

namespace AGV
{

CameraNode::CameraNode() : Node("camera_node")
{
    // Camera Subscriber
    rclcpp::SubscriptionOptions camera_callback_options;
    camera_callback_group_                      = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    camera_callback_options.callback_group      = camera_callback_group_;

    camera_subscriber_                          = this->create_subscription<sensor_msgs::msg::Image>(
                                                    Constants::CameraTopic, Constants::QueueSize,
                                                    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                                                        cameraCallback(msg);
                                                    }, camera_callback_options);
    
    rclcpp::SubscriptionOptions sub_options_odometry;
    odometry_callback_group_                    = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options_odometry.callback_group         = odometry_callback_group_;

    odometry_subscriber_                        = this->create_subscription<nav_msgs::msg::Odometry>(
                                                    Constants::OdometryTopic, Constants::QueueSize,
                                                    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                                                        odometryCallback(msg);
                                                    },sub_options_odometry);
    
    // State Subscriber
    rclcpp::SubscriptionOptions state_callback_options;
    state_callback_group_                       = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_callback_options.callback_group       = state_callback_group_;

    state_subscriber_                           = this->create_subscription<std_msgs::msg::String>(
                                                    Constants::StateTopic, Constants::QueueSize,
                                                    [this](const std_msgs::msg::String::SharedPtr msg) {
                                                        this->stateCallback(msg);
                                                    }, state_callback_options);



    // State Publisher
    state_publisher_                            = this->create_publisher<std_msgs::msg::String>(
                                                    Constants::StateTopic, Constants::QueueSize);

    // Speed Publisher
    speed_publisher_                            = this->create_publisher<geometry_msgs::msg::Twist>(
                                                    Constants::TwistTopic, Constants::QueueSize);

    // Publish 2D goal pose for navigation
    goal_pose_publisher_                        = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                                                    Constants::GoalPosePublish, Constants::QueueSize);

    RCLCPP_DEBUG(this->get_logger(), "CameraNode has been started.");
}

CameraNode::~CameraNode()
{
    RCLCPP_DEBUG(this->get_logger(), "CameraNode has been stopped.");
}

// Callback function for camera data
void CameraNode::cameraCallback(const sensor_msgs::msg::Image::SharedPtr& msg)
{
    std::string state; 
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = state_;
    }
    double yaw_z;       //< 90 derecelik dönüş yapmak için aracın açı bilgisi alınıyor
    {
        std::lock_guard<std::mutex> guard(odom_mutex_);
        yaw_z = yaw_z_;
    }

    if (state == "LineFollow")
    {
        ////////////////////////////////////////////////////////////////
        my_timer_.reset(); // Nesne algılandıktan sonra 5 saniye beklemeliyiz ancak beklerken başka bir state çalışırsa saat kaldığı yerden devam ediyor.
        ////////////////////////////////////////////////////////////////
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        ImageProcessor::ContourAnalysisResult result = ImageProcessor::process(image);

        if (result.valid && result.extent > Constants::MinExtent)
        {
            publishState("LineFollow"); //TODO: CHECK

            // Dönüş alanları
            if (result.area > Constants::MinimumArea && result.mid_pixel == Constants::MidPixelValue && (result.left_black_pixel_count == Constants::BlackPixelValue || result.right_black_pixel_count == Constants::BlackPixelValue) && is_turn_in_progress_)
            {
                // Eğer bu dönüşten dönmemiz gerekiyorsa sağ mu sol mu olduğunu kontrol et
                if (my_route_.isTurn())
                {
                    if (my_route_.getCurrentTurnType() == Route::TurnType::LEFT)
                    {
                        turning_direction_ = 1;
                        publishState("TurnLeft");
                        RCLCPP_DEBUG(this->get_logger(), "State set to TurnLeft.");

                    }
                    else if (my_route_.getCurrentTurnType() == Route::TurnType::RIGHT)
                    {
                        turning_direction_ = -1;
                        publishState("TurnRight");
                        RCLCPP_DEBUG(this->get_logger(), "State set to TurnRight.");
                    }

                    {
                        std::lock_guard<std::mutex> guard(odom_mutex_);
                        location_x_ = x_;
                        location_y_ = y_;
                    }
                }
                else if (result.area > Constants::MinimumArea && result.mid_pixel == Constants::MidPixelValue)
                {
        
                    publishSpeed(Constants::LinearSpeed, 0.0);
                }
                else
                {
                    my_route_.getCurrentTurnOrder()--;
                }
                is_turn_in_progress_ = false;
            }
            // Dönüş alanına gelirken sağa sola kaymaması için düz devam ediyor
            else if (result.area > Constants::MinimumArea && result.mid_pixel == Constants::MidPixelValue)
            {
        
                publishSpeed(Constants::LinearSpeed, 0.0);

            }
            // Dönüş yok çizgiyi takip ediyoruz
            else
            {
                is_turn_in_progress_ = true;

                publishSpeed(Constants::LinearSpeed, static_cast<double> ((result.middle_x - result.contour_center.x) * Constants::AngularSpeedScale));

            }

        }
        // Eğer çizgi yoksa duruyoruz
        else
        {
            publishSpeed(0.0, 0.0);
        }
    }

    if (state == "TurnLeft" || state == "TurnRight")
    {
        ////////////////////////////////////////////////////////////////
        my_timer_.reset(); // Nesne algılandıktan sonra 5 saniye beklemeliyiz ancak beklerken başka bir state çalışırsa saat kaldığı yerden devam ediyor.
        ////////////////////////////////////////////////////////////////
        double distance_travelled;
        {
            std::lock_guard<std::mutex> guard(odom_mutex_);
            distance_travelled = getDistanceTwoPoints(x_,y_,location_x_,location_y_);
        }

        if (distance_travelled <= Constants::Distance)
        {
            publishSpeed(Constants::LinearSpeed, 0.0);
        
            initial_yaw_ = yaw_z;
        }
        else if (std::abs(yaw_z - initial_yaw_) < 1.57)
        {
            publishSpeed(0.0, turning_direction_*Constants::TurningSpeed);
        }          
        else
        {
            my_route_.removeCurrentTurn();
            publishState("LineFollow"); // Dönüşten sonra tekrar Idle state'ine girilsin
        }
    }

    if (state == "Obstacle")
    {
        if (!my_timer_.isRunning())
        {
            my_timer_.start();
        }

        if (my_timer_.elapsedMilliseconds() / Constants::MsToSeconds >= Constants::WaitingTimeSeconds)
        {
            my_timer_.reset();

            auto message = geometry_msgs::msg::PoseStamped();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "map"; 

            // Calculate the goal position 1 meter ahead of the robot
            double distance_ahead = 2.0;  // 1 meter
            double goal_x = x_ + distance_ahead * std::cos(yaw_z_);
            double goal_y = y_ + distance_ahead * std::sin(yaw_z_);

            message.pose.position.x = goal_x;
            message.pose.position.y = goal_y;
            message.pose.position.z = 0.0;  // Keep Z at 0 for 2D navigation

            // Maintain the same orientation as the robot
            message.pose.orientation.x = q_x;
            message.pose.orientation.y = q_y;
            message.pose.orientation.z = q_z;   // Quaternion Z component
            message.pose.orientation.w = w;     // Quaternion W component
            
            RCLCPP_INFO(this->get_logger(), "Publishing goal pose: x=%.2f, y=%.2f",
                        message.pose.position.x, message.pose.position.y);

            publishState("ObstacleAvoidance"); 

            goal_pose_publisher_->publish(message);
        }
    }  
    
    if (state == "ObstacleAvoidance") 
    {
        if (navigation_complated_)
        {   
            navigation_complated_ = false;
            publishState("Idle");
        }
    }
}

void CameraNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
    std::lock_guard<std::mutex> guard(odom_mutex_);

    q_x = msg->pose.pose.orientation.x;
    q_y = msg->pose.pose.orientation.y;
    q_z = msg->pose.pose.orientation.z;
    w = msg->pose.pose.orientation.w;
    x_ = msg->pose.pose.position.x;  
    y_ = msg->pose.pose.position.y; 
    yaw_z_ = eulerFromQuaternion(q_x, q_y, q_z, w);
    
}

void CameraNode::stateCallback(const std_msgs::msg::String::SharedPtr& msg)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_ = msg->data;
    }

}



void CameraNode::publishSpeed(const double linear_speed, double angular_speed)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x      = linear_speed;
    twist.angular.z     = angular_speed;

    speed_publisher_->publish(twist);
}

void CameraNode::publishState(const std::string& state)
{
    auto message        = std_msgs::msg::String();
    message.data        = state;  
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_          = message.data;
    }
    state_publisher_->publish(message);
}

double CameraNode::eulerFromQuaternion(double x, double y, double z, double w) 
{
    // Roll (x-axis rotation)
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y * y);
    double roll_x = std::atan2(t0, t1);

    // Pitch (y-axis rotation)
    double t2 = +2.0 * (w * y - z * x);
    t2 = std::clamp(t2, -1.0, +1.0); 
    double pitch_y = std::asin(t2);

    // Yaw (z-axis rotation)
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y * y + z * z);
    double yaw_z = std::atan2(t3, t4);

    //double yaw_degrees = yaw_z * (180.0 / M_PI);

    return yaw_z; 
}

double CameraNode::getDistanceTwoPoints(double x_1, double y_1, double x_2, double y_2)
{
    double dx = x_2 - x_1;
    double dy = y_2 - y_1;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace AGV
