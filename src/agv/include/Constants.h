#pragma once


namespace Constants {

    // Topics
    constexpr char CameraTopic[]            = "/camera/image_raw";
    constexpr char TwistTopic[]             = "/cmd_vel";
    constexpr char LidarTopic[]             = "/scan";
    constexpr char OdometryTopic[]          = "/odom";
    constexpr char LidarAlertTopic[]        = "/lidar_alert";
    constexpr char ImageInfoTopic[]         = "/image_info";    
    constexpr char OccupancyGridTopic[]     = "/map";
    constexpr char PathTopic[]              = "/plan";   
    constexpr char StateTopic[]             = "/state";   

    // Publisher
    constexpr char SpeedPublish[]           = "cmd_vel";
    constexpr char MarkerPublish[]          = "visualization_marker";
    constexpr char GoalPosePublish[]        = "goal_pose";

    // Queue Size for Communication
    constexpr int QueueSize                 = 10;

    // Motion Control Parameters
    constexpr double TurningSpeed           = 0.35;
    constexpr double LinearSpeed            = 0.3;
    constexpr double AngularSpeedScale      = 0.003;

    // Motion Duration Constants
    constexpr double TurnLeftDurationSec    = 4.05;  // seconds
    constexpr double TurnRightDurationSec   = 4.05;  // seconds
    constexpr int AdjustmentTimerMs         = 1800;  // milliseconds
    constexpr double Distance               = 0.58;   // meters

    // Image Processing Constants
    constexpr double BINARY_THRESHOLD_VALUE = 20.0;
    constexpr double MinExtent              = 0.2;

    // Circle Detection Parameters
    constexpr int CircleRadius              = 5;
    constexpr int MiddleCircleRadius        = 3;

    // Pixel Value Constants
    constexpr int BlackPixelValue           = 300;
    constexpr int MidPixelValue             = 255;

    // Region of Interest (ROI) for Left and Right Sides of the Camera
    constexpr int LEFT_Y_START              = 420;
    constexpr int LEFT_Y_END                = 480;
    constexpr int LEFT_X_START              = 0;
    constexpr int LEFT_X_END                = 5;
    
    constexpr int RIGHT_Y_START             = 420;
    constexpr int RIGHT_Y_END               = 480;
    constexpr int RIGHT_X_START             = 635;
    constexpr int RIGHT_X_END               = 640;

    // Minimum Area for Detection
    constexpr double MinimumArea            = 205000.0;

    // Obstacle Detection Parameters
    constexpr double DetectionDistance      = 1.3;
    constexpr double MsToSeconds            = 1000.0;
    constexpr double WaitingTimeSeconds     = 5.0;

}
