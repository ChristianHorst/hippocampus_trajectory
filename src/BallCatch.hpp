#ifndef BALLCATCH_HPP
#define BALLCATCH_HPP
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensor_msgs/Imu.h"
#include <mavros/mavros_plugin.h>
#include <mavros_msgs/HippocampusControl.h>
#include "data_subscriber.hpp"
#include "rviz.hpp"
#include <chrono>
#include "RapidTrajectoryGenerator.h"
#include "TrajectoryCreator.hpp"
#include <math.h> 
#include <time.h>
#include <tf2/LinearMath/Quaternion.h>
class BallCatch
{
public:
    BallCatch(ros::NodeHandle* nodehandle);
    
   
    void CatchTheBall();
    void ControlFunction();
private:
    ros::NodeHandle nh;
    ros::Publisher publish_desired_values;
    ros::WallTime timer;
    
    data_subscriber boatdata;
    rviz rviz_publisher;
    TrajectoryCreator trajcreator;
    void SetTimer();
    void Reset();
    void initializePublisher();
    void PublishDesiredValues(const double thrust_value, const Vec3 axis);
    double calculateThrust(const double force);
    void GoToStart();
    void OrientateToGoal();
    //void ControlFunction();
    
    
    bool IsAtStart;
    bool PerformCatching;
    bool SetTheTime;
    bool OrientateTowardsGoal;
    
    double DesiredThrust;
    Vec3 DesiredAxis,StartPosition;
    
};

#endif // BALLCATCH_HPP
