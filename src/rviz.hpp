#ifndef RVIZ_HPP
#define RVIZ_HPP
#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include "Vec3.h"
#include "RapidTrajectoryGenerator.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
class rviz
{
public:
    rviz(ros::NodeHandle* nodehandle);
    void publishTrajectory(const RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator &ptr, const double tmr);
    void publishGoal(const Vec3 pose);
    void publishBoatPosition( const Vec3 position, const int counter);
private:
    ros::NodeHandle nh;
    ros::Publisher rviz_traject;
    ros::Publisher goal_visual;
    ros::Publisher boat_position;
    
    
    void initializePublisher();
    
};

#endif // RVIZ_HPP
