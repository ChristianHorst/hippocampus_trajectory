#ifndef RVIZ_HPP
#define RVIZ_HPP
#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include "CommonMath/Vec3.hpp"
#include "RapidQuadcopterTrajectories/RapidTrajectoryGenerator.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
using namespace CommonMath;
class rviz
{
public:
    rviz(ros::NodeHandle* nodehandle);
    void publishTrajectory(const RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator &ptr, const double tmr,const int coloroption);
    void publishGoal(const Vec3 pose);
    void publishBoatPosition( const CommonMath::Vec3 position, const int counter);
    void publishSingleObstacle( const CommonMath::Vec3 position, const double radius);
    void publishWalls();
    void publishVelocityText(const CommonMath::Vec3 velocity);
    void publishDataText(const CommonMath::Vec3 data);
private:
    ros::NodeHandle nh;
    ros::Publisher rviz_traject;
    ros::Publisher goal_visual;
    ros::Publisher boat_position;
    ros::Publisher single_obstacle;
    ros::Publisher tank_walls;
    ros::Publisher velo_info;
    ros::Publisher data_info;
    
    
    void initializePublisher();
    
};

#endif // RVIZ_HPP
