#include "rviz.hpp"

rviz::rviz(ros::NodeHandle* nodehandle):nh(*nodehandle)
{

    initializePublisher(); 
}  
    void rviz::initializePublisher(){
       rviz_traject = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
       goal_visual = nh.advertise<visualization_msgs::Marker>("goal_visualization", 1);
       boat_position = nh.advertise<visualization_msgs::Marker>("boat_position", 1);
    }

    void rviz::publishTrajectory(const RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator &ptr, const double tmr){
        ROS_INFO("TEST");
        int trajectory_count = 1;
        int trajectory_markerpoints = 50;
        double r = 0.05 * 0.7;
        double tf = tmr;
        visualization_msgs::MarkerArray trajectory_marker; 
        trajectory_marker.markers.resize(trajectory_markerpoints);
        for(int i = 0; i < trajectory_markerpoints; i++){
            
            //visualization_msgs::Marker marker;
            trajectory_marker.markers[i].header.frame_id = "global_tank";
            trajectory_marker.markers[i].id = i;
        
            trajectory_marker.markers[i].type = visualization_msgs::Marker::SPHERE;;
            trajectory_marker.markers[i].action = visualization_msgs::Marker::ADD;;
            trajectory_marker.markers[i].scale.x = r;  
            trajectory_marker.markers[i].scale.y = r;
            trajectory_marker.markers[i].scale.z = r;
            trajectory_marker.markers[i].color.b = 1.0;
            trajectory_marker.markers[i].color.r = 1.0;
            trajectory_marker.markers[i].color.g = 1.0;

            trajectory_marker.markers[i].color.a = 1.0;  
            trajectory_marker.markers[i].pose.orientation.w = 1.0;
            Vec3 markerposition = ptr.GetPosition(((tf) / trajectory_markerpoints) * i);
            
            trajectory_marker.markers[i].pose.position.x = markerposition[0];  
            trajectory_marker.markers[i].pose.position.y = markerposition[1]; 
            trajectory_marker.markers[i].pose.position.z = markerposition[2];  
            //trajectory_marker.marker[i] = marker;
        }
        
            rviz_traject.publish(trajectory_marker);
        }
        
    void rviz::publishGoal(const Vec3 pose){
        visualization_msgs::Marker goal_pose;
        double r = 0.2;
        goal_pose.header.frame_id = "global_tank";
        
        goal_pose.id = 0;
        goal_pose.type =  goal_pose.CYLINDER;
        goal_pose.action =  goal_pose.ADD;
        goal_pose.scale.x = r;  
        goal_pose.scale.y = r;
        goal_pose.scale.z = 0.01;
        goal_pose.color.r = 1.0;
        goal_pose.color.g = 1.0;
        goal_pose.color.b = 0.6;
        goal_pose.color.a = 0.5 ; 
        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.7071068;
        goal_pose.pose.orientation.z = 0.0;
        goal_pose.pose.orientation.w = 0.7071068;
        goal_pose.pose.position.x = pose[0]; 
        goal_pose.pose.position.y = pose[1]; 
        goal_pose.pose.position.z = pose[2];  
        goal_visual.publish(goal_pose);
    }
    void rviz::publishBoatPosition(const Vec3 position, const int counter){
        double r = 0.05 * 0.7;
        visualization_msgs::Marker boat_pose;
        boat_pose.header.frame_id = "global_tank";
        boat_pose.id = counter;
        boat_pose.type =  boat_pose.SPHERE;
        boat_pose.action =  boat_pose.ADD;
        boat_pose.scale.x = r;  
        boat_pose.scale.y = r;
        boat_pose.scale.z = r;
        boat_pose.color.b = 1.0;
        boat_pose.color.a = 0.5 ; 
        boat_pose.pose.orientation.w = 1.0;
        boat_pose.pose.position.x = position[0]; 
        boat_pose.pose.position.y = position[1]; 
        boat_pose.pose.position.z = position[2];  
        goal_visual.publish(boat_pose);
        
    }