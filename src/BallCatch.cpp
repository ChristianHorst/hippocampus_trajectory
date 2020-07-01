#include "BallCatch.hpp"

BallCatch::BallCatch(ros::NodeHandle* nodehandle):nh(*nodehandle), boatdata(nodehandle), rviz_publisher(nodehandle)
{   
    initializePublisher(); 
    DesiredThrust=0.0;
    DesiredAxis = Vec3(1,0,0);
    StartPosition = Vec3(0.3,0.3,0);
    IsAtStart = false;
    OrientateTowardsGoal=false;
    PerformCatching = false;
    SetTheTime = true;
   
    ControlFunction();
}
void BallCatch::initializePublisher(){
        publish_desired_values = nh.advertise<mavros_msgs::HippocampusControl>("hippocampus/desired_values", 1);
       
    }
//-------------MAIN LOOP------------------------------------    
void BallCatch::ControlFunction(){
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
    
    if(IsAtStart == false){
        ROS_INFO("GO TO START \n");
        BallCatch::GoToStart();
        }
    if(OrientateTowardsGoal == true){
        ROS_INFO("Orientate To Goal \n");
        BallCatch::OrientateToGoal();
    }
    if(PerformCatching == true){
            if(SetTheTime == true){
                BallCatch::SetTimer();
            }
            ROS_INFO("Catch the ball \n");
            
            
            BallCatch::CatchTheBall();
        }
    ros::spinOnce();
    loop_rate.sleep();
    }
}

//------------------------------------------------------------


void BallCatch::GoToStart(){
    DesiredThrust = 0.5;
    DesiredAxis = (StartPosition - boatdata.GetPosition() ).GetUnitVector();
    
    BallCatch::PublishDesiredValues(DesiredThrust,DesiredAxis);
    
    if((StartPosition - boatdata.GetPosition()).GetNorm2() < 0.2){
        BallCatch::PublishDesiredValues(0.0,DesiredAxis);
        IsAtStart = true;
        OrientateTowardsGoal = true;
    }

}
void BallCatch::OrientateToGoal(){
    Vec3 Goal = Vec3(2,1.5,0);
    DesiredThrust = 0.0;
    DesiredAxis = (Goal - boatdata.GetPosition() ).GetUnitVector();
    tf2::Quaternion myQuat, curaxis ,qinv;
    tf2::Vector3 axis;
    myQuat[0] = boatdata.GetOrientation()[0];
    myQuat[1] = boatdata.GetOrientation()[1];
    myQuat[2] = boatdata.GetOrientation()[2];
    myQuat[3] = boatdata.GetOrientation()[3];
    curaxis[0]=0;curaxis[1]=0;curaxis[2]=0;curaxis[3]=1;
    qinv = curaxis*myQuat.inverse();
    axis = qinv.getAxis();
 
    Vec3 CurrentAxis = Vec3(boatdata.GetOrientation()[0],boatdata.GetOrientation()[1],boatdata.GetOrientation()[2]);
    ROS_INFO("Current Axis %f %f %f: ",axis[0],axis[1],axis[2]);
    BallCatch::PublishDesiredValues(DesiredThrust,DesiredAxis);
    ros::Duration(5.0).sleep();
    OrientateTowardsGoal = false;
    PerformCatching = true;
    /*
    if((DesiredAxis - CurrentAxis).GetNorm2() < 0.2){
        ROS_INFO("IN IF COND\n");
        BallCatch::PublishDesiredValues(0.0,DesiredAxis);
        OrientateTowardsGoal = false;
        PerformCatching = true;
    }
    */    
    
}
void BallCatch::CatchTheBall(){
    double trajectory_duration = 6.0;
    ros::WallTime go = ros::WallTime::now();
    double execution_time =( (go.toSec()) - (timer.toSec()) );
    double evaluation_time = 2.0 / 30.0;
    int timecorrection = 1;
    
    ROS_INFO_STREAM("Execution time (s): " << (execution_time ) );
    
    if(execution_time > trajectory_duration + 2.0){
        ROS_INFO("TIME OVER\n");
        BallCatch::Reset();
    }
//Current State
    Vec3 pos0 = boatdata.GetPosition(); //position
    Vec3 vel0 = boatdata.GetVelocity(); //velocity
    Vec3 acc0 = boatdata.GetAcceleration() *0; //acceleration
//Goal State
    Vec3 posf = Vec3(2.0, 1.5, 0.5); //position
    Vec3 velf = Vec3(0.5, 0, 0); //velocity
    Vec3 accf = Vec3(0, 0, 0); //acceleration
    if((posf - boatdata.GetPosition()).GetNorm2() < 0.2){
        ROS_INFO("Position Reached\n");
        BallCatch::Reset();
    }
        
        
    double Tf = trajectory_duration - execution_time; //Duration
    
    trajcreator.SetGoalPosition(posf);
    trajcreator.SetGoalVelocity(velf);
    trajcreator.SetGoalAcceleration(accf);
    
//After Position and Input Feasibility Tests 1 Trajectory gets returned
    RapidTrajectoryGenerator traj = trajcreator.GenerateTrajectories(pos0,vel0,acc0,50,Tf);
    trajcreator.DeleteTrajectoryList();
    if(Tf <= 0.7) timecorrection=0;
    DesiredAxis =  traj.GetNormalVector((evaluation_time + 0.6 * timecorrection));
    
    DesiredThrust = BallCatch::calculateThrust(traj.GetThrust(evaluation_time));
    if(traj.GetThrust(evaluation_time) < 0.8) DesiredThrust = 0.4;
    
    ROS_INFO("THRUST NEWTON  & TF SYS %f %f: \n", traj.GetThrust(evaluation_time),evaluation_time );
    ROS_INFO("THRUST & TF %f %f: \n", DesiredThrust, traj.GetTf());
    ROS_INFO("DesAxis %f %f %f: \n", DesiredAxis[0],DesiredAxis[1],DesiredAxis[2]);
 
    BallCatch::PublishDesiredValues(DesiredThrust,DesiredAxis);
    rviz_publisher.publishTrajectory(traj, traj.GetTf());
    rviz_publisher.publishGoal(posf);
    rviz_publisher.publishBoatPosition(pos0, 1);
    
    
}
void BallCatch::PublishDesiredValues(const double thrust_value, const Vec3 axis){
    
    mavros_msgs::HippocampusControl msg;
    msg.frame_stamp = ros::Time::now();
    msg.thrust = thrust_value;
    msg.roll_effort =  axis[0];
    msg.pitch_effort=  axis[1];
    msg.yaw_effort  =  axis[2];
    publish_desired_values.publish(msg);
} 

double BallCatch::calculateThrust(const double force){
    double thrust = 0.0;
    double  a = 2.26892684e-05,
            b = -8.14731295e-04,
            c = 1.58229761e-01 - force;
    double d = b * b  - 4 * a * c;
    double x1 = 0.0;
    double x2 = 0.0;
   
    if (d < 0){
        double dummy=0.0;
            
    }else if (d == 0){
            x1 = -b / (2 * a);
        
    }else if (d > 0){ 
        
            x1 = (-b + sqrt(d)) / (2 * a);
            x2 = (-b - sqrt(d)) / (2 * a);
    }
    if (x1 >0 and x1>x2){
        thrust= x1/500;
    }
    if (x2 >0 and x2>x1){
        thrust = x2/500;
    }

    return thrust;
};
void BallCatch::SetTimer(){
    timer = ros::WallTime::now();
    SetTheTime = false;
}
void BallCatch::Reset(){
    IsAtStart = false;
    PerformCatching = false;
    SetTheTime = true;
}