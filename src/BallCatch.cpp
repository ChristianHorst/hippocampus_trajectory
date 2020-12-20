#include "hippocampus_test/BallCatch.hpp"
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <random>
#include <fstream>
#include <memory>
#include <vector>

using std::cerr;
using std::endl;

using std::ofstream;
#include <cstdlib> // for exit function
BallCatch::BallCatch(ros::NodeHandle* nodehandle):nh(*nodehandle), boatdata(nodehandle), rviz_publisher(nodehandle), 
                                    traj(CommonMath::Vec3(1,0,0),CommonMath::Vec3(1,0,0),CommonMath::Vec3(1,0,0))
{  // trajalll = RapidTrajectoryGenerator(CommonMath::Vec3(1,0,0),CommonMath::Vec3(1,0,0), CommonMath::Vec3(1,0,0) );
    initializePublisher(); 
    DesiredThrust=0.0;
    DesiredAxis = CommonMath::Vec3(1,0,0);
    StartPosition = CommonMath::Vec3(0.5,0.5,0.0);
    IsAtStart = false;
    OrientateTowardsGoal=false;
    PerformCatching = false;
    CloseToGoal = false;
    SetTheTime = true;
    counter = 0;

    ControlFunction();
}
void BallCatch::initializePublisher(){
        publish_desired_values = nh.advertise<mavros_msgs::HippocampusDesired>("hippocampus/desired", 1);
        output_values = nh.advertise<mavros_msgs::HippocampusOutput>("hippocampus/output", 1);
       
    }
//-------------MAIN LOOP------------------------------------    
void BallCatch::ControlFunction(){
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
    
    if(IsAtStart == false){
        //ROS_INFO("GO TO START \n");
        BallCatch::GoToStart();
        }
    if(OrientateTowardsGoal == true){
        ROS_INFO("Orientate To Goal \n");
        BallCatch::OrientateToGoal();
    }
    if(PerformCatching == true){
            if(SetTheTime == true){
                BallCatch::SetTimer(); //set timer + random position
            }
            //ROS_INFO("Catch the ball \n");
            
            BallCatch::CatchTheBall();
        }
    if(CloseToGoal == true){
        //ROS_INFO("Close To Goal \n");
        BallCatch::closeGoal();
    }
    ros::spinOnce();
    loop_rate.sleep();
    }
}

//------------------------------------------------------------
void BallCatch::calculateGoal(){
    //Random Goal 
    std::random_device rd;
    mt19937 gen(rd()); 
    uniform_real_distribution<> randPosX(1.5, 3.0);
    uniform_real_distribution<> randPosY(1.0,2.5);
    uniform_real_distribution<> randPosZ(0.0, 0.8);
    
    uniform_real_distribution<> randVelX(0.0, 0.3);
    uniform_real_distribution<> randVelY(-0.3, 0.3);
    uniform_real_distribution<> randVelZ(-0.2, 0.2);
    
    uniform_real_distribution<> randAccX(0.0, 0.0);
    uniform_real_distribution<> randAccY(0.0, 0.0);
    uniform_real_distribution<> randAccZ(0.0, 0.0);
    
    uniform_real_distribution<> randomTf(9.0, 13.0);
    
    BallCatch::goalPosition = Vec3(randPosX(gen), randPosY(gen), randPosZ(gen)); //position
    BallCatch::goalVelocity = Vec3(randVelX(gen), randVelY(gen), randVelZ(gen)); //velocity
    //BallCatch::goalAcceleration = Vec3(randAccX(gen), randAccY(gen), randAccZ(gen)); //acceleration
    BallCatch::goalAcceleration = Vec3( 0.0, 0.0, 0.0);
    
    trajectory_duration = randomTf(gen);
    
    trajcreator.SetNewPos(goalPosition);
    trajcreator.SetNewVel(goalVelocity);
    trajcreator.SetNewAccel(goalAcceleration);
    
    ROS_INFO("Random X %f:",goalPosition[0] );
    ROS_INFO("Random Y %f:",goalPosition[1] );
    ROS_INFO("Random Z %f:",goalPosition[2] );
    ROS_INFO("RandomVel X %f:",goalVelocity[0] );
    ROS_INFO("RandomVel Y %f:",goalVelocity[1] );
    ROS_INFO("RandomVel Z %f:",goalVelocity[2] );
    ROS_INFO("Random Time Tf %f:",trajectory_duration );
    
    
}

void BallCatch::GoToStart(){
   // BallCatch::calculateGoal();
    DesiredThrust = 0.5;//0.5
    DesiredAxis = (StartPosition - boatdata.GetPosition() ).GetUnitVector();
    
    BallCatch::PublishDesiredValues(DesiredThrust,DesiredAxis);
    
    if((StartPosition - boatdata.GetPosition()).GetNorm2() < 0.3){
        BallCatch::PublishDesiredValues(0.0,DesiredAxis);
        IsAtStart = true;
        OrientateTowardsGoal = true;
    }

}
void BallCatch::OrientateToGoal(){
    CommonMath::Vec3 Goal = CommonMath::Vec3(2,2.0,0);
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
 
    CommonMath::Vec3 CurrentAxis = CommonMath::Vec3(boatdata.GetOrientation()[0],boatdata.GetOrientation()[1],boatdata.GetOrientation()[2]);
    ROS_INFO("Current Axis %f %f %f: ",axis[0],axis[1],axis[2]);
    BallCatch::PublishDesiredValues(DesiredThrust,DesiredAxis);
    BallCatch::calculateGoal();
    ros::Duration(3.0).sleep();
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
//calculate the goal function 4
void BallCatch::CatchTheBall(){
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    int coloroption = 0;  
    counter = counter + 1;
    ros::WallTime go = ros::WallTime::now();
    execution_time =( (go.toSec()) - (timer.toSec()) );
    double evaluation_time = 2.0 / 30.0;
    int timecorrection = 1;
    
   // ROS_INFO_STREAM("Execution time (s): " << (execution_time ) );
    
   // if(execution_time > trajectory_duration + 0.5){
     //   ROS_INFO("TIME OVER\n");
       // BallCatch::Reset();
    //}
//Current State
    CommonMath::Vec3 pos0 = boatdata.GetPosition(); //position
    CommonMath::Vec3 vel0 = boatdata.GetVelocity(); //velocity
    CommonMath::Vec3 acc0 = boatdata.GetAcceleration() *0; //acceleration
    CommonMath::Vec3 current_axis = boatdata.GetCurrentAxis();
    /*
    //Close To Goal-------------------------------------------------
    if((goalPosition - boatdata.GetPosition()).GetNorm2() < 0.4 || execution_time == trajectory_duration - 0.5)  {
        ROS_INFO("Close To Goal\n");
        PerformCatching = false;
        CloseToGoal = true;
        SetTheTime = true;

    }
    */
   //current_cost= norm(omega * thrust - c*accel)**2
   //accel=traj.GetAcceleration(execution_time)
    //OBSTACLE------------------
    Vec3 obsPos(1.5, 1.0, 0.0);
    Vec3 obsPos2(2.0, 1.5, 0.0);
    Vec3 obsPos3(1.8, 1.25, 0.5);
    Vec3 obsPos4(1.5, 0.15, 0.0);
    double obsRadius = 0.15;
    shared_ptr<ConvexObj> obstacle = make_shared < Sphere
        > (obsPos, obsRadius);
    shared_ptr<ConvexObj> obstacle2 = make_shared < Sphere
        > (obsPos2, obsRadius);
    shared_ptr<ConvexObj> obstacle3 = make_shared < Sphere
    > (obsPos3, obsRadius);
     shared_ptr<ConvexObj> obstacle4 = make_shared < Sphere
    > (obsPos4, obsRadius);
        
    Tf = trajectory_duration - execution_time; //Duration

   
    
//After Position and Input Feasibility Tests 1 Trajectory gets returned
  
    traj = trajcreator.GenerateTrajectories(pos0,vel0,acc0,3000,Tf);
    

 //   CollisionChecker checker(traj.GetTrajectory());
 //   CollisionChecker::CollisionResult stateFeas = checker.CollisionCheck(obstacle, 0.02);
 //   CollisionChecker::CollisionResult stateFeas2 = checker.CollisionCheck(obstacle2, 0.02);
 //   CollisionChecker::CollisionResult stateFeas3 = checker.CollisionCheck(obstacle3, 0.02);
 //   CollisionChecker::CollisionResult stateFeas4 = checker.CollisionCheck(obstacle4, 0.02);
    
   // std::cout << "Time difference Collision = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    //std::cout << "Time difference Collision= " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
    
  //  if(stateFeas == 1 || stateFeas2 == 1 || stateFeas3 == 1 || stateFeas4 == 1)coloroption = 1;
    
   // ROS_INFO("Collision Check %d : \n", stateFeas);
   // ROS_INFO("Collision Check2 %d : \n", stateFeas2);
    
    trajcreator.DeleteTrajectoryList();
    if(Tf <= 0.7) timecorrection=0;
    
    DesiredAxis =  traj.GetNormalVector((evaluation_time + 0.6 * timecorrection));
    Vec3 thrustvector = traj.GetThrustVector(evaluation_time + 0.3 * timecorrection);
    
 //   ROS_INFO("Thrust X %f:", thrustvector[0]); 
  //  ROS_INFO("Thrust Y %f: \n", thrustvector[1]);
   // ROS_INFO("Thrust Z %f:\n", thrustvector[2]); 
    DesiredThrust = BallCatch::calculateThrust(traj.GetThrust(evaluation_time + 0.3 * timecorrection));
    if(traj.GetThrust(evaluation_time) < 0.7) DesiredThrust = 0.4;

   
    //ROS_INFO("THRUST NEWTON  & TF SYS %f %f: \n", traj.GetThrust(evaluation_time),evaluation_time );
    //ROS_INFO("THRUST & TF %f %f: \n", DesiredThrust, traj.GetTf());
    //ROS_INFO("DesAxis %f %f %f: \n", DesiredAxis[0],DesiredAxis[1],DesiredAxis[2]);
    CommonMath::Vec3 datainfo = Vec3(execution_time, DesiredThrust, traj.GetThrust(evaluation_time + 0.3 * timecorrection)); //traj.GetMaxThrust()
   // ROS_INFO("Maximal Rate %f : \n", traj.GetMaxRate());
    BallCatch::PublishDesiredValues(DesiredThrust,DesiredAxis);
    
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
   // std::cout << "Time difference Traj Gen= " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
   // std::cout << "Time difference Traj Gen= " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
    rviz_publisher.publishTrajectory(traj, traj.GetTf(), coloroption);
    rviz_publisher.publishGoal(goalPosition);
    rviz_publisher.publishBoatPosition(pos0, counter);
  //  rviz_publisher.publishSingleObstacle(obsPos, obsRadius);
   // rviz_publisher.publishSingleObstacle2(obsPos2, obsRadius);
   // rviz_publisher.publishSingleObstacle3(obsPos3, obsRadius);
   // rviz_publisher.publishSingleObstacle4(obsPos4, obsRadius);
   // rviz_publisher.publishWalls();
    rviz_publisher.publishVelocityText(vel0);
    rviz_publisher.publishDataText(datainfo);
    //Publish rviz Text Velocity
    //--------------------------------------------------------------OUTPUT DATA ROSBAG-------------------
mavros_msgs::HippocampusOutput output;
    output.frame_stamp = ros::Time::now();
    CommonMath::Vec3 pos_output=traj.GetPosition(execution_time);
    CommonMath::Vec3 vel_output=traj.GetVelocity(execution_time);
    CommonMath::Vec3 acc_output=traj.GetAcceleration(execution_time);
    output.des_position.x =pos_output[0]; 
    output.des_position.y =pos_output[1]; 
    output.des_position.z =pos_output[2]; 
    
    output.des_velocity.x =vel_output[0]; 
    output.des_velocity.y =vel_output[1]; 
    output.des_velocity.z =vel_output[2];
    output.des_acceleration.x =acc_output[0]; 
    output.des_acceleration.y =acc_output[1]; 
    output.des_acceleration.z =acc_output[2]; 
    
    output.current_position.x = pos0[0];
    output.current_position.y = pos0[1];
    output.current_position.z = pos0[2];
    
    output.current_velocity.x = vel0[0];
    output.current_velocity.y = vel0[1];
    output.current_velocity.z = vel0[2];
    
    output.current_acceleration.x = acc0[0];
    output.current_acceleration.y = acc0[1];
    output.current_acceleration.z = acc0[2];
    
    output.des_orientation.x=DesiredAxis[0];
    output.des_orientation.y=DesiredAxis[1];
    output.des_orientation.z=DesiredAxis[2];
    output.current_orientation.x=current_axis[0];
    output.current_orientation.y=current_axis[1];
    output.current_orientation.z=current_axis[2];
    
    output.thrust_time.x=DesiredThrust;
    output.thrust_time.y=execution_time;
    output.thrust_time.z=0.0;
    
    output_values.publish(output);
    //Goal State
    if((goalPosition - boatdata.GetPosition()).GetNorm2() < 0.05 ||execution_time > trajectory_duration + 0.5){
        ROS_INFO("Position Reached\n");
        BallCatch::writeData(BallCatch::goalPosition,pos0,BallCatch::goalVelocity,vel0,trajectory_duration,execution_time,current_axis);
        BallCatch::PublishDesiredValues(0.0,DesiredAxis);
        
        ros::Duration(2.0).sleep();
        
        BallCatch::Reset();
    }
}
void BallCatch::PublishDesiredValues(const double thrust_value, const Vec3 axis){
    
    mavros_msgs::HippocampusDesired msg;
    msg.frame_stamp = ros::Time::now();
    msg.thrust = thrust_value;
    msg.rollrate =  axis[0];
    msg.pitchrate=  axis[1];
    msg.yawrate  =  axis[2];
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
    if(thrust > 1.0) thrust = 1.0;
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
    counter=0;
    CloseToGoal=false;
    
    ofstream outdata; 
    
}
void BallCatch::writeData(const CommonMath::Vec3 des_pos,const CommonMath::Vec3 curr_pos,const CommonMath::Vec3 des_vel,
                   const CommonMath::Vec3 curr_vel,double des_time,double final_time,const CommonMath::Vec3 curr_orient){
    ofstream outdata; 
    ofstream outdata_ext; 
    ofstream outdata_visual;
    outdata.open("multitraject_test.txt",std::ios_base::app);
    outdata_ext.open("multitraject_test_extended.txt",std::ios_base::app);
    outdata_visual.open("multitraject_test_visual.txt",std::ios_base::app);
    
    ROS_INFO("Writing in File");
    
    outdata_visual << "DesiredPos: " << des_pos[0] << ","  << des_pos[1] << ","  << des_pos[2] <<std::endl;
    outdata_visual << "FinalPos: " << curr_pos[0] << ","  << curr_pos[1] << ","  << curr_pos[2] <<std::endl;
    outdata_visual << "DesiredVel: " << des_vel[0] << ","  << des_vel[1] << ","  <<des_vel[2] <<std::endl;
    outdata_visual << "FinalVel: " << curr_vel[0] << ","  << curr_vel[1] << ","  <<curr_vel[2] <<std::endl;
    outdata_visual << "DesiredTime: " << des_time <<std::endl;
    outdata_visual << "FinalTime: " <<  final_time <<std::endl;
    outdata_visual << "\n\n" <<std::endl;
    
    outdata << des_pos[0] << ","  << des_pos[1] << ","  << des_pos[2] <<
      "," << curr_pos[0] << ","  << curr_pos[1] << ","  << curr_pos[2] <<
      "," << des_vel[0] << ","  << des_vel[1] << ","  <<des_vel[2] <<
     "," << curr_vel[0] << ","  << curr_vel[1] << ","  <<curr_vel[2] <<
      "," << des_time <<
     "," <<  final_time <<std::endl;
     
     outdata_ext << des_pos[0] << ","  << des_pos[1] << ","  << des_pos[2] <<
      "," << curr_pos[0] << ","  << curr_pos[1] << ","  << curr_pos[2] <<
      "," << des_vel[0] << ","  << des_vel[1] << ","  <<des_vel[2] <<
     "," << curr_vel[0] << ","  << curr_vel[1] << ","  <<curr_vel[2] <<
      "," << des_time <<
     "," <<  final_time <<
    "," << curr_orient[0] << ","  << curr_orient[1] << ","  <<curr_orient[2] <<
     std::endl;
    
    
    
    
    
    outdata.close();
    outdata_visual.close();
    
}   
void BallCatch::closeGoal(){
    ROS_INFO("In close Goal Function\n");
    ros::WallTime go = ros::WallTime::now();
    execution_time =( (go.toSec()) - (timer.toSec()) );
    if((goalPosition - boatdata.GetPosition()).GetNorm2() < 0.2 || execution_time > trajectory_duration + 0.5)  {
         BallCatch::Reset();
     }
     
    double evaluationTime = Tf - (trajectory_duration - execution_time);
    DesiredThrust = BallCatch::calculateThrust(traj.GetThrust(evaluationTime));
    DesiredAxis =  traj.GetNormalVector((evaluationTime));
    
    ROS_INFO("Eval Time: %f\n", evaluationTime);
    BallCatch::PublishDesiredValues(DesiredThrust,DesiredAxis);
    

    CommonMath::Vec3 datainfo = Vec3(execution_time, DesiredThrust, 0.0);
    rviz_publisher.publishVelocityText(boatdata.GetVelocity());
    rviz_publisher.publishDataText(datainfo);
}