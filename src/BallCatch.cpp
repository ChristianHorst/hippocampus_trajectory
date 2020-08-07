#include "hippocampus_test/BallCatch.hpp"

BallCatch::BallCatch(ros::NodeHandle* nodehandle):nh(*nodehandle), boatdata(nodehandle), rviz_publisher(nodehandle)
{   
    initializePublisher(); 
    DesiredThrust=0.0;
    DesiredAxis = CommonMath::Vec3(1,0,0);
    StartPosition = CommonMath::Vec3(0.5,0.5,0.5);
    IsAtStart = false;
    OrientateTowardsGoal=false;
    PerformCatching = false;
    SetTheTime = true;
    counter = 0;
   
    ControlFunction();
}
void BallCatch::initializePublisher(){
        publish_desired_values = nh.advertise<mavros_msgs::HippocampusDesired>("hippocampus/desired", 1);
       
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
    CommonMath::Vec3 Goal = CommonMath::Vec3(2,1.5,0);
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
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    double trajectory_duration = 7.0;
    int coloroption = 0;
    counter = counter + 1;
    ros::WallTime go = ros::WallTime::now();
    double execution_time =( (go.toSec()) - (timer.toSec()) );
    double evaluation_time = 2.0 / 30.0;
    int timecorrection = 1;
    
    ROS_INFO_STREAM("Execution time (s): " << (execution_time ) );
    
    if(execution_time > trajectory_duration + 0.5){
        ROS_INFO("TIME OVER\n");
        BallCatch::Reset();
    }
//Current State
    CommonMath::Vec3 pos0 = boatdata.GetPosition(); //position
    CommonMath::Vec3 vel0 = boatdata.GetVelocity(); //velocity
    CommonMath::Vec3 acc0 = boatdata.GetAcceleration() *0; //acceleration
//Goal State
    CommonMath::Vec3 posf = Vec3(2.0, 1.0, 0.5); //position
    //posf = Vec3(1.5, 1.5, 0.5);
    CommonMath::Vec3 velf = Vec3(0.5, 0, 0); //velocity
    CommonMath::Vec3 accf = Vec3(0, 0, 0); //acceleration
    if((posf - boatdata.GetPosition()).GetNorm2() < 0.2){
        ROS_INFO("Position Reached\n");
        BallCatch::Reset();
    }
    
    //OBSTACLE------------------
    Vec3 obsPos(1.0, 1.0, 0.5);
    double obsRadius = 0.8;
    shared_ptr<ConvexObj> obstacle = make_shared < Sphere
        > (obsPos, obsRadius);
        
    double Tf = trajectory_duration - execution_time; //Duration
    
    trajcreator.SetGoalPosition(posf);
    trajcreator.SetGoalVelocity(velf);
    trajcreator.SetGoalAcceleration(accf);
    
//After Position and Input Feasibility Tests 1 Trajectory gets returned
    RapidTrajectoryGenerator traj = trajcreator.GenerateTrajectories(pos0,vel0,acc0,300,Tf);
    
   // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    CollisionChecker checker(traj.GetTrajectory());
    CollisionChecker::CollisionResult stateFeas = checker.CollisionCheck(obstacle, 0.02);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    
    std::cout << "Time difference Collision = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    std::cout << "Time difference Collision= " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
    
    if(stateFeas == 1)coloroption = 1;
    
    ROS_INFO("Collision Check %d : \n", stateFeas);
    
    trajcreator.DeleteTrajectoryList();
    if(Tf <= 0.7) timecorrection=0;
    DesiredAxis =  traj.GetNormalVector((evaluation_time + 0.6 * timecorrection));
    
    DesiredThrust = BallCatch::calculateThrust(traj.GetThrust(evaluation_time + 0.3 * timecorrection));
    if(traj.GetThrust(evaluation_time) < 0.8) DesiredThrust = 0.45;
    
    //ROS_INFO("THRUST NEWTON  & TF SYS %f %f: \n", traj.GetThrust(evaluation_time),evaluation_time );
    //ROS_INFO("THRUST & TF %f %f: \n", DesiredThrust, traj.GetTf());
    //ROS_INFO("DesAxis %f %f %f: \n", DesiredAxis[0],DesiredAxis[1],DesiredAxis[2]);
    CommonMath::Vec3 datainfo = Vec3(execution_time, traj.GetMaxThrust(), traj.GetThrust(evaluation_time + 0.3 * timecorrection));
    ROS_INFO("Maximal Rate %f : \n", traj.GetMaxRate());
    BallCatch::PublishDesiredValues(DesiredThrust,DesiredAxis);
    rviz_publisher.publishTrajectory(traj, traj.GetTf(), coloroption);
    rviz_publisher.publishGoal(posf);
    rviz_publisher.publishBoatPosition(pos0, counter);
    //rviz_publisher.publishSingleObstacle(obsPos, obsRadius);
    rviz_publisher.publishWalls();
    rviz_publisher.publishVelocityText(vel0);
    rviz_publisher.publishDataText(datainfo);
    //Publish rviz Text Velocity
    
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

    return thrust;
};
void BallCatch::SetTimer(){
    timer = ros::WallTime::now();
    SetTheTime = false;
    mt19937 gen(0); 
    
    uniform_real_distribution<> randGoalX(1.0, 2.5);
    uniform_real_distribution<> randGoalY(0.4,1.5);
    uniform_real_distribution<> randGoalZ(0.1,0.6);
    //BallCatch::posf[0] =randGoalX(gen);
    //BallCatch::posf[1] =randGoalY(gen);
    //BallCatch::posf[2] =randGoalZ(gen);
  // BallCatch::posf[0] =2.5;
  // BallCatch::posf[1] =1.5;
  // BallCatch::posf[2] =0.5;
    ROS_INFO("RAndom Goal %f %f %f: \n", posf[0],posf[1],posf[2]);
}
void BallCatch::Reset(){
    IsAtStart = false;
    PerformCatching = false;
    SetTheTime = true;
    counter=0;

    
}
