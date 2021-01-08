#include "hippocampus_test/TrajectoryCreator.hpp"
#include <random>
#include <fstream>
#include <iostream>
using std::ofstream;
using namespace std;
using namespace CommonMath;
TrajectoryCreator::TrajectoryCreator()
{
    //Stuff to do when initialized
    
}
RapidTrajectoryGenerator TrajectoryCreator::GenerateTrajectories(const Vec3 position, const Vec3 velocity, const Vec3 acceleration, const int iterations, const double timeduration){
    double fmin = 0.0;//[m/s**2]
    double fmax = 6.0;//[m/s**2]
    double wmax = 6;//[rad/s]
    double minTimeSec = 0.02;//[s]
    
    int inputfeasibility = 0;

    int positionfeasibility =0;
    
    CommonMath::Vec3 floorPos = CommonMath::Vec3(0,0,0);//any point on the boundary
    CommonMath::Vec3 floorNormal = CommonMath::Vec3(0,0,1);
    
    mt19937 gen(0); 
    
    uniform_real_distribution<> randPosX(goal_pos[0] - 0.05, goal_pos[0] + 0.05);
    uniform_real_distribution<> randPosY(goal_pos[1] - 0.05, goal_pos[1] + 0.05);
    uniform_real_distribution<> randPosZ(goal_pos[2] - 0.01, goal_pos[2] + 0.01);
    
    uniform_real_distribution<> randVelX(goal_vel[0] - 0.05, goal_vel[0] + 0.05);
    uniform_real_distribution<> randVelY(goal_vel[1] - 0.05, goal_vel[1] + 0.05);
    uniform_real_distribution<> randVelZ(goal_vel[2] - 0.05, goal_vel[2] + 0.05);
    
    uniform_real_distribution<> randAccX(goal_accel[0] - 0.0, goal_accel[0] + 0.05);
    uniform_real_distribution<> randAccY(goal_accel[1] - 0.0, goal_accel[1] + 0.05);
    uniform_real_distribution<> randAccZ(goal_accel[2] - 0.0, goal_accel[2] + 0.05);
    //Time Sampling+
    double upperTimeBound = timeduration + 1.0;
    double lowerTimeBound = timeduration - 0.5;
    if(lowerTimeBound <= 0.3) lowerTimeBound = 0.3;
    uniform_real_distribution<> totaltime(lowerTimeBound , upperTimeBound);
    ofstream outdata; 
    outdata.open("comptime_inputfeas.txt",std::ios_base::app);
 

    
    for(int i = 0; i < iterations; i++)
    {   
    
        CommonMath::Vec3 posf =   Vec3(randPosX(gen), randPosY(gen) , randPosZ(gen)); //position
        CommonMath::Vec3 velf =Vec3(randVelX(gen), randVelY(gen), randVelZ(gen)); //velocity
        //CommonMath::Vec3 accf = Vec3(randAccX(gen), randAccY(gen), randAccZ(gen)); //acceleration
        CommonMath::Vec3 accf = Vec3(0.0, 0.0,0.0);
        RapidTrajectoryGenerator traj(position, velocity, acceleration);
        traj.SetGoalPosition(posf);
        traj.SetGoalVelocity(velf);
        traj.SetGoalAcceleration(accf);
        traj.Generate(totaltime(gen));
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        inputfeasibility =    traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        outdata <<std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count()<< "," <<std::endl;
        //traj.Generate(timeduration + i * 0.02);
      //  
      
        
       // positionfeasibility = traj.CheckPositionFeasibility(Vec3(0.0, 0.0,0.0),Vec3(0.0, 0.0,1.0));
        
        //std::cout << "Time difference Traj Gen= " << std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count() << "[us]" << std::endl;
        
        //positionfeasibility = traj.CheckPositionFeasibility(floorPos, floorNormal);
        
        
        //ROS_INFO("RAndom X %f:", posf[0]);
    

        //ROS_INFO("In generation");
        if(inputfeasibility==0){
       // ROS_INFO("In feasible test");
        //OBSTACLE-----------------
  /*      Vec3 obsPos(1.5, 1.0, 0.0);
        Vec3 obsPos2(2.0, 1.5, 0.0);
        Vec3 obsPos3(1.8, 1.25, 0.5);
        Vec3 obsPos4(1.5, 0.15, 0.0);
        double obsRadius = 0.15;
        
        shared_ptr<ConvexObj> obstacle = make_shared < Sphere> (obsPos, obsRadius);
        shared_ptr<ConvexObj> obstacle2 = make_shared < Sphere> (obsPos2, obsRadius);
        shared_ptr<ConvexObj> obstacle3 = make_shared < Sphere> (obsPos3, obsRadius);
        shared_ptr<ConvexObj> obstacle4 = make_shared < Sphere> (obsPos4, obsRadius);
        CollisionChecker checker(traj.GetTrajectory());
       // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        CollisionChecker::CollisionResult stateFeas = checker.CollisionCheck(obstacle, 0.02);
        // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        //outdata <<std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count()<< "," <<std::endl;
        CollisionChecker::CollisionResult stateFeas2 = checker.CollisionCheck(obstacle2, 0.02);
        CollisionChecker::CollisionResult stateFeas3 = checker.CollisionCheck(obstacle3, 0.02);
        CollisionChecker::CollisionResult stateFeas4 = checker.CollisionCheck(obstacle4, 0.02);
    //    ROS_INFO("Collision ?\t %d", stateFeas);
        if(stateFeas == 0 || stateFeas==2){
            if(stateFeas2 == 0 || stateFeas2==2){
                if(stateFeas3 == 0 || stateFeas3==2){
                    if(stateFeas4 == 0 || stateFeas4==2){
           // ROS_INFO("In obstacle test");*/
                        TrajectoryCreator::trajectory_list.push_back(traj);
                 //       }
                  //  }
              //  }
           // }
        }
        //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
       // std::cout << "Time difference Traj Gen= " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
        //std::cout << "Time difference Traj Gen= " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
    }
    //print length of trajectory list
    //ROS_INFO("Length of Traj List : %ld", trajectory_list.size() ); 
    int index =  TrajectoryCreator::ChooseBestTrajectory();   
   // ROS_INFO("BEST TRAJ %i:", index); 
    outdata.close();
    return trajectory_list[index];
}
int TrajectoryCreator::ChooseBestTrajectory(){
    std::vector<double> cost_list;
    std::vector<double> duration_list;
    for(int k = 0; k < TrajectoryCreator::trajectory_list.size(); k++) {
        //cost_list.push_back( trajectory_list[k].GetCost());
        //duration_list.push_back( trajectory_list[k].GetCost());
        //printf("TimeOfTraject %f:", trajectory_list[k].GetDuration());
        duration_list.push_back( trajectory_list[k].GetDuration());
    }
    return std::min_element(duration_list.begin(),duration_list.end()) - duration_list.begin();
}

void TrajectoryCreator::SetNewPos(const CommonMath::Vec3 in)
{   
     TrajectoryCreator::goal_pos = in;
}
void TrajectoryCreator::SetNewVel(const CommonMath::Vec3 in)
{   
     TrajectoryCreator::goal_vel = in;
}
void TrajectoryCreator::SetNewAccel(const CommonMath:: Vec3 in)
{   
     TrajectoryCreator::goal_accel = in;  
}
