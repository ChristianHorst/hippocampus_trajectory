#include "hippocampus_test/TrajectoryCreator.hpp"
#include <random>
using namespace std;
using namespace CommonMath;
TrajectoryCreator::TrajectoryCreator()
{
    //Stuff to do when initialized
    
}
RapidTrajectoryGenerator TrajectoryCreator::GenerateTrajectories(const Vec3 position, const Vec3 velocity, const Vec3 acceleration, const int iterations, const double timeduration){
    double fmin = 0.0;//[m/s**2]
    double fmax = 10.0;//[m/s**2]
    double wmax = 20;//[rad/s]
    double minTimeSec = 0.02;//[s]
    
    int inputfeasibility=0;
    int positionfeasibility =0;
    
    CommonMath::Vec3 floorPos = CommonMath::Vec3(0,0,0);//any point on the boundary
    CommonMath::Vec3 floorNormal = CommonMath::Vec3(0,0,1);
    
    mt19937 gen(0); 
    
    uniform_real_distribution<> randPosX(goal_pos[0] - 0.1, goal_pos[0] + 0.1);
    uniform_real_distribution<> randPosY(goal_pos[1] - 0.1, goal_pos[1] + 0.1);
    uniform_real_distribution<> randPosZ(goal_pos[2] - 0.1, goal_pos[2] + 0.1);
    
    uniform_real_distribution<> randVelX(goal_vel[0] - 0.05, goal_vel[0] + 0.05);
    uniform_real_distribution<> randVelY(goal_vel[1] - 0.0, goal_vel[1] + 0.05);
    uniform_real_distribution<> randVelZ(goal_vel[2] - 0.0, goal_vel[2] + 0.05);
    
    uniform_real_distribution<> randAccX(goal_accel[0] - 0.0, goal_accel[0] + 0.05);
    uniform_real_distribution<> randAccY(goal_accel[1] - 0.0, goal_accel[1] + 0.05);
    uniform_real_distribution<> randAccZ(goal_accel[2] - 0.0, goal_accel[2] + 0.05);
    //Time Sampling+
    double upperTimeBound = timeduration + 1.0;
    double lowerTimeBound = timeduration - 0.5;
    if(lowerTimeBound <= 0.3) lowerTimeBound = 0.3;
    uniform_real_distribution<> totaltime(lowerTimeBound , upperTimeBound);


    
    for(int i = 0; i < iterations; i++)
    {   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    
        CommonMath::Vec3 posf = Vec3(randPosX(gen), randPosY(gen) , randPosZ(gen)); //position
        CommonMath::Vec3 velf = Vec3(randVelX(gen), randVelY(gen), randVelZ(gen)); //velocity
        //CommonMath::Vec3 accf = Vec3(randAccX(gen), randAccY(gen), randAccZ(gen)); //acceleration
        CommonMath::Vec3 accf = Vec3(0.0, 0.0,0.0);
        RapidTrajectoryGenerator traj(position, velocity, acceleration);
        traj.SetGoalPosition(posf);
        traj.SetGoalVelocity(velf);
        traj.SetGoalAcceleration(accf);
        //traj.SetGoalPosition(TrajectoryCreator::goal_pos);
        //traj.SetGoalVelocity(TrajectoryCreator::goal_vel);
        //traj.SetGoalAcceleration(TrajectoryCreator::goal_accel);
        //traj.Generate(timeduration + i * 0.02);
        traj.Generate(totaltime(gen));
        inputfeasibility =    traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec);
        //positionfeasibility = traj.CheckPositionFeasibility(floorPos, floorNormal);
        
        
        //ROS_INFO("RAndom X %f:", posf[0]);
    
       
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
       // std::cout << "Time difference Traj Gen= " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
       // std::cout << "Time difference Traj Gen= " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
        if(inputfeasibility==0){
            TrajectoryCreator::trajectory_list.push_back(traj);
        }
        
   
    }
    //print length of trajectory list
    ROS_INFO("Length of Traj List : %ld", trajectory_list.size() ); 
    int index =  TrajectoryCreator::ChooseBestTrajectory();   
    ROS_INFO("BEST TRAJ %i:", index); 
    return trajectory_list[index];
}
int TrajectoryCreator::ChooseBestTrajectory(){
    std::vector<double> cost_list;
    std::vector<double> duration_list;
    for(int k = 0; k < TrajectoryCreator::trajectory_list.size(); k++) {
        //cost_list.push_back( trajectory_list[k].GetCost());
        duration_list.push_back( trajectory_list[k].GetCost());
        //printf("TimeOfTraject %f:", trajectory_list[k].GetDuration());
        //duration_list.push_back( trajectory_list[k].GetDuration());
    }
    return std::min_element(duration_list.begin(),duration_list.end()) - duration_list.begin();
}

void TrajectoryCreator::SetGoalPosition(const CommonMath::Vec3 in)
{   
     TrajectoryCreator::goal_pos = in;
}
void TrajectoryCreator::SetGoalVelocity(const CommonMath::Vec3 in)
{   
     TrajectoryCreator::goal_vel = in;
}
void TrajectoryCreator::SetGoalAcceleration(const CommonMath:: Vec3 in)
{   
     TrajectoryCreator::goal_accel = in;  
}
