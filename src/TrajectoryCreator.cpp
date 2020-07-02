#include "TrajectoryCreator.hpp"
using namespace std;
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
    
    Vec3 floorPos = Vec3(0,0,0);//any point on the boundary
    Vec3 floorNormal = Vec3(0,0,1);
    RapidTrajectoryGenerator traj(position, velocity, acceleration);
    
    for(int i = 0; i < iterations; i++)
    {   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    
       // RapidTrajectoryGenerator traj(position, velocity, acceleration);
        traj.SetGoalPosition(TrajectoryCreator::goal_pos);
        traj.SetGoalVelocity(TrajectoryCreator::goal_vel);
        traj.SetGoalAcceleration(TrajectoryCreator::goal_accel);
        traj.Generate(timeduration + i * 0.2);
        inputfeasibility =    traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec);
        //positionfeasibility = traj.CheckPositionFeasibility(floorPos, floorNormal);
       // ROS_INFO("Feasibility %i:", feasibility);
       
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
        if(inputfeasibility==0){
            TrajectoryCreator::trajectory_list.push_back(traj);
        }
        
    traj.Reset();
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
        //printf("TimeOfTraject %f:", trajectory_list[k].GetDuration());
        duration_list.push_back( trajectory_list[k].GetDuration());
    }
    return std::min_element(duration_list.begin(),duration_list.end()) - duration_list.begin();
}

void TrajectoryCreator::SetGoalPosition(const Vec3 in)
{   
     TrajectoryCreator::goal_pos = in;
}
void TrajectoryCreator::SetGoalVelocity(const Vec3 in)
{   
     TrajectoryCreator::goal_vel = in;
}
void TrajectoryCreator::SetGoalAcceleration(const Vec3 in)
{   
     TrajectoryCreator::goal_accel = in;  
}