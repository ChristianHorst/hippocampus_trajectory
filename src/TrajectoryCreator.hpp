#ifndef TRAJECTORYCREATOR_HPP
#define TRAJECTORYCREATOR_HPP
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include "RapidTrajectoryGenerator.h"
#include "Vec3.h"
#include <list> 
#include <algorithm>
#include <chrono>
using namespace RapidQuadrocopterTrajectoryGenerator;
using namespace std;
class TrajectoryCreator
{
public:
    TrajectoryCreator();
    RapidTrajectoryGenerator GenerateTrajectories(const Vec3 position, const Vec3 velocity, const Vec3 acceleration, const int iterations, const double timeduration);
    void SetGoalPosition(const Vec3 in);
    void SetGoalVelocity(const Vec3 in);
    void SetGoalAcceleration(const Vec3 in);
    void DeleteTrajectoryList() { TrajectoryCreator::trajectory_list.clear(); };
    
private:
    Vec3 goal_pos;
    Vec3 goal_vel;
    Vec3 goal_accel;
    std::vector<RapidTrajectoryGenerator> trajectory_list;
    
    int ChooseBestTrajectory();
};

#endif // TRAJECTORYCREATOR_HPP
