#ifndef TRAJECTORYCREATOR_HPP
#define TRAJECTORYCREATOR_HPP
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include "RapidQuadcopterTrajectories/RapidTrajectoryGenerator.hpp"
#include "RapidCollisionDetection/CollisionChecker.hpp"
#include "CommonMath/Vec3.hpp"
#include <list> 
#include <algorithm>
#include <chrono>
#include "CommonMath/ConvexObj.hpp"
#include "CommonMath/Sphere.hpp"
using namespace RapidQuadrocopterTrajectoryGenerator;
using namespace std;
using namespace RapidCollisionChecker;

class TrajectoryCreator
{
public:
    TrajectoryCreator();
    RapidTrajectoryGenerator GenerateTrajectories(const CommonMath::Vec3 position, const CommonMath::Vec3 velocity, const CommonMath::Vec3 acceleration, const int iterations, const double timeduration);
    void SetGoalPosition(const CommonMath::Vec3 in);
    void SetGoalVelocity(const CommonMath::Vec3 in);
    void SetGoalAcceleration(const CommonMath::Vec3 in);
    void DeleteTrajectoryList() { TrajectoryCreator::trajectory_list.clear(); };
    
private:
    CommonMath::Vec3 goal_pos;
    CommonMath::Vec3 goal_vel;
    CommonMath::Vec3 goal_accel;
    std::vector<RapidTrajectoryGenerator> trajectory_list;
    
    int ChooseBestTrajectory();
};

#endif // TRAJECTORYCREATOR_HPP
