#include "active_3d_planning_mav/module/trajectory_generator/feasible_rrt_star.h"

#include <cmath>
#include <memory>
#include <random>
#include <vector>
#include "mav_trajectory_generation/trajectory_sampling.h"

#include "active_3d_planning_core/data/system_constraints.h"

namespace active_3d_planning {

namespace trajectory_generator {

ModuleFactoryRegistry::Registration<FeasibleRRTStar>
    FeasibleRRTStar::registration("FeasibleRRTStar");

FeasibleRRTStar::FeasibleRRTStar(PlannerI& planner) : RRTStar(planner) {}

void FeasibleRRTStar::setupFromParamMap(Module::ParamMap* param_map) {
  // Setup parent and segment creator
  RRTStar::setupFromParamMap(param_map);

  setParam<bool>(param_map, "all_semgents_feasible", &p_all_semgents_feasible_,
                 false);

  setParam<int>(param_map, "semgents_num", &Semgents_Num_,
                 3);

  setParam<bool>(param_map, "add_waypoints", &add_waypoints_,
                 false);

  segment_generator_.setConstraints(
      planner_.getSystemConstraints().v_max,
      planner_.getSystemConstraints().a_max,
      planner_.getSystemConstraints().yaw_rate_max,
      planner_.getSystemConstraints().yaw_accel_max, p_sampling_rate_);

      map_ = dynamic_cast<map::VoxbloxMap*>(&(planner_.getMap()));
      if (!map_) {
      planner_.printError(
      "'FeasibleRRTStar' requires a map of type 'VoxbloxMap'!");
      }
      esdf_server_=&map_->getESDFserver();
      local_segment_generator_.setDistanceAndGradientFunction(
      std::bind(&FeasibleRRTStar::getMapDistanceAndGradient, this,
                std::placeholders::_1, std::placeholders::_2));     
      std::cout <<"=============Semgents_Num_=============="<<Semgents_Num_<<std::endl;
      local_segment_generator_.setNumSegments(Semgents_Num_);
      local_segment_generator_.setAddWaypoints(add_waypoints_);

      steps_count=1;
      temp_traj.clear();
}

double FeasibleRRTStar::getMapDistanceAndGradient(
    const Eigen::Vector3d& position, Eigen::Vector3d* gradient) {
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_server_->getEsdfMapPtr()->getDistanceAndGradientAtPosition(
          position, kInterpolate, &distance, gradient)) {
    return 0.0;
  }
  return distance;
}

bool FeasibleRRTStar::connectPoses(const EigenTrajectoryPoint& start,
                                   const EigenTrajectoryPoint& goal,
                                   EigenTrajectoryPointVector* result,
                                   bool check_collision) {
  if (check_collision) {
    // try creating a linear trajectory and check for collision
    Eigen::Vector3d direction = goal.position_W - start.position_W;
    int n_points =
        std::ceil(direction.norm() / planner_.getSystemConstraints().v_max *
                  p_sampling_rate_);
    for (int i = 0; i < n_points; ++i) {
      if (!checkTraversable(start.position_W +
                            static_cast<double>(i) /
                                static_cast<double>(n_points) * direction)) {
        return false;
      }
    }
  }
  if (p_all_semgents_feasible_) {
    return segment_generator_.createTrajectory(start, goal, result);
  } else {
    return segment_generator_.simulateTrajectory(start, goal, result);
  }
}

bool FeasibleRRTStar::extractTrajectoryToPublish(
    EigenTrajectoryPointVector* trajectory, const TrajectorySegment& segment) {
        // std::cout<<" FeasibleRRTStar::extractTrajectoryToPublish"<<std::endl;

  if (p_all_semgents_feasible_) {
    // All segments are already feasible
    *trajectory = segment.trajectory;
    return true;
  } else {
    // Create a smooth semgent for execution
    if (segment_generator_.createTrajectory(segment.trajectory.front(),
                                            segment.trajectory.back(),
                                            trajectory)) {
      return true;
    } else {
      *trajectory = segment.trajectory;
      return false;
    }
  }
}

bool FeasibleRRTStar::SetRRTStepLength(int steps_length, const TrajectorySegment& segment, int replan_signal) 
{
steps_count=steps_length;
temp_traj.clear();

int size=segment.trajectory.size();
  if( replan_signal>0)
   {
    temp_traj.push_back( segment.trajectory[size- replan_signal +2]);
   }
replan_signal=-1;
}


bool FeasibleRRTStar::extractTrajectoryToPublish(
    EigenTrajectoryPointVector* trajectory, const TrajectorySegment& segment, bool multi_step, int steps_length) {
        // std::cout<<" FeasibleRRTStar::extractTrajectoryToPublish"<<std::endl;
      if (!multi_step)
      {
            if (p_all_semgents_feasible_) {
              // All segments are already feasible
              *trajectory = segment.trajectory;
              return true;
            } else {
              // Create a smooth semgent for execution
              if (segment_generator_.createTrajectory(segment.trajectory.front(),
                                                      segment.trajectory.back(),
                                                      trajectory)) {
                return true;
              } else {
                *trajectory = segment.trajectory;
                return false;
              }
            }
      }else{
            if (p_all_semgents_feasible_) 
            {               
                            for (auto trajectory_point: segment.trajectory)
                            {
                                   trajectory->push_back(trajectory_point);
                            }

                 return true;
            }
            else{       

                      //  fill the trajectory with the default trajectory which wiil be used when the trajsmooth is failed.
                      for (auto trajectory_point: segment.trajectory)
                            {
                                   trajectory->push_back(trajectory_point);        
                            }

                            temp_traj.push_back( segment.trajectory.front());  //  collect the waypoints from the front of the default traj.
                            if(steps_count<steps_length)
                              {
                                  steps_count++;
                                  return true;
                              }
                              else
                              {
                                 temp_traj.push_back( segment.trajectory.back()); //  collect the waypoints

                                 EigenTrajectoryPointVector traj;
                                mav_msgs::EigenTrajectoryPoint::Vector local_traj;                  // get the Waypoints which can be used to get traj.
                                 for( auto &local_traj_point : temp_traj)
                                {
                                        mav_msgs::EigenTrajectoryPoint r= 
                                        *reinterpret_cast<const mav_msgs::EigenTrajectoryPoint*> (&local_traj_point) ;
                                        local_traj.push_back(r);                          
                                }

                                mav_trajectory_generation::Trajectory local_smooth_traj;                 // trajectory   in mav_trajectory_generation::Trajectory
                                 local_segment_generator_.setNumSegments(steps_length);            // segments_num ==  steps_num   , the traj may be smoother.

                                 if(local_segment_generator_.getTrajectoryBetweenWaypoints(local_traj,&local_smooth_traj))
                                     {       
                                                  //  sample traj into the mav_msgs  which can be published 
                                                  mav_msgs::EigenTrajectoryPoint::Vector tmp_states;
                                                  if (!mav_trajectory_generation::sampleWholeTrajectory(
                                                  local_smooth_traj, 1.0 / 20, &tmp_states)) {
                                                  std::cout<<"SampleWholeTrajectory Failed"<<std::endl;
                                                  return false;
                                                  }

                                                  ///    fill the mav_msgs traj  into the type "EigenTrajectoryPointVector" ;
                                                  for( auto &ts : tmp_states)
                                                  {
                                                  EigenTrajectoryPoint r= *reinterpret_cast<const EigenTrajectoryPoint*> (&ts) ;
                                                  traj.push_back(r);
                                                  }
                                                  trajectory->clear();
                                                  for (auto trajectory_point: traj)
                                                  {
                                                  trajectory->push_back(trajectory_point);
                                                  }
                                                  return true;

                                      }
                                      else
                                      {
                                        return false;
                                      }
                              }

            }
                  //  *trajectory = traj;
      }
}
     


}  // namespace trajectory_generator
}  // namespace active_3d_planning
