#ifndef ACTIVE_3D_PLANNING_MAV_MODULE_TRAJECTORY_GENERATOR_FEASIBLE_RRT_STAR_H_
#define ACTIVE_3D_PLANNING_MAV_MODULE_TRAJECTORY_GENERATOR_FEASIBLE_RRT_STAR_H_

#include "active_3d_planning_core/module/trajectory_generator/rrt_star.h"
//  the voxblox.h is include in the rrt_star/rrt/trajectory_generator.h
#include "active_3d_planning_mav/tools/loco_smoother.h"
#include "active_3d_planning_mav/tools/linear_mav_trajectory_generator.h"
// #include "active_3d_planning_core/map/tsdf_map.h"
#include "active_3d_planning_voxblox/map/voxblox.h"



namespace active_3d_planning {
namespace trajectory_generator {
    
class FeasibleRRTStar : public RRTStar {
 public:
  explicit FeasibleRRTStar(PlannerI& planner);  // NOLINT

  void setupFromParamMap(Module::ParamMap* param_map) override;

  bool SetRRTStepLength(int steps_length, const TrajectorySegment& segment, int replan_signal) override;

 double getMapDistanceAndGradient(const Eigen::Vector3d& position, Eigen::Vector3d* gradient);

protected:

  static ModuleFactoryRegistry::Registration<FeasibleRRTStar> registration;

  // Segment creator
  LinearMavTrajectoryGenerator segment_generator_;
  mav_planning::LocoSmoother     local_segment_generator_;

  map::VoxbloxMap* map_;
  voxblox::EsdfServer* esdf_server_;
  // params
  bool p_all_semgents_feasible_;
  int  steps_count;
  bool  Flag_Multi_steps_Tra_is_ready;
  bool Traj_is_created;
  int  Semgents_Num_;
  bool add_waypoints_;

  EigenTrajectoryPointVector    temp_traj;

    // Overwrite virtual method to create constrained trajectories
  bool connectPoses(const EigenTrajectoryPoint& start,
                    const EigenTrajectoryPoint& goal,
                    EigenTrajectoryPointVector* result,
                    bool check_collision) override;

  bool extractTrajectoryToPublish(EigenTrajectoryPointVector* trajectory,
                                  const TrajectorySegment& segment) override;


  bool extractTrajectoryToPublish( EigenTrajectoryPointVector* trajectory, 
                                const TrajectorySegment& segment, bool multi_step, int steps_length) override;



};

}  // namespace trajectory_generator
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_MAV_MODULE_TRAJECTORY_GENERATOR_FEASIBLE_RRT_STAR_H_
