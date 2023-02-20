#include "active_3d_planning_core/module/trajectory_generator.h"

#include <string>

#include "active_3d_planning_core/map/map.h"
#include "active_3d_planning_core/module/module_factory.h"

namespace active_3d_planning {

TrajectoryGenerator::TrajectoryGenerator(PlannerI& planner) : Module(planner) {}

void TrajectoryGenerator::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<bool>(param_map, "collision_optimistic", &p_collision_optimistic_,
                 false);
  setParam<double>(param_map, "clearing_radius", &p_clearing_radius_, 0.0);
  std::string ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "segment_selector_args", &p_selector_args_,
                        ns + "/segment_selector");
  setParam<std::string>(param_map, "generator_updater_args", &p_updater_args_,
                        ns + "/generator_updater");
  std::string temp_args;
  setParam<std::string>(param_map, "bounding_volume_args", &temp_args,
                        ns + "/bounding_volume");
  bounding_volume_ = planner_.getFactory().createModule<BoundingVolume>(
      temp_args, planner_, verbose_modules_);
  setParam<std::string>(param_map, "system_constraints_args", &temp_args,
                        ns + "/system_constraints");
}
bool TrajectoryGenerator::SetRRTStepLength(int steps_length, const TrajectorySegment& segment,int replan_signal)
{
std::cout<<" virtual SetRRTStepLength";
return true;
}

// bool TrajectoryGenerator::SetVoxbloxMap(voxblox::EsdfServer& voxbloxESDFserver)
// {
//   std::cout<<" virtual SetVoxbloxMap";
// }

Map& TrajectoryGenerator::getMap()
  {
    // std::cout<<"Get the Map for Gradient calculation"<<std::endl;
    return planner_.getMap();
  }

bool TrajectoryGenerator::checkTraversable(const Eigen::Vector3d& position) {
  // check bounding volume
  if (!bounding_volume_->contains(position)) {
    return false;
  }
  //
  if (planner_.getMap().isObserved(position)) {
    return planner_.getMap().isTraversable(position);
  }
  if (p_clearing_radius_ > 0.0) {
    if ((planner_.getCurrentPosition() - position).norm() <          // distance between currentpose and target pose < p_clearing_radius_ ?
        p_clearing_radius_) {
      return true;
    }
  }
  return p_collision_optimistic_;
}

bool TrajectoryGenerator::selectSegment(TrajectorySegment** result,
                                        TrajectorySegment* root) {
  // If not implemented use a (default) module
  if (!segment_selector_) {
    segment_selector_ = planner_.getFactory().createModule<SegmentSelector>(
        p_selector_args_, planner_, verbose_modules_);
  }
  return segment_selector_->selectSegment(result, root);
}

bool TrajectoryGenerator::updateSegment(TrajectorySegment* segment) {
  // If not implemented use a (default) module
  if (!generator_updater_) {
    generator_updater_ = planner_.getFactory().createModule<GeneratorUpdater>(
        p_updater_args_, planner_, verbose_modules_);
  }
  return generator_updater_->updateSegment(segment);
}

bool TrajectoryGenerator::extractTrajectoryToPublish(
    EigenTrajectoryPointVector* trajectory, const TrajectorySegment& segment) {
  // Default does not manipulate the stored trajectory
  std::cout<<"  Default does not manipulate the stored trajectory"<<std::endl;
  *trajectory = segment.trajectory;
  return true;
}

bool  TrajectoryGenerator:: extractTrajectoryToPublish(
      EigenTrajectoryPointVector* trajectory, const TrajectorySegment& segment, bool multi_step, int steps_length)
      {
  return true;
      }

SegmentSelector::SegmentSelector(PlannerI& planner) : Module(planner) {}

GeneratorUpdater::GeneratorUpdater(PlannerI& planner) : Module(planner) {}
}  // namespace active_3d_planning
