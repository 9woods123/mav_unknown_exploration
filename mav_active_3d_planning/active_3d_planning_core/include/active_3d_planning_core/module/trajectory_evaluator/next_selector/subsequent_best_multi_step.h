#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_SUBSEQUENT_BEST_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_SUBSEQUENT_BEST_H_

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace next_selector {

// Select the child node which contains the highest value segment in its subtree
class MultiStep : public NextStepsSelector {
 public:
  explicit MultiStep(PlannerI& planner);  // NOLINT

  // override virtual functions
  // int selectNextBest(TrajectorySegment* traj_in) override;
  bool selectNextBestMultiStep(TrajectorySegment*   traj_in,
  std::vector<int> & children_index) override;
  
  void setupFromParamMap(Module::ParamMap* param_map) override;
  std::vector<int> Children_Index;
 protected:
  static ModuleFactoryRegistry::Registration<MultiStep> registration;
 double evaluateSingle(TrajectorySegment* traj_in, std::vector<int> &temp);
  // methods
  // TrajectorySegment* evaluateSingle(TrajectorySegment* traj_in);

  bool findChildrenIndex(TrajectorySegment* traj_in, std::vector<int> &children_index);
};

}  // namespace next_selector
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_SUBSEQUENT_BEST_H_
