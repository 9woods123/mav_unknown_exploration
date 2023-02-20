#include "active_3d_planning_core/module/trajectory_evaluator/value_computer/global_normalized_gain.h"

#include <algorithm>

namespace active_3d_planning {
namespace value_computer {

// GlobalNormalizedGain
ModuleFactoryRegistry::Registration<GlobalNormalizedGain>
    GlobalNormalizedGain::registration("GlobalNormalizedGain");

GlobalNormalizedGain::GlobalNormalizedGain(PlannerI& planner)
    : ValueComputer(planner) {}

void GlobalNormalizedGain::setupFromParamMap(Module::ParamMap* param_map) {
    setParam<std::string>(param_map, "cost_function", &cost_function_, "x");
}

bool GlobalNormalizedGain::computeValue(TrajectorySegment* traj_in) {
  double gain = 0.0;
  double cost = 0.0;
  TrajectorySegment* current = traj_in->parent;
  while (current) {
    // propagate the new value up to the root
    gain += current->gain;
    cost += current->cost;
    current = current->parent;
  }
  traj_in->value = findBest(traj_in, gain, cost);
  return true;
}

double GlobalNormalizedGain::findBest(TrajectorySegment* current, double gain,
                                      double cost) {
  // recursively iterate towards leaf, then iterate backwards and select best
  // value of children
  double value = 0.0;
  gain += current->gain;
  cost += current->cost;
  if (cost > 0) {
    if (cost_function_=="x")
    {
    value = gain /cost;
    }
    else if (cost_function_=="logx")
    {
    value = gain /std::log2 (cost/1.5+1);
    }
    else if (cost_function_=="x^2")
    {
    value = gain /(cost*cost);
    }
    else if (cost_function_=="e^(-x)")
    {
    value = gain*std::exp(-cost/2);
    }
    else if (cost_function_=="e^(-x)/x")
    {
    value = gain*std::exp(-cost/2)*1/cost;
    }
    else if(cost_function_=="linear")
    {
     value = gain-0.1*cost;
    }
  else if(cost_function_=="log(gain)")
    {
     value = std::log2 (gain)/cost;
    }
    // else if(cost_function_=="linear")
    // {
    //  value = gain-0.5*cost;
    // }
  }
  for (int i = 0; i < current->children.size(); ++i) {
    value = std::max(value, findBest(current->children[i].get(), gain, cost));
  }
  return value;
}

}  // namespace value_computer
}  // namespace active_3d_planning
