#include "active_3d_planning_core/module/trajectory_evaluator/next_selector/subsequent_best_multi_step.h"

#include <algorithm>
#include <vector>

namespace active_3d_planning {
namespace next_selector {

// SubsequentBestMultiStep
ModuleFactoryRegistry::Registration<MultiStep>
    MultiStep::registration("MultiStep");

MultiStep::MultiStep(PlannerI& planner) : NextStepsSelector(planner) {}

void MultiStep::setupFromParamMap(Module::ParamMap* param_map) {}

bool MultiStep::selectNextBestMultiStep(TrajectorySegment*  traj_in, std::vector<int> & children_index)
 {
          std::vector<TrajectorySegment*> tree;
          traj_in->getTree(&tree);
          double max_value = tree[0]->value;
          TrajectorySegment* goal = tree[0];
          if (tree.size()==1)
          {
            return true;
          }

          for (int i = 1; i < tree.size(); ++i) {
            if (tree[i]->value >= max_value) {
              max_value = tree[i]->value;
              goal = tree[i];
            }
          }
           
          while (goal->parent)
                      {
                        std::vector<TrajectorySegment*> childrens_of_parent;
                        goal->parent->getChildren(&childrens_of_parent);
                        for(int i=0;i<childrens_of_parent.size();i++)
                        {
                              if (childrens_of_parent[i]==goal)
                              {
                              // std::cout<<"children_index.push_back(i)===="<<i<<std::endl;
                              children_index.push_back(i);
                              }
                        }
                        goal = goal->parent;
                      }
          std::reverse(children_index.begin(),children_index.end());
          return true;
        }

}  // namespace next_selector
}  // namespace active_3d_planning
