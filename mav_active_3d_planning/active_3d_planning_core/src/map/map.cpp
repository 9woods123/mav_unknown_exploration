#include "active_3d_planning_core/map/map.h"

namespace active_3d_planning {

Map::Map(PlannerI& planner) : Module(planner) {}

bool Map::isTraversablePath(const EigenTrajectoryPointVector& trajectory) {
  // default just checks every point
  for (const EigenTrajectoryPoint& point : trajectory) {
    if (!isTraversable(point.position_W, point.orientation_W_B)) {
      return false;
    }
  }
  return true;
}

bool Map::getDistanceAndGradientAtPosition(const Eigen::Vector3d& position,
                                        double* distance,Eigen::Vector3d* gradient)
 {
   std::cout <<"Voxblox is required for Gradient calculation"<<std::endl;
   return false;
}
 double Map::getVoxelSize()
 {
   std::cout <<"Voxblox is required for getVoxelSize()"<<std::endl;
 }
int  Map::getMapSize() 
{
   std::cout <<"Voxblox is required for getMapSize()"<<std::endl;

}


}  // namespace active_3d_planning
