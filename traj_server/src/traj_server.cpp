
#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>


// =========== Function declarations =============
void cmd_pub_callback(const ros::TimerEvent& e);
void traj_server_callback(const trajectory_msgs::MultiDOFJointTrajectory& msg);
void publishCompletedTrajectoryVisualization();

trajectory_msgs::MultiDOFJointTrajectory temp_traj;
trajectory_msgs::MultiDOFJointTrajectory target_point;
std_msgs::Int32 replan_singal;
ros::Publisher    cmd_pub_ ;
ros::Publisher replan_singal_pub_;
ros::Publisher pub_markers_;
ros::Publisher current_target_pub_;

visualization_msgs::Marker traj_point;
visualization_msgs::Marker current_target_point;;
int replan_signal_time_interval=50;
int traj_count ; 
bool traj_ready;
static int n_seq = 0;
int traj_size;
// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle n;   ////n 命名空间为/node_namespace    launch 文件中 ns=="node_namespace"
  ros::NodeHandle nh("~");  //命名空间为/node_namespace/node_name    firefly/traj_server/
  // 创建ros 定时器
  std::cout<<"=============traj_server created============="<<std::endl;

  cmd_pub_ = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);
  replan_singal_pub_ = n.advertise<std_msgs::Int32>("replan_singal", 10);
  replan_singal.data=-1;
  traj_ready=false;
  traj_count=0;

  traj_point.type =   visualization_msgs::Marker::CUBE;
  traj_point.type =   visualization_msgs::Marker::CUBE;

  traj_point.header.frame_id="world";
  traj_point.ns = "completed_trajectory";
  // traj_point.id = vis_completed_count_;
  traj_point.action =   visualization_msgs::Marker::ADD;

  pub_markers_ =nh.advertise<visualization_msgs::Marker>("trajectory_markers", 1);
  current_target_pub_=nh.advertise<visualization_msgs::Marker>("trajectory_markers", 1);

  ros::Subscriber traj_sub_ = nh.subscribe("trajectory", 10 , &traj_server_callback);
  ros::Timer cmd_pub_timer = nh.createTimer(ros::Duration(0.05), cmd_pub_callback);

  ros::spin();
}


// void publishCompletedTrajectoryVisualization(
//     const trajectory_msgs::MultiDOFJointTrajectory& Traj_Point) {
//   // Continuously increment the already traveled path
//   visualization_msgs::Marker traj_point;

//   traj_point.type =   visualization_msgs::Marker::POINTS;
//   traj_point.ns = "completed_trajectory";
//   // traj_point.id = vis_completed_count_;
//   traj_point.action =   visualization_msgs::Marker::ADD;

//   traj_point.pose.position.x =Traj_Point.point ;
//   traj_point.pose.position.y = node->state_[1];
//   traj_point.pose.position.z = node->state_[2];
//   traj_point.color.r = 1.0;
//   traj_point.color.g = 0.0;
//   traj_point.color.b = 0.0;
//   traj_point.color.a = 0.5;

//   // points
//   for (int i = 0; i < trajectories.trajectory.size(); ++i) {
//     msg.points.push_back(trajectories.trajectory[i].position_W);
//   }
//   VisualizationMarkers array_msg;
//   array_msg.addMarker(msg);
//   publishVisualization(array_msg);
// }



 void traj_server_callback(const trajectory_msgs::MultiDOFJointTrajectory& msg) 
 {
          // ROS_INFO_STREAM ( "       traj is received      "  );
          temp_traj=msg;
          traj_count=0;
          traj_ready=true;
          target_point.header=temp_traj.header;
          traj_size=temp_traj.points.size();

}
 

// Planing loop
void cmd_pub_callback(const ros::TimerEvent& e)
{         
          if(traj_ready==true)
          {       
                  if(traj_count !=traj_size)
                  {
                      target_point.points.clear();
                      target_point.points.push_back(temp_traj.points[traj_count]);
                      cmd_pub_.publish(target_point);
                      
                      if (traj_size>replan_signal_time_interval && traj_size - traj_count == replan_signal_time_interval)         //50 * 0.05  = 2.5 second?
                          {
                              // std::cout <<" replan before end "<<temp_traj.points.back().time_from_start.toSec()  - 
                              // temp_traj.points[traj_count].time_from_start.toSec()<<std::endl;

                              replan_singal.data=replan_signal_time_interval;
                              replan_singal_pub_.publish(replan_singal);
                          } else if(10<traj_size<=replan_signal_time_interval && traj_size - traj_count == 1)
                          {
                              replan_singal.data=10;
                              replan_singal_pub_.publish(replan_singal);
                          }
                          


                                traj_point.header.stamp = ros::Time::now();
                                traj_point.header.seq++;
                                // traj_point.id=traj_point.header.seq;
                                // traj_point.pose.position.x =target_point.points[0].transforms[0].translation.x;
                                // traj_point.pose.position.y =target_point.points[0].transforms[0].translation.y;
                                // traj_point.pose.position.z = target_point.points[0].transforms[0].translation.z;
                                // traj_point.scale.x = 0.1;
                                // traj_point.scale.y = 0.1;
                                // traj_point.scale.z = 0.1;
                                // traj_point.color.r = 0.0;
                                // traj_point.color.g = 0.0;
                                // traj_point.color.b = 1.0;
                                // traj_point.color.a = 0.5;
                                // traj_point.action =   visualization_msgs::Marker::ADD;
                                // traj_point.lifetime=ros::Duration(0);
                                // pub_markers_.publish(traj_point);



                      traj_count++;
                  }
                  else{
                    traj_ready=false;
                  }
          }

}


                  // if(replan_singal==false)
                  // {
                  //         if (temp_traj.points.back().time_from_start.toSec()  -
                  //         temp_traj.points[traj_count].time_from_start.toSec()
                  //         <=0.05*20)
                  //         {
                  //             replan_singal=true;
                  //             replan_singal_pub_.publish(replan_singal);
                  //         }
                  // }
