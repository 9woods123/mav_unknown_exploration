
#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
#include <math.h>


// =========== Function declarations =============

#define SQ(x) ((x)*(x))

trajectory_msgs::MultiDOFJointTrajectory temp_traj;
trajectory_msgs::MultiDOFJointTrajectoryPoint target_point;
std_msgs::Int32 replan_singal;
ros::Publisher    cmd_pub_ ;
ros::Publisher replan_singal_pub_;
msr::airlib::MultirotorRpcLibClient client;


void init_airsim_control()
{
    client.enableApiControl(true);
    std::cout << "Press Enter to arm the drone\n"; std::cin.get();
    client.armDisarm(true);
    std::cout << "Press Enter to takeoff\n"; std::cin.get();
    client.takeoffAsync(5)->waitOnLastTask();


    std::cout << "Press Enter to move 5 meters in x direction with 1 m/s velocity\n"; std::cin.get();
    auto position = client.getMultirotorState().getPosition(); // from current location
    client.moveToPositionAsync(position.x() + 1, position.y(), position.z(), 1)->waitOnLastTask();

    position = client.getMultirotorState().getPosition(); // from current location
    auto drivetrain_ = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    msr::airlib::YawMode yaw_mode_ =msr::airlib::YawMode(0, -180*1.57/M_PI);

    client.moveToPositionAsync(position.x(), position.y()-1, position.z(),
                               1, 3e38, drivetrain_,yaw_mode_
                               )->waitOnLastTask();

    std::cout<<client.getMultirotorState().getPosition()<<std::endl;

    std::cout << "OK \n"; std::cin.get();

    //std::cout << "Press Enter to land\n"; std::cin.get();
    //client.landAsync()->waitOnLastTask();
}

void nwu2ned(   Eigen::Vector3d nwu_position ,Eigen::Quaterniond nwu_quat,
                Eigen::Vector3d &ned_position, Eigen::Quaterniond &ned_quat)
{

    Eigen::Matrix3d R_wu = nwu_quat.toRotationMatrix();

    Eigen::Matrix3d nwu_to_ned;
    nwu_to_ned <<1,    0,   0.0,
            0,    -1,  0.0,
            0.0, 0.0, -1.0;

    Eigen::Matrix3d R_ed = nwu_to_ned * R_wu * nwu_to_ned.transpose();


    ned_position = nwu_to_ned * nwu_position;
    ned_quat = Eigen::Quaterniond(R_ed);

}



void traj_server_callback(const trajectory_msgs::MultiDOFJointTrajectory& msg)
{
    target_point=msg.points[0];

    float velocity=sqrt(SQ(target_point.velocities.begin()->linear.x)+
    SQ(target_point.velocities.begin()->linear.y)+
    SQ(target_point.velocities.begin()->linear.z));

    Eigen::Vector3d    point_position(target_point.transforms.begin()->translation.x,
                                      target_point.transforms.begin()->translation.y,
                                      target_point.transforms.begin()->translation.z);
    Eigen::Quaterniond point_quat(target_point.transforms.begin()->rotation.w,
                                  target_point.transforms.begin()->rotation.x,
                                  target_point.transforms.begin()->rotation.y,
                                  target_point.transforms.begin()->rotation.z);

    Eigen::Vector3d  ned_target_position;
    Eigen::Quaterniond ned_target_quat;
    nwu2ned(point_position,point_quat,ned_target_position,ned_target_quat);


    // Compute RPY angles from quaternion
    Eigen::Vector3d ned_target_rpy = ned_target_quat.toRotationMatrix().eulerAngles(0, 1, 2);
    // Extract roll, pitch, and yaw angles in degrees
    //    roll = rpy(0) * 180.0 / M_PI;
    //    pitch = rpy(1) * 180.0 / M_PI;

    float point_yaw = ned_target_rpy(2) * 180.0 / M_PI;
    auto drivetrain_ = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    msr::airlib::YawMode yaw_mode_ =msr::airlib::YawMode(0, point_yaw);

    client.moveToPositionAsync(ned_target_position.x(),
                               ned_target_position.y(),
                               ned_target_position.z(),
                               velocity,0.1, drivetrain_,yaw_mode_);



}
// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "cmd2airsim");
  ros::NodeHandle n;   ////n 命名空间为/node_namespace    launch 文件中 ns=="node_namespace"
  ros::NodeHandle nh("~");  //命名空间为/node_namespace/node_name    firefly/traj_server/

  // cmd_pub_ = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);
  ros::Subscriber cmd_sub_ = n.subscribe("command/trajectory", 10 , &traj_server_callback);
  ros::Publisher  current_target_pub_=nh.advertise<visualization_msgs::Marker>("trajectory_markers", 1);



    std::cout << "Press Enter to init_airsim_control\n"; std::cin.get();
  init_airsim_control();

  ros::spin();
}
