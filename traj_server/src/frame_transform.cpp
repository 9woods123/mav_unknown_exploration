
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

ros::Publisher      odom_pub_ ;
nav_msgs::Odometry  odom_;

void odom_callback(const nav_msgs::OdometryConstPtr & msg)
{
    odom_=*msg;
    odom_.header.frame_id="world";


    // 定义NED坐标系中的位姿             coded by chatgpt
    Eigen::Vector3d ned_position(odom_.pose.pose.position.x, odom_.pose.pose.position.y,
                                 odom_.pose.pose.position.z);
    Eigen::Quaterniond ned_orientation(odom_.pose.pose.orientation.w
            ,odom_.pose.pose.orientation.x
            ,odom_.pose.pose.orientation.y
            ,odom_.pose.pose.orientation.z); 
    Eigen::Matrix3d R_ned = ned_orientation.toRotationMatrix();

    // 将ENU坐标系的位姿转换为右手坐标系下的位姿
    Eigen::Matrix3d ned_to_nwu;
    ned_to_nwu <<1,    0,   0.0,
                 0,    -1,  0.0,
                 0.0, 0.0, -1.0;


    Eigen::Vector3d position = ned_to_nwu * ned_position;
    
    Eigen::Matrix3d R_nwu = ned_to_nwu * R_ned * ned_to_nwu.transpose();
    Eigen::Quaterniond quat = Eigen::Quaterniond(R_nwu);


    odom_.pose.pose.position.x=position.x()+2;     // const distance =2
    odom_.pose.pose.position.y=position.y();
    odom_.pose.pose.position.z=position.z();
    odom_.pose.pose.orientation.x=quat.x();
    odom_.pose.pose.orientation.y=quat.y();
    odom_.pose.pose.orientation.z=quat.z();
    odom_.pose.pose.orientation.w=quat.w();


    odom_pub_.publish(odom_);
}

// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "frame_transform");
  ros::NodeHandle n;   ////n 命名空间为/node_namespace    launch 文件中 ns=="node_namespace"
  ros::NodeHandle nh("~");  //命名空间为/node_namespace/node_name    firefly/traj_server/

  ros::Subscriber odom_sub_ = n.subscribe("/airsim_node/drone_1/odom_local_ned", 10 , &odom_callback);
  odom_pub_ = n.advertise<nav_msgs::Odometry>("/airsim_node/drone_1/odom_world", 10);

  ros::spin();

}






//    //set the transform to turn the ned pose to world pose.
//    Eigen::Isometry3d T_ned2world=Eigen::Isometry3d::Identity();
//    Eigen::AngleAxisd rotation_vector(3.14,Eigen::Vector3d(1,0,0));
//    Eigen::Matrix3d rotation_matrix=rotation_vector.matrix();
//    T_ned2world.rotate ( rotation_matrix );
//    T_ned2world.pretranslate ( Eigen::Vector3d ( 0,0,0 ) );
//
//    //  init the transform matrix from odom_
//    Eigen::Isometry3d T_odom2ned=Eigen::Isometry3d::Identity();
//    Eigen::Quaterniond q_odom2ned(odom_.pose.pose.orientation.w
//            ,odom_.pose.pose.orientation.x
//            ,odom_.pose.pose.orientation.y
//            ,odom_.pose.pose.orientation.z);
//    T_odom2ned.rotate ( q_odom2ned.toRotationMatrix() );
//    T_odom2ned.pretranslate ( Eigen::Vector3d ( odom_.pose.pose.position.x,
//                                                odom_.pose.pose.position.y,
//                                                odom_.pose.pose.position.z ) );
//
//
//    Eigen::Isometry3d T_odom2world=T_ned2world*T_odom2ned;
//
//
//    Eigen::Vector3d position=T_odom2world.translation();
//    Eigen::Matrix3d  rotation=T_odom2world.rotation();
//    Eigen::Quaterniond quat(rotation);
