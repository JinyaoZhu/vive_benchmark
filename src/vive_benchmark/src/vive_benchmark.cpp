#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <tf/transform_broadcaster.h>

std::string RESULT_PATH = "/home/jinyao/ros_ws/vive_benchmark/";


geometry_msgs::Pose bm_vive_world_frame;
geometry_msgs::Pose bm_vins_world_frame;

ros::Publisher pub_pose_vins;
ros::Publisher pub_pose_tracker;

// tf::Transform tf_1; // cam_world -> cam
// tf::Transform tf_2; // tracker -> cam
// tf::Transform tf_3; // vive_world -> tracker
// tf::Transform tf_4; // vive_world -> cam_world

bool isVOInit = false;
bool isTfInit = false;
bool isVOUpdated = false;

tf::Transform T_1;
tf::Transform T_2;

void vive_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  static int loop_cnt = 0;
  geometry_msgs::Pose pose = msg->pose.pose;
  
  tf::Transform T_3;
  static tf::Transform T_4;

  T_3.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z) );
  T_3.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));

  // initialize, compute translation between two world frame (tf4)
  if((isTfInit == false)&&(isVOInit == true)){
    isTfInit = true;
    T_4 = T_3*(T_2*T_1.inverse());
    ROS_INFO("Translation T4(vive_world ---> cam_world) = ");
    ROS_INFO("qx = %4.3f, qy = %4.3f, qz = %4.3f, qw = %4.3f",
    T_4.getRotation().getX(),T_4.getRotation().getY(),T_4.getRotation().getZ(),T_4.getRotation().getW());
    ROS_INFO("tx = %4.3f, ty = %4.3f, tz = %4.3f",
    T_4.getOrigin().getX(),T_4.getOrigin().getY(),T_4.getOrigin().getZ());

    ROS_INFO("Translation T3(vive_world ---> tracker) = ");
    ROS_INFO("qx = %4.3f, qy = %4.3f, qz = %4.3f, qw = %4.3f",
    T_3.getRotation().getX(),T_3.getRotation().getY(),T_3.getRotation().getZ(),T_3.getRotation().getW());
    ROS_INFO("tx = %4.3f, ty = %4.3f, tz = %4.3f",
    T_3.getOrigin().getX(),T_3.getOrigin().getY(),T_3.getOrigin().getZ());
  }
  else if(isVOInit == true){
    // static tf::TransformBroadcaster tf_br;
    // tf_br.sendTransform(tf::StampedTransform(tf_4.inverseTimes(tf_3.inverse()*tf_2.inverse()), msg->header.stamp, "bm_vive_tracker", "world"));
    // tf_br.sendTransform(tf::StampedTransform(tf_3.inverse(), msg->header.stamp, "bm_vive_tracker", "world"));
    // geometry_msgs::PoseStamped pub_pose;
    // pub_pose.header.stamp = msg->header.stamp;
    // pub_pose.header.frame_id = "world";
    // pub_pose.pose = pose;
    // pub_pose_tracker.publish(pub_pose);
    // isVOUpdated = false;
    if(loop_cnt >= 2){
      loop_cnt=0;
      geometry_msgs::PoseStamped pub_pose;
      tf::Transform T_1_;
      T_1_ = T_4.inverse()*(T_3*T_2);
      pub_pose.header.stamp = msg->header.stamp;
      pub_pose.header.frame_id = "world";
      pub_pose.pose.position.x = T_1_.getOrigin().getX();
      pub_pose.pose.position.y = T_1_.getOrigin().getY();
      pub_pose.pose.position.z = T_1_.getOrigin().getZ();
      pub_pose.pose.orientation.x = T_1_.getRotation().getX(); 
      pub_pose.pose.orientation.y = T_1_.getRotation().getY(); 
      pub_pose.pose.orientation.z = T_1_.getRotation().getZ(); 
      pub_pose.pose.orientation.w = T_1_.getRotation().getW(); 

      std::ofstream result_file(RESULT_PATH+"result_vive.csv", std::ios::app);
      result_file.setf(std::ios::fixed, std::ios::floatfield);
      result_file.precision(0);
      result_file << pub_pose.header.stamp << ",";
      result_file.precision(5);
      result_file  << pub_pose.pose.position.x << ","
            << pub_pose.pose.position.y << ","
            << pub_pose.pose.position.z << ","
            << pub_pose.pose.orientation.w << ","
            << pub_pose.pose.orientation.x << ","
            << pub_pose.pose.orientation.y << ","
            << pub_pose.pose.orientation.z << ","
            << std::endl;
      result_file.close();

      pub_pose_tracker.publish(pub_pose);
    }
    else
      loop_cnt++;
  }
}

void vins_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  static int loop_cnt = 0;
  geometry_msgs::Pose pose = msg->pose.pose;

  T_1.setOrigin( tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  T_1.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));

  
  if(loop_cnt >= 24){
    loop_cnt = 0;
    geometry_msgs::PoseStamped pub_pose;
    pub_pose.header.stamp = msg->header.stamp;
    pub_pose.header.frame_id = "world";
    pub_pose.pose.position.x = T_1.getOrigin().getX();
    pub_pose.pose.position.y = T_1.getOrigin().getY();
    pub_pose.pose.position.z = T_1.getOrigin().getZ();
    pub_pose.pose.orientation.x = T_1.getRotation().getX(); 
    pub_pose.pose.orientation.y = T_1.getRotation().getY(); 
    pub_pose.pose.orientation.z = T_1.getRotation().getZ(); 
    pub_pose.pose.orientation.w = T_1.getRotation().getW(); 

    std::ofstream result_file(RESULT_PATH+"result_vins.csv", std::ios::app);
    result_file.setf(std::ios::fixed, std::ios::floatfield);
    result_file.precision(0);
    result_file << pub_pose.header.stamp << ",";
    result_file.precision(5);
    result_file  << pub_pose.pose.position.x << ","
          << pub_pose.pose.position.y << ","
          << pub_pose.pose.position.z << ","
          << pub_pose.pose.orientation.w << ","
          << pub_pose.pose.orientation.x << ","
          << pub_pose.pose.orientation.y << ","
          << pub_pose.pose.orientation.z << ","
          << std::endl;
    result_file.close();

    pub_pose_vins.publish(pub_pose);
  }
  else
    loop_cnt++;

  isVOInit = true;
  isVOUpdated = true;

  // tf::Transform T_5;
  // T_5 = T_1*T_2.inverse();

  // pub_pose.header.stamp = msg->header.stamp;
  // pub_pose.header.frame_id = "world";
  // pub_pose.pose.position.x = T_5.getOrigin().getX();
  // pub_pose.pose.position.y = T_5.getOrigin().getY();
  // pub_pose.pose.position.z = T_5.getOrigin().getZ();
  // pub_pose.pose.orientation.x = T_5.getRotation().getX(); 
  // pub_pose.pose.orientation.y = T_5.getRotation().getY(); 
  // pub_pose.pose.orientation.z = T_5.getRotation().getZ(); 
  // pub_pose.pose.orientation.w = T_5.getRotation().getW(); 
  // pub_pose_tracker.publish(pub_pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vive_benchmark");

  ros::NodeHandle nh("~");

  std::ofstream fout1(RESULT_PATH+"result_vive.csv", std::ios::out);
  fout1.close();

  std::ofstream fout2(RESULT_PATH+"result_vins.csv", std::ios::out);
  fout2.close();

  //T_2.setOrigin( tf::Vector3(0.05,0.01,0.01) );
  //T_2.setRotation(tf::Quaternion(-0.5,0.5,-0.5,0.5));

  T_2.setOrigin( tf::Vector3(0.052,-0.035,0.015) );
  T_2.setRotation(tf::Quaternion(0,-0.7071,0,0.7071));

  pub_pose_vins = nh.advertise<geometry_msgs::PoseStamped>("vins_pose", 100);
  pub_pose_tracker = nh.advertise<geometry_msgs::PoseStamped>("vive_pose", 100);
  ros::Subscriber sub_pose1 = nh.subscribe("/vive_odom", 1000, vive_odom_callback);
  ros::Subscriber sub_pose2 = nh.subscribe("/vins_odom", 1000, vins_odom_callback);

  ros::spin();

  return 0;
}

