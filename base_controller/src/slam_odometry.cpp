#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "ros/ros.h"
#include <vector> // for vector 

#define RAD_SEC_TO_RPM 30.0/M_PI
#define NUM_STATES 6

nav_msgs::Odometry slam_odometry_, slam_odometry_0_;
std::string odom_topic_name_,
            odom_frame_id_,
            robot_frame_id_,
            slam_pose_topic_name_
            ;

bool publish_tf_=false;

tf::Stamped< tf::Pose > tf_odom_2_robot_; //Transformation from world_ned to base_link

ros::Time last_time_stamp_;

double publish_rate_=50.0, 
       last_yaw_ =0
       ;

std::vector<double> covariance_pose_vec_(6, 1.0),
                    covariance_twist_vec_(6, 1.0);

// ros::Duration tfCacheDuration;
tf::TransformListener* listener_;

double cutoff_freq_linear_vel_=10,cutoff_freq_angular_vel_ = 10
  , damping_linear_vel_ = 1.0, damping_angular_vel_=1.0,
  cut_off_value_linear_=0.5, cut_off_value_angular_=2.0;

/**
* Applies a simple second order filter to two measured samples of the same data
* 
* @param measure - a measure comming from sensor
* @param filtered_0 - the last filtered sample (instant k-1)
* @param filtered_00 - the last filtered sample (instant k-2)
* @param omega_f - cutoff frequency
* @param xi - damping coefficient
* @param cut_off_value - acceptable range for measure
* @param dt0 - time interval
*
* @return the new filtered sample
*/
double applyFilter(double measure, double filtered_0, double filtered_00, double omega_f, double xi, double cut_off_value, double dt0){
    measure = (measure > cut_off_value) ? cut_off_value : measure;
    measure = (measure < -cut_off_value) ? -cut_off_value : measure;

    return  pow(dt0,2)*pow(omega_f,2)*measure 
      +(2-2*dt0*omega_f*xi)*filtered_0
      +(2*dt0*omega_f*xi-1-pow(dt0,2)*pow(omega_f,2))*filtered_00;
}

void slamPoseCallbackPose(const geometry_msgs::PoseWithCovarianceStamped& slam_pose){

  ros::Time actual_time_stamp = ros::Time::now();

  double elapsed_time = (actual_time_stamp - last_time_stamp_).toSec();

  tf::StampedTransform transform;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(slam_odometry_.pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  double delta_x = (slam_pose.pose.pose.position.x-slam_odometry_.pose.pose.position.x)*cos(yaw)
                  -(slam_pose.pose.pose.position.y-slam_odometry_.pose.pose.position.y)*sin(yaw);
  double linear_vel_robot = delta_x/elapsed_time, //angular velocity of robot
        angular_vel_robot = -(last_yaw_-yaw)/elapsed_time; //angular velocity of robot
  last_yaw_ = yaw;

  nav_msgs::Odometry slam_odometry_00 = slam_odometry_0_;
  slam_odometry_0_ = slam_odometry_;

  //Update odometry twist
  slam_odometry_.twist.twist.linear.x =  applyFilter(linear_vel_robot, 
                                      slam_odometry_0_.twist.twist.linear.x, 
                                      slam_odometry_00.twist.twist.linear.x, 
                                      cutoff_freq_linear_vel_, 
                                      damping_linear_vel_,
                                      cut_off_value_linear_, 
                                      1.0/publish_rate_);
  slam_odometry_.twist.twist.linear.y = 0;
  slam_odometry_.twist.twist.linear.z = 0;

  slam_odometry_.twist.twist.angular.x = 0; 
  slam_odometry_.twist.twist.angular.y = 0;
  slam_odometry_.twist.twist.angular.z = applyFilter(angular_vel_robot, 
                                      slam_odometry_.twist.twist.angular.z, 
                                      slam_odometry_00.twist.twist.angular.z, 
                                      cutoff_freq_angular_vel_, 
                                      damping_angular_vel_,
                                      cut_off_value_angular_, 
                                      1.0/publish_rate_);

  for(int i=0;i<6;i++){
    slam_odometry_.twist.covariance[i*6+i] = covariance_twist_vec_.at(i);
  }

   // ROS_INFO_STREAM("valor: "<<slam_pose.pose.pose.position.x);

  //Update odometry pose
  slam_odometry_.pose.pose.position.x = slam_pose.pose.pose.position.x;
  slam_odometry_.pose.pose.position.y = slam_pose.pose.pose.position.y;
  slam_odometry_.pose.pose.position.z = 0;
  slam_odometry_.pose.pose.orientation = slam_pose.pose.pose.orientation;

  for(int i =0;i<6;i++){
    if(slam_pose.pose.covariance[i*6+i]!=0)
      slam_odometry_.pose.covariance[i*6+i] = covariance_pose_vec_.at(i)/slam_pose.pose.covariance[i];
    else
      slam_odometry_.pose.covariance[i*6+i] = 0;
  }

  //Setting msg header
  slam_odometry_.header.stamp = actual_time_stamp;
  slam_odometry_.header.frame_id = odom_frame_id_;
  slam_odometry_.child_frame_id = robot_frame_id_;

  //update last time
  last_time_stamp_ = actual_time_stamp;
}
void loadParams(ros::NodeHandle nhPrivate){

  nhPrivate.param<std::string>("odom_topic_name", odom_topic_name_, "/odometry/slam");
  nhPrivate.param<std::string>("slam_pose_topic_name", slam_pose_topic_name_, "/hector/poseupdate");
  nhPrivate.param<std::string>("robot_frame_id", robot_frame_id_, "base_link");
  nhPrivate.param<std::string>("odom_frame_id", odom_frame_id_, "slam_odom");
  nhPrivate.param<double>("publish_rate", publish_rate_, 50.0);                                        
  nhPrivate.param<bool>("publish_tf", publish_tf_, false);                                        
  nhPrivate.param<double>("cutoff_freq_linear_vel",cutoff_freq_linear_vel_,10.0);
  nhPrivate.param<double>("cutoff_freq_angular_vel",cutoff_freq_angular_vel_, 10.0);
  nhPrivate.param<double>("damping_linear_vel", damping_linear_vel_, 1.0); 
  nhPrivate.param<double>("damping_angular_vel",damping_angular_vel_, 1.0);
  nhPrivate.param<double>("cut_off_value_linear",cut_off_value_linear_,0.5), 
  nhPrivate.param<double>("cut_off_value_angular",cut_off_value_angular_, 2.0);

  if(nhPrivate.hasParam("pose_covariance"))
    nhPrivate.getParam("pose_covariance", covariance_pose_vec_);
  // else
  //   ROS_ERROR("No given pose_covariance.");

  // for(int i=0;i<6;i++){
  //   slam_odometry_.twist.covariance[i*6+i] = covariance_pose_vec_.at(i);
  // }

  if(nhPrivate.hasParam("twist_covariance"))
    nhPrivate.getParam("twist_covariance", covariance_twist_vec_);
  // else
  //   ROS_ERROR("No given twist_covariance.");

  // for(int i=0;i<6;i++){
  //     slam_odometry_.pose.covariance[i*6+i] = covariance_twist_vec_.at(i);
  // }


  slam_odometry_.twist.twist.linear.x = 0;
  slam_odometry_.twist.twist.linear.y = 0;
  slam_odometry_.twist.twist.linear.z = 0;
  slam_odometry_.twist.twist.angular.x = 0; 
  slam_odometry_.twist.twist.angular.y = 0;
  slam_odometry_.twist.twist.angular.z = 0;

  slam_odometry_.pose.pose.position.x = 0;
  slam_odometry_.pose.pose.position.y = 0;
  slam_odometry_.pose.pose.position.z = 0;
  slam_odometry_.pose.pose.orientation.x = 0;
  slam_odometry_.pose.pose.orientation.y = 0;
  slam_odometry_.pose.pose.orientation.z = 0;
  slam_odometry_.pose.pose.orientation.w = 1;

  slam_odometry_0_ = slam_odometry_;


  if(nhPrivate.hasParam("initial_pose/pos_x_meters"))
    nhPrivate.getParam("initial_pose/pos_x_meters", slam_odometry_.pose.pose.position.x);
    

  if(nhPrivate.hasParam("initial_pose/pos_y_meters"))
    nhPrivate.getParam("initial_pose/pos_y_meters", slam_odometry_.pose.pose.position.y);

  double yaw;
  if(nhPrivate.hasParam("initial_pose/ang_z_rad"))
    nhPrivate.getParam("initial_pose/ang_z_rad", yaw);

  tf::Quaternion quat;
  quat.setRPY(0.0,0.0,yaw);
  tf::quaternionTFToMsg(quat, slam_odometry_.pose.pose.orientation);
}


int main (int argc, char **argv){

  // Inicia o modulo no ros.
  ros::init(argc, argv, "slam_odom");
  ros::NodeHandle nhPrivate("~");
  ros::NodeHandle nh;
  listener_ = new tf::TransformListener();
  loadParams(nhPrivate);

  ros::Publisher slam_odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic_name_, 1);
  ros::Subscriber slam_pose_sub = nh.subscribe(slam_pose_topic_name_, 10, &slamPoseCallbackPose);      
  ros::Rate loop_rate(publish_rate_);

  
  // last_time_stamp_ = ros::Time::now();
  while(ros::ok()) {     
    slam_odom_pub.publish(slam_odometry_);
    if(publish_tf_){

      //Update transform from odom to world
      static tf::TransformBroadcaster br;
      geometry_msgs::PoseStamped robot_pose_stamped;
      robot_pose_stamped.pose = slam_odometry_.pose.pose;
      robot_pose_stamped.header.stamp = ros::Time::now();
      robot_pose_stamped.header.frame_id = odom_frame_id_;

      tf::poseStampedMsgToTF(robot_pose_stamped, tf_odom_2_robot_);
      
      br.sendTransform(tf::StampedTransform(tf_odom_2_robot_, ros::Time::now(), odom_frame_id_, robot_frame_id_));
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
