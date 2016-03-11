#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "geometry_msgs/PoseStamped.h"
#include "iiwa_msgs/ConfigureSmartServo.h"

iiwa_msgs::JointPosition current_joint_position;
geometry_msgs::PoseStamped current_cartesian_position;
std::string joint_position_topic, cartesian_position_topic;
int ros_rate = 20000;
bool isRobotConnected = false;

void jointPositionCallback(const iiwa_msgs::JointPosition& jp)
{
  if (!isRobotConnected)
    ~isRobotConnected;
  current_joint_position = jp;
}

void cartesianPositionCallback(const geometry_msgs::PoseStamped& ps)
{
  if (!isRobotConnected)
    ~isRobotConnected;
  current_cartesian_position = ps;
}

int main (int argc, char **argv) {
  
  // Initialize ROS
  ros::init(argc, argv, "CommandRobotMoveit");
  ros::NodeHandle nh("~");
  
  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh.param<std::string>("joint_position_topic", joint_position_topic, "/iiwa/state/JointPosition");
  nh.param<std::string>("cartesian_position_topic", cartesian_position_topic, "/iiwa/state/CartesianPose");
  
  ros::ServiceClient client = nh.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
  iiwa_msgs::ConfigureSmartServo config;
  
  // Dynamic parameter to choose the rate at wich this node should run
  nh.param("ros_rate", ros_rate, 20000); // 20 sec
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
  
  // Subscribers and publishers
  ros::Subscriber sub_joint_position = nh.subscribe(joint_position_topic, 1, jointPositionCallback);
  ros::Subscriber sub_cartesian_position = nh.subscribe(cartesian_position_topic, 1, cartesianPositionCallback);
  
  
  while (ros::ok()) {
    if (isRobotConnected) {
      
      config.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
      config.request.mode.relative_velocity = 0.05;
      config.request.mode.cartesian_stiffness.stiffness.x = 1000;
      config.request.mode.cartesian_stiffness.stiffness.y = 1000;
      config.request.mode.cartesian_stiffness.stiffness.z = 350;
      config.request.mode.cartesian_stiffness.stiffness.a = 400;
      config.request.mode.cartesian_stiffness.stiffness.b = 400;
      config.request.mode.cartesian_stiffness.stiffness.c = 400;
      
      config.request.mode.cartesian_damping.damping.x = 0.7;
      config.request.mode.cartesian_damping.damping.y = 0.7;
      config.request.mode.cartesian_damping.damping.z = 0.7;
      config.request.mode.cartesian_damping.damping.a = 0.7;
      config.request.mode.cartesian_damping.damping.b = 0.7;
      config.request.mode.cartesian_damping.damping.c = 0.7;
      
      if (client.call(config))
      {
	if(!config.response.success)
	  ROS_ERROR("Config failed, Java error: %s", config.response.error.c_str());
	else
	  ROS_INFO("SmartServo Service successfully called.");
      }
      else
      {
	ROS_ERROR("Config failed - service could not be called - QUITTING NOW !");
      }
      
      loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 20 seconds in this example.
    }
    else {
      ROS_ERROR("Robot is not connected...");
      ros::Duration(5.0).sleep();
    }
  }
  
  std::cerr<<"Stopping spinner..."<<std::endl;
  spinner.stop();
  
  std::cerr<<"Bye!"<<std::endl;
  
  return 0;
  
}; 