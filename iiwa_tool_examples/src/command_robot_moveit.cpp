#include "ros/ros.h"
#include "iiwa_msgs/JointPosition.h"
#include "geometry_msgs/PoseStamped.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>

iiwa_msgs::JointPosition current_joint_position;
geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position;
std::string joint_position_topic, cartesian_position_topic, movegroup_name, ee_link;
double ros_rate = 0.1;
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
  nh.param<std::string>("move_group", movegroup_name, "manipulator");
  nh.param<std::string>("ee_link", ee_link, "tool_link_ee");
  
  // Dynamic parameter to choose the rate at wich this node should run
  nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
  
  // Subscribers and publishers
  ros::Subscriber sub_joint_position = nh.subscribe(joint_position_topic, 1, jointPositionCallback);
  ros::Subscriber sub_cartesian_position = nh.subscribe(cartesian_position_topic, 1, cartesianPositionCallback);
  
  int direction = 1;
  
  // Create MoveGroup
  move_group_interface::MoveGroup group(movegroup_name);
  moveit::planning_interface::MoveGroup::Plan myplan;
      
  // Configure planner 
  group.setPlanningTime(0.5);
  group.setPlannerId(movegroup_name+"[RRTConnectkConfigDefault]");
  group.setEndEffectorLink(ee_link);

  
  while (ros::ok()) {
    if (isRobotConnected) {
      
      command_cartesian_position = current_cartesian_position;
      command_cartesian_position.pose.position.z -= direction * 0.10;
  
      group.setStartStateToCurrentState();
      group.setPoseTarget(command_cartesian_position);
      bool success = group.plan(myplan);
      if (success)
      group.execute(myplan);
      
      direction *= -1; // In the next iteration the motion will be on the opposite direction
      
      loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
    }
    else {
      ROS_ERROR("Robot is not connected...");
      ros::Duration(5.0).sleep(); // 5 seconds
    }
  }
  
  std::cerr<<"Stopping spinner..."<<std::endl;
  spinner.stop();
  
  std::cerr<<"Bye!"<<std::endl;
  
  return 0;
  
}; 