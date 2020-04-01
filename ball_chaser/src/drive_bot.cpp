#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;

bool handle_drive_request_callback(ball_chaser::DriveToTarget::Request& req,
     ball_chaser::DriveToTarget::Response& res){
	
	// ROS_INFO("DriveToTarget request received: -linear: %1.2f, -angular: %1.2f", (float)req.linear_x, (float)req.angular_z);
//
	geometry_msgs::Twist motor_commands;
	motor_commands.linear.x = req.linear_x;
	motor_commands.angular.z = req.angular_z;

	motor_command_publisher.publish(motor_commands);
	res.msg_feedback = "DriveToTarget request submitted: -linear: " + std::to_string(motor_commands.linear.x) + ", -angular: " + std::to_string(motor_commands.angular.z);
	ROS_INFO_STREAM(res.msg_feedback);
	return true;

}
int main(int argc, char** argv){
	
	ros::init(argc, argv, "drive_bot");
	ros::NodeHandle nodeHandler;

	motor_command_publisher = nodeHandler.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::ServiceServer service = nodeHandler.advertiseService("/ball_chaser/command_robot", handle_drive_request_callback);

	ROS_INFO("Ready to send drive commands");	
	ros::spin();
	
	return 0;
}
