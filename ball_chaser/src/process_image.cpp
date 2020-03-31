#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <math.h>

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z){
    ROS_INFO("Moving robot");
    ROS_INFO("Received drive velocities - linear: %1.2f, -angular:%1.2f", lin_x, ang_z);

    ball_chaser::DriveToTarget velocity;
    velocity.request.linear_x = lin_x;
    velocity.request.angular_z = ang_z;

    if (!client.call(velocity)){
        ROS_ERROR("Failed to call drive_robot node with given velocity");
    }
    else{
        ROS_INFO("drive_robot node called with given velocity");
    }
}

bool ball_check(int idx, sensor_msgs::Image img){
  //process data for all 3 channels(serialized)
  std::vector<int> rgb_values = {img.data[idx], img.data[idx+1], img.data[idx+2]};
  int white_pixel = 255;
  int white_pixel_count = std::count(rgb_values.begin(), rgb_values.end(), white_pixel);
  if (white_pixel_count == 3)
    return true;
  return false;
}

int find_max(int l, int m, int r){
  int max_el = std::max(std::max(l, m), r);
  return max_el;
}

void process_image_callback(const sensor_msgs::Image img){
    float left_idx = 0;
    float middle_idx = 0.33;
    float right_idx = 0.67;
  
    int l = 0;
    int r = 0;
    int m = 0;
  
    for (int i=0; i < img.height * img.step; i++){
      if(ball_check(i, img)){
        ROS_INFO("Ball found in image stream. Proceeding to send velocity");
        float row_position_idx = fmod((float(i)/float(img.step)), 1); 
        
        if (left_idx <= row_position_idx < middle_idx)
          	l++;
        else if (middle_idx<= row_position_idx < right_idx)
          	m++;
        else
          	r++;
      }else{
        drive_robot(0.0, 0.0);
      }
    }
  	
    if (find_max(l, r, m) == l){
      ROS_INFO("Moving along left direction");
      drive_robot(0.0, 0.1);
    }else if (find_max(l, r, m) == m){
      ROS_INFO("Moving forward");
      drive_robot(0.1, 0.0);
    }else{
      ROS_INFO("Moving along the right direction");
      drive_robot(0.0, -0.1);
    }

}

int main(int argc, char** argv){

    ros::init(argc, argv, "process_image");
    ros::NodeHandle nodeHandler;

    client = nodeHandler.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    ros::Subscriber subscriber = nodeHandler.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    ros::spin();
    return 0;
}