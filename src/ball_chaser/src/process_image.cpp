#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
	
    // Call the command_robot service and pass the requested joint angles
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service command_robot");
    }	

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int left_count = 0;
    int front_count = 0;
    int right_count = 0;


    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
   for (int i = 0; i < img.height * img.step; i += 3) 
   {
        int pos = (i % img.step)/3;
	
        if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel) 
	{
            if(pos <= (img.width / 3) ) 
	    {
		left_count += 1;                
            }
            if(pos > (img.width / 3) && pos <= (img.width * 2 / 3)) 
            {
		front_count += 1;               
            }
            if(pos > (img.width * 2 / 3)) 
            {
		right_count += 1;                
            }
            //ROS_INFO("Ball Pos- p:%d", pos);
	}
    }
    //ROS_INFO("Ball Process - w:%d, 1:%d, 2:%d, s:%d", img.width, (img.width / 3), (img.width * 2 / 3), img.step);
    //ROS_INFO("Ball Pos Counter - l:%d, f:%d, r:%d", left_count, front_count, right_count);

    if (left_count > front_count && left_count > right_count) 
    {
	drive_robot(0.0, 0.5);  // turn left
    }
    else if (front_count > left_count && front_count > right_count)  
    {
        drive_robot(0.5, 0.0);  // drive forward
    }
    else if (right_count > left_count && right_count > front_count) 
    {
        drive_robot(0.0, -0.5); // turn right
    }
    else
    {
	drive_robot(0.0, 0); // stop
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
