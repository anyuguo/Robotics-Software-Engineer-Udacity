#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
using namespace std;

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Driving the robot");

    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;

    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service and pass the requested motor commands
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service command_robot");
    }

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    int i;
    
    int ball_position;
    int ball_position_center;
    int ball_position_sum = 0;
    int white_pixel_num = 0;
    
    

    for (i = 0; i + 2 < img.data.size(); i = i + 3)
    {
        if ((img.data[i] == 255) && (img.data[i+1] == 255) && (img.data[i+2] == 255))
        {
            ball_position = (i % (img.width * 3)) / 3;
            ball_position_sum += ball_position;
            white_pixel_num++;
        }
    }
    
    if (white_pixel_num == 0)
    {
        drive_robot(0, 0);
    }
    else
    {
        ball_position_center = ball_position_sum / white_pixel_num;
        
        if(ball_position_center < img.width / 3)
        {
            drive_robot(0.1, 0.5);
        }
        else if(ball_position_center > img.width * 2 / 3)
        {
            drive_robot(0.1, -0.5);
        }
        else
        {
            drive_robot(0.1, 0);
        }
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
