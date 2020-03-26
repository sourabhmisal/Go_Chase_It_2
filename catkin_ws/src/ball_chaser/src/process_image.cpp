#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>  
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <list>

namespace enc = sensor_msgs::image_encodings;

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Driving the robot towards the ball");
    
    // Create service msg
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = (float)lin_x;
    srv.request.angular_z = (float)ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
            ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    // Convert to gray
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    cv::Mat src_gray;
    cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );

    // Reduce the noise so we avoid false detection
    cv::GaussianBlur( src_gray, src_gray, cv::Size(9, 9), 2, 2);

    // Apply the Hough Transform to find the circles
    std::vector<cv::Vec3f> circles;

    // Detect the circles
    cv::HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 70, 35, 0, 0);

    // Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ ) {
        cv::Point centre(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        // circle centre
        cv::circle( cv_ptr->image, centre, 3, cv::Scalar(0,255,0), -1, 8, 0);

        // circle outline
        cv::circle( cv_ptr->image, centre, radius, cv::Scalar(0,0,255), 3, 8, 0);

    }

    cv::namedWindow("Detected balls", CV_WINDOW_AUTOSIZE);
    cv::imshow("Hough Circle Transform Demo", cv_ptr->image);

    cv::waitKey(10);


    float lin_x;
    float ang_z;

    if (circles.size() != 1) {
        ROS_WARN_STREAM(circles.size() << " circles detected");
        drive_robot(0,0);
    } else {
        if (circles[0][0] < cv_ptr->image.cols/3) {
            ROS_INFO_STREAM("Moving left");
            lin_x = 0.5;
            ang_z = 1.5;
        }  else if (circles[0][0] < cv_ptr->image.cols/3) {
            ROS_INFO_STREAM("Moving straight");
            lin_x = 0.5;
            ang_z = 0;
        }  else {
            ROS_INFO_STREAM("Moving right");
            lin_x = 0.5;
            ang_z = -1.5;
        }
    }
    drive_robot(lin_x, ang_z);        
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

