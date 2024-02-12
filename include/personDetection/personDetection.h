#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/console.h>


#include <fstream>
#include <sstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <ctime>

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <ctype.h>
#include <iostream>
#include <string>

#ifndef ROS
//   The code below runs for the command line code
//   #include <conio.h>

#else
  #include <sys/select.h>
  #include <termios.h>
  #include <stropts.h>
  #include <sys/ioctl.h>

#endif

//opencv
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h> // sensor_msgs::Image
#include <cv_bridge/cv_bridge.h> // For converting between OpenCV images and ROS messages
// #include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <map>
#include <fstream>
// #include <ros/package.h> // Include the package header to get the package path
#include <iostream>
#include <cstdlib>
#include <iostream>
#include <unistd.h> // For changing directories

#define TRUE  1
#define FALSE 0
#define MAX_STRING_LENGTH 200 // Length was modified to avoid stack overflow NM
#define MAX_FILENAME_LENGTH 200

using namespace boost;
using namespace std;
using namespace cv;

#define ROS

#ifdef ROS
  // ncurses.h must be included after opencv2/opencv.hpp to avoid incompatibility
  #include <ncurses.h>
  #include <ros/ros.h>
  #include <ros/package.h>
#endif 

#ifdef ROS
  int _kbhit();
#endif

/* Call back functions executed when a sensor data arrived */
void jointStateCallBack(const sensor_msgs::JointState& state); 
void imageCallBack(const sensor_msgs::ImageConstPtr& msg);


void frontCamera(ros::NodeHandle nh);
void bottomCamera(ros::NodeHandle nh);
void depthCamera(ros::NodeHandle nh);

void frontCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
void bottomCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);
void depthCameraMessageReceived(const sensor_msgs::ImageConstPtr& msg);


std::vector<string> extract_tests(string key);
string extract_topic(string set);

void prompt_and_exit(int err);
void prompt_and_continue();

/* 
  This program publishes different images for person detection testing when there's no access to the cameras
  -----------------------------------------------------------------------
 
  (This is the interface file: it contains the declarations of dedicated functions to implement the application.
  These function are called by client code in the application file. The functions are defined in the implementation file.)

  Natasha Mutangana
  15 July 2023
*/

#define GCC_COMPILER (defined(__GNUC__) && !defined(__clang__))

#if GCC_COMPILER
  #ifndef ROS
      #define ROS
   #endif
  #ifndef ROS_PACKAGE_NAME
      #define ROS_PACKAGE_NAME "personDetection"
   #endif
#endif

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <ctype.h>
#include <iostream>
#include <string>

#ifndef ROS
  #include <conio.h>

#else
  #include <sys/select.h>
  #include <termios.h>
  #include <stropts.h>
  #include <sys/ioctl.h>

#endif

//opencv
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h> // sensor_msgs::Image
#include <cv_bridge/cv_bridge.h> // For converting between OpenCV images and ROS messages
// #include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <map>
#include <fstream>
// #include <ros/package.h> // Include the package header to get the package path
#include <iostream>
#include <cstdlib>
#include <iostream>
#include <unistd.h> // For changing directories
 
#define TRUE  1
#define FALSE 0
#define MAX_STRING_LENGTH 200 // Length was modified to avoid stack overflow NM
#define MAX_FILENAME_LENGTH 200

using namespace std;
using namespace cv;


/* function prototypes go here */
void prompt_and_exit(int status);

#ifdef ROS
  // ncurses.h must be included after opencv2/opencv.hpp to avoid incompatibility
  #include <ncurses.h>
  #include <ros/ros.h>
  #include <ros/package.h>
#endif 

#ifdef ROS
  int _kbhit();
#endif