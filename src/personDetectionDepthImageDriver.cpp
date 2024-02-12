/*
    The personDetectection module uses the physical robot as the primary driver to generate test data. 
    However, in the abscence of a physical robot, a driver is needed to generate test data.

    This program acts as a driver to generate test data (RGB and depth images) for the personDetection task.
    
    These images are acquired from 
    - a USB camera or a USB RGB-D camera, or
    - online datasets.
    
    The data is then published on the same topics as those used by the physical robot's RGB and RGB-D cameras.

    The names of the topics to be used for each sensor will be read from a data file comprising a sequence of 
    key-value pairs. The key is the name of the sensor. The value is the topic name.

    This driver reads a sequence lines from an input file personDetectionInput.txt.
    Each line contains a filename of an image to be processed. 

    It is assumed that the input file is located in a data directory given by the path ../data/ 
    defined relative to the location of executable for this application.

    Natasha Mutangana
    27th July 2023
*/

#include "personDetection/personDetection.h"

void prompt_and_exit(int status) {
    printf("Press any key to continue and close terminal ... \n");
    getchar();
    exit(status);
}

int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

int main(int argc, char **argv) { 
    ros::init(argc, argv, "publish_images"); // Initialize the ROS system 
    ros::NodeHandle nh;                        // Become a node 

    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/naoqi_driver/camera/depth/image_raw", 1000);

    // const char inputWindowName[MAX_STRING_LENGTH] = "Input Image";
    cv::Mat inputImage;
    // cv::namedWindow(inputWindowName, cv::WINDOW_AUTOSIZE);

    const char input_filename[MAX_FILENAME_LENGTH] = "personDetectionRGBDInput.txt";    
    char input_path_and_filename[MAX_FILENAME_LENGTH];    
    char data_dir[MAX_FILENAME_LENGTH];
    char file_path_and_filename[MAX_FILENAME_LENGTH];
    int end_of_file;
    bool debug = true;
    char filename[MAX_FILENAME_LENGTH];
    ros::Rate rate(2); // Loop at 2Hz until the node is shut down 

    FILE *fp_in;   

    #ifdef ROS   
        strcpy(data_dir, ros::package::getPath(ROS_PACKAGE_NAME).c_str()); // get the package directory
    #else
        strcpy(data_dir, "..");
    #endif

    strcat(data_dir, "/data/");
    strcpy(input_path_and_filename, data_dir);
    strcat(input_path_and_filename, input_filename);

    printf("Attempting to open file: %s\n", input_path_and_filename);


    if ((fp_in = fopen(input_path_and_filename,"r")) == 0) {
        printf("Error can't open input personDetectionRGBDInput.txt\n");
        prompt_and_exit(1);
    }

    printf("Opened input file: personDetectionRGBDInput.txt\n\n");
   
    do {
        end_of_file = fscanf(fp_in, "%s", filename);
      
        if (end_of_file != EOF) {          
            printf("\nReading image from: %s \n", filename);

            if (strcmp(filename, "camera") != 0) {

                // Read the image using OpenCV
                cv::Mat inputImage = cv::imread(data_dir + std::string(filename), cv::IMREAD_COLOR);
                
                if (!inputImage.empty()) {

                    // Show the input image in the window
                    // cv::imshow(inputWindowName, inputImage);

                    // Convert the image to a ROS image message
                    cv_bridge::CvImage img_bridge;
                    img_bridge.image = inputImage;
                    img_bridge.encoding = "mono8"; // Assuming it's a color image
                    sensor_msgs::ImagePtr ros_img_msg = img_bridge.toImageMsg();

                    // Publish the ROS image message
                    image_pub.publish(ros_img_msg);

                    // Send a message to rosout with the details
                    ROS_INFO_STREAM("Publishing image: " << filename);

                } else {
                    ROS_ERROR_STREAM("Failed to read image: " << filename);
                }

                ros::spinOnce(); // Handle callbacks to publish the image

                rate.sleep(); // Sleep for 0.5 seconds

            } 
            
            else {
                printf("\nOpen camera\n");
                int desired_width = 320;
                int desired_height = 240;

                // Initialize the camera
                VideoCapture camera;
                camera.open(0);  // Open the default camera
                // camera.open(1);  // Open the external camera

                if (camera.isOpened()) {
                    printf("Camera is opened. Press any key to stop streaming ...\n");
                    camera >> inputImage;  // Read a frame from the camera

                    int width = static_cast<int>(camera.get(cv::CAP_PROP_FRAME_WIDTH));
                    int height = static_cast<int>(camera.get(cv::CAP_PROP_FRAME_HEIGHT));
                    
                    std::cout << "Resolution: Width: " << width << " Height: " << height << std::endl; 
                    

                    do {
                        camera >> inputImage;

                        // camera >> frame;                          // read a frame from the camera                    
                        // imshow(windowName, frame);
                        cv::imshow("inputWindowName", inputImage);

                        // Convert the image to a ROS image message
                        cv_bridge::CvImage img_bridge;
                        img_bridge.image = inputImage;
                        img_bridge.encoding = "mono8"; // Since it's a color image
                        sensor_msgs::ImagePtr ros_img_msg = img_bridge.toImageMsg();

                        // Publish the ROS image message
                        image_pub.publish(ros_img_msg);

                        // Send a message to rosout with the details
                        ROS_INFO_STREAM("Publishing camera image");

                        cvWaitKey(30); // this is essential as it allows openCV to handle the display event ... 
                                        // the argument is the number of milliseconds to wait 
                    } while (!_kbhit());

                    getchar(); // flush the buffer from the keyboard hit

                }

                camera.release();
                cv::destroyAllWindows();
            }
        }
    } while (end_of_file != EOF);

    fclose(fp_in); 
   
    return 0;        
}
