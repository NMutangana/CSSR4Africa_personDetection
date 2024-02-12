/*
    This program detects the presence of people in the robot’s field of view and computes their position in an 
    image frame of reference and in the robot’s head Cartesian frame of reference.
    
    The region that the person occupies in the image is determined by computing the bounding box surrounding the 
    person. If more than one person is present in the robot’s field of view, all of them are detected and localized. 
    To ensure coherence in detection and localization over time, each detected person is labelled (e.g., “Person 1”) 
    and the same label is assigned to that person in subsequent images. The label and the bounding box are 
    colour-coded, assigning different colours to different people, and the same colour to a given person in each image 
    in a sequence of images. If that person is no longer detected in an image, then that label is not reused. 
    If that person reappears in a subsequent image, they are given a new label.

    The input is in the form of an RGB image and a depth image retrieved by subscribing to topics the robot’s cameras 
    and depth sensors publish data to.
    
    The output is 
    - an RGB image published on a topic named personDetectionRGBImage, with bounding boxes drawn around each detected person, 
    - a depth image published on a topic named personDetectionDepthImage, and 
    - an array of records published on a topic named personDetectionData, one record for each person detected.
        The components of a record are 
        - the person ID
        - a confidence value between 0 and 1 indicating likelihood that the detection is not a false positive,
        - the 2D image coordinates denoting the centroid of the bounding box (X and Y), 
        - the width and height of the bounding box,  and
        - the depth value at the centroid point. This value combined with the x and y coordinates of the centroid provide the 
          3D coordinates of the person in the image.

    Natasha Mutangana
    16th August 2023
*/

#include "personDetection/personDetection.h"

// Global variables
int messageCount = 0;
int depthMessageCount = 0;
int personIDCounter;
ros::Publisher rgb_image_pub;
ros::Publisher depth_image_pub;
ros::Publisher array_of_records_pub;
cv_bridge::CvImagePtr depthInputImage; // Global variable for depth image


// Callback function for handling received depth image messages. 
// Converts the depth image message to a OpenCV image format and updates the depthInputImage global variable.
void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
    depthMessageCount++;

    ROS_INFO_STREAM("Received depth image message #" << depthMessageCount);
    depthInputImage = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO16);

}

// This function returns the current working directory path as a string.
std::string getCurrentDirectory() {
    char current_dir[PATH_MAX];
    if (getcwd(current_dir, sizeof(current_dir)) != nullptr) {
        return std::string(current_dir);
    }
    return "";
}

/*
 The imageCallback function is used for handling received RGB image messages. 
 It performs the following steps:
    Converts the RGB image message to an OpenCV image format.
    Saves the image to a temporary file for passing to Darknet.
    Changes the current directory to the Darknet directory.
    Executes the Darknet command for object detection on the temporary image file.
    Reads detection results from the "detections.txt" file and stores them in vectors.
    Processes detection results and draws bounding boxes on the RGB and depth images (if available).
    Publishes the modified RGB and depth images.
    Saves the RGB and depth images with bounding boxes.
    Formats the detection data and publishes it as a Float64MultiArray message.
    Saves the detection data to a text file.
*/ 
void imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg) {
    messageCount++;

    ROS_INFO_STREAM("Received image message #" << messageCount);

    cv_bridge::CvImagePtr rgbInputImage = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    // cv_bridge::CvImagePtr depthInputImage = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO16);

    // Save the image to a temporary file
    std::string temp_image_path = "/tmp/input_image.jpg";
    cv::imwrite(temp_image_path, rgbInputImage->image);

    // Change to the desired directory
    std::string desired_directory = "/home/cram/workspace/cram_tutorial/src/darknet"; //Update this path to where the darknet directory is located on your end
    std::string current_directory = getCurrentDirectory();
    
    int chdir_result = chdir(desired_directory.c_str());
    if (chdir_result == 0) {
        // Run Darknet for object detection
        std::string command = "./darknet detector test cfg/coco.data cfg/yolov3.cfg yolov3.weights " + temp_image_path;
        int system_result = system(command.c_str());
        
        if (system_result == 0) {
            ROS_INFO("Object detection completed successfully.");

            // Read detection results from a file named detectiions.txt and store them in a list
            std::string detection_filename = "detections.txt"; // Modify this path if needed
            std::ifstream detection_file(detection_filename);

            std::vector<std::array<double, 13>> detections; // Adjust the array size to match the number of columns in your file
            std::vector<double> depth_values; // Store depth values for each detection

            if (detection_file.is_open()) {
                std::string line;
                std::getline(detection_file, line); // Read and discard the header line
                // Read the data stored in the detection file and store it in the detections vector
                while (std::getline(detection_file, line)) {
                    std::istringstream iss(line);
                    std::array<double, 13> personInfo;
                    char comma; // To read the commas
                    iss >> personInfo[0] >> comma
                        >> personInfo[1] >> comma
                        >> personInfo[2] >> comma
                        >> personInfo[3] >> comma
                        >> personInfo[4] >> comma
                        >> personInfo[5] >> comma
                        >> personInfo[6] >> comma
                        >> personInfo[7] >> comma
                        >> personInfo[8] >> comma
                        >> personInfo[9] >> comma
                        >> personInfo[10] >> comma
                        >> personInfo[11] >> comma
                        >> personInfo[12];

                    detections.push_back(personInfo);
                }
                detection_file.close();
                ROS_INFO_STREAM("Read " << detections.size() << " detection results from file: " << detection_filename);

                // Process detection results and draw bounding boxes
            for (const auto& personInfo : detections) {
                cv::Rect boundingBox(personInfo[6], personInfo[7], personInfo[4], personInfo[5]);
                ROS_INFO_STREAM("Bounding box values. Left: " << personInfo[6] << ", Top:" << personInfo[7] << ", Width:" << personInfo[4] << ", Height:" << personInfo[5]);
                // cv::Rect boundingBox(left, top, width, height);
                // cv::Scalar color(personInfo[10], personInfo[11], personInfo[12]);
                cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
                std::string label = "Person " + std::to_string(static_cast<int>(personInfo[0]));

                // Draw bounding boxes on the RGB image and label them with random colors
                cv::rectangle(rgbInputImage->image, boundingBox, color, 2);
                cv::putText(rgbInputImage->image, label, cv::Point(boundingBox.x, boundingBox.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
                
                // Similar drawing on depthInputImage if available
                if (!depthInputImage) {
                    ROS_WARN_STREAM("Depth image is not available.");
                    return;
                }

                double depth_value = 0.0; // Initializa the depth_value
                
                if (personInfo[2] >= 0 && personInfo[2] < depthInputImage->image.cols && personInfo[3] >= 0 && personInfo[3] < depthInputImage->image.rows) {
                    depth_value = depthInputImage->image.at<uint16_t>(personInfo[3], personInfo[2]) / 1000.0;
                    // Push the computed depth value into the vector
                    depth_values.push_back(depth_value);

                    // personInfo[13] = depth_value;
                    ROS_INFO_STREAM("Depth value #" << depth_value);
                }

                // Draw bounding boxes on the depth image and label them with random colors
                cv::rectangle(depthInputImage->image, boundingBox, color, 2);
                cv::putText(depthInputImage->image, label, cv::Point(boundingBox.x, boundingBox.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

            }
        
        // Publish the RGB image with bounding boxes 
        sensor_msgs::ImagePtr rgb_image_msg = cv_bridge::CvImage(rgb_msg->header, "bgr8", rgbInputImage->image).toImageMsg();
        rgb_image_pub.publish(rgb_image_msg);

        ROS_INFO_STREAM("Published RGB image message #" << messageCount);

        // Save the person detection outputs in the package's directory
        std::string packagePath = ros::package::getPath("personDetection");
        
        std::string rgb_image_filename = packagePath + "/rgb_image_" + std::to_string(messageCount) + ".jpg";
        cv::imwrite(rgb_image_filename, rgbInputImage->image);

        if (depthInputImage) {
            // Publish the depth image with bounding boxes 
            sensor_msgs::ImagePtr depth_image_msg = cv_bridge::CvImage(depthInputImage->header, "mono16", depthInputImage->image).toImageMsg();
            depth_image_pub.publish(depth_image_msg);
        }

        ROS_INFO_STREAM("Published depth image message #" << messageCount);

        // Publish the array of records
        std::string depth_image_filename = packagePath + "/depth_image_" + std::to_string(messageCount) + ".png";
        cv::imwrite(depth_image_filename, depthInputImage->image);

        std_msgs::Float64MultiArray detection_msg;
        detection_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        detection_msg.layout.dim[0].label = "detections";
        detection_msg.layout.dim[0].size = detections.size();
        detection_msg.layout.dim[0].stride = 7 * detections.size();
        detection_msg.data.resize(7 * detections.size());

        // Send the following values {person_ID,  confidence level, centroid.x, centroid.y, width, height, depth_value}
        for (size_t i = 0; i < detections.size(); i++) {
            detection_msg.data[i * 7 + 0] = detections[i][0];
            detection_msg.data[i * 7 + 1] = detections[i][1];
            detection_msg.data[i * 7 + 2] = detections[i][2];
            detection_msg.data[i * 7 + 3] = detections[i][3];
            detection_msg.data[i * 7 + 4] = detections[i][4];
            detection_msg.data[i * 7 + 5] = detections[i][5];
            detection_msg.data[i * 7 + 6] = depth_values[i];
        }

        array_of_records_pub.publish(detection_msg);
        ROS_INFO_STREAM("Published array of records message #" << messageCount);

        std::string records_filename = packagePath + "/image_" + std::to_string(messageCount) + "data.txt";
        std::ofstream records_file(records_filename);
        if (records_file.is_open()) {
            for (size_t i = 0; i < detections.size(); i++) {
                records_file << detections[i][0] << " " << detections[i][1] << " "
                            << detections[i][2] << " " << detections[i][3] << " "
                            << detections[i][4] << " " << detections[i][5] << " " 
                            << depth_values[i] <<"\n";
            }
            records_file.close();
            ROS_INFO_STREAM("Saved array of records to file: " << records_filename);
        } else {
            ROS_WARN_STREAM("Unable to save records to file: " << records_filename);
        }

    } else {
        ROS_WARN_STREAM("Unable to open detection file: " << detection_filename);
    }

        } else {
            ROS_ERROR("Error running object detection command.");
        }
        
        // Return to the original directory
        int restore_result = chdir(current_directory.c_str());
        if (restore_result != 0) {
            ROS_ERROR("Failed to restore original directory.");
        }
    } else {
        ROS_ERROR("Failed to change directory to %s", desired_directory.c_str());
    }
    
}

/* Main function */
int main(int argc, char **argv){
    string path;
    #ifdef ROS
        path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        path = "..";
    #endif
    
    // Initialize the ROS node
    ros::init(argc, argv, "yolo_person_detection");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Hello World! This program detects people using the You Only Look Once algorithm.");

    // Subscribe to the image topics, both RGB and depth images
    // If the RGB image is received by subscribing to the front camera
    string rgb_image_topic_name = extract_topic("FrontCamera");
    // // Uncomment the line below if the RGB image is received by subscribing to the bottom camera
    // string rgb_image_topic_name = extract_topic("BottomCamera");
    string depth_image_topic_name = extract_topic("DepthCamera");

    ROS_INFO_STREAM("Subscribing to :" << rgb_image_topic_name << "\n"  ); // Print the topic name
    ros::Subscriber rgb_image_sub = nh.subscribe(rgb_image_topic_name, 1000, imageCallback);
    ROS_INFO_STREAM("Subscribing to :" << depth_image_topic_name << "\n"  ); // Print the topic name
    ros::Subscriber depth_image_sub = nh.subscribe(depth_image_topic_name, 1000, depthImageCallback);

    // Create a publisher for the RGB and depth person images
    rgb_image_pub = nh.advertise<sensor_msgs::Image>("/personDetection/rgb_image", 1000);
    depth_image_pub = nh.advertise<sensor_msgs::Image>("/personDetection/depth_image", 1000);


    // Publish the array of records message
    array_of_records_pub = nh.advertise<std_msgs::Float64MultiArray>("personDetection/data", 1000);


    // Create an OpenCV window to display the received image
    // cv::namedWindow("Received Image", cv::WINDOW_AUTOSIZE);

    // Create an OpenCV window to display the received image with detections
    // cv::namedWindow("Person Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Person Image", cv::WINDOW_NORMAL); // Allow resizing
    cv::resizeWindow("Person Image", 800, 600); // Set the window size to 800x600


    // Spin to receive and process ROS messages
    ros::spin();

    // Close OpenCV windows before exiting
    cv::destroyAllWindows();

    return 0;
}

/* Helper Functions */
void prompt_and_exit(int status){
    printf("Press any key to continue ... \n");
    getchar();
    exit(status);
}

void prompt_and_continue(){
    printf("Press any key to proceed ...\n");
    getchar();
}

/* Extract topic names for the respective simulator or physical robot */
string extract_topic(string key){
    bool debug = false;   // used to turn debug message on
    
    std::string conf_file = "personDetectionConfiguration.ini";  // configuration filename
    std::string config_path;                                  // configuration path
    std::string config_path_and_file;                         // configuration path and filename
    
    std::string platformKey = "platform";                     // platform key 
    std::string robotTopicKey = "robotTopics";                // robot topic key
    std::string simulatorTopicKey = "simulatorTopics";        // simulator topic key

    std::string platformValue;                                // platform value
    std::string robotTopicValue;                              // robot topic value
    std::string simulatorTopicValue;                          // simulator topic value
    
    std::string topic_file;                                   // topic filename
    std::string topic_path;                                   // topic filename path
    std::string topic_path_and_file;                          // topic with path and file 

    std::string topic_value = "";                             // topic value

    // Construct the full path of the configuration file
    #ifdef ROS
        config_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        config_path = "..";
    #endif

    // set configuration path
    config_path += "/config/";
    config_path_and_file = config_path;
    config_path_and_file += conf_file;

    if (debug) printf("Config file is %s\n", config_path_and_file.c_str());

    // Open configuration file
    std::ifstream conf_if(config_path_and_file.c_str());
    if (!conf_if.is_open()){
        printf("Unable to open the config file %s\n", config_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string configLineRead;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(conf_if, configLineRead)){
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);
        
        if (paramKey == platformKey){ platformValue = paramValue;}
        
        else if (paramKey == robotTopicKey){ robotTopicValue = paramValue;}

        else if (paramKey == simulatorTopicKey){ simulatorTopicValue = paramValue;}
    }
    conf_if.close();

    // set the topic file based on the config extracted above
    if (platformValue == "simulator") { topic_file = simulatorTopicValue; }
    else if (platformValue == "robot") { topic_file = robotTopicValue; }
    
    if (debug) printf("Topic file: %s\n", topic_file.c_str());

    // Construct the full path of the topic file
    #ifdef ROS
        topic_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        topic_path = "..";
    #endif

    // set topic path    
    topic_path += "/data/";
    topic_path_and_file = topic_path;
    topic_path_and_file += topic_file;

    if (debug) printf("Topic file is %s\n", topic_path_and_file.c_str());

    // Open topic file
    std::ifstream topic_if(topic_path_and_file.c_str());
    if (!topic_if.is_open()){
        printf("Unable to open the topic file %s\n", topic_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string topicLineRead;   // variable to read the line in the file
    // Get key-value pairs from the topic file
    while(std::getline(topic_if, topicLineRead)){
        std::istringstream iss(topicLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);
        if (paramKey == key) {
            topic_value = paramValue;
            break;
        }
    }
    topic_if.close();

    // verify the topic_value is not empty
    if (topic_value == ""){
        printf("Unable to find a valid topic.\n");
        prompt_and_exit(1);
    }
    return topic_value;
}