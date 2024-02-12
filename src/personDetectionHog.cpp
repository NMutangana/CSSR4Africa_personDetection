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
        - the 2D image coordinates denoting the centroid of the bounding box, 
        - the width and height of the bounding box, 
        - a confidence value between 0 and 1 indicating likelihood that the detection is not a false positive, and 
        - the 3D coordinates that define the point that corresponds to the centroid of the bounding box surrounding the 
          person in the image.

    Natasha Mutangana
    27th July 2023
*/

#include "personDetection/personDetection.h"

int messageCount = 0;
int depthMessageCount = 0;
int personIDCounter;
ros::Publisher rgb_image_pub;
ros::Publisher depth_image_pub;
ros::Publisher array_of_records_pub;
cv_bridge::CvImagePtr depthInputImage; // Global variable for depth image

// Thos function performs HOG-based person detection on an input RGB image using a pre-trained SVM model. 
// Returns a vector of detected bounding boxes.
std::vector<cv::Rect> performHOGPersonDetection(const cv::Mat& rgbImage) {
    cv::HOGDescriptor hog;
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    std::vector<cv::Rect> people;
    hog.detectMultiScale(rgbImage, people);

    return people;
}

// Callback function for handling received depth image messages. Converts the depth image 
// message to a OpenCV image format and updates the depthInputImage global variable
void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
    depthMessageCount++;

    ROS_INFO_STREAM("Received depth image message #" << depthMessageCount);
    depthInputImage = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO16);

}

/*
 The imageCallback function is used for handling received RGB image messages. It performs the following steps:
    Converts the RGB image message to an OpenCV image format.
    Performs HOG-based person detection on the RGB image.
    Processes detected bounding boxes to calculate centroid, dimensions, and depth information.
    Draws bounding boxes on RGB and depth images.
    Publishes modified RGB and depth images.
    Saves RGB and depth images with bounding boxes.
    Formats and publishes detection data as a Float64MultiArray message.
    Saves detection data to a text file.
*/
void imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg) {
    messageCount++;

    ROS_INFO_STREAM("Received image message #" << messageCount);

    cv_bridge::CvImagePtr rgbInputImage = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    // cv_bridge::CvImagePtr depthInputImage = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO16);

    std::vector<cv::Rect> peopleBoundingBoxes = performHOGPersonDetection(rgbInputImage->image);

    static std::map<int, std::pair<cv::Scalar, std::string>> personInfoMap;
    personIDCounter = 0;

    std::vector<std::array<double, 5>> detections;

    if (!depthInputImage) {
        ROS_WARN_STREAM("Depth image is not available.");
        return;
    }

    // For each detected person, compute their bounding box's centroid coordinates, width, and height
    for (const auto& boundingBox : peopleBoundingBoxes) {
        cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
        std::string label = "Person " + std::to_string(personIDCounter + 1);
        cv::Point2d centroid(boundingBox.x + boundingBox.width / 2.0, boundingBox.y + boundingBox.height / 2.0);
        double width = boundingBox.width;
        double height = boundingBox.height;

        double depth_value = 0.0;

        // Using the bounding box centroid coordinates, compute the depth value
        if (centroid.x >= 0 && centroid.x < depthInputImage->image.cols && centroid.y >= 0 && centroid.y < depthInputImage->image.rows) {
            depth_value = depthInputImage->image.at<uint16_t>(centroid.y, centroid.x) / 1000.0;
        }

        // Draw bounding boxes on both the RGB and depth images and label them with random colors
        cv::rectangle(rgbInputImage->image, boundingBox, color, 2);
        cv::putText(rgbInputImage->image, label, cv::Point(boundingBox.x, boundingBox.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

        cv::rectangle(depthInputImage->image, boundingBox, color, 2);
        cv::putText(depthInputImage->image, label, cv::Point(boundingBox.x, boundingBox.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

        // Store the array of records for each detected person
        std::array<double, 5> personInfo = {centroid.x, centroid.y, width, height, depth_value};
        detections.push_back(personInfo);

        personInfoMap[personIDCounter] = std::make_pair(color, label);
        personIDCounter++;
    }

    // Publish the RGB and depth images with bounding boxes
    sensor_msgs::ImagePtr rgb_image_msg = cv_bridge::CvImage(rgb_msg->header, "bgr8", rgbInputImage->image).toImageMsg();
    rgb_image_pub.publish(rgb_image_msg);

    ROS_INFO_STREAM("Published RGB image message #" << messageCount);

    // Save the person detection outputs in the package's directory
    std::string packagePath = ros::package::getPath("personDetection");
    
    std::string rgb_image_filename = packagePath + "/rgb_image_" + std::to_string(messageCount) + ".jpg";
    cv::imwrite(rgb_image_filename, rgbInputImage->image);

    if (depthInputImage) {
        sensor_msgs::ImagePtr depth_image_msg = cv_bridge::CvImage(depthInputImage->header, "mono16", depthInputImage->image).toImageMsg();
        depth_image_pub.publish(depth_image_msg);
    }

    ROS_INFO_STREAM("Published depth image message #" << messageCount);

    std::string depth_image_filename = packagePath + "/depth_image_" + std::to_string(messageCount) + ".png";
    cv::imwrite(depth_image_filename, depthInputImage->image);

    // Puublish the arrays of records
    std_msgs::Float64MultiArray detection_msg; // Declare the message type
    detection_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    detection_msg.layout.dim[0].label = "detections";
    detection_msg.layout.dim[0].size = detections.size();
    detection_msg.layout.dim[0].stride = 5 * detections.size();
    detection_msg.data.resize(5 * detections.size());

    for (size_t i = 0; i < detections.size(); i++) {
        detection_msg.data[i * 5 + 0] = detections[i][0];
        detection_msg.data[i * 5 + 1] = detections[i][1];
        detection_msg.data[i * 5 + 2] = detections[i][2];
        detection_msg.data[i * 5 + 3] = detections[i][3];
        detection_msg.data[i * 5 + 4] = detections[i][4];
    }

    array_of_records_pub.publish(detection_msg);
    ROS_INFO_STREAM("Published array of records message #" << messageCount);

    std::string records_filename = packagePath + "/image_" + std::to_string(messageCount) + "data.txt";
    std::ofstream records_file(records_filename);
    if (records_file.is_open()) {
        for (size_t i = 0; i < detections.size(); i++) {
            records_file << detections[i][0] << " " << detections[i][1] << " "
                         << detections[i][2] << " " << detections[i][3] << " "
                         << detections[i][4] << "\n";
        }
        records_file.close();
        ROS_INFO_STREAM("Saved array of records to file: " << records_filename);
    } else {
        ROS_WARN_STREAM("Unable to save records to file: " << records_filename);
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
    ros::init(argc, argv, "hog_person_detection");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Hello World! This program detects people using the Histogram of Oriented Gradients (HOG) algorithm.");

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


