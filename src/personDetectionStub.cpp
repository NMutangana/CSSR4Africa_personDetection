/*
    This program was developed as a stub to sink test the output data of the personDetection task.
    
    The output data will be obtained by subscribing to the following topics:
    - /personDetection/rgb_image,
    - /personDetection/depth_image, and 
    - personDetection/data.

    The output data will be displayed to the console

    Natasha Mutangana
    29th July 2023
*/

#include "personDetection/personDetection.h"

// Callback function for the /personDetectionData topic
void personDataCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    // Extract the data from the message
    int num_people = msg->layout.dim[0].size;
    std::vector<double> data = msg->data;

    // Display the information for each person
    ROS_INFO_STREAM("Total people detected: " << num_people);
    // for (int i = 0; i < num_people; i++) {
    //     // Uncomment this loop if using HOG
    //     double centroidX = data[i * 5];
    //     double centroidY = data[i * 5 + 1];
    //     double width = data[i * 5 + 2];
    //     double height = data[i * 5 + 3];
    //     double depth = data[i * 5 + 4];

    //     ROS_INFO_STREAM("Person " << i + 1 << ": [" << centroidX << "," << centroidY << "," << width << "," << height << "," << depth << "]");
    // }

    for (int i = 0; i < num_people; i++) {
        // This loop is used if using YOLO
        double person_ID = data[i * 7];
        double confidence = data[i * 7 + 1];
        double centroidX = data[i * 7 + 2];
        double centroidY = data[i * 7 + 3];
        double width = data[i * 7 + 4];
        double height = data[i * 7 + 5];
        double depth = data[i * 7 + 6];

        ROS_INFO_STREAM("Person " << i + 1 << ": [" << person_ID << "," << confidence << ","  << centroidX << "," << centroidY << "," << width << "," << height << "," << depth << "]");
    }
}

// Callback function for the /personDetectionRGBImage topic
void rgbImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // Convert the ROS image message to an OpenCV image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // Display the RGB image with bounding boxes
    cv::imshow("RGB Image with Bounding Boxes", cv_ptr->image);
    cv::waitKey(30);
}

// Callback function for the /personDetectionDepthImage topic
void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // Convert the ROS image message to an OpenCV image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);

    // Display the depth image with bounding boxes
    cv::imshow("Depth Image with Bounding Boxes", cv_ptr->image);
    cv::waitKey(30);
}

int main(int argc, char** argv) {
    // Initialize the ROS system and become a node
    ros::init(argc, argv, "person_detection_stub");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Hello World! This program receives output of the person detection task");

    // Subscribe to the /personDetectionData topic
    ros::Subscriber person_data_sub = nh.subscribe("/personDetection/data", 1000, personDataCallback);

    // Subscribe to the /personDetectionRGBImage topic
    ros::Subscriber rgb_image_sub = nh.subscribe("/personDetection/rgb_image", 1000, rgbImageCallback);

    // Subscribe to the /personDetectionDepthImage topic
    ros::Subscriber depth_image_sub = nh.subscribe("/personDetection/depth_image", 1000, depthImageCallback);

    // Create OpenCV windows for image display
    cv::namedWindow("RGB Image with Bounding Boxes", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Depth Image with Bounding Boxes", cv::WINDOW_AUTOSIZE);

    // Spin to receive and process ROS messages
    ros::spin();

    // Close OpenCV windows before exiting
    cv::destroyAllWindows();

    return 0;
}
