//
// Created by tipakorng on 6/1/16.
//

#include <cwru_opencv_common/manual_labeling.h>

ManualLabeling::ManualLabeling(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle) {
    manualLabelingServer_ = nodeHandle_.advertiseService("manual_labeling_service",
                                                         &ManualLabeling::manualLabelingCallback, this);
    ROS_INFO("Manual image labeling  server initialized");
}

ManualLabeling::~ManualLabeling() { }

bool ManualLabeling::manualLabelingCallback(cwru_opencv_common::image_label::Request& request,
                                            cwru_opencv_common::image_label::Response& response) {
    // Put sensor_msgs::Image in cv::Mat
    const sensor_msgs::Image::Ptr &image = boost::make_shared<sensor_msgs::Image>(request.imageQuery);
    cv::Mat rawImageIn = cv_bridge::toCvShare(image, "bgr8")->image.clone();
    // Declare labeled image
    cv::Mat labeledImage;
    // Apply adaptive threshold
    // create a click window:
    cv::Point imagePt(-1, -1);
    cv::namedWindow("Selectable Points");
    cv::setMouseCallback("Selectable Points", cv_ui::getCoordinates, &imagePt);
    
    response.pointsResp.points.clear();
    int imageCount(0);
    // fill the blobs.
    while (true)
    {
        imshow("Selectable Points", labeledImage);
        char keyIn = cv::waitKey(50);
        if (imagePt.x > 0)
        {
            geometry_msgs::Point32 localPt;
            localPt.x = static_cast<float> (imagePt.x);
            localPt.y = static_cast<float> (imagePt.y);
            localPt.z = 0.0;
            response.pointsResp.points.push_back(localPt);
            imageCount++;
            imagePt.x = -1;
        }
        // if the Esc key is pressed, break out.
        if (keyIn == 27 || imageCount >= request.requestedPoints) break;
    }
    cv::destroyWindow("Selectable Points");
    ROS_INFO("Finished the acquiring the point list. \n");

    // Merge blob to label image
    if (response.pointsResp.points.size() > 0)
    {
        return true;
    }
    else return false;
}
