//
// Created by tipakorng on 6/1/16.
//

#ifndef CWRU_OPENCV_COMMON_MANUAL_LABELING_H
#define CWRU_OPENCV_COMMON_MANUAL_LABELING_H

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <cwru_opencv_common/opencv_local.h>
#include <cwru_opencv_common/opencv_ui.h>
#include <cv_bridge/cv_bridge.h>
#include <cwru_opencv_common/image_label.h>
#include <gtest/gtest.h>

/**
 * \brief This class is a manual labeling server.
 */
class ManualLabeling {

public:

    /**
     * \brief Constructor
     */
    ManualLabeling(ros::NodeHandle &nodeHandle);

    /**
     * \brief Destructor
     */
    ~ManualLabeling();

    /**
     * \brief Manual labeling callback function
     */
    bool manualLabelingCallback(cwru_opencv_common::image_label::Request& request,
                                cwru_opencv_common::image_label::Response& response);

protected:

    /**
     * \brief Grow blob from seed
     */
    int growFromSeedRaw(const cv::Mat& rawImage, cv::Mat& labeledImage, cv::Point2i seedPoint);

    /**
     * \brief Grow from seed
     */
    int growFromSeed(const cv::Mat& segmentedImage, cv::Mat& labeledImage, cv::Point2i seedPoint);

    /**
     * \brief Flood fill segmentation
     */
    int catheterFloodFillSegmentation(const cv::Mat & inputImage, cv::Mat &outputImage, cv::Point2i seedPoint, cv::Rect regionIn);

    /**
     * \brief ROS node handle
     */
    ros::NodeHandle nodeHandle_;

    /**
     * \brief Manual image labeling server
     */
    ros::ServiceServer manualLabelingServer_;

};

#endif //CWRU_OPENCV_COMMON_MANUAL_LABELING_H
