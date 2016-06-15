/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *    Russell C Jackson <rcj33@case.edu>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve Univeristy, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "cwru_opencv_common/projective_geometry.h"
#include "cwru_opencv_common/ellipse_modeling.h"


using namespace cv;


struct ManualLabelingTest : testing::Test {

    // member information:
    Mat P;
    Mat 

   
    ManualLabelingTest() {
        
    }

    ~ManualLabelingTest() {
        delete manualLabeling;
    }
};


TEST_F(ManualLabelingTest, testManualLabelingCallback) {
    // Import image
    
    // 1. create an ellipse image:
    // Plot the image 



    // 2. create an alternative image.


    std::string imagePathStr = ros::package::getPath("cwru_opencv_common") + "/test/test_images/test_4_comb_2016-02-01-135224-0000.raw";
    const char* imagePath = imagePathStr.c_str();
    cv::Mat imageCv;
    importRawImage(imagePath, imageCv);
    // Convert cv::Mat to sensor_msgs::Image
    sensor_msgs::Image image;
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageCv).toImageMsg(image);
    // Put image in service request
    cwru_opencv_common::image_label srv;
    srv.request.imageQuery = image;
    srv.request.requestedPoints = 10;
    // Call service
    manualLabeling->manualLabelingCallback(srv.request, srv.response);
    // Convert sensor_msgs::Mat back to cv::Mat
    cv_bridge::CvImagePtr cvImagePtr;
    cvImagePtr = cv_bridge::toCvCopy(srv.response.imageResp, sensor_msgs::image_encodings::BGR8);
    // Show image
    cv::namedWindow("Labeled image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Labeled image", cvImagePtr->image);
    cv::waitKey(0);

}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}