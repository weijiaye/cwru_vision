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
#include <vector>
#include "cwru_opencv_common/projective_geometry.h"
#include "cwru_opencv_common/ellipse_modeling.h"


using namespace cv;


struct ManualLabelingTest : testing::Test
{
    // member information:
    Mat P;
    Mat G;

    // tracked circle:
    Point3d circleCenter(0.0, 0.0, 1.0);
    double rad(0.2);


    //

    ManualLabelingTest():
    P(3, 4, CV_64FC1)
    {
    	// populate the projection Matrix:
    	P.setTo(0.0);
    	P.at<double>(0, 0) = 1000.0;
    	P.at<double>(1, 1) = 1000.0;
    	P.at<double>(0, 2) = 100.0;
    	P.at<double>(1, 2) = 100.0;
    	P.at<double>(2, 2) = 1.0;

        G = Mat::Eye(4, 4, CV_64FC1);
    }

    ~ManualLabelingTest() {
        delete manualLabeling;
    }
};


TEST_F(ManualLabelingTest, testManualLabelingCallback) {
    // Import image

    // 1. create an ellipse image:
    // create the image:
    Mat test_image(200, 200, CV8UC3);


    // project the base circle:
    std::vector< std::vector<Point> > ptList;
    ptList.resize(1);
    ptList[0].clear();
    cv::Rect circleRect(projectCirclePoints(ptList[0], P, G, circleCenter, rad));

    drawContours(test_image, ptList, 0, Scalar(255, 255, 255), -1, 8, noArray(), INT_MAX, imageROI.tl()*-1);

    cv::namedWindow("circle image", cv::WINDOW_AUTOSIZE);
    cv::imshow("circle image", test_image);
    cv::waitKey(0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}