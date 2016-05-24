/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *    Russell Jackson <rcj33@case.edu>
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


#ifndef FIDUCIALCIRCLES_H
#define FIDUCIALCIRCLES_H
// Opencv Includes
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>


/*
 * This function generates an ellipse based on an initial seed point.
 */
cv::RotatedRect fillEllipseBW(const cv::Mat & inputImg, cv::Point seedPt);

namespace cv_circle
{



//Uses a single point to grow a list of ellipses
//rotatedRectStereoCorr growEllipse(const stereoImage &, stereoCorrespondence, Scalar &);


 /*
  * growEllipseImage: grows an ellipse from a single point and identifies the color region.
  */
//  RotatedRect growEllipseImage(const Mat & inputImgPair , Point2f startPt, Scalar &meanColor,Scalar &lowColor,Scalar &highColor );


 /*
  * growEllipseBW: grows a Black ellipse on a white background using region growth.
  */
//  RotatedRect growEllipseBW(const Mat &, Point2f);

  /** \brief void exportEllipseFile(const char*,std::vector<RotatedRect> &): This function exports a list of circles.
   *
   *
   */
//  void exportEllipseFile(const char*,std::vector<RotatedRect> &);





/*
 * This function optimizes an ellipse based on an initial seed ellipse.
 */
void optimizeEllipseBW(const cv::Mat & inputImg, cv::RotatedRect &ellipse, int padding);

}; //  cv_circle

#endif
