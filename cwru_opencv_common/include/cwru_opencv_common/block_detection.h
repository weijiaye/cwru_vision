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


#ifndef BLOCKDETECTION_H
#define BLOCKDETECTION_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "cwru_opencv_common/opencv_local.h"


/** \brief FiducialBase is the base class for segmenting and fitting simple geometric shapes to image segmentation.
 *
 *         This class should have a simple user interface.
 *
 *
 */
/*class FiducialBase {
 public:

  FiducialBase::FiducialBase() {}
  //constructor; initialize with a Mat, threshold values, range of blob size in pixels, number of objects to detect, and display status
  FiducialBase(Mat image, std::vector<int> thresholdRange, std::vector<int> blobRange,int numObjects,bool displayOn);  
  Mat getImage();    // return the original image
  void setImage(Mat imageIn);   //Sets the image
  std::vector<int> getThresholdRange(); //return values to be used for thresholding
  std::vector<int> getBlobRange(); //return lob area pixel ranges to be used for detection
  int getNumObjects(); //return the number of objects to seek for in the search
  
  //Get and set the location of the centroid of the fidcuial(s)
  std::vector<Point2f> getCentroids();
  void setCentroids(std::vector<Point2f>);

  void ApproxRect();

  void setThresholdRange(std::vector<int> thresholdRangeIn);

  Mat thresholdImage(Mat*); 
  Mat locateContours(Mat);
  void locateBlobs(Mat, int);
  std::vector<Point2f> search();
  void displayStatus(bool);
  std::vector<Point> returnCorners();

  std::vector<Point> search2();

  int loadBlobFile(const char*);


protected:
  Mat image; //Original image
  Mat imageHSV; //HSVImage;
  std::vector <int> thresholdRange; //Chroma0 target, chroma1 target, chroma0 tolerance, chroma1 target
  std::vector <int> blobRange; //Blob size in pixels ie 160,180
  int numObjects; //Number of object to be detected


  std::vector<std::vector<Point>> contours; //Detected contours in the image.
  std::vector<Point2f> centroids; //Location of the fiducial(s) in the image

  bool displayOn;   //Turns on and off the display elements:


};*/


/** \breif growBlobBW uses iterative segmentation in order to identify a local black region on a white background.
 *
 *         This function is used for the BW ellipse segmentation.
 *         As well as quadrilateral segmentation.
 *
 * /
void growBlobBW(const Mat &,Mat &,Point2f); */


/** \breif growQuadrilateralBW uses iterative segmentation in order to identify a quadrilateral.
 *
 *         This function uses BW growth segmentation.
 *
 */
//void growQuadrilateralBW(const cv::Mat &inputImg, cv::Point2f startPt, std::vector<cv::Point2f> &corners);

/** \brief quadrilateralBW uses canny edge detection and the hough transform.
 *
 *         This function uses BW growth segmentation.
 *
 */
void detectBlock(const cv::Mat &, cv::Point, std::vector<cv::Point2f> &,bool=false);


int refineBlock(const cv::Mat&, std::vector<cv::Point2f> &,bool=false);


/** \brief sortPtGrid sorts the point array into a grid. (won't work that well).
 *
 *
 *
 */
void sortPtGrid(std::vector<cv::Point2f> &,cv::Size ,bool = true);


/** \brief exportPointFile(const char*,std::vector<std::vector<Point3f>> &) 
 *         save the point array of arrays.
 *
 *
 *
 */
void exportPointFile(const char*,std::vector< std::vector< cv::Point3f > > &);


#endif
