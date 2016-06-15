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

#include "cwru_opencv_common/projective_geometry.h"

#include "cwru_opencv_common/ellipse_modeling.h"

using namespace cv;
using namespace cv_local;

namespace cv_ellipse_num
{


double circleEnergy(const cv::Mat &segmentedImage, cv::Mat &P, cv::Mat &G_co, cv::Point3d &center, double rad, int segments)
{
	// The circle energy is maximizd when the segmented image is dark where the circle is dark and the gradient around image
	// matches the circle's edge.
	Mat pt_o(4, 1, CV_64FC1);
	
	
	//create the cirlcle
	std::vector< std::vector<Point> > imagePts;
	
	imagePts.clear();
	imagePts.resize(1);
	Rect imageROI(-1, -1, 0, 0);
	Point oldPt(-1,-1);
	for (int ind(0); ind < segments; ind++)
	{
		// compute the point:
		pt_o.at<double>(0) = cos(ind*3.141/segments)*rad+center.x;
		pt_o.at<double>(1) = sin(ind*3.141/segments)*rad+center.y;
		pt_o.at<double>(2) = center.z;
		pt_o.at<double>(3) = center.z;
		
		Mat pt_c = G_co*pt_o;
		Point3d result;
		result.x = pt_c.at<double>(0);
		result.y = pt_c.at<double>(1);
		result.z = pt_c.at<double>(2);
		cv::Point2d ptLoc(reprojectPoint(result, P);
		
		cv::Point newPt(static_cast<Point> (ptLoc));
		
		if (norm(newPt-oldPt) > 0)	
		{
			imagePts[0].push_back(newPt);
			
			if (imageROI.x > 0)
			{
				imageROI |= Rect(newPt+Point(-5, -5), newPt+Point(5, 5));
			}
			else imageROI = Rect(newPt+Point(-5, -5), newPt+Point(5, 5));
		}
	}
	// now create a convex hull.
	Mat edgeImage(imageROI.height, imageROI.width, CV_8UC1);
	Mat fillImage(imageROI.height, imageROI.width, CV_8UC1);
	drawContours(edgeImage, imagePts, 0, Scalar(255, 255, 255), 3, 8, noArray(), INT_MAX, imageROI.tl()*-1);
	
	drawContours(fillImage, imagePts, 0, Scalar(255, 255, 255), -1, 8, noArray(), INT_MAX, imageROI.tl()*-1);
	
	//get the ROI of the base image.
	
	
	// make derivative magnitude images.
	Mat derivX, derivY;
	
	scharr()
	
	// use cross correlation for the fill match, as well as the edge match.
	
	return energy;
}

};  // namespace cv_ellipse_num
