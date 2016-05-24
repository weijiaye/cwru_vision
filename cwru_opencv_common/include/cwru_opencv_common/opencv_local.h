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


/*
	This header file includes structs that are used thoughout the rest of the cwru_opencv_common library.
*/
#ifndef _OPENCV_LOCAL_H
#define _OPENCV_LOCAL_H


#include <opencv2/opencv.hpp>
#include <vector>

namespace cv_local
{


/* Stereo Storage structs */

//Storage struct for  stereo camera points
struct stereoCorrespondence{
	cv::Point2f left;
	cv::Point2f right;

	cv::Point2f& operator[](int i) {
		assert(i==0 || i==1);
		return i==0? left : right;
	}
	
	const cv::Point2f& operator[](int i) const {
		assert(i==0 || i==1);
		return i==0? left : right;
	}
	
	stereoCorrespondence& operator = (const stereoCorrespondence & in_)
	{
		left = in_.left;
		left = in_.right;
	}

};


typedef std::vector< std::vector< cv::Point> > ContourList;

struct rotatedRectStereoCorr
{
    cv::RotatedRect rectLeft;
    cv::RotatedRect rectRight;

    cv::RotatedRect& operator[](int i)
    {
        assert(i == 0 || i == 1);
        return i == 0? rectLeft : rectRight;
    }

    const cv::RotatedRect& operator[](int i) const
    {
        assert(i == 0 || i == 1);
        return i == 0 ? rectLeft : rectRight;
    }
};

struct rectStereoCorr
{
    cv::Rect rectLeft;
    cv::Rect rectRight;

    cv::Rect& operator[](int i)
    {
        assert(i == 0 || i == 1);
        return i == 0 ? rectLeft : rectRight;
    }

    const cv::Rect& operator[](int i)  const
    {
        assert(i == 0 || i == 1);
        return i == 0? rectLeft : rectRight;
    }
};

struct stereoImage
{
    cv::Mat view[2];

    cv::Mat& left()
    {
        return view[0];
    }
    cv::Mat& right()
    {
        return view[1];
    }

    cv::Mat& operator[](int i)
    {
        assert(i == 0 || i == 1);
        return view[i];
    }

    const cv::Mat& operator[](int i) const
    {
        assert(i == 0 || i == 1);
        return view[i];
    }
};

/* End of Stereo Storage */


/*
 * circleTracker3d defines a planar circle in 3 space. 
 * The vector zN is the outward normal to the circle surface.
 * The vector zN_Alt is the inward normal to the circle surface. (used for deprojection)
 * errorXY is the error (measured as the inner product between the x and y vectors after deprojection)
 * this is idealy 0.
 */
struct circleTracker3d
{
    cv::Point3d center;
    cv::Point3d zN;
    cv::Point3d zN_Alt;
    double rad;
    double errorXY;
};

/*
 * objectCircles3d defines a list of object circles
 * The vector colorList
 */
struct objectCircles3d
{
    std::vector< std::vector<circleTracker3d> > circleList;
    std::vector<cv::Scalar> colorList;
};



//  This function compute the error in hue
int byteError(int, int);

void defaultWindowLocation(int &lx, int &ly, int &rx, int &ry);


template<typename _Tp, typename _Tp2> inline cv::Point3_<_Tp2> normalizePt3(const cv::Point3_<_Tp>& inputPt)
{
    _Tp normI = 1/norm(inputPt);
    cv::Point3_<_Tp2> output = cv::Point3_<_Tp2>(inputPt*normI);
    return output;
}

bool contourCompare(std::vector< cv::Point> contour1, std::vector<cv::Point> contour2);


int countOutputChannels(int);

};  //  cv_local

#endif
