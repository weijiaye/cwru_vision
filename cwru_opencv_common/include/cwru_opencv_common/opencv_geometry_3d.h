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


/**
 * @brief functions and models required for 3d object detecion and tracking
 *
 */
#ifndef OPENCV_GEOMETRY_3D_H
#define OPENCV_GEOMETRY_3D_H


#include <opencv2/opencv.hpp>


namespace cv_3d{

    /* Three dimensional model structs */

    /**
     * @brief This is a sphere in 3d space
     */
    struct sphere{

        cv::Point3d center;
        double radius;

        // constructor
        sphere(const cv::Point3d &center_=cv::Point3d(0.0,0.0,0.0) , double radius_ = 1.0 ):
        center(center_),
        radius(radius_)
        {
        }

    };

    /**
     * @brief This is a cylinder in 3d space
     */
struct cylinder
{
    cv::Point3d center;
    double theta;
    double phi;
    double height;
    double radius;
    // constructor
    cylinder(const cv::Point3d& center_ = cv::Point3d(0.0, 0.0, 0.0), double theta_ = 0.0,
             double phi_ = 0.0, double height_ = 0.0, double radius_ = 0.0):
    center(center_),
    theta(theta_),
    phi(phi_),
    height(height_),
    radius(radius_)
    {
    }
};

    cylinder baseCylinder();


    void iterateSphere_3d(const cv::Mat & , sphere & , const cv::Mat& , const cv::Mat&);


    cv::Rect renderSphere(cv::Mat &,const sphere& , const cv::Mat &, cv::OutputArray = cv::noArray(),  cv::OutputArray = cv::noArray(), const cv::Scalar& = cv::Scalar(255, 255, 255)  );
    cv::RotatedRect renderCylinder(cv::Mat &,const cylinder&, const cv::Mat &,cv::OutputArray = cv::noArray(),  cv::OutputArray = cv::noArray(), const cv::Scalar& = cv::Scalar(255, 255, 255));

    double optimizeSphere(sphere &, const cv::Mat&, const cv::Mat&, const cv::Mat& , int, double, bool = false  );

    double optimizeCylinder(cylinder &, const cv::Mat&, const cv::Mat&, const cv::Mat& , int, double, bool = false );

    /**
     * @brief compute the mirror normal from theta and phi with an option jacobian.
     */
    cv::Point3d computeNormalFromSpherical(double , double , cv::OutputArray = cv::noArray());

    /**
     * @brief compute the theta and phi from the mirror normal
     */
    cv::Point2d computeSphericalFromNormal(cv::Point3d);
};

#endif
