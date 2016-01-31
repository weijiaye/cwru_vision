/* 
* opencv_geometry_3d.h:
* 
* Created By Russell Jackson
* 10/19/2012
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
    radius(radius_),
    {
    }
};

    cylinder baseCylinder();


    void iterateSphere_3d(const cv::Mat & , sphere & , const cv::Mat& , const cv::Mat&);


    cv::Rect renderSphere(cv::Mat &,const sphere& , const cv::Mat &, cv::OutputArray = cv::noArray(),  cv::OutputArray = cv::noArray()  );
    cv::RotatedRect renderCylinder(cv::Mat &,const cylinder&, const cv::Mat &,cv::OutputArray = cv::noArray(),  cv::OutputArray = cv::noArray());

    void optimizeSphere(sphere &, const cv::Mat&, const cv::Mat&, const cv::Mat& , int, double, bool = false  );

    void optimizeCylinder(cylinder &, const cv::Mat&, const cv::Mat&, const cv::Mat& , int, double, bool = false );

    /**
     * @brief compute the mirror normal from theta and phi with an option jacobian.
     */
    cv::Point3d computeNormalFromSpherical(double , double , cv::OutputArray=cv::noArray());

};

#endif
