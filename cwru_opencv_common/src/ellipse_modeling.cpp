/* 
 * projective_geometry.h
 * Copyright 2014 Russell Jackson
 * All rights reserved.
 */

/*
 * The functions declared in this file are meant to handle the projective geometry of the camera image space and P^2 to p^3.
 */



#include "cwru_opencv_common/projective_geometry.h"

using namespace cv;
using namespace cv_local;

Mat ellipse2Mat(RotatedRect input)
{

    Mat conicMat(3,3,CV_64FC1);

    //generate the first stage of parameters.
    double a = input.size.width/2;
    double b = input.size.height/2;
    double xc = input.center.x;
    double yc = input.center.y;
    double theta = (double) input.angle*3.14159265359/180;

    double stheta = sin(theta);
    double ctheta = cos(theta);

    //compute the Polynomial coefficients.
    double A = a*a*stheta*stheta+b*b*ctheta*ctheta;
    double B = 2*(b*b-a*a)*stheta*ctheta;
    double C = a*a*ctheta*ctheta+b*b*stheta*stheta;
    double D = -2*A*xc-B*yc;
    double E = -B*xc-2*C*yc;
    double F = A*xc*xc+B*xc*yc+C*yc*yc-a*a*b*b;


    conicMat.at<double>(0,0) = A;
    conicMat.at<double>(1,1) = C;
    conicMat.at<double>(2,2) = F;

    conicMat.at<double>(0,1) = B/2;
    conicMat.at<double>(1,0) = B/2;

    conicMat.at<double>(2,0) = D/2;
    conicMat.at<double>(0,2) = D/2;

    conicMat.at<double>(2,1) = E/2;
    conicMat.at<double>(1,2) = E/2;

    return conicMat;
}


Mat findEllipseRotTransMat(RotatedRect ellipse, double radius, Mat intrinsics)
{

    Mat circleMat = Mat::eye(3,3,CV_64FC1);
    circleMat.at<double>(2,2) = -radius*radius;
	
	std::cout << std::endl;
	std::cout << circleMat;
	std::cout << std::endl;
    
	Mat ellipseMat = ellipse2Mat(ellipse);

	std::cout << std::endl;
	std::cout << ellipseMat;
	std::cout << std::endl;

    Mat rotTrans;

	std::cout << std::endl;
	std::cout << intrinsics;
	std::cout << std::endl;

    rotTrans = intrinsics.inv()*ellipseMat*circleMat.inv();

    Mat r0 = rotTrans.col(0);
    Mat r1 = rotTrans.col(1);

	std::cout << std::endl;
	std::cout << rotTrans;
	std::cout << std::endl;

    double n0 = norm(r0);
    double n1 = norm(r1);

    double nMean = sqrt(n0*n1);

    rotTrans *= 1/(nMean);

    return rotTrans;
}



stereoCorrespondence reprojectPointStereo(const Point3d &point, const Mat &P_l, const Mat &P_r)
{

        stereoCorrespondence output;
        Mat ptMat(4,1,CV_64FC1);
        Mat results(3,1,CV_64FC1);

        Mat stP[2];

        stP[0]= P_l;
        stP[1]= P_r;

        ptMat.at<double>(0,0) = point.x;
        ptMat.at<double>(1,0) = point.y;
        ptMat.at<double>(2,0) = point.z;
        ptMat.at<double>(3,0) = 1.0;

        for(int lr = 0; lr < 2; lr++){
            results = stP[lr]*ptMat;
            output[lr].x = results.at<double>(0,0)/results.at<double>(2,0);
            output[lr].y = results.at<double>(1,0)/results.at<double>(2,0);
        }

        return output;
}


stereoCorrespondence reprojectPointTangent(const cv::Point3d &point,const cv::Point3d &pointDeriv,const Mat & P_l , const Mat & P_r )
{

    stereoCorrespondence tempDeriv;
    stereoCorrespondence tempOutput;
    Mat ptDMat(4,2,CV_64FC1);
    Mat dResults(3,2,CV_64FC1);
    Mat stP[2];

    stP[0]= P_l;
    stP[1]= P_r;


    ptDMat.at<double>(0,0) = point.x;
    ptDMat.at<double>(1,0) = point.y;
    ptDMat.at<double>(2,0) = point.z;
    ptDMat.at<double>(3,0) = 1.0;
    ptDMat.at<double>(0,1) = pointDeriv.x;
    ptDMat.at<double>(1,1) = pointDeriv.y;
    ptDMat.at<double>(2,1) = pointDeriv.z;
    ptDMat.at<double>(3,1) = 0.0; //This is a vector and not a point...


    for(int lr = 0; lr < 2; lr++){
        dResults = stP[lr]*ptDMat;
        tempDeriv[lr].x = (dResults.at<double>(2,0)*dResults.at<double>(0,1)-dResults.at<double>(0,0)*dResults.at<double>(2,1))/(dResults.at<double>(2,0)*dResults.at<double>(2,0));
        tempDeriv[lr].y = (dResults.at<double>(2,0)*dResults.at<double>(1,1)-dResults.at<double>(1,0)*dResults.at<double>(2,1))/(dResults.at<double>(2,0)*dResults.at<double>(2,0));
    }
    return tempDeriv;
}



cv::Point3d deprojectStereoPoint(const cv_local::stereoCorrespondence inputPts,const  cv::Mat &P_l ,const cv::Mat &P_r)
{
    //construct the output mat:
    Mat results(4,1,CV_64FC1);
    Mat p_l(2,1,CV_64FC1);
    Mat p_r(2,1,CV_64FC1);

    p_l.at<double>(0) = inputPts[0].x;
    p_l.at<double>(1) = inputPts[0].y;

    p_r.at<double>(0) = inputPts[1].x;
    p_r.at<double>(1) = inputPts[1].y;

    triangulatePoints(P_l,P_r,p_l,p_r,results);

    Point3d output;

    output.x =  results.at<double>(0)/results.at<double>(3);
    output.y =  results.at<double>(1)/results.at<double>(3);
    output.z =  results.at<double>(2)/results.at<double>(3);

    return output;
}
