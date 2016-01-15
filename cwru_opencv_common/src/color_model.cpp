/*
 * color_model.cpp
 * Copyright 2016  Russell Jackson, All Rights Reserved
 */
 
/*
 * This file defines a class of color tracking model.
 * The model assumes that a color distribution is a gaussian in RGB space. 
 * The segmentation is simply a multivariate gaussian probability (the final result is normalized using the l_infinity norm).
 * A mask is used to generate the initial fit.
 */


#include <cwru_opencv_common/color_model.h>

using cv::Mat;

namespace cv_color_model{

ColorModel::colorModel(const Mat & sourceImage, const cv::Mat maskImage){
    // To obtain a full covariance matrix, use the calcCovar
    int nonZero(countNonZero(maskImage));
    Mat samplesMat(1,nonZero,sourceImage.type());
    // For now, a really slow painful form is implemented.
    int nonZeroIndex(0); 
    for ( int i(0); i < maskImage.rows*maskImage.cols; i++)
    {
        if (maskImage.at<uchar>(i) > 0 )
        {
             samplesMat.at<vec3b>(nonZeroIndex) = sourceImage.at<vec3b>(i);
        }
    }
    Mat covar,mean;
    calcCovarMatrix(samplesMat,covar,  mean, CV_COVAR_NORMAL);
    //Generate the full data.
    //assign it to the mean and covariance.
}

cv::Mat ColorModel::segmentImage(Mat &inputImage){
 
 Mat result(inputImage.size(),CV_32FC1);
 float maxResult = -1;
 Matx<3,1>
 for (int i(0); i < inputImage.rows*imputImage.cols)
 {
     // Fill this line in with the covariant probability density function.
     // Available: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    Vec3b  tempVec(sourceImage.at<Vec3b>(i));
    
 }
 //normalize result
 result = result/-1;
 return result;
}

}; //namespace cv_color_model
