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
    
    Mat colorStdDev;
    meanStdDev(sourceImage, static_cast<Mat> (colorMean), (colorStdDev), maskImage);
    void calcCovarMatrix(InputArray samples, OutputArray covar, OutputArray mean, int flags, int ctype=CV_64F);
    //Generate the full data.
}

ColorModel::colorModel(const Mat & sourceImage){
    
}

cv::Mat ColorModel::segmentImage(Mat &inputImage){
 
 
}

}; //namespace cv_color_model
