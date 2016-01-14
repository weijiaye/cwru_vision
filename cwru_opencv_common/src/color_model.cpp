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

}

ColorModel::colorModel(const Mat & sourceImage){
    
}

cv::Mat ColorModel::segmentImage(Mat &); 

}; //namespace cv_color_model
