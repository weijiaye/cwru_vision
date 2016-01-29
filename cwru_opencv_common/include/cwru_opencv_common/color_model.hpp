/*
 * color_model.hpp
 * Copyright 2016  Russell Jackson, All Rights Reserved
 */

/*
 * This file defines a class of color tracking model.
 * The model assumes that a color distribution is a gaussian in RGB space. 
 * The segmentation is simply a multivariate gaussian probability (the final result is normalized using the l_infinity norm).
 * A mask is used to generate the initial fit.
 */


#ifndef COLORMODEL_H
#define COLORMODEL_H  

#include <cv.h>


namespace cv_color_model{

class ColorModel{

public:
  explicit ColorModel(const cv::Mat & , const cv::Mat &); //uses a pre-defined mask
  explicit ColorModel(const ColorModel&);  // copy constructor;
  explicit ColorModel(const cv::Mat &); //manually defined mask.

  cv::Mat segmentImage(const cv::Mat &); 

  void printModelInfo();

  void floatMaskInit(const cv::Mat &, const cv::Mat&);
  void binaryMaskInit(const cv::Mat &, const cv::Mat&);

private:
    cv::Matx<float, 3, 3> colorVariance;
    cv::Matx<float, 3, 1> colorMean;
};

};  // namespace cv_color_model



#endif
