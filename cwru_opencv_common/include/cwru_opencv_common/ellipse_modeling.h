/* 
 * projective_geometry.h
 * Copyright 2014 Russell Jackson
 * All rights reserved.
 */

/*
 * The functions declared in this file are meant to handle the projective geometry of the cameras
 */



#include <iostream>
#include <opencv2/opencv.hpp>

#include "cwru_opencv_common/opencv_local.h"

/**
 *  @brief ellipse2Mat takes a RotatedRect (ellipse) and generates a 3x3 Conic Mat.
 *
 * The input is an opencv RotatedRect, the output is a 3x3 CV_64FC1 matrix.
 * The matrix is generated based on the conic matrix representation (v^T)A_qv = 0.
 * The polynoimial is A x^2 + B xy +C^2 + Dx + Ey + F = 0;
 * A_q = [ A B/2 D/2; B/2 C E/2; D/2 E/2 F]
 * v = [x y 1]^T
 */
cv::Mat ellipse2Mat(cv::RotatedRect);


/** \brief findEllipseRotTransMat takes a detected ellipse from an image. 
 *  Then computes the 3x3 rotation translation [r1 r2 t]  matrix from it.
 *
 * The input is an opencv RotatedRect of the detected ellipse as well as the known radius of the original circle
 * The camera intrinsic matrix is the 3rd input.
 * The 3x3 output matrix has 2 normalized orthogonal rotation basis vectors as well as a translation vector.
 */
cv::Mat findEllipseRotTransMat(cv::RotatedRect, double, cv::Mat);

