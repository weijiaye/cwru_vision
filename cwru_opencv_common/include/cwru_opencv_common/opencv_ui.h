
#include <opencv2/opencv.hpp>
#include <cstdio>

#include "cwru_opencv_common/opencv_local.h"

#ifndef CV_UI_H
#define CV_UI_H



namespace cv_ui{

	/*
		On left click:
			if param is null, print clicked co-ordinates
			otherwise, treat as cv::Point* and store clicked co-ordinates
	*/
	void getCoordinates(int event,int x,int y,int flags,void * param);

	

	/*
		On left click:
			Treat param as cv::Mat* and print its contents at clicked co-ordinates
	*/
	void displayPixel(int event,int x,int y,int flags,void * param);

	typedef std::vector< std::vector<cv::Point> > ContourList;


	void createContours(int event, int x, int y, int flags, void *param);
};


#endif

