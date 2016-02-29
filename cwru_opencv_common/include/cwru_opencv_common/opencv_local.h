/* 
* opencv.h:
* 
* Created By Russell Jackson & Connor Balin
* 10/19/2012
*/


/*
	This header file includes all the necessary code to use the opencv library
	and defines several callbacks for use with highgui
*/
#ifndef _OPENCV_LOCAL_H
#define _OPENCV_LOCAL_H


// #define PI 3.14159265359

#include <opencv2/opencv.hpp>


namespace cv_local{


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
	
};


typedef std::vector< std::vector< cv::Point> > ContourList;


struct rotatedRectStereoCorr{
	
	
    cv::RotatedRect rectLeft;
    cv::RotatedRect rectRight;
	
    cv::RotatedRect& operator[](int i){
		assert(i==0 || i ==1);
		return i==0? rectLeft : rectRight;
	}

    const cv::RotatedRect& operator[](int i) const  {
        assert(i==0 || i ==1);
        return i==0? rectLeft : rectRight;
    }


};

struct rectStereoCorr{
	
	
    cv::Rect rectLeft;
    cv::Rect rectRight;
	
    cv::Rect& operator[](int i){
		assert(i==0 || i ==1);
		return i==0? rectLeft : rectRight;
	}

    const cv::Rect& operator[](int i)  const{
		assert(i==0 || i ==1);
		return i==0? rectLeft : rectRight;
	}


};



struct stereoImage{
	cv::Mat view[2];

	cv::Mat& left(){
		return view[0];
	}
	cv::Mat& right(){
		return view[1];
	}

	cv::Mat& operator[](int i){
		assert(i==0 || i==1);
		return view[i];
	}

	const cv::Mat& operator[](int i) const{
		assert(i==0 || i==1);
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
struct circleTracker3d{
	
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
struct objectCircles3d {
	
	std::vector< std::vector<circleTracker3d> > circleList;
	std::vector<cv::Scalar> colorList;

};



//This function compute the error in hue
int byteError(int,int);

void defaultWindowLocation(int &lx, int &ly, int &rx, int &ry);



template<typename _Tp,typename _Tp2> inline cv::Point3_<_Tp2> normalizePt3(const cv::Point3_<_Tp>& inputPt)
{
	_Tp normI = 1/norm(inputPt);

    cv::Point3_<_Tp2> output =cv::Point3_<_Tp2>(inputPt*normI);

	return output;

}

bool contourCompare(std::vector< cv::Point> contour1, std::vector<cv::Point> contour2);



int countOutputChannels(int);

};

#endif
