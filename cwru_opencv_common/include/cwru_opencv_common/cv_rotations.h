/*rotation_operations.h
 *Case Western Reserve Mechatronics Lab -- Professor Murat Cavusoglu
 *Author: Russell Jackson, Viraj Desai
 * 3/4/2015
 */

/*This file relies on the following external libraries:

OpenCV (2.3.1)
Eigen  (?????)
*/

//This file generates the function prototypes that will become common to multiple projects.
//The library will rely on opencv to define the basic math, but will be expanded to include
//other material, ie screws, twists, D-H params, and Quaternions.

#ifndef CV_ROTATIONS_H
#define ROBOTOPERATIONS_H


#include <opencv2/opencv.hpp>
//transformation Quaternion

namespace cv_rot{

typedef cv::Vec<double,7> transQuatTransform;

template <class T>  double computePointArrayError(cv::Point3_<T>  newPoint, int &index, cv::Point3_<T>* inputArray, int arrayCount){




	double errorOutput = 0.0;
	for(int i = 0; i < arrayCount; i++)
	{
		
		errorOutput+= (double) norm(newPoint-inputArray[i]);
		
	}
	index = (index+1)%arrayCount;
	inputArray[index] = newPoint;

	return errorOutput/((double) arrayCount);
};




struct Quaternion{

	///* data[i] = qi and v = (q1,q2,q3) *///
	double data[4];
	double operator[](int index)
	{
		return data[index];
	}
	Quaternion() {}
	Quaternion log() const
	{	
		///* log(q) =  log(q0)* + v/||v||*arccos( q0/||v|| )*///
		Quaternion Q;
		if (data[0] < 0)
		{			
			for (unsigned int i = 0; i < 4; i++)
			{
				Q.data[i] = -data[i];
			}			
		}
		else
		{
			for (unsigned int i = 0; i < 4; i++)
			{
				Q.data[i] = data[i];
			}			
		}
		double a;
		if (Q.data[0]/length() > 1 || Q.data[0]/length() < -1)
		{
			a = 0;
		}
		else
		{
			a = (double)acos(Q.data[0]/length());
		}
		double vlength = sqrt(pow(Q.data[1],2) + pow(Q.data[2],2) + pow(Q.data[3],2));
		//double sina = (double)sin(a);
		Quaternion ret;
		


		ret.data[0] = std::log(Q.data[0]);
		ret.data[1] = Q.data[1]/vlength*a;
		ret.data[2] = Q.data[2]/vlength*a;
		ret.data[3] = Q.data[3]/vlength*a;


		//if (sina > 0)
		//{
		//	ret.data[1] = a*data[0]/sina;
		//	ret.data[2] = a*data[1]/sina;
		//	ret.data[3] = a*data[2]/sina;
		//} else {
		//	ret.data[1] = ret.data[2] = ret.data[3] = 0;
		//}
		return ret;
	}
	Quaternion exp() const
	{
		///* exp(q) =  exp(q0)*(cos||v|| + v/||v||*sin||v||)*///
		double vlength = sqrt(pow(data[1],2) + pow(data[2],2) + pow(data[3],2));
		double a = (double)vlength;
		double sina = (double)sin(a);
		double cosa = (double)cos(a);
		Quaternion ret;

		double exp_term = std::exp(data[0]);
		ret.data[0] = exp_term*cosa;
		ret.data[1] = exp_term*sina/vlength*data[1];
		ret.data[2] = exp_term*sina/vlength*data[2];
		ret.data[3] = exp_term*sina/vlength*data[3];
		//if (a > 0)
		//{
		//	ret.data[1] = sina * data[1] / a;
		//	ret.data[2] = sina * data[2] / a;
		//	ret.data[3] = sina * data[3] / a;
		//} else {
		//	ret.data[1] = ret.data[2] = ret.data[3] = 0;
		//}
		return ret;
	}
	void conjugate()
	{ 
		data[1] = -data[1]; 
		data[2] = -data[2]; 
		data[3] = -data[3]; 
	}
	double length() const
	{
		return (float) sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3]); 
	}
	double length_squared() const
	{
		return (float)(data[0]*data[0] + data[1]*data[1] + data[2]*data[2] + data[3]*data[3]); 
	}
	const Quaternion operator /(double scale) const
	{
		Quaternion ret;
		for( unsigned int i = 0; i < 4; i++)
		{
			ret.data[i] /= scale;
		}
		return ret; 
	}
	const Quaternion &operator /= (double scale)			
	{
		for( unsigned int i = 0; i < 4; i++)
		{
			data[i] /= scale;
		}
		return *this; 
	}
	Quaternion invert() const
	{
		Quaternion ret;
		ret.data[0] = data[0]/length_squared();
		ret.data[1] = -data[1]/length_squared();
		ret.data[2] = -data[2]/length_squared();
		ret.data[3] = -data[3]/length_squared();
	 
		return ret;
	}
	Quaternion norm() const
	{
		Quaternion ret;
		ret.data[0] = data[0]/length();
		ret.data[1] = data[1]/length();
		ret.data[2] = data[2]/length();
		ret.data[3] = data[3]/length();
		return ret;
	}
	Quaternion neg() const
	{
		Quaternion ret;
		ret.data[0] = -data[0];
		ret.data[1] = -data[1];
		ret.data[2] = -data[2];
		ret.data[3] = -data[3];
	 
		return ret;
	}
	const Quaternion operator *(double scale) const
	{ 
		Quaternion ret;
		for( unsigned int i = 0; i < 4; i++)
		{
			ret.data[i] *= scale;
		}
		return ret;
	}
	const Quaternion operator *(const Quaternion &q) const	
	{	
		Quaternion ret;
		ret.data[0] = data[0]*q.data[0] - data[1]*q.data[1] - data[2]*q.data[2] - data[3]*q.data[3];
		ret.data[1] = data[2]*q.data[3] - data[3]*q.data[2] + data[0]*q.data[1] + data[1]*q.data[0];
		ret.data[2] = data[3]*q.data[1] - data[1]*q.data[3] + data[0]*q.data[2] + data[2]*q.data[0];
		ret.data[3] = data[1]*q.data[2] - data[2]*q.data[1] + data[0]*q.data[3] + data[3]*q.data[0];
		return  ret;
	}

	const Quaternion &operator +=(const Quaternion &q)		
	{ 
		for (unsigned int i = 0; i < 4; i++)
		{
			data[i] += q.data[i];
		}
		return *this; 
	}
	const Quaternion &operator -=(const Quaternion &q)		
	{ 
		for (unsigned int i = 0; i < 4; i++)
		{
			data[i] -= q.data[i];
		}
		return *this; 
	}
	Quaternion &operator =(const Quaternion &q)		
	{ 
		for (unsigned int i = 0; i < 4; i++)
		{
			data[i] = q.data[i];
		} 
		return *this; 
	}
	const Quaternion operator +(const Quaternion &q) const	
	{ 
		Quaternion ret;
		for (unsigned int i = 0; i < 4; i++)
		{
			ret.data[i] = data[i] + q.data[i];
		}
		return ret;
	}
	const Quaternion operator -(const Quaternion &q) const	
	{ 
		Quaternion ret;
		for (unsigned int i = 0; i < 4; i++)
		{
			ret.data[i] = data[i] - q.data[i];
		}
		return ret;
	}

};


cv::Mat Rot_xFun(double dbInput);//unit--degree
cv::Mat Rot_yFun(double dbInput);//unit--degree
cv::Mat Rot_zFun(double dbInput);//unit--degree

cv::Mat Ginv(cv::Mat matInput);

Quaternion MatrixToQuat(cv::Mat& matInput); //The matrix should either be a 3x3 or a 4x4. 
	//The matrix will be confirmed to be in SO(3)


cv::Mat QuatToMatrix(Quaternion);

Quaternion EulerXYZToQuat(cv::Point3d inputEXYZ);
cv::Point3d MatrixToEulerXYZ(cv::Mat inputMat);
cv::Mat EulerXYZToMatrix(cv::Point3d inputEXYZ);


Quaternion QuatNormalize(Quaternion inputQ);


cv::Mat InvRotMatrix(cv::Mat inputM);

cv::Vec3d QuatRotVec(Quaternion, cv::Vec3d);

cv::Point3d QuatToEulerXYZ(Quaternion inputQ);

double QuatError(Quaternion inputQ1,Quaternion inputQ2);

};

#endif


