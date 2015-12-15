/*This file relies on the following external libraries
OpenCV (2.3.1)
PGRflyCapture (2.3.3.18)
*/


#include "cwru_opencv_common/circle_detection.h"

#include <vector>


/*bool pointYCompare(Point2f pt1, Point2f pt2)
{
    //Right now the comparison is number of points
    //A beeter comparison might be area:

    //Descending size:
    if(pt1.y >  pt2.y) return true;
    else return false;

}*/

using cv::RotatedRect;
using cv::Mat;
using cv::Point;
using cv::Point2f;
using cv::Point2d;
using cv::Scalar;
using cv::Size2f;
using cv::MORPH_ELLIPSE;
using cv::MORPH_OPEN;
using cv::Size;
using cv::FLOODFILL_FIXED_RANGE;
using cv::FLOODFILL_MASK_ONLY;

using cv::Range;


cv::RotatedRect fillEllipseBW(const cv::Mat & inputImg, cv::Point seedPt)
{
    std::vector< std::vector<Point> > contours;


    if (seedPt.x > inputImg.cols || seedPt.y > inputImg.rows || seedPt.x < 0 || seedPt.y < 0)
    {
        return RotatedRect(Point2f(-1, -1), Size2f(-1, -1), 0);
    }

    int newMaskVal = 255;
    Scalar newVal = Scalar(120, 120, 120);

    int connectivity = 8;
    int flags = connectivity | (newMaskVal << 8) | FLOODFILL_FIXED_RANGE | FLOODFILL_MASK_ONLY;

    int lo = 20;
    int up = 10;

    Mat mask2 = Mat::zeros(inputImg.rows + 2, inputImg.cols + 2, CV_8UC1);
    floodFill(inputImg, mask2, seedPt, newVal, 0, Scalar(lo, lo, lo), Scalar(up, up, up), flags);
    Mat mask = mask2(Range(1, mask2.rows - 1), Range(1, mask2.cols - 1));


    Mat elementOpen = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    Mat elementClose = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));

    Mat openMask;
    Mat closeMask;

    morphologyEx(mask, openMask, MORPH_OPEN, elementOpen);
    //morphologyEx(openMask, closeMask, MORPH_CLOSE, elementClose);

    //outputImg = closeMask.clone();

/*#ifdef DEBUGCATHSEG
    //imshow( "Open Mask", openMask );
    //imshow( "Close Mask", closeMask );

    //waitKey(0);

    //destroyWindow("Open Mask");
    //destroyWindow("Close Mask");

#endif*/

    findContours(openMask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    if(contours[0].size() > 5)
    {
        RotatedRect outputRect=fitEllipse(Mat(contours[0]));
        return outputRect;
    }

    RotatedRect outputRect(Point2f(-1,-1),Size2f(-1,-1),0);
    return outputRect;

}


void ellipseError(const RotatedRect & originalEllipse, const RotatedRect & newEllipse, Point2d& offset , double& angle)
{
    //compute the 4 points.

    Point2f origList[4];
    Point2f newList[4];

    originalEllipse.points(origList);
    newEllipse.points(newList);

    /*
     * @todo Finish this function
     */

}

/*bool RotatedRectYCompare(RotatedRect ep1, RotatedRect ep2)
{
	//Right now the comparison is number of points
	//A beeter comparison might be area:
	

	//Descending size:
	if(ep1.center.y >  ep2.center.y) return true;
	else return false;

}*/

//Prune matches from the cross matching vector:
//pass in the pruned vectors by reference
/*void pruneMatches(std::vector<std::vector<int>> &matches_CT, std::vector<std::vector<int>> &matches_TC, int ind_C, int ind_T){

	//Once a new point is appended, the match list must be culled to remove the matching points:
	
	//1. remove all references to tool ind_T from matches_CT
	int cullTC = matches_TC[ind_T].size();
	for(int j = 0; j < cullTC; j++){
		int removeC  = matches_TC[ind_T][j];
		int kLim = matches_CT[removeC].size();
		for(int k = 0; k < kLim; k++){
			if(matches_CT[removeC][k] == ind_T){
				matches_CT[removeC].erase(matches_CT[removeC].begin()+k);
				break;
			}
		}
	}

	//2. erase the entry of matches_TC at ind_T
	matches_TC[ind_T].clear();

	//3. remove all references to camera ind_C from matches_TC.
	int cullCT = matches_CT[ind_C].size();
	for(int j = 0; j < cullCT; j++){
		int removeT  = matches_CT[ind_C][j];
		int kLim = matches_TC[removeT].size();
		for(int k = 0; k < kLim; k++){
			if(matches_TC[removeT][k] == ind_C){
				matches_TC[removeT].erase(matches_TC[removeT].begin()+k);
				break;
			}
		}
	}

	//4. erase the entry of matches_CT at ind_C.
	matches_CT[ind_C].clear();
}




//Constructor function:
FiducialCircles::FiducialCircles(Mat imageIn,std::vector<int> markerThreshold, std::vector<int> markerBlobsize,int fiducialCount,bool display=false)
{
	this->image=imageIn;
	this->thresholdRange = markerThreshold;
	this->blobRange = markerBlobsize;
	this->numObjects = fiducialCount;
	this->displayOn=display;
	
	imageMatchedColorStatus = -1;

	
}

//Constructor function:
FiducialCircles::FiducialCircles(stereoImage imagesIn,std::vector<int> markerThreshold, std::vector<int> markerBlobsize,int fiducialCount,bool display=false)
{
	this->images=imagesIn;
	this->thresholdRange = markerThreshold;
	this->blobRange = markerBlobsize;
	this->numObjects = fiducialCount;
	this->displayOn=display;

	imageMatchedColorStatus = -1;
}


//matching function:
// finds thebest match of a Camera point from a list of camera points and
// a specific Tool point:
int FiducialCircles::bestMatch_C(std::vector<int> matchIndi_C, std::vector<int> matchIndi_T, std::vector<int> options_C, int pt_Ti, double &errorBest){


	double* distances_T;

	int matchCount = matchIndi_T.size(); 

	distances_T = new double [matchCount];
	
	std::vector<double> errorVals;
	errorVals.clear();
	
		//Find the distances to the index Tool point from matched points.
	for(int i  = 0; i < matchCount; i++){

		int indTemp_T = matchIndi_T[i];
		Point3d distV = circleList_T[indTemp_T].center - circleList_T[pt_Ti].center; 
		distances_T[i] = norm(distV);

	}

	//errorBest
	double errorMinTC = 2;
	int bestIndC = -1;
	int bestjTC  = -1;

	int optSize = options_C.size();
	for(int j = 0; j < optSize; j++)	
	//Run through potential matches on the camera and find the best one!!
	//(based on distance)
	{
		int indMatch = options_C[j];
		Point3d final_C = circleList_C[indMatch].center;

		double errorSum = 0;
		bool faceNorm = true;
		for(int k = 0; k < matchIndi_T.size(); k++)
		{
			int indTemp_C = matchIndi_C[k];
			errorSum = errorSum+abs(norm(final_C-circleList_C[indTemp_C].center)-distances_T[k]);									 
		}
		double error = errorSum/((double)matchCount);
		//double dotError = 
		errorVals.push_back(error);
		if(error < 1 && errorMinTC < 1){
			bestIndC = -1;
			bestjTC = -1;
			errorMinTC = 0.0;
		}
		if(error < errorMinTC)
		{
			errorMinTC = error;
			bestIndC = indMatch;
			bestjTC = j;
						
		}
	}
	if(errorVals.size() > 1){
		sort(errorVals.begin(),errorVals.end()); //sorts the error values:

		double improvement = abs(errorVals[1]-errorVals[0]);

		if(improvement < 1.5){ //errors are too close to call:
			bestIndC = -1;
		}
	}
		errorBest = errorMinTC;
		delete [] distances_T;

	if(errorBest < 2) return bestIndC;
	else return -1;

}


int FiducialCircles::bestMatch_T(std::vector<int> matchIndi_C, std::vector<int> matchIndi_T, int pt_C, std::vector<int> options_T,double &errorBest){

	double* distances_C;

	int matchCount = matchIndi_T.size(); 

	std::vector<double> errorVals;
	errorVals.clear();

	distances_C = new double [matchCount];

	for(int i  = 0; i < matchCount; i++){

		int indTemp_C = matchIndi_C[i];
		Point3d distV = circleList_C[indTemp_C].center - circleList_C[pt_C].center; 
		distances_C[i] = norm(distV);

	}

	//errorBest
	double errorMinCT = 10;
	int bestIndT = -1;
	int bestjCT  = -1;

	int optSize = options_T.size();
	for(int j = 0; j < optSize; j++)	
	//Run through potential matches on the camera and find the best one!!
	//(based on distance)
	{
		int indMatch = options_T[j];
		Point3d final_T = circleList_T[indMatch].center;

		double errorSum = 0;
		bool faceNorm = true;
		for(int k = 0; k < matchCount; k++)
		{
			int indTemp_T = matchIndi_T[k];
			errorSum = errorSum+abs(norm(final_T-circleList_T[indTemp_T].center)-distances_C[k]);									 
		}
		double error = errorSum/((double)matchCount);
		//double dotError = 
		errorVals.push_back(error);
		
		if(error < 1 && errorMinCT < 1){
			bestIndT = -1;
			bestjCT = -1;
			errorMinCT = 0.0;
		}
		if(error < errorMinCT)
		{
			errorMinCT = error;
			bestIndT = indMatch;
			bestjCT = j;
						
		}
	}
	
	if(errorVals.size() > 1){
		sort(errorVals.begin(),errorVals.end()); //sorts the error values:

		double improvement = abs(errorVals[1]-errorVals[0]);

		if(improvement < 1.5){ //errors are too close to call:
			bestIndT = -1;
		}
	}


	errorBest = errorMinCT;
	delete [] distances_C;

	if(errorBest < 10) return bestIndT;
	else return -1;

}



FiducialCircles::FiducialCircles(stereoImage imagesIn, const char * fileName,bool display = false){
	this->images=imagesIn;
	this->displayOn=display;
	this->loadFidFile(fileName);

	imageMatchedColorStatus = -1;

}



Scalar FiducialCircles::CircleColorMean(Mat imageIn, RotatedRect circleIn){
//This private function finds the mean of a color space and rounds it to an integer for a cv scalar.
	Mat imgMask(imageIn.size(),CV_8UC1,Scalar(0));
	ellipse(imgMask,circleIn,Scalar(255),-1);
	Scalar scalarOut =  mean(imageIn,imgMask);
	return scalarOut;

}


Scalar FiducialCircles::CircleHaloColorMean(Mat imageIn, RotatedRect circleInside,RotatedRect circleOutside){
//This private function finds the mean of a color space and rounds it to an integer for a cv scalar.
	Mat imgMaskS(imageIn.size(),CV_8UC1,Scalar(0));
	Mat imgMaskA(imageIn.size(),CV_8UC1,Scalar(0));
	Mat imgMaskB(imageIn.size(),CV_8UC1,Scalar(0));
	


	ellipse(imgMaskA,circleOutside,Scalar(255),-1);
	ellipse(imgMaskB,circleInside,Scalar(255),-1);
	bitwise_xor(imgMaskA,imgMaskB,imgMaskS);

	//Display the options:
	/*imshow("outerCircle",imgMaskA);
	imshow("innerCircle",imgMaskB);
	imshow("netHalo",imgMaskS);
	waitKey(0);

	cvDestroyWindow("outerCircle");
	cvDestroyWindow("innerCircle");
	cvDestroyWindow("netHalo"); * /

	Scalar scalarOut =  mean(imageIn,imgMaskS);
	return scalarOut;

}


std::vector<Scalar> FiducialCircles::calcCircleColorMean(Mat imageIn, std::vector<RotatedRect> circlesIn){

	std::vector<Scalar> colorVectOut;
	
	int nC = circlesIn.size();
	colorVectOut.resize(nC);


	for(int i = 0; i < nC; i++){
		colorVectOut[i] = CircleColorMean(imageIn,circlesIn[i]);
	}

	return colorVectOut;


}


void FiducialCircles::drawColorList(){

	Mat displayOutput;
	for(int j = 0; j < 2; j++){
		
	displayOutput = this->images[j].clone();
	for(unsigned int i = 0; i < locatedStereoCircles.size(); i++)
	{
		unsigned int a = ((i)*70)%255;
		unsigned int b = ((i+1)*70)%255;
		unsigned int c = ((i+2)*70)%255;
		char number[10];
		sprintf(number,"%d",i);
		string outputTxt;
		outputTxt.assign(number);
		ellipse(displayOutput,locatedStereoCircles[i][j],Scalar(a,b,c),-1);
		putText(displayOutput, outputTxt, locatedStereoCircles[i][j].center,FONT_HERSHEY_SIMPLEX, 1.5, Scalar(255,255,255));
	}	
	imshow("ContourDisplay",displayOutput);
	waitKey(0);
	cvDestroyWindow("ContourDisplay");
	}


}

//This function thresholds out the colors.
Mat FiducialCircles::ThresholdCircleImage()
{
	//Step 1: Use the base fiducial thresholding:
	Mat localThreshold= thresholdImage(NULL);



	//Dilate the white image:
		int kSizeErode;
		int kSizeDilate;
		Size imgSize = localThreshold.size(); 
		if(imgSize.width >  650){
			kSizeErode = 3;
			kSizeDilate = 5;
		}
		else{
			kSizeErode = 5;
			kSizeDilate = 3;
		}

		//Perform some erosion before dilation:
		Mat elementErode = getStructuringElement( MORPH_RECT, Size( kSizeErode, kSizeErode ));
		Mat erodeOutput;
		erode(localThreshold, erodeOutput, elementErode);


		Mat elementDilate = getStructuringElement( MORPH_RECT, Size( kSizeDilate, kSizeDilate ));
		Mat dilOutput;
		dilate( erodeOutput, dilOutput, elementDilate,Point(-1,-1),3);




		//Subtract white space and dark spaces:
		Mat colorLess;
		Mat Dark;
		Mat colorLight;
		Mat colorLessDark;
		
		//TODO: remove hard coding?
		//HARDCODE
		inRange(imageHSV,Scalar(0,0,0),Scalar(255,70,255),colorLess);
		inRange(imageHSV,Scalar(0,0,0),Scalar(255,255,20),Dark);

		bitwise_or(colorLess,Dark,colorLessDark);
		bitwise_not(colorLessDark,colorLight);

		Mat finalThresh;

		bitwise_and(dilOutput,colorLight,finalThresh);


	//Mat dilOutput;

	//Step 2: Use dilation to help expand the white area: (allows for using
	//Tighter threshold values

	//Mat element = getStructuringElement( MORPH_RECT, Size( 2 + 1, 2+1 ),Point( 1, 1 ) );

	//dilate( localThreshold, dilOutput, element );


	if(this->displayOn)
	{
		imshow("Final_HSV 0",localThreshold);
		imshow("Final_HSV 1",finalThresh);
		waitKey(0);
		cvDestroyWindow("Final_HSV 0");
		cvDestroyWindow("Final_HSV 1");
	}
	return finalThresh;
}


std::vector<Point2f>  FiducialCircles::locateContourCircles(Mat Thresholded){

		//They do....
		//
		
		vector<RotatedRect> boxes;

		boxes = locateCircles(Thresholded);

		vector<Point2f> output;	
		output.clear();
		for(unsigned int i = 0; i < boxes.size(); i++)
		{
			output.push_back(boxes[i].center);
		}
		//Right here...consider sorting the information;

		sort(output.begin(),output.end(),pointYCompare);
	
		return output;	
	
}


std::vector<RotatedRect> FiducialCircles::locateCircles(Mat Thresholded){

	vector<vector<Point>> contours,contoursOut;

	vector<RotatedRect> boxes;
	
	findContours(Thresholded,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
	

	/*Once the contours are found, iterate throught them for two goals:
	1. verify the circuliarity of the contour:
	2. Only consider contours of a certain size:
	* /
	
	//sort the contours by size
	sort(contours.begin(),contours.end(),contourCompare);
	
	//Only use the 
	contoursOut.clear();	
	for(unsigned int j = 0; j < contours.size(); j++){//iterate contours
		
		//Stop the loop after the circles have been found:
		//This flag was removed. The elements must be filtered later
		//if(contoursOut.size() >= (unsigned int) numObjects) break;
		
		//the size filter may be added here:
		//TODO: What is a good size filter?
		if(contours[j].size()  > 25){
			double p = arcLength(contours[j],true);
			double a = contourArea(contours[j]);
			double circularity = 4 * CV_PI * a / (p * p);
			if(circularity > 0.4)	//not too eccentric to allow
			{
				contoursOut.push_back(contours[j]);
			}	
		}
	} // for(unsigned int j = 0; j < contours.size(); j++)
	
	//draw the output contours:
	if(displayOn)
	{
		Mat displayOutput;
		displayOutput = this->image.clone();
		drawContours(displayOutput,contours,-1,Scalar(255,0,0),-1);
		if(contoursOut.size() > 0)
		{
			drawContours(displayOutput,contoursOut,-1,Scalar(0,0,255),-1);
		}
		imshow("ContourDisplay",displayOutput);
		waitKey(0);
		cvDestroyWindow("ContourDisplay");
	}


	//fit ellipses to all of the circles:
		boxes.clear();
		for(unsigned int i = 0; i < contoursOut.size(); i++)
		{
			boxes.push_back(cv::fitEllipse(Mat(contoursOut[i])));
			//Append the color info...?

		}
		//Now that the boxes are generated, sort them by y position.
		sort(boxes.begin(),boxes.end(),RotatedRectYCompare);
		//Now verify the boxes look right....
		if(displayOn)
		{
			Mat displayOutput;
			displayOutput = this->image.clone();
			
			
			for(unsigned int i = 0; i < boxes.size(); i++)
			{
				unsigned int a = ((i)*70)%255;
				unsigned int b = ((i+1)*70)%255;
				unsigned int c = ((i+2)*70)%255;
				char number[10];
				sprintf(number,"%d",i);
				string outputTxt;
				outputTxt.assign(number);
				ellipse(displayOutput,boxes[i],Scalar(a,b,c),-1);
				putText(displayOutput, outputTxt, boxes[i].center,FONT_HERSHEY_SIMPLEX, 1.5, Scalar(255,255,255));
			}	
			imshow("ContourDisplay",displayOutput);
			waitKey(0);
			cvDestroyWindow("ContourDisplay");
		}
	
		return boxes;	
}


std::vector<Point2f> FiducialCircles::searchImage()
{


	std::vector<Point2f> outputVect;
	Mat thresholdImage;
	thresholdImage = ThresholdCircleImage();
	
	outputVect= locateContourCircles(thresholdImage);


	return outputVect;
}



void FiducialCircles::setToolEstimate(Point3d transEst, Quaternion QuatEst){


//TODO	
//Turn  the parameters into a HomoGeneous Transformation FrameTransform3D:
//At this point move the frame transform stuff into the robotGeometry.

//gEstSet = true;


}
 




std::vector<stereoCorrespondence> FiducialCircles::searchStereoImage(stereoImage* imagesIn=NULL)
{


	if(imagesIn != NULL)
	{
		this->images[0] = imagesIn[0][0].clone();	
		this->images[1] = imagesIn[0][1].clone();
	}
	
	//Intermediary data structures:
	std::vector<Point2f> outputVectL;
 	std::vector<Point2f> outputVectR;
	
	std::vector<stereoCorrespondence> outputStereoCorr;
	Mat thresholdImageL;
	Mat thresholdImageR;
	image = images[0];
	thresholdImageL = ThresholdCircleImage();
	imagesHSV[0] = imageHSV; 
	outputVectL = locateContourCircles(thresholdImageL);
	
	image = images[1];
	thresholdImageR = ThresholdCircleImage();
	imagesHSV[1] = imageHSV;
	outputVectR = locateContourCircles(thresholdImageR);
	
	outputStereoCorr.clear();
	if(outputVectL.size() == outputVectR.size())
	{
		for(unsigned int i = 0; i < outputVectL.size(); i++)
		{
			stereoCorrespondence tempPoint;
			tempPoint[0] = outputVectL[i];
			tempPoint[1] = outputVectR[i];
			outputStereoCorr.push_back(tempPoint);
		}
	}



	return outputStereoCorr;
}


int FiducialCircles::searchStereoImageEllipse(stereoImage* imagesIn, const StereoCameraModel& modelIn )
{
	//This function finds the ellipses in both the left and right images, then aligns them in order to create a set of colored circles in 3d.
	int foundAligned = 0;

	if(imagesIn != NULL)
	{
		this->images[0] = imagesIn[0][0].clone();	
		this->images[1] = imagesIn[0][1].clone();
	}
	
	//Intermediary data structures:
	std::vector<RotatedRect> outputBoxL;
	std::vector<RotatedRect> outputBoxR;
	
	
	Mat thresholdImageL;
	Mat thresholdImageR;
	
	
	image = images[0];
	thresholdImageL = ThresholdCircleImage();
	imagesHSV[0] = imageHSV.clone();
	outputBoxL = locateCircles(thresholdImageL);
	std::vector<Scalar> detectedColorsTempLeft = calcCircleColorMean(imagesHSV[0], outputBoxL); 

	std::vector<int> colorPts = colorAlignment(detectedColorsTempLeft);

	image = images[1];
	thresholdImageR = ThresholdCircleImage();
	imagesHSV[1] = imageHSV.clone();
	outputBoxR = locateCircles(thresholdImageR);
	std::vector<Scalar> detectedColorsTempRight = calcCircleColorMean(imagesHSV[1], outputBoxR);

	//After Finding the contours, align them:
	//Since both sets of rotated rectangles are sorted by y location
	//Validate the color (ensure it is semi close):	
	locatedStereoCircles.clear();
	
	detectedColorList.clear();
	detectedColors[0].clear();
	detectedColors[1].clear();
	if(outputBoxL.size()  > 0 &&  outputBoxR.size() > 0)
	{
		
		
		for(unsigned int i = 0; i < outputBoxL.size(); i++)
		{
			//Only look at ellipses that are ringed with white
			//Halo detection
			RotatedRect leftHalo(outputBoxL[i].center,Size2f(outputBoxL[i].size.width*1.2,outputBoxL[i].size.height*1.2),outputBoxL[i].angle);
			Scalar colorRing = CircleHaloColorMean(imagesHSV[0],outputBoxL[i],leftHalo);
			
			//Forget it if the halo is not colorless enough! 
			if(colorRing[1] > 90) continue;
			rotatedRectStereoCorr tempPoint;

			//Compare the color:
			//images[0]->row(y).col(x)
			int dataType = images[0].type();
			//CvScalar colorLeft =  images[0].row((int)outputBoxL[i].center.y).col((int)outputBoxL[i].center.x);
			//CvScalar colorRight =  images[1].row((int)outputBoxR[i].center.y).col((int)outputBoxR[i].center.x);
			Scalar leftMean = detectedColorsTempLeft[i];

			//Now cycle through the boxes on the right image
			bool toHigh = false;
			bool toLow  = false;
			for(int j = 0; j < (outputBoxR.size())*2+outputBoxL.size(); j++)
			{
				if(toHigh && toLow) break;
				int offset = floor((float)j/2);
				int output = 0; 
				if(j%2 == 1)
				{
					output = i+offset;
					
				}
				else
				{
					output = i-offset;
					
				}
				if(output >= outputBoxR.size()){
						toHigh = true;
						continue; 
					}
				if(output < 0){
						toLow = true;
						continue; 
					}
				
				
				Scalar rightMean = detectedColorsTempRight[output];

				int hueError= byteError((int)leftMean[0],(int)rightMean[0]);
				int satError= abs(leftMean[1]-rightMean[1]);
				int valError= abs(leftMean[2]-rightMean[2]);

				float yErr = abs(outputBoxL[i].center.y-outputBoxR[output].center.y);
				//TODO: remove the HARDCODE numbers
				if(hueError < 17 && yErr < 4.0) //This could be better, hue needs to be dynamic to match the best ranges::
				{
					tempPoint[0] = outputBoxL[i];
					tempPoint[1] = outputBoxR[output];
					detectedColors[0].push_back(detectedColorsTempLeft[i]);
					detectedColors[1].push_back(detectedColorsTempRight[output]);
					locatedStereoCircles.push_back(tempPoint);
					Scalar color = leftMean*0.5+rightMean*0.5;
					detectedColorList.push_back(color);
					break;
				}
			} //for(int j = 0; j < outputBoxR.size()+outputBoxL.size(); j++)
		}//for(unsigned int i = 0; i < outputBoxL.size(); i++)
		
		//At this point the located circles in the left and right images are
		//aligned. Is there a good way to fit the image sets to the tool
		//circles?

		//Addendum:
		//Add detection of the faces by looking at r1 and r2.
		foundAligned = locatedStereoCircles.size();

		//Initalize the appropriate vectors:
		detectedFaceListFP.clear();
		detectedFaceListPF.clear();
		
		detectedFaceListPF.resize(foundAligned);

		//TODO: implement a y-location error:

	}//if(outputBoxL.size()  > 0 &&  outputBoxR.size() > 0)
	
	//At this point, The stereo model can be used to generate the 3d list of
	//circles. 
	
	

	circleList_C.resize(foundAligned);

	for(int i = 0; i<foundAligned; i++)
	{
	  circleList_C[i] = modelIn.deprojectEllipse(locatedStereoCircles[i]);
	}

	return foundAligned;

}


void FiducialCircles::matchColorList(){






}



int FiducialCircles::loadFidFile(const char * fileName){

//TODO 
//1. Open a file 
//2. read in the color list, then get the point locations
//3. validate the results
//4. return: -1 on failure, 1 on success.
//

	//New vectors:
	std::vector<int> thresholdRangeTemp;
	std::vector<std::vector<int>>  colorListCirclesTemp;    
	std::vector<int> circleListColorTemp;
	std::vector<Scalar> fileColorListTemp;
	std::vector<Point3f> centerListTemp;
	std::vector<Point3f> faceNormLTemp;
	std::vector<std::vector<int>> faceListFPTemp; 
	std::vector<int> faceListPFTemp;


	


	//Clear the new vectors:
	faceListFPTemp.clear();
	faceListPFTemp.clear();
	thresholdRangeTemp.clear();
	
	centerListTemp.clear();
	faceNormLTemp.clear();
	circleList_T.clear();
	colorListCirclesTemp.clear();
	circleListColorTemp.clear();
	fileColorListTemp.clear();

	std::ifstream infile(fileName,std::ifstream::in);

	if(!infile) return -1; //On fail return -1 
	
	string line;

	int lineNum=0;

	int mode = 0;

	int colorCount = 0;
	int ptCount   = 0;

	while(getline(infile,line))
	{
		string item;
		std::istringstream linestream(line); 
		
		switch(mode)
		{

		case 0: 
			//look for the color line:
			if(line.compare("color:")==0)
			{
				mode = 1;
			}
			break;

		case 1: 
			//Assign the color vector:

			//Break if empty line
			if(line.length()==0) break;

			if(line.compare("location:")==0)
			{
				colorListCirclesTemp.resize(colorCount);
				mode = 2;
			}
			else 
			{
				//Append to the color list:
				getline(linestream,item,',');

				int c0 = atoi(item.c_str())%256;
			
				getline(linestream,item,',');

				int c0t = atoi(item.c_str())%128;
			
				getline(linestream,item,',');

				int c1 = atoi(item.c_str())%256;

				getline(linestream,item,',');

				int c1t = atoi(item.c_str())%128;

				//TODO verify file load:
				//If all that worked,
			
				thresholdRangeTemp.push_back(c0);
				thresholdRangeTemp.push_back(c1);
				thresholdRangeTemp.push_back(c0t);
				thresholdRangeTemp.push_back(c1t);
				
				fileColorListTemp.push_back(Scalar(c0,c1,-1));
				colorCount++;
			}
			break;

		case 2:
			//catch the location/color label:
			//Obtain the vals, then add them to the vectors:
			
			//Break if empty line
			if(line.length()==0) break;
			
			getline(linestream,item,',');

			int colorL = atoi(item.c_str());
			
			Point3f loc;
			Point3f face;

			getline(linestream,item,',');


			loc.x  = (float) atof(item.c_str());
			
			getline(linestream,item,',');
			
			loc.y  = (float)atof(item.c_str());
			
			getline(linestream,item,',');
			
			loc.z  = (float)atof(item.c_str());
				
			getline(linestream,item,',');

			face.x  = (float)atof(item.c_str());
			
			getline(linestream,item,',');
			
			face.y  = (float)atof(item.c_str());
			
			getline(linestream,item,',');
			
			face.z  = (float)atof(item.c_str());
			
			//Try loading the radius:
			getline(linestream,item,',');
		
			double radius  = -1;

			//The face counter of the 
			if(!linestream.eof()){
				radius  = atof(item.c_str());
				//Perform the necessary adendums here:
				//TODO
			}

			//At this point, the specific face can be loaded,
			getline(linestream,item,',');
		
			int faceInt = -1;

			//The face counter of the 
			if(!linestream.eof()){
				faceInt  = atoi(item.c_str());
				//Perform the necessary adendums here:
				//TODO
			}
			else { //compare the face normal to other faces to see if it matches.
				int faceSize = faceNormLTemp.size();
				int indexOut = -1;
				for(int i = 0; i < faceSize; i++){
					bool match = false;

					//compare with the 
					if((faceNormLTemp[i].x - face.x) < 0.01){
						if((faceNormLTemp[i].y - face.y) < 0.01){
							if((faceNormLTemp[i].z - face.z) < 0.01){
								match = true;
								indexOut = i;
							}
						}
					}
					if(match) break;


				}
				if(indexOut > -1) {//Append to the list of face cross matching.
					faceListFPTemp[indexOut].push_back(ptCount+1); 
					faceListPFTemp.push_back(indexOut);

				}
				else { //Make a new face.

					//Add face to the face List:
					faceNormLTemp.push_back(face);
					
					//Add the point reference to the face List 
					std::vector<int> faceListPointAdd;
					faceListPointAdd.clear();
					faceListPointAdd.push_back(ptCount+1);
					faceListFPTemp.push_back(faceListPointAdd);
					
					//Add the face reference to the int counter
					faceListPFTemp.push_back(faceSize);

				}

			}

			//TODO verify file load:
			//If all that worked,
		    int colorHue = thresholdRangeTemp[(colorL-1)*4];
			int colorSat = thresholdRangeTemp[(colorL-1)*4+1];
			colorListCirclesTemp[colorL-1].push_back(ptCount);
			circleListColorTemp.push_back(colorL-1);

			ptCount++;
			
			centerListTemp.push_back(loc);

			circleTracker3d circleTemp;

			circleTemp.center = loc;
			circleTemp.zN     = face;
			circleTemp.rad    = radius;
			circleTemp.errorXY = 0.0;

			circleList_T.push_back(circleTemp);

			break;
		}
	}

	//Once the file load is complete, transfer the temporary vectors to the
	//object vectors: 
	numObjects = (int) centerListTemp.size();
	
	fileColorList = fileColorListTemp;
	colorListCircles = colorListCirclesTemp;
	circleListColor = circleListColorTemp;
	thresholdRange = thresholdRangeTemp;
	faceListFP =faceListFPTemp;
	faceListPF = faceListPFTemp;

	return 1;
}

void FiducialCircles::set3DEllipses(std::vector<circleTracker3d> circleList_CIn){
	circleList_C = circleList_CIn;
}

std::vector<Scalar> FiducialCircles::getDetectedColorList()
{
	return detectedColorList;


}

std::vector<int> FiducialCircles::colorAlignment(std::vector<Scalar> colorLists)
{
	//Goals: 
	//Use the color wheel to match the detected colors to their target points.
	//inputs:
	
	//Initial color Centers:
	int colorCount = thresholdRange.size()/4;
	int ptCount    = colorLists.size();

	std::vector<int> colorCodeTemp;
	colorCodeTemp.resize(colorLists.size());

	int* meanHue;

	meanHue = new int[colorCount];

	//Initial assigment:
	for(int i = 0; i < colorCount; i++){
		meanHue[i] = thresholdRange[i*4];
	}

	//Once the means are first found, find the errors:
	//iterate the errors:
	while(true){
		
		double rot = 0;
		
		for(int i = 0; i < ptCount; i++){
			
			int minError = 40;
			int bestC    = -1;
			for(int j = 0; j < colorCount; j++){
		
				int bError =  byteError((int)meanHue[j],(int)colorLists[i][0]);
				if(bError < minError){
					minError = bError;
					bestC = j;
				}
				//Once the best point is found, look at where it pushes the
				//results:
				if(bestC > -1){
					double errorP =  (colorLists[i][0]-meanHue[bestC]);    
					rot = rot + errorP/ptCount;
				}


			}
				

		}
		
		
		if(rot < 2) break;
			
		for(int i = 0; i < colorCount; i++){
			
			meanHue[i] = meanHue[i]+rot;

		}
	}
	//output assigment:
	for(int i = 0; i < ptCount; i++){
			
			int minError = 40;
			int bestC    = -1;
			for(int j = 0; j < colorCount; j++){
		
				int bError =  byteError((int)meanHue[j],(int)colorLists[i][0]);
				if(bError < minError){
					minError = bError;
					bestC = j;
				}
				//Once the best point is found, look at where it pushes the
				//results:
				colorCodeTemp[i] = bestC;

			}
	}

	delete [] meanHue;


	return colorCodeTemp;
}

int FiducialCircles::setLocalModel(StereoCameraModel _cameraModel)
{
	localCamera = _cameraModel;
	return 1;
}

double FiducialCircles::alignHomoMatch(std::vector<int> matchIndi_C,std::vector<int> matchIndi_T, FrameTransform3D &homoResult_TC){
		
	//Using the lists of matched points, find the transformation.
	//
	
	double errorOutput = 500000.0; //Large initial error.

	int mSize = matchIndi_C.size();
	if( mSize < 3){
	//insufficient Points:
		return errorOutput;	
	}

	else {
		std::vector<Point3f> Pts_C;
		std::vector<Point3f> Pts_T;
		Pts_C.resize(mSize);
		Pts_T.resize(mSize);

		//Generate the point Lists:
		for(int iM = 0; iM < mSize; iM++)
		{
			int ind_C = matchIndi_C[iM];
			int ind_T = matchIndi_T[iM];
			//Verify point alignment
			//Point the normal towards the camera!!
			Point3f ptZtemp; 
			if(circleList_C[ind_C].zN.z > 0) ptZtemp=-circleList_C[ind_C].zN; 
			else  ptZtemp = circleList_C[ind_C].zN; 
			Point3f ptOut_C = (ptZtemp*10)+((Point3f)circleList_C[ind_C].center); 
			//Pts_C.push_back(ptOut_C);
			
			
			////Pts_T.push_back(ptOut_T);

			//Point3f ptOut_C;
			Pts_C[iM] = circleList_C[ind_C].center;

			Pts_T[iM] = circleList_T[ind_T].center;
		}
		
		//At this point, the vector list has been found, generate the transform
		//matrix G!!
		//cout << Mat(Pts_C) << endl;
		//cout << Mat(Pts_T) << endl;
		double errorRMS;
		FrameTransform3D _g_TC(Mat(Pts_C),Mat(Pts_T),errorRMS);
		//_g_CT.getTransform().copyTo(results);
		//Once that is done

		//perform extraction:
		if(errorRMS < 3){
			homoResult_TC = _g_TC;

			g_TC = _g_TC;
		}
		return errorRMS;
	


	}// else if(mSize < 3)
}

int FiducialCircles::seedPointAlign(std::vector<int> &matchIndi_C, std::vector<int> &matchIndi_T, std::vector<std::vector<int>> matchListi_CT, std::vector<std::vector<int>> matchListi_TC){

	int mT = circleList_T.size();
	int mC = circleList_C.size();


	int matchCount = matchIndi_C.size();
	int matchCountOld = matchCount-1;

	while(matchCount > matchCountOld){
		for(int i = 0; i < mT; i++){
			int ind_T = i;
			int ind_C;
			int matchSize_C = matchListi_TC[i].size();
			switch(matchSize_C)
			{
			
				case 0:
					//std::cout << "There were no matches!!\n";
					break;
				
				case 1: //One match from T to C, but verify the reverse.
					ind_C = matchListi_TC[i][0];
					if(matchListi_CT[ind_C].size() == 0){ //See if the match is mutually exclusive

						matchIndi_C.push_back(ind_C);
						matchIndi_T.push_back(ind_T);

						pruneMatches(matchListi_CT,matchListi_TC, ind_C, ind_T);
						
						break;
					} //if not, then proceed with below:
				default:
					//std::cout << "many potential matches \n";
					double errorMin_C;
					int bestInd_C =  bestMatch_C(matchIndi_C, matchIndi_T, matchListi_TC[i], i, errorMin_C);

					 //Right now the best point from the tools to the cameras has been found.
					 //Next step is to look at the camera to the tool points
					//Once the best point is found, set it:
					//TODO remove the matched index from the matchList.
					//TODO do a better job of the matching indexing.
					//HARDCODE

					if(bestInd_C > -1){ //Only use the best point if it is a 'good' match
									//This finds the best matched camera point for each tool point, but this will result in multiple matches!
						//Look at the camera coordinates:
						double errorMin_T;
						int bestInd_T =  bestMatch_T(matchIndi_C, matchIndi_T, bestInd_C, matchListi_CT[bestInd_C],errorMin_T);	
						
						if(bestInd_T == i){
							//This indicates that the point is the best
							//cross-match, However, verify that it can be on
							//the correct face:
							//TODO FACE-verify

							
							//Find the tool Face, 
							//int tFaceInd = faceListPF[bestIndT];
							//int 
								
													//Add indexing:
							matchIndi_C.push_back(bestInd_C);
							matchIndi_T.push_back(bestInd_T);
							//Once a new point is appended, the match list must be culled to remove the matching points:							
							pruneMatches(matchListi_CT,matchListi_TC, bestInd_C, bestInd_T);
						} // 	if(bestInd_T == i)

					} //	if(bestInd_C > -1)
			} //switch(matchSize_C)
		} //for(int i = 0; i < mT; i++)

		matchCountOld = matchCount;
		matchCount = matchIndi_C.size();

		if(matchCount == mT || matchCount == mC) break;

	} // while(matchCount > matchCountOld)
	return 1;
							
} 

FrameTransform3D FiducialCircles::generateHomo(){

	//TODO
	//This function generates the homogenous (4x4) transform
	//From the camera frame to the coordinates defined by the 
	//points listed:
	//
	//
	//
	//
	FrameTransform3D _g_TC;
	//Start with g_CT being the identity
	g_TC = FrameTransform3D();

	//Right now, the function assumes that there is only one color per
	//Ergo, run through the point list and assign it accordingly
	
	int nT = circleList_T.size();

	int mC  =locatedStereoCircles.size();

	//std::vector<int> circleIndM_T,circleIndM_C;

	circleIndM_T.clear();
	circleIndM_C.clear();

	/* //TODO:
	//Once face alignment is complete, match detected faces?
	//This is done as part of the three d matching:	
		/ *Face matching plan: * /
		// 1. match by the best set of colors
		// 2.
		int detectedFaceCount = detectedFaceListFP.size();
		int expectedFaceCount = faceListFP.size();

		//if there is a discrepancy, fix it?

		for(int dF = 0; dF < detectedFaceCount; dF++){
			for(int eF = 0; eF < expectedFaceCount; eF++){

				//Fross match the points: look at the
				int dCC = detectedFaceListFP[dF].size();   //detected color count.
				int eCC = faceListFP[eF].size();
				
				if(dCC == eCC) //The numbers match up ergo, perform a comparison;
				for(int dC = 0; dC < dCC; dC++){ //for each detected face element, find the best match:
					for(int eC = 0; eC < eCC;  eC++){
						
					}
				}

			}
		} * /

	
		//The output vector of matched points.
		std::vector<int> outputList_T;
		std::vector<int> outputList_C;
		outputList_T.clear();
		outputList_C.clear();


		//vectors of cross matches.
		std::vector<std::vector<int>> matchList_TC; //first index is Tool points, second index is the camera points 
		std::vector<std::vector<int>> matchList_CT; //first index is camera points, second index is the tool point

	
		std::vector<std::vector<double>> matchList_CTErrors;
		matchList_TC.resize(nT);
		matchList_CT.resize(mC);
		matchList_CTErrors.resize(mC);

		for(int i = 0; i < nT; i++) matchList_TC[i].clear();
		for(int i = 0; i < mC; i++) matchList_CT[i].clear();
		//for(int i = 0; i < mC; i++) matchList_CTErrors[i].clear();

		//Match potentials based on colors and radius if possible:
		for(int i = 0; i < mC; i++){     //Move through camera point list

			Scalar bestMatches[2];
			int hueError[2] = {260,260};
			int satError[2] = {-1,-1};
			int bestC[2] = {-1,-1};  //best 2 colors.
			int colorSize = colorListCircles.size();

			for( int j = 0; j < colorSize; j++){  //matches colors to circles.
				

				if(colorListCircles[j].size() > 0){
				int tempHueError = byteError((int) detectedColorList[i][0],(int) fileColorList[j][0]);
				int tempSatError = abs( detectedColorList[i][1]-fileColorList[j][1]);

				if(tempHueError < hueError[0] && tempSatError < 60 ){ //TODO remove the light invariance problem!!
					//Pushback:
					hueError[1] = hueError[0];
					satError[1] = satError[0];
					bestC[1]    = bestC[0];
					//replace vals
					hueError[0] = tempHueError;
					satError[0] = tempSatError;
					bestC[0]    = j;
					
					}
				else {
					if(tempHueError < hueError[1] && tempSatError < 60 ){
					//replace vals
					hueError[1] = tempHueError;
					satError[1] = tempSatError;
					bestC[1]  = j;
					}
				}
				}
			}
			//At this point Do both need to be included?
			if(bestC[0] > -1){
				if(hueError[1]-hueError[0] > 20 || abs(satError[1]-satError[0]) > 70) { //No
					int tList = colorListCircles[bestC[0]].size(); 
					for(int k = 0; k < tList; k++){
						int toolInd = colorListCircles[bestC[0]][k];

						//Before appending to the match list, look at the radius
						//and verify it:
						float rError = abs(circleList_T[toolInd].rad-circleList_C[i].rad);

						if(rError < 0.8){ //one half millimeter error range:
							//The introduction of the radial term should improve
							//performance be removing false ellipses from
							//consideration
							matchList_TC[toolInd].push_back(i);
							matchList_CT[i].push_back(toolInd);
						}

					}
				}
				else{
					for(int j = 0; j < 2; j++){
						int tList = colorListCircles[bestC[j]].size(); 
						for(int k = 0; k < tList; k++){
							int toolInd = colorListCircles[bestC[j]][k];
							matchList_TC[toolInd].push_back(i);
							matchList_CT[i].push_back(toolInd);
						}

					}
				}
			} //if(bestC[0] > 0)
		}
		//At this point all of the potential matches are tracked.
		//Ideally there will be at least one unique color.
		//This unique color gives a fixed point.
		

		//Now use the single matches to sort out the non matched ones.
		//Since the interpoint distance must match, use that to find other
		//matches from the first match. (This puts some goemetric constraints
		//on the fiducials).
		//circleIndM_C.clear();
		//circleIndM_T.clear();
		for(int i = 0; i < nT; i++)
		{
			if(matchList_TC[i].size() == 1)  //The tool Point has only one match! 
			{
				int Cind = matchList_TC[i][0];
				if(matchList_CT[Cind].size() == 1){ //The Camera point also has only one match!!
					//Append to the list of starting points for distance.
					//circleIndM_C.push_back(circleList_C[matchList_TC[i][0]]); //Camera frame:
					//circleIndM_T.push_back(circleList_T[i]);		//Tool frame:

					//Append to the matched point list indeciis.
					circleIndM_C.push_back(matchList_TC[i][0]);
					circleIndM_T.push_back(i); //The center list must only include matches
											// This allows the possiblity of
											// not matching all the points due
											// to oc0lusions etc. 
					//Remove the matched items....
					matchList_TC[i].erase(matchList_TC[i].begin());
					matchList_CT[Cind].erase(matchList_CT[Cind].begin());
				}
			}	
		}



		//Whith no Single good matches, bifrucate the starting matches.
		//
		if(circleIndM_T.size() == 0){

			//std::cout << "No Single good points found!!!\n";
			std::vector<std::vector<int>> circleIndn_C;
			std::vector<std::vector<int>> circleIndn_T;
			std::vector<double> meanMatchErrors;
			std::vector<int> matchSizes;

			circleIndn_C.clear();
			circleIndn_T.clear();
			meanMatchErrors.clear();
			matchSizes.clear();
			//Pick the first option:
			//using tool zero as an initialMat
			
			std::vector<double> errorList;
			errorList.clear();
			//matchCounter_CT.resize(mC);

			double errorBest = 5000;
			int bestMInd = -1; 
			int bestMSize = -1;
			//matchCounter[i] = matchList_CT[0].size();
			for(int i = 0; i < mC; i++){
				int toolOptions = matchList_CT[i].size();
				int ind_C = i;
				std::vector<std::vector<int>> matchListTemp_CT = matchList_CT;
				std::vector<std::vector<int>> matchListTemp_TC = matchList_TC;

				for(int j = 0; j < toolOptions; j++){
					//increment the match size
					//
					std::vector<std::vector<int>> matchListTemp_CT = matchList_CT;
					std::vector<std::vector<int>> matchListTemp_TC = matchList_TC;
					
					std::vector<int> circleIndt_C; //temporary seed matches
					std::vector<int> circleIndt_T; //temporary seed matches

					circleIndt_C.clear();
					circleIndt_T.clear();
					int ind_T = matchListTemp_CT[i][j];
					
					circleIndt_C.push_back(i);  //start with the camera point;
					circleIndt_T.push_back(ind_T); //the tool point
					//Add the pruning:

					pruneMatches(matchListTemp_CT,matchListTemp_TC, ind_C, ind_T);

					//To improve reliability, seed with an additional point:
					//for(int k = 0; k < toolOptions; k++){
						//second seed point:


						//Once the system is seeded, Find best matches:
						int results = seedPointAlign(circleIndt_C, circleIndt_T, matchListTemp_CT, matchListTemp_TC); 
						//After the seed points are found, generate the homoTrans.
						FrameTransform3D frameOutput;
						double errorTest = alignHomoMatch(circleIndt_C, circleIndt_T,frameOutput);
						bool cond3 = true;
						for(int ptTi = 0; ptTi < circleIndt_T.size(); ptTi++)
						{
							//Find the normal direction of the different found circles. 
							int loc = circleIndt_T[ptTi];
							Point3d normalVect_T = circleList_T[loc].zN;
							Point3d normalVect_C = frameOutput.getInverseTransform()(normalVect_T)-frameOutput.getInverseTransform()(Point3d(0,0,0));
							if(normalVect_C.z > 0 || frameOutput.getTransform().at<double>(3,3) < 0)
							{
								//Cheap pause point
								cond3 = false;
							}
						}
						errorList.push_back(errorTest);

						circleIndn_T.push_back(circleIndt_T);
						circleIndn_C.push_back(circleIndt_C);
						int matchCount = circleIndt_T.size();
						double errorTestM=errorTest/matchCount;
						meanMatchErrors.push_back(errorTestM);
						matchSizes.push_back(matchCount);
						bool cond1 = (errorTestM < 2) && (matchCount > bestMSize);
						bool cond2 = errorTestM < errorBest && (matchCount >= bestMSize);

						//if(errorTestM < 2.0 && 
						if((cond1 || cond2) && cond3){ 
							errorBest = errorTestM;
							bestMInd = circleIndn_T.size()-1;
							bestMSize = matchCount;
						}//if(errorTest<errorBest)
					//}//for(int k = 0; k < toolOptions; k++){
				} //for(int j = 0; j < toolOptions; j++);

			} //for(int i = 0; i < mC; i++);
			//At this point cycle through the different options, one of the options
			//will have the lowest overall error. Use that one!!!

			if(bestMInd > -1){
				
				alignHomoMatch(circleIndn_C[bestMInd], circleIndn_T[bestMInd],g_TC);
				circleIndM_C = circleIndn_C[bestMInd];
				circleIndM_T = circleIndn_T[bestMInd];

				
				alignColorMatch();


				if(displayOn){
					cout << "Point Matches" << endl;
					for(int i = 0; i < circleIndn_C[bestMInd].size(); i++){
						cout << "Image Pt # " << circleIndn_C[bestMInd][i] << " Tool Pt # " << circleIndn_T[bestMInd][i] << endl;

					}
				}
			}

		} //if(circleListM_T.size() == 0;
		else{ //Some single good points found.

		//Run with that:
		seedPointAlign(circleIndM_C, circleIndM_T, matchList_CT, matchList_TC); 
		if(displayOn){
			cout << "Point Matches" << endl;
			for(int i = 0; i < circleIndM_C.size(); i++){
				cout << "Image Pt # " << circleIndM_C[i] << " Tool Pt # " << circleIndM_T[i] << endl;

			}
		}
	

		double errorRMS = alignHomoMatch(circleIndM_C, circleIndM_T,_g_TC); 


		//Once that is done

		//perform extraction:
		if(errorRMS < 3)
		{
			g_TC = _g_TC;
			alignColorMatch();
			//record the new color values, if possible.

		}
		}
		//else cout << "Not enough good points identified" << endl;
		
		//Run
	return g_TC;
}


//this function aligns a new set of template colors based on the image tracking results.
void FiducialCircles::alignColorMatch()
{
	//circleIndM_C[i]  //matched indeces from detection
	//circleIndM_T[i]  //matched indeces from 
	std::vector<std::vector<Scalar>> sensedColorList;
	sensedColorList.clear();
	sensedColorList.resize(fileColorList.size());
	for(int index = 0; index < fileColorList.size(); index++)
	{
		sensedColorList[index].clear();

	}

	int sizeM_C = circleIndM_C.size();
	int sizeM_T = circleIndM_T.size();

	if(sizeM_C == sizeM_T && sizeM_T> 0)
	{
		for(int index = 0; index < sizeM_C; index++)
		{
			int colorIndex = circleListColor[circleIndM_T[index]];
			Scalar newColor = detectedColorList[circleIndM_C[index]];
			sensedColorList[colorIndex].push_back(newColor);
		}

		imageMatchedColorList.clear();
		imageMatchedColorList.resize(fileColorList.size(),Scalar(-1,-1,-1));


		int replacedColors  = 0;

		for(int index = 0; index < fileColorList.size(); index++)
		{
			if(sensedColorList[index].size() > 0)
			{
				Scalar meanVal(0,0,0);
				for(int jndex = 0; jndex < sensedColorList[index].size(); jndex++)
				{
					meanVal += sensedColorList[index][jndex]*(1/((double)sensedColorList[index].size()));

				}
				imageMatchedColorList[index] = meanVal;
				replacedColors++;
			}
		}
		if(replacedColors == fileColorList.size()) imageMatchedColorStatus = 2;
		else imageMatchedColorStatus = 1;

	}

	



}

FrameTransform3D FiducialCircles::updateHomo(){
	
	std::vector<Point3f> ptsC;
	std::vector<Point3f> ptsT;
	int mSize = circleList_C.size(); //number of cameraPoints.
	ptsC.resize(mSize);
	ptsT.resize(mSize);
	//use the points from the frame transform:

	for(int index = 0; index < mSize; index++)
	{

		ptsC[index] = circleList_C[circleIndM_C[index]].center;
		ptsT[index] = circleList_T[circleIndM_T[index]].center;

	}

	//cout << g_TC.getTransform() << endl;

	g_TC.LMoptimize(Mat(ptsC),Mat(ptsT));

	//cout << g_TC.getTransform() << endl;


	return g_TC;


}


std::vector<Point3d> FiducialCircles::getFidCenters_T()
{
	std::vector<Point3d> vectorList;
	vectorList.resize(circleList_T.size());

	for(int i = 0; i < vectorList.size(); i++)
	{
		vectorList[i] = circleList_T[i].center;
	}

	return vectorList;


}

int FiducialCircles::drawEstEllipse(stereoImage& inputImages, const StereoCameraModel& modelIn)
{
	int returnStatus = 0;

	int circleCount = circleList_T.size();
	
	std::vector<rotatedRectStereoCorr> localEllipses;
	localEllipses.clear();
	
	for(int index = 0 ; index < circleCount; index++)
	{
		Point3d centerTemp_T =  circleList_T[index].center;
		Point3d normTemp_T   =  circleList_T[index].zN;

		
		Point3d centerTemp_C = g_TC.getInverseTransform()(centerTemp_T);
		Point3d normalTemp_C = g_TC.getInverseTransform()(normTemp_T)-g_TC.getInverseTransform()(Point3d(0,0,0));

		//verify that the norm is pointing in the -z direction of the camera:
		if(normalTemp_C.z < 0)
		{
			circleTracker3d tempCircle_C = g_TC.getInverseTransform()(circleList_T[index]); 
			//compute the 2 dimensional projection of the circle.
			rotatedRectStereoCorr tempEllipsePair = modelIn.reprojectEllipse(tempCircle_C);

			for(int lr = 0; lr < 2; lr++)
			{

				ellipse(inputImages[lr],tempEllipsePair[lr],Scalar(0,0,0),4);

			}
		
		}
		//



	}



	return returnStatus;
}



int FiducialCircles::findEstEllipse(const stereoImage& inputImages, const StereoCameraModel& modelIn)
{
	
	stereoImage localImages,mask;

	for(int lr = 0; lr < 2; lr++)
	{
		localImages[lr] = inputImages[lr].clone();
		cvtColor(localImages[lr],mask[lr],CV_BGR2GRAY);
	}
	
	int returnStatus = 0;

	int circleCount = circleList_T.size();
	

	std::vector<rotatedRectStereoCorr> localEllipses;
	std::vector<Scalar> localScalars;
	localEllipses.clear();
	localScalars.clear();
	circleIndM_T.clear();
	circleIndM_C.clear();

	for(int index = 0 ; index < circleCount; index++)
	{
		Point3d centerTemp_T =  circleList_T[index].center;
		Point3d normTemp_T   =  circleList_T[index].zN;

		
		Point3d centerTemp_C = g_TC.getInverseTransform()(centerTemp_T);
		Point3d normalTemp_C = g_TC.getInverseTransform()(normTemp_T)-g_TC.getInverseTransform()(Point3d(0,0,0));
		//verify that the norm is pointing in the -z direction of the camera:
		if(normalTemp_C.z < 0) // Potenitally a find.
		{
			
			circleTracker3d tempCircle_C = g_TC.getInverseTransform()(circleList_T[index]);
			stereoCorrespondence center = modelIn.reproject(tempCircle_C.center);
			//compute the 2 dimensional projection of the circle.
			rotatedRectStereoCorr tempEllipsePair = modelIn.reprojectEllipse(tempCircle_C);
			
			//find the center point.
			stereoCorrespondence startPt;
			Scalar outputColor;
			for(int lr = 0; lr < 2; lr++)
			{
				startPt[lr] = tempEllipsePair[lr].center;	
			}
				
			rotatedRectStereoCorr endRects= growEllipse(inputImages ,startPt, outputColor);
			
			Scalar errorVal = fileColorList[circleListColor[index]]-outputColor;
			double normError = sqrt(errorVal[0]*errorVal[0]*4+errorVal[1]*errorVal[1]*1+errorVal[2]*errorVal[2]*0.5);

			localEllipses.push_back(endRects);
			localScalars.push_back(outputColor);
			//measure color error:
			//update the matchList:
			circleIndM_T.push_back(index);
		}
		//
	}
	//once everything is done, push back the list.
	//overwrite the detected lists.
	//colorLists = localScalars;
	locatedStereoCircles =localEllipses;
	int mSize = localEllipses.size();
	circleList_C.resize(mSize);
	circleIndM_C.resize(mSize);
	for(int index = 0; index < mSize; index++)
	{
		//cout << "original stated Val at index " << index << " : " << fileColorList[circleListColor[index]] << endl;
		/*if(index < detectedColorList.size())
		{
			//cout << "Old recorded stated Val at index " << index << " : " << detectedColorList[index] << endl;
		} * /
		//else cout << "Old recorded stated Val at index " << index << " : N/A"  << endl;
		//cout << "New recorded stated Val at index " << index << " : " << localScalars[index] << endl;
		circleList_C[index] =  modelIn.deprojectEllipse(localEllipses[index]);
		circleIndM_C[index] = index;
	}
	detectedColorList = localScalars;
	returnStatus = 1;

	return returnStatus;
}




void FiducialCircles::initFromPoints(const stereoImage & inputImgPair, const StereoCameraModel& modelIn,vector<stereoCorrespondence> startPtList)
{
	
	int listSize = startPtList.size();
	
	circleList_C.clear();
	locatedStereoCircles.clear();
	detectedColorList.clear();

	for(int i = 0; i < listSize;i++)
	{
		Scalar ellipseColor;
		rotatedRectStereoCorr generatedCircle = growEllipse(inputImgPair ,  startPtList[i], ellipseColor);
	
		locatedStereoCircles.push_back(generatedCircle);
		//artificially track the colors.
		detectedColorList.push_back(ellipseColor); 
		circleList_C.push_back(modelIn.deprojectEllipse(generatedCircle));
		
	}
	return;

}


//This function uses a starting point to identify an ellipse

FrameTransform3D FiducialCircles::transformDirMove(Quaternion inputQ,Point3d inputTrans){

	g_TC.moveTransformDir(inputQ,inputTrans);

	return g_TC;

}

FrameTransform3D FiducialCircles::transformRelMove(Quaternion inputQ,Point3d inputTrans){

	g_TC.moveTransformRel(inputQ,inputTrans);

	return g_TC;

}



//This function uses a starting point to identify an ellipse
//if this function works with out the class definition then it would be perfect for the tissue point growth.
rotatedRectStereoCorr growEllipse(const stereoImage & inputImgPair , stereoCorrespondence startPt, Scalar &meanColor)
{

	rotatedRectStereoCorr outputCircle;
	Scalar colorMatch[2];

	//convert to HSV...

	
//	cvNamedWindow("local Image 01");
//	cvNamedWindow("local Image 02"); 
	
	for(int lr = 0; lr < 2; lr++)
		{
			Mat tempHSV;
			cvtColor(inputImgPair[lr],tempHSV,CV_BGR2HSV);

			//take the mean of the estimated center:

			Rect centerROI((Point) startPt[lr]-Point(1,1),Size(3,3));  //center point

			Mat localRegion = tempHSV(centerROI);

			Mat localMask,localMaskROI;
			cvtColor(tempHSV,localMask,CV_BGR2GRAY);

			localMask = 0;

			localMaskROI = localMask(centerROI);
			localMaskROI = 255;  //dataType is 8 bit unsigned.

			Scalar meanSm,stdDevSm;
			
			meanStdDev(localRegion,meanSm,stdDevSm);
			
			//since this is a single point start, just use the local 3x3 mean
			//to start the segmentation. 


			Mat segmentedImage;
			
			Scalar lowerBound,upperBound;
			lowerBound[2] = 0;
			upperBound[2] = 255;
			lowerBound[0] = meanSm[0] - 2*stdDevSm[0];
			lowerBound[1] = meanSm[1] - 3*stdDevSm[1];
			upperBound[0] = meanSm[0] + 2*stdDevSm[0];
			upperBound[1] = meanSm[1] + 3*stdDevSm[1];
			inRange(tempHSV,lowerBound,upperBound,segmentedImage);
						 
			Scalar oldSum = sum(localMask);
			//dilate and segment the mask
			//in theory this should reach an equilibrium... (maybe not)
			while(true)
			{
				dilate(localMask,localMask,Mat());
				Mat tempMask;
				
				bitwise_and(localMask,localMask,tempMask,segmentedImage);
				
				localMask = tempMask.clone();

				meanStdDev(tempHSV,meanSm,stdDevSm,localMask);

				lowerBound[0] = meanSm[0] - 2*stdDevSm[0];
				lowerBound[1] = meanSm[1] - 3*stdDevSm[1];
				upperBound[0] = meanSm[0] + 2*stdDevSm[0];
				upperBound[1] = meanSm[1] + 3*stdDevSm[1];
				
				inRange(tempHSV,lowerBound,upperBound,segmentedImage);

				Scalar newSum = sum(localMask);

			//	imshow("local Image 01",localMask);
			//	imshow("local Image 02",segmentedImage);
			//	cvWaitKey(30);

				if(newSum[0] == oldSum[0] ) break;
				else oldSum = newSum;
				
			}
		
			//cvDestroyWindow("local Image 01");
			//cvDestroyWindow("local Image 02");
			//Here we need to use the local mean/stdDev to identify the full
			//circle

			vector<vector<Point>> contours;

			findContours(localMask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);


			outputCircle[lr] =  fitEllipse(contours[0]);
			colorMatch[lr] = meanSm;
		}
	//how similar are the colors?

	meanColor = colorMatch[0]*0.5+colorMatch[1]*0.5;

	return outputCircle;

}


//This function uses a starting point to identify an ellipse
RotatedRect growEllipseImage(const Mat & inputImg , Point2f startPt, Scalar &meanColor,Scalar &lowColor,Scalar &highColor )
{

	RotatedRect outputCircle;
	Scalar colorMatch;

	//convert to HSV...
	Mat tempHSV;
	cvtColor(inputImg,tempHSV,CV_BGR2HSV);

	//take the mean of the estimated center:

	Rect centerROI((Point) startPt-Point(1,1),Size(3,3));  //center point

	Mat localRegion = tempHSV(centerROI);

	Mat localMask,localMaskROI;
	cvtColor(tempHSV,localMask,CV_BGR2GRAY);
	localMask = 0;

	localMaskROI = localMask(centerROI);
	localMaskROI = 255;  //dataType is 8 bit unsigned.

	Scalar meanSm,stdDevSm;
			
	meanStdDev(localRegion,meanSm,stdDevSm);
			
	//since this is a single point start, just use the local 3x3 mean
	//to start the segmentation. 


	Mat segmentedImage;
			
	Scalar lowerBound,upperBound;
	
	lowerBound[0] = meanSm[0] - 2*stdDevSm[0];
	lowerBound[1] = meanSm[1] - 3*stdDevSm[1];
	upperBound[0] = meanSm[0] + 2*stdDevSm[0];
	upperBound[1] = meanSm[1] + 3*stdDevSm[1];
	lowerBound[2] = meanSm[2] - 4*stdDevSm[2];
	upperBound[2] = meanSm[2] + 4*stdDevSm[2];

			inRange(tempHSV,lowerBound,upperBound,segmentedImage);
						 
			Scalar oldSum = sum(localMask);
			//dilate and segment the mask
			//in theory this should reach an equilibrium... (maybe not)
			while(true)
			{
				dilate(localMask,localMask,Mat());
				Mat tempMask;
				
				bitwise_and(localMask,localMask,tempMask,segmentedImage);
				
				localMask = tempMask.clone();

				meanStdDev(tempHSV,meanSm,stdDevSm,localMask);

				lowerBound[0] = meanSm[0] - 2*stdDevSm[0];
				lowerBound[1] = meanSm[1] - 3*stdDevSm[1];
				upperBound[0] = meanSm[0] + 2*stdDevSm[0];
				upperBound[1] = meanSm[1] + 3*stdDevSm[1];
				lowerBound[2] = meanSm[2] - 4*stdDevSm[2];
				upperBound[2] = meanSm[2] + 4*stdDevSm[2];

				inRange(tempHSV,lowerBound,upperBound,segmentedImage);

				Scalar newSum = sum(localMask);

			//	imshow("local Image 01",localMask);
			//	imshow("local Image 02",segmentedImage);
			//	cvWaitKey(30);

				if(newSum[0] == oldSum[0] ) break;
				else oldSum = newSum;
				
			}
		
			//cvDestroyWindow("local Image 01");
			//cvDestroyWindow("local Image 02");
			//Here we need to use the local mean/stdDev to identify the full
			//circle

			vector<vector<Point>> contours;

			findContours(localMask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);


			outputCircle =  fitEllipse(contours[0]);
			colorMatch = meanSm;
			lowColor = lowerBound;
			highColor = upperBound;

	return outputCircle;

}

//#define DEBUG_GROWELLIPSEBW
//This function uses a starting point to identify an ellipse
RotatedRect growEllipseBW(const Mat & inputImg , Point2f startPt)
{

    Mat segmentedImg;
    growBlobBW(inputImg,segmentedImg,startPt);
	//cannySegment(inputImg,segmentedImg,startPt);

#ifdef DEBUG_GROWELLIPSEBW
    //draw contour
    Mat display = segmentedImg.clone();
   

    imshow("Segmented Image",display);

    waitKey(0);

    destroyWindow("Segmented Image");
#endif

    vector<vector<Point>> contours;

    findContours(segmentedImg,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

#ifdef DEBUG_GROWELLIPSEBW
    //draw contour
    display = inputImg.clone();
    drawContours(display,contours,-1,Scalar(255,0,0));

    imshow("final Contour",display);

    waitKey(0);

    destroyWindow("final Contour");
#endif

    if(contours.size() == 0)
    {
        RotatedRect dummyEllipse;
        dummyEllipse.center = Point2f(-1.0,-1.0);
        return dummyEllipse; 
    }
    RotatedRect outputCircle =  fitEllipse(contours[0]);

#ifdef DEBUG_GROWELLIPSEBW
    display = inputImg.clone();
    ellipse(display,outputCircle,Scalar(0,255,0));


    imshow("final Ellipse",display);

    waitKey(0);

    destroyWindow("final Ellipse");
#endif


    return outputCircle;
}



//This function uses a starting ellipse to fit and identify an ellipse
//this function can work one of two ways.
//1. Iterate the ellipse numerically.
//2. use the original ellipse as a mask and refine the mask.
//
/*double iterateEllipseBW(const Mat & inputImg, RotatedRect &ellipseUpdate)
{

    //In this version, assume that the image needs to be edge segmented.

    Rect ROI =  ellipseUpdate.boundingRect();

    Point pointOff(8,8);
    Size  sizeOff(16,16);

    ROI -= pointOff;
    ROI +=sizeOff;

    //bound the ROI so that it is inside the image.
    Rect imgBox(Point(0,0),inputImg.size());

    ROI &= imgBox;

    Mat subImg = inputImg(ROI);

    Mat greySubImg;
    cvtColor(subImg,greySubImg,CV_BGR2GRAY);

    Mat dx;
    Mat dy;

    Mat dxx;
    Mat dxy;
    Mat dyy;


    Scharr(greySubImg,dx,CV_32F,1,0);
    Scharr(greySubImg,dy,CV_32F,0,1);

    Scharr(greySubImg,dxx,CV_32F,2,0);
    Scharr(greySubImg,dxx,CV_32F,1,1);
    Scharr(greySubImg,dyy,CV_32F,0,2);


    //compute ellipse gradient here!
    //should match value as a function of center, r1, r2, and angle.

    //computeEllipseMatch

    double oldMatch = computeEllipseMatch(dx,dy,ellipseUpdate);
    double dE = 10;

    while(dE > 0.001)
    {

        gradientEllipseUpdate(dxx,dxy,dyy,ellipseUpdate);

        double newMatch = computeEllipseMatch(dx,dy,ellipseUpdate);

        dE = oldMatch - newMatch;

        if(dE < 0)
        {
            printf("The ellipse is not improving");
            oldMatch = -1.0;  //indicates a bad fit.
            break; //don't bother if not improving.
        }

        oldMatch = newMatch;

    }

#ifdef DEBUG_ITERATEELLIPSEBW
    //draw contour
    display = inputImg.clone();
    ellipse(display,ellipseUpdate,Scalar(0,255,0));

    imshow("final Ellipse",display);

    waitKey(0);

    destroyWindow("final Ellipse");
#endif

    return oldMatch;

}* /

#define DEBUG_ITERATEELLIPSEBW

RotatedRect updateEllipseBWCanny(const Mat & inputImg, RotatedRect ellipseUpdate)
{


    //In this version, assume that the image needs to be edge segmented.

    Rect ROI =  ellipseUpdate.boundingRect();

    Point pointOff(8,8);
    Size  sizeOff(16,16);

    ROI -= pointOff;
    ROI +=sizeOff;

    //bound the ROI so that it is inside the image.
    Rect imgBox(Point(0,0),inputImg.size());

    ROI &= imgBox;

	Mat greySubImg;
    Mat subImg = inputImg(ROI);
    cvtColor(subImg,greySubImg,CV_BGR2GRAY);

    Mat cannyImg;

    Canny(greySubImg,cannyImg,50,10,3,true);


#ifdef DEBUG_ITERATEELLIPSEBW
    //draw contour
    Mat display = cannyImg.clone();

    imshow("canny Image",display);

    waitKey(0);

    destroyWindow("canny Image");
#endif

    Point offset = ROI.tl();

    vector<vector<Point>> contours;

    findContours(cannyImg,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,offset);

    RotatedRect outputCircle =  fitEllipse(contours[0]);

	//compare the new ellipse to the old ellipse


#ifdef DEBUG_ITERATEELLIPSEBW
    display = inputImg.clone();
    ellipse(display,outputCircle,Scalar(0,255,0));
	ellipse(display,ellipseUpdate,Scalar(0,0,255));


    imshow("final Ellipse",display);

    waitKey(0);

    destroyWindow("final Ellipse");
#endif

	return  outputCircle;

}

void exportEllipseFile(const char* filePath,std::vector<RotatedRect> &inputEllipses){

    FILE* pFile = fopen(filePath,"w");

    if(pFile ==NULL)
    {
        printf("Unable to open the ellipse file.\n");
        return;
    }

    for(int ind = 0; ind < inputEllipses.size(); ind++)
    {
        fprintf(pFile,"%d, %lf, %lf, %lf, %lf, %lf \n",ind
													  ,inputEllipses[ind].center.x
													  ,inputEllipses[ind].center.y
												  	  ,inputEllipses[ind].size.height
													  ,inputEllipses[ind].size.width
													  ,inputEllipses[ind].angle);
	}
	fclose(pFile);
}
*/
