/*FiducialBase.cpp
 *Case Western Reserve Mechatronics Lab -- Professor Murat Cavusoglu
 *Author: Eddie E. Massey III
 *A class to determine the location of a fiducial in an image using 
 * thresholding and blob detection Simple: and expandable.
 * Dec 2012 
 */

 //Standard library

#include "cwru_opencv_common/block_detection.h"
#include <stdio.h>

#include <ros/ros.h>

using namespace cv;
using namespace cv_local;

//This function helps sort the points by their y positions.
bool pointYSortingHL(Point2f A,Point2f B)
{
    if(A.y > B.y) return true;
    else return false;
}

bool pointXSortingHL(Point2f A,Point2f B)
{
    if(A.x > B.x) return true;
    else return false;
}

bool pointYSortingLH(Point2f A,Point2f B)
{
    if(A.y < B.y) return true;
    else return false;
}

bool pointXSortingLH(Point2f A,Point2f B)
{
    if(A.x < B.x) return true;
    else return false;
}

/*bool contourAreaSortingLH(std::vector<Point> A,std::vector<Point> B)
{

    double areaA,areaB;
    std::vector<Point> convexHullA,convexHullB;


    convexHull(A,convexHullA);
    convexHull(B,convexHullB);

    areaA = contourArea(convexHullA);
    areaB = contourArea(convexHullB);


    if(areaA < areaB) return true;
    else return false;
}

bool contourAreaSortingHL(std::vector<Point> A,std::vector<Point> B)
{
    return contourAreaSortingLH(B,A);
}

void FiducialBase::displayStatus(bool settingDisp)
{
    this->displayOn = settingDisp;
}
*/
/* Find the needle 
 * The constructor initalizes the FiducialBase class with an image to search against.
 * @param tissue a cv::Mat image of the tissue without the needle.
 */

/*
FiducialBase::FiducialBase(Mat image, std::vector<int> thresholdRange, std::vector<int> blobRange,int numObjects, bool displayOn = false)
{
  this->image = image;
  this->thresholdRange = thresholdRange;
  this->blobRange = blobRange;
  this->numObjects = numObjects;
  this->displayOn = displayOn;
}

void FiducialBase::setImage(Mat imageIn){
    this->image = imageIn;
}

std::vector<cv::Point> FiducialBase::returnCorners()
{

    //cvb::CvBlobs::const_iterator it=blobs.begin();
    std::vector<cv::Point> fiducialCorners;
    fiducialCorners.resize(4);

    /*fiducialCorners[0].x = it->second->minx;
    fiducialCorners[0].y = it->second->miny;

    fiducialCorners[1].x = it->second->maxx;
    fiducialCorners[1].y = it->second->miny;

    fiducialCorners[2].x = it->second->minx;
    fiducialCorners[2].y = it->second->maxy;

    fiducialCorners[3].x = it->second->maxx;
    fiducialCorners[3].y = it->second->maxy;
    */

    //return fiducialCorners;

//}

/* Getter functions for the tissue and needle images */
/*
Mat FiducialBase::getImage()
{
    return image;
}

std::vector<int> FiducialBase::getThresholdRange()
{
    return thresholdRange;
}

void FiducialBase::setThresholdRange(std::vector<int> thresholdRangeIn)
{
    thresholdRange=thresholdRangeIn;
}

std::vector<int> FiducialBase::getBlobRange()
{
    return blobRange;
}


int FiducialBase::getNumObjects()
{
    return numObjects;
}


std::vector<Point2f> FiducialBase::getCentroids()
{
    return centroids;
}

void FiducialBase::setCentroids(std::vector<Point2f> centroids)
{
    this->centroids = centroids;
}
*/

/*  Give the entry point of the needle create a sub image or "region of intrest" 
 *  that is focused soley on the needles exit point.
 * @param tissueCentroid - Location of pink tissue marker
 * @param start - The distance from the top left corner pixel of the roi rectangle to the needle entry point
 * @param width - The desired width of the roi rectangle 
 * @param height - The desired height of the roi rectangle
 */


/** This function performs hsv thresholding on a given image in the range of
 *  target - tolerance < target + tolerance
 * @param chroma0Target - Hue component threshold value
 * @param chroma0Offset - Hue component tolerance 
 * @param chroma1Target - Saturation component threshold value
 * @param chroma1Offset - Saturation component tolerance
 */


/*
Mat FiducialBase::thresholdImage(Mat* imageIn=NULL)
{
    if(imageIn != NULL) this->image = imageIn->clone();
    Mat frame = getImage(); //Source image


    //Allow for the possibility of multiple colors simultaneously.


    std::vector<int> thresholdValues = getThresholdRange();


    Mat threshOut;
    Mat finalThresh;
    inRange(frame,Scalar(0,0,0),Scalar(0,0,0),threshOut);
    threshOut.setTo(0);


    //imageHSV;  //HSV Destination matrix
    cvtColor(frame, imageHSV, CV_BGR2HSV_FULL);  //Convert to HSV



    int size = thresholdValues.size();

    if(size%4 == 0){ 
        //The for loop allows for multiple colors to be detected
        //simultaneously. Only one color is still supported.	
        int count = size/4;
        for(int i = 0; i<count; i++){

            int c0 = thresholdValues[i*4]%256; //chroma0 target
            int c1 = thresholdValues[i*4+1]%256; //chroma1 target
            int c0t = thresholdValues[i*4+2]%128; //chroma0 tolerance
            int c1t = thresholdValues[i*4+3]%128; //chroma1 tolerance

            Mat thresh;
            Mat thresha;
            Mat threshb;

            //cvtColor(frame, hsv, CV_BGR2GRAY);  //Convert to gray  
            //std::cout << frame.channels() << frame.depth() << std::endl;
            //Compute the thresholding for the tolerance state:
            //if hue is 

            if(c0-c0t < 0)
			{
				int cLower = 256+(c0-c0t);
				int cUpper = c0+c0t;
				inRange(imageHSV,Scalar(0,c1-c1t,0),Scalar(cUpper,c1+c1t,255),thresha);
				inRange(imageHSV,Scalar(cLower,c1-c1t,0),Scalar(255,c1+c1t,255),threshb);
				bitwise_or(thresha,threshb,thresh);
			}
			else if(c0+c0t > 255)
			{
				int cLower = c0-c0t;
				int cUpper = c0+c0t-256;
				inRange(imageHSV,Scalar(0,c1-c1t,0),Scalar(cUpper,c1+c1t,255),thresha);
				inRange(imageHSV,Scalar(cLower,c1-c1t,0),Scalar(255,c1+c1t,255),threshb);
				bitwise_or(thresha,threshb,thresh);
			}
			else inRange(imageHSV,Scalar(c0-c0t,c1-c1t,0),Scalar(c0+c0t,c1+c1t,255),thresh);

			//continue to append additional thresholds into the 
			bitwise_or(thresh,threshOut,threshOut);
		}
		//Dilate the white image:
		int kSize;
		Size imgSize = threshOut.size(); 
		if(imgSize.width >  650) kSize = 7;
		else kSize = 3;

		Mat element = getStructuringElement( MORPH_RECT, Size( kSize, kSize ),Point( 1, 1 ) );
		Mat dilOutput;
		dilate( threshOut, dilOutput, element );


		//Subtract white space and dark spaces:
		Mat colorLess;
		Mat Dark;
		Mat colorLight;
		Mat colorLessDark;
		
		//TODO: remove hard coding?
		//HARDCODE
		inRange(imageHSV,Scalar(0,0,0),Scalar(255,90,255),colorLess);
		inRange(imageHSV,Scalar(0,0,0),Scalar(255,255,20),Dark);

		bitwise_or(colorLess,Dark,colorLessDark);
		bitwise_not(colorLessDark,colorLight);

		bitwise_and(dilOutput,colorLight,finalThresh);


		if(this->displayOn){
			imshow("Original", frame);
			imshow("HSV", imageHSV);
			imshow("Thresholding", threshOut);
			imshow("Dilation", dilOutput);
			imshow("colorLight", colorLight);
			imshow("FinalOutput", finalThresh);
			waitKey(0);
			cvDestroyWindow("Original");
			cvDestroyWindow("HSV");
			cvDestroyWindow("Thresholding");
			cvDestroyWindow("Dilation");
			cvDestroyWindow("colorLight");
			cvDestroyWindow("FinalOutput");


		}
	}

	return finalThresh;
}
*/

/*
//This file loads in the 6 numbers associated with the 
//thresholding
int FiducialBase::loadBlobFile(const char* fileName){

	
	std::vector<int> thresholdRangeTemp;
	std::vector<int> blobRangeTemp;
	
	//Clear the new vectors:
	thresholdRangeTemp.clear();
	blobRangeTemp.clear();


	std::ifstream infile(fileName,std::ifstream::in);

	if(!infile) return -1; //On fail return -1 
	string line;
	getline(infile,line);	

	
	string item;
	std::istringstream linestream(line); 

	//Append to the color list:
	getline(linestream,item,',');

	int c0 = atoi(item.c_str())%256;
			
	getline(linestream,item,',');

	int c0t = atoi(item.c_str())%128;
			
	getline(linestream,item,',');

	int c1 = atoi(item.c_str())%256;

	getline(linestream,item,',');

	int c1t = atoi(item.c_str())%128;

	getline(linestream,item,',');
	
	int lower  = atoi(item.c_str());

	getline(linestream,item,',');
	
	int upper = atoi(item.c_str());
		
	thresholdRangeTemp.push_back(c0);
	thresholdRangeTemp.push_back(c1);
	thresholdRangeTemp.push_back(c0t);
	thresholdRangeTemp.push_back(c1t);

	blobRangeTemp.push_back(lower);
	blobRangeTemp.push_back(upper);

	blobRange = blobRangeTemp;
	thresholdRange = thresholdRangeTemp;

	return 1;
}
*/


/*
void FiducialBase::ApproxRect()
{
	//Go through the contours

	std::vector<std::vector<Point>> contourApprox;
	
	int n = (int) contours.size();
	contourApprox.resize(n);
	for(int i = 0; i < n; i++){

		approxPolyDP(contours[i],contourApprox[i] , 50, true);

	}
	Mat contourA(image.rows,image.cols,CV_8UC1);
	Mat contourB(image.rows,image.cols,CV_8UC1); 
	Scalar color = Scalar(255,255,255);
	contourA = 00;
	vector<Vec4i> lines;
	Mat contourAlined = contourA.clone();
	contourB = 00;

	drawContours(contourA, contours, -1, color);
	
	HoughLinesP(contourA, lines, 1, CV_PI/180, 80, 30,10);
    for( size_t i = 0; i < lines.size(); i++ )
    {
			line( contourAlined, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(255), 3, 8 );
    }

	drawContours(contourB, contourApprox, -1, color);

	Mat contourC =contourA*0.5+contourB*0.5;
	/*
	imshow("contoursLined",contourAlined);
	imshow("contours",contourC);
	waitKey(0);
	destroyWindow("contours"); * /
	//Display Approximations:

}*/

/** This function search for contours in the thresholded image
 * @param thresh Mat An image that has gone though thresholding.
 */

/*
Mat FiducialBase::locateContours(Mat thresh)
{

 
  //vector<vector<Point> > contours; //Destination vector for the list of contours in the image
  vector<Vec4i> hierarchy; //Destination vector for hierarchial organization oft he contours
  Mat drawing = Mat::zeros(thresh.size(), CV_8UC3); //Destination matrix for the image contours

  //Find the contours of the image
  findContours( thresh, contours, hierarchy,
                CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
	
  	//Draw contours
	  for(unsigned int i = 0; i< contours.size(); i++ )
	  {
			 Scalar color = Scalar(255,255,255);
			drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
	}

	cvtColor(drawing, drawing, CV_BGR2GRAY);

  if(this->displayOn)
  {

	imshow("Contours", drawing);
	waitKey(0);
	cvDestroyWindow("Contours");
  }
  return drawing;
   
}*/

/** Searchs for blobs in an image and returns the farthest left blob fits the size description of a needle
 * @param diff Mat a binary image with contours highlighted if detecting needle, if detecting tissue a simple thresholded image
 * @param numObject the number of objects being searched for
 * @return centroidPoint a vector of the object(s) points
 */
/*

void FiducialBase::locateBlobs(Mat diff, int numObjects)
{ 

	std::vector<int> blobSize = getBlobRange();
	int lowerBound = blobSize[0];
	int upperBound = blobSize[1];

	std::vector<double> contourAreas;
	std::vector<std::vector<Point>> contourHulls;
	std::vector<std::vector<Point>> outputContourHulls;
	contourHulls.resize(contours.size());
	contourAreas.resize(contours.size());


	for(int i = 0; i < contours.size(); i++){
		convexHull(contours[i],contourHulls[i]);
		contourAreas[i] = contourArea(contourHulls[i]);
	}
	//Find the needle exit 	

	//Find the blobs in the image
	//IplImage diffIpl = diff; //Convert the image Mat into an IPL header                                                     
	//IplImage *label = cvCreateImage(cvSize(diffIpl.width,diffIpl.height), IPL_DEPTH_LABEL, 1);


	
//	unsigned int result = cvb::cvLabel(&diffIpl, label, blobs); //make a label                                               \

//	IplImage *blobIplPtr = cvCreateImage(cvSize(diffIpl.width, diffIpl.height),IPL_DEPTH_8U,3);

	//Sort and Filter blobs by size
	sort(contourHulls.begin(),contourHulls.end(),contourAreaSortingHL);

	outputContourHulls.clear();	
	std::vector<double> outputAreas;
	outputAreas.clear();
	for(int i = 0; i < contourHulls.size(); i++){
		double hullArea = contourArea(contourHulls[i]);
		if(hullArea < upperBound && hullArea > lowerBound){
			outputContourHulls.push_back(contourHulls[i]);
			outputAreas.push_back(hullArea);
		}
	}
	
	


	//Find the centroids of each blob
	std::vector <Point2f> centroidPoint(numObjects);

	int i = 0; //Iterator for filling the vector of points

	//Add each blob to a vector of centroids and display its information
	for (int j = 0; j < outputContourHulls.size(); j++)
	{
		Moments tempMoment=moments(outputContourHulls[j]);
		Point2f point( tempMoment.m10/tempMoment.m00, tempMoment.m01/tempMoment.m00);
		centroidPoint[i] = point;
		
		if(this->displayOn)
		{
			std::cout << "Blob #" << j << ": Area=" << outputAreas[j] << ", Centroid=(" << point.x << ", " << point.y << ")\n";
		}

		i++;
		if(i >= numObjects) break;
	}

	//Now that the points have been found, sort them:
	
	sort(centroidPoint.begin(),centroidPoint.end(),pointYSortingHL);


	for(i = 0; i < numObjects; i++)
	{
		std::cout << "Blob #: x= " << centroidPoint[i].x << ", y= " << centroidPoint[i].y << std::endl;
		std::cout << std::endl;
	}


	setCentroids(centroidPoint);



	//Display Blobed Image
	if(this->displayOn)
	{
		Mat display = image.clone();
		drawContours( display, outputContourHulls, -1, Scalar(255,255,0), CV_FILLED, 8);
	//	cvNamedWindow("Blobs", CV_WINDOW_AUTOSIZE);
		imshow("Blobs", display);
		cvWaitKey(0);
		cvDestroyWindow("Blobs");
	}
	//cvReleaseImage(&blobIplPtr);
	//cvReleaseImage(&label);
}

std::vector<Point2f> FiducialBase::search()
{

    int numberOfObjects = getNumObjects();


    Mat thresholded = thresholdImage();
    Mat contoured = locateContours(thresholded);
    locateBlobs(contoured, numberOfObjects);

    return getCentroids();
}

std::vector<Point> FiducialBase::search2()
{
    int numberOfObjects = getNumObjects();
    Mat thresholded = thresholdImage();
    Mat contoursOut = locateContours(thresholded);

    sort(contours.begin(),contours.end(),contourCompare);
    std::vector<std::vector<Point>> contourApprox;
    int n = (int) contours.size();
    contourApprox.resize(1);
    //Only use the largest contour by area:
    //This function is meant to be called a few times only at the beginning: no subsequent
    //calls: (the robots might be interfering etc:


    approxPolyDP(contours[0],contourApprox[0] , 10, true);

	int i = 5;
	if(contourApprox[0].size() > 4){
		while(true){
			i = i+15;
			std::vector<Point> contourApproxTemp = contourApprox[0];
			approxPolyDP(contourApproxTemp,contourApprox[0] , i, true);
			if(contourApprox[0].size() < 5) break;
		} 
	}

	if(contourApprox[0].size() == 4){
		//	The data is now going to be sorted:
		sort(contourApprox[0].begin(),contourApprox[0].end(),pointYSortingLH);
		sort(contourApprox[0].begin(),contourApprox[0].begin()+2,pointXSortingHL);
		sort(contourApprox[0].begin()+2,contourApprox[0].begin()+4,pointXSortingLH);

		if(this->displayOn){
			Scalar color = Scalar(0,255,255);
			Mat display2 = image.clone();
			drawContours(display2, contourApprox, -1, color);
			Mat display3 = image.clone();
			drawContours(display3, contours, -1, color);

			imshow("contours V0",display3);
	
			imshow("contours V1",display2);
			waitKey(0);
			destroyWindow("contours V1");
			destroyWindow("contours V0");
		}

		
		std::vector<std::vector<Point>> contourTemp; 
		contourTemp.resize(1);
		contourTemp[0] = contourApprox[0];
		//Fit the quadrilateral..
		double errorQuad = 1000000;

		int cLength = contours[0].size();
		double cLengthDiv = 1/((double) cLength);

		while(errorQuad > 0.5){

			//Go through each point of the tissue contour:
			Point2d deltaSum[4]; //point update vector:
			double errorOut = 0.0;
			for(int j = 0; j < 4; j++){
				deltaSum[j].x = 0.0;
				deltaSum[j].y = 0.0;
			}


			for(int k = 0; k < cLength; k++){
				
				//find the line that is closest to the particular point.
				double localError = 100000;
				double localLambda = -1;
				int lineC       = -1;
				Point2d errVOut;

				for(int j = 0; j < 4; j++){
					//pts:
					Point Pt0i =  contourTemp[0][j];
					Point Pt1i =  contourTemp[0][(j+1)%4];
					Point PtTi = contours[0][k];
					
					Point2d Pt0;
					Pt0.x = (double) Pt0i.x;
					Pt0.y = (double) Pt0i.y;

					Point2d Pt1;
					Pt1.x = (double) Pt1i.x;
					Pt1.y = (double) Pt1i.y;

					Point2d PtT;
					PtT.x = (double) PtTi.x;
					PtT.y = (double) PtTi.y;


					Point2d vL = Pt1-Pt0;
					Point2d vE = PtT-Pt0;
					double vLNormI = 1/norm(vL);
					//Compute the convex parameter lambda:
					double lambda = (vL.x*vE.x+vL.y*vE.y)*vLNormI*vLNormI;
					
					//constrain lambda to be zero or 1.
					if(lambda < 0) lambda = 0.0;
					if(lambda > 1) lambda = 1.0;

					//error:
					Point2d errorV = (PtT-((lambda*Pt1)+(1-lambda)*Pt0));
					double errorH  = norm(errorV);

					//Check if the line is a best Fit:
					if(errorH < localError){
						localError = errorH;
						localLambda = lambda;
						lineC      = j;
						errVOut    = errorV;
					}		

				}
				//Now the fit for the line has been established, add a little
				//bit to the offset vector:
				errorOut = errorOut+localError;
				if(localLambda > -1){
					deltaSum[lineC] = deltaSum[(lineC+1)%4]+errVOut*localLambda*cLengthDiv; 
					deltaSum[(lineC+1)%4] = deltaSum[lineC]+errVOut*(1-localLambda)*cLengthDiv;
				}
				
			
			}
			//Done going through the contour Now update the point information:

			if(errorOut >= errorQuad) break;
			errorQuad = errorOut;
			for(int j = 0; j < 4; j++){
				Point2d pt1 = contourTemp[0][j];
				Point2d pt2 = deltaSum[j];
				contourTemp[0][j] = pt1+pt2*0.2;
			}
			
			if(this->displayOn){
			Scalar color = Scalar(0,255,255);
			Mat display2 = image.clone();
			drawContours(display2, contourTemp, -1, color);
			imshow("contours VU",display2);
			waitKey(10);
			destroyWindow("contours VU");
		}
	
		
		}


		if(this->displayOn){
			Scalar color = Scalar(0,255,255);
			Mat display2 = image.clone();
			drawContours(display2, contourApprox, -1, color);
			imshow("contours V1",display2);
			waitKey(0);
			destroyWindow("contours V1");
		}
	//Display Approximations:

	//Sort the points appropriately:


	return contourTemp[0];
	//return contourApprox[0];
	}
	else
	{
		contourApprox[0].clear();	
		return contourApprox[0];
	}

}
*/


/*
//This function uses a starting point to grow a segmented blob.
void growBlobBW(const Mat & inputImg , Mat & outputImg,Point2f startPt)
{

    //Since the ellipse is Black on White,  convert to greyscale
    Mat grayImg;
    cvtColor(inputImg,grayImg,CV_BGR2GRAY);


    //define a initial ROI for the segmentation and mask.
    int roiSide = 15;
    Rect centerROI((Point) startPt-Point(roiSide,roiSide),Size(roiSide*2+1,roiSide*2+1));  //center point
    Mat localRegion = grayImg(centerROI);


    Mat localMask,localMaskROI;
    localMask.create(grayImg.size(),CV_8UC1);
    localMask = 0;

    localMaskROI = localMask(centerROI);
    localMaskROI = 255;  //dataType is 8 bit unsigned.


    //initially use the entire image mean
    Scalar meanSm,stdDevSm;
    meanStdDev(grayImg,meanSm,stdDevSm);

    double cutoff = meanSm[0];

    Mat segmentedImg;

    for(int count = 0; count < 10; count++)
    {

        //segment the image and combine it with the mask
        threshold(grayImg,segmentedImg,cutoff,255,THRESH_BINARY_INV);

        bitwise_and(segmentedImg,localMask,segmentedImg);

        //use the segmentation as a mask for the new cutoff.
        meanStdDev(grayImg,meanSm,stdDevSm,segmentedImg);


        cutoff = (1-stdDevSm[0]/meanSm[0])*127;

        //dilate the segmentation into the image mask.
        dilate(segmentedImg,localMask,Mat());


#ifdef DEBUG_GROWBLOBBW
        imshow("Segmented Image",segmentedImg);
        cvWaitKey(0);
#endif
    }

    outputImg = localMask.clone();

}
*/

//#define DEBUG_GROWQUADBW
//#define DEBUG_QUADITERATE

void detectBlock(const Mat &inputImg, Point seedPt, std::vector<Point2f> &corners,bool display)
{
    bool localDisplay = true;

    int newMaskVal = 255;
    Scalar newVal = Scalar( 120, 120, 120 );

    int connectivity = 8;
    int flags = connectivity | (newMaskVal << 8 ) | FLOODFILL_FIXED_RANGE | FLOODFILL_MASK_ONLY;

    int lo = 20;
    int up = 20;

    Mat mask2 = Mat::zeros( inputImg.rows + 2, inputImg.cols + 2, CV_8UC1 );
    floodFill( inputImg, mask2, Point(seedPt.x,seedPt.y), newVal, 0, Scalar( lo, lo, lo ), Scalar( up, up, up), flags );
    Mat mask = mask2( Range( 1, mask2.rows - 1 ), Range( 1, mask2.cols - 1 ) );


    Mat elementOpen = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    Mat elementClose = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

    Mat openMask;
    Mat closeMask;

    morphologyEx(mask, openMask, MORPH_OPEN, elementOpen);
    morphologyEx(openMask, closeMask, MORPH_CLOSE, elementClose);

    //outputImg = closeMask.clone();
    if(display)
    {
        imshow( "TempImage", openMask);
        waitKey(0);
        destroyWindow("TempImage");
    }


    std::vector< std::vector<Point> > contours;

    findContours(closeMask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

    if(display)
    {
        ROS_INFO("There are  %d total contours",(int) contours.size());
    }

    sort(contours.begin(),contours.end(),contourCompare);

    if(display)
    {
        ROS_INFO("The largest contour is %d.", (int) contours[0].size());
    }

    RotatedRect minRect = minAreaRect(contours[0]);
    Point2f ptArray[4];

    minRect.points(ptArray);

    std::vector<cv::Point2f> outputPts(ptArray,ptArray+sizeof(ptArray)/sizeof(Point2f));

    if(display)
    {
        ROS_INFO("Finished finding the RotatedRect and its associated point array");
        for(int ind=0; ind <4; ind++)
        {
            ROS_INFO_STREAM(outputPts[ind]);
        }
    }

    refineBlock(inputImg,outputPts,display);
    corners = outputPts;


}


int refineBlock(const cv::Mat& inputImg, std::vector<cv::Point2f> & points,bool display)
{


     if(display) ROS_INFO("started refinement");
     //sort all of the y vectors from low to high
     std::vector<cv::Point2f> tempPoints = points;
     sort(tempPoints.begin(),tempPoints.end(),pointYSortingLH);
     //first row from High to Low
     sort(tempPoints.begin(),tempPoints.begin()+2,pointXSortingHL);
     //second row from Low to High
     sort(tempPoints.begin()+2,tempPoints.begin()+4,pointXSortingLH);

     /* Point layout (rough)
         *     1      0
         *     2      3
        */
    //Now spread the points by 2 pixels each.
    float spread = 4.0;
    tempPoints[0] += Point2f(spread,-spread);
    tempPoints[1] += Point2f(-spread,-spread);
    tempPoints[2] += Point2f(-spread,spread);
    tempPoints[3] += Point2f(spread,spread);

    /*std::vector<Point2f> rectPts = tempPoints;

    std::vector< std::vector<Point> > contourTemp; 
    contourTemp.resize(1);
    contourTemp[0].resize(4);

    for(int ind = 0; ind < 4; ind++)
    {
        contourTemp[0][ind].x = tempPoints[ind].x;
        contourTemp[0][ind].y = tempPoints[ind].y;
    }
    //Fit the quadrilateral..
    double dPt = 10;

    //iterate the quadrilateral:
    //change the iteration so that it works by moving the line by maximizing the gradient.
    while(dPt > 1){

    if(display)
    {
        ROS_INFO_STREAM("Finding the ROI ");
    }

    //bounding Rect
    Rect tempRegion = boundingRect(tempPoints);

    if(display)
    {
        ROS_INFO_STREAM("The ROI rectangle is " << tempRegion);
    }

    //pad the rectangle by 8 pixels in all directions.
    tempRegion += Point(-8,-8);
    tempRegion += Size(16,16);

            //points are sorted to be clockwise
            Mat subGImg = inputImg(tempRegion);
            //Mat subGImg;

            //cvtColor(subImg,subGImg,CV_BGR2GRAY);

            Mat subX,subY;
            Mat subXX,subXY,subYY;

            Sobel(subGImg,subXX,CV_32F,2,0,5);
            Sobel(subGImg,subXY,CV_32F,1,1,5);
            Sobel(subGImg,subYY,CV_32F,0,2,5);
            Sobel(subGImg,subX,CV_32F,2,0,5);
            Sobel(subGImg,subY,CV_32F,1,1,5);



            //Go through each point of the tissue contour:
            Point2f deltaSum[4]; //point update vector:
            for(int j = 0; j < 4; j++){
                deltaSum[j].x = 0.0;
                deltaSum[j].y = 0.0;
            }

            //find the agregate mean:
            Mat binImg;
            double threshVal = threshold(subGImg,binImg,0,255,THRESH_BINARY_INV+THRESH_OTSU);


            //cycle through the 4 quadrilateral lines.
            //optimize each one.
            for(int k = 0; k < 4; k++){

                //create the line:
                //offset the points to be within the ROI:
                Point pt1(rectPts[k].x-tempRegion.tl().x,rectPts[k].y-tempRegion.tl().y);
                Point pt2(rectPts[(k+1)%4].x-tempRegion.tl().x,rectPts[(k+1)%4].y-tempRegion.tl().y);

                //tangent and normal vectors:
                //normal vector points out of the shape.
                Point2f ptT(rectPts[(k+1)%4].x-rectPts[k].x,rectPts[(k+1)%4].y-rectPts[k].y);
                ptT *= 1/(norm(ptT));

                Point2f ptN;
                ptN.x = -ptT.y;
                ptN.y = ptT.x;

                LineIterator side(subGImg,pt1,pt2,8);

                //iterate through the line points.
                for(int i = 0; i < side.count; i++, ++side)
                {
                    double lambda = norm(side.pos()-pt1)/norm(pt1-pt2);

                    int val   = (int) subGImg.at<char>(side.pos());
                    float valX  = subX.at<float>(side.pos());
                    float valY  = subY.at<float>(side.pos());
                    float valXX = subXX.at<float>(side.pos());
                    float valXY = subXY.at<float>(side.pos());
                    float valYY = subYY.at<float>(side.pos());

                    //now that the derivative values are computed, the points should be pushed
                    //compute the local line offset...

                    //compute the derivative magnitude:
                    Point2f dMagd;
                    dMagd.x = (valX*valXX+valY*valXY)/(255*25*side.count*10);
                    dMagd.y = (valX*valXY+valY*valYY)/(255*25*side.count*10);



                    //align the dMag to the normal direction. (maximize)
                    //float ip = dMagd.dot(ptN);
                    //simple mean version:
                    float ip = ((float) threshVal-(float) val)/800.0; //val < threshold means to expand.
                    //val > threshold contract

                    deltaSum[k] += ptN*ip*(1-lambda);
                    deltaSum[(k+1)%4] += ptN*ip*(lambda);
                }
            }// for(int k = 0; k < 4; k++) //corner iteration

            dPt = 0.0;
            //update the line here.
            for(int i = 0; i < 4; i++)
            {
                rectPts[i] += deltaSum[i];
                dPt += norm(deltaSum[i])*norm(deltaSum[i]);
            }

            /*if(display)
            {
                Scalar color = Scalar(255,255,255);
                Mat displayImg = inputImg.clone();
                for(int ind = 0; ind < 4; ind++)
                {
                    line(displayImg,(Point) rectPts[ind],(Point) rectPts[(ind+1)%4],color);
                }
                imshow("Quadrilateral Fit",displayImg);
                waitKey(0);
                destroyWindow("Quadrilateral Fit");
            }* /

        }//while(dPt > 0.01); */

        if(display) ROS_INFO("Completed iterating the points");

        std::vector< Point2f > oldPoints = points;

        //Done going through the contour Now update the point information:
        cornerSubPix(inputImg,points,Size(spread,spread),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,3,0.01));

        int n = points.size();

        for(int indi = 0; indi < n; indi++)
        {
            for(int indj=0; indj < indi; indj++)
            {
                double dist = norm(points[indi]-points[indj]);
                if(dist < 3)
                {
                    points = oldPoints;
                    return -1;
                }
            }
        }
        return 1;
//        points = rectPts;
}


/*
void quadrilateralBW(const Mat &inputImg, Point2f startPt, std::vector<Point> &corners)
{
    bool localDisplay = true;
    //segment the image:

	Rect imgROI((Point)(startPt-Point2f(50,50)),Size(101,101));

	Mat inputImgROI = inputImg(imgROI);

    Mat edgeImg;

	std::ifstream infile("localFiles\\canny.local.seg",std::ifstream::in);

	int c1 = 10;
	int c2 = 35;
	int a = 3;

	if(infile)
	{
			 
		string line;
		getline(infile,line);	
		string item;
		std::istringstream linestream(line); 

		//Append to the color list:
		getline(linestream,item,',');

		c1 = atoi(item.c_str());
			
		getline(linestream,item,',');

	    c2 = atoi(item.c_str());
			
	    getline(linestream,item,',');

		a = atoi(item.c_str());
	
	}

    if(localDisplay){
            imshow("Quadrilateral Image",inputImgROI);
            waitKey(0);
            destroyWindow("Quadrilateral Image");
    }

    Canny(inputImgROI,edgeImg,c1,c2,a,true);

    if(localDisplay){
        imshow("Quadrilateral Edge Image",edgeImg);
        waitKey(0);
        destroyWindow("Quadrilateral Edge Image");
    }

    //contours:

}
*/



void sortPtGrid(std::vector<Point2f> &pointArray,Size gridSize,bool lr)
{
	//always 

	sort(pointArray.begin(),pointArray.end(),pointYSortingLH);
    for(int i = 0; i < gridSize.height; i++)
	{
		if(lr) sort(pointArray.begin()+i*gridSize.width,pointArray.begin()+(i+1)*gridSize.width,pointXSortingLH);
		else sort(pointArray.begin()+i*gridSize.width,pointArray.begin()+(i+1)*gridSize.width,pointXSortingHL);
	}
		

}



void exportPointFile(const char* filePath,std::vector< std::vector< Point3f > > &inputPoints){

    FILE* pFile = fopen(filePath,"w");

    if(pFile ==NULL)
    {
        printf("Unable to open the ellipse file.\n");
        return;
    }

    for(int ind = 0; ind < inputPoints.size(); ind++)
    {
        fprintf(pFile,"%d",ind);
        for(int jnd = 0; jnd < inputPoints[ind].size(); jnd++)
        {
            fprintf(pFile,", %lf, %lf,%lf"
								,inputPoints[ind][jnd].x
								,inputPoints[ind][jnd].y
                                ,inputPoints[ind][jnd].z);
        }
        fprintf(pFile,"\n");
	}
	fclose(pFile);
}

