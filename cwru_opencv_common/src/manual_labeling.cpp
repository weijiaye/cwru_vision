//
// Created by tipakorng on 6/1/16.
//

#include <cwru_opencv_common/manual_labeling.h>

ManualLabeling::ManualLabeling(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle) {
    manualLabelingServer_ = nodeHandle_.advertiseService("manual_labeling_service",
                                                         &ManualLabeling::manualLabelingCallback, this);
    ROS_INFO("Manual image labeling  server initialized");
}

ManualLabeling::~ManualLabeling() { }

bool ManualLabeling::manualLabelingCallback(cwru_opencv_common::image_label::Request& request,
                                            cwru_opencv_common::image_label::Response& response) {
    // Put sensor_msgs::Image in cv::Mat
    const sensor_msgs::Image::Ptr &image = boost::make_shared<sensor_msgs::Image>(request.imageQuery);
    cv::Mat rawImageIn = cv_bridge::toCvShare(image, "bgr8")->image.clone();
    // Declare labeled image
    cv::Mat labeledImage;
    // Initialize blobs
    std::vector<cv::Mat> labeledBlobs;
    labeledBlobs.clear();
    // Apply adaptive threshold
    cv::Mat imgHSV;
    cv::cvtColor(rawImageIn, imgHSV, CV_BGR2HSV);
    // create a click window:
    cv::Point imagePt(-1, -1);
    cv::namedWindow("Selectable Points");
    cv::setMouseCallback("Selectable Points", cv_ui::getCoordinates, &imagePt);
    int imageCount(0);
    // fill the blobs.
    while (true) {
        int blobCount = (int) floor((double) imageCount / 2.0);
        int lr = imageCount % 2; // select the points from right side to left side
        imshow("Selectable Points", rawImageIn);

        char keyIn = cv::waitKey(50);

        if (imagePt.x > 0) {

            cv::Mat labeledBlob;
            labeledBlob = cv::Mat::zeros(rawImageIn.size(), CV_8UC1);

            ManualLabeling::growFromSeedRaw(rawImageIn, labeledBlob, imagePt);
            std::vector<std::vector<cv::Point2i> > contours;
            cv::Mat hierarchy;
            findContours(labeledBlob, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
            cv::RotatedRect newRect(fitEllipse(contours[0]));
            ROS_INFO("Finished point at index %d \n Image side %d \n next point.\n", blobCount, lr);
            labeledBlobs.push_back(labeledBlob);
            imageCount++;
            imagePt.x = -1;
        }
        // if the Esc key is pressed, break out.
        if (keyIn == 27) break;
    }
    cv::destroyWindow("Selectable Points");
    printf("Finished the catheter display image. \n");

    // Merge blob to label image
    if(labeledBlobs.size() > 0)
    {
        merge(labeledBlobs, labeledImage);
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", labeledImage).toImageMsg(response.imageResp);
        return true;
    }
    else
        return false;
}

int ManualLabeling::growFromSeedRaw(const cv::Mat& rawImage, cv::Mat&labeledImage, cv::Point2i seedPoint) {
    //assume that the labeled Mat is allocated.
    cv::Mat segmentedImage;
    int rectOff = 100;
    int rectSide = rectOff*2+1;
    //The model of the image is that is a orange blob.
    cv::Rect ROI(seedPoint.x - rectOff, seedPoint.y - rectOff, rectSide, rectSide);

    catheterFloodFillSegmentation(rawImage, segmentedImage, seedPoint, ROI);
    //catheterImageSegmentation(rawImage, segmentedImage,seedPt,ROI);

    return growFromSeed(segmentedImage, labeledImage, seedPoint);
}

int ManualLabeling::growFromSeed(const cv::Mat& segmentedImage, cv::Mat& labeledImage, cv::Point2i seedPoint)
{
    //assume that the labeled Mat is allocated.

    //if the seed point is meaningful, use it to set the prelabeled image.
    if(seedPoint.x > 0)
    {
        labeledImage = cv::Scalar(0);
        labeledImage.at<uchar>(seedPoint) = 255;
    }
        //else perform an initial guess.
    else dilate(labeledImage, labeledImage, cv::Mat(), cv::Point2i(-1, -1), 15);
    int sumState = 0;
    int iterations = 0;
    while(iterations < 50)
    {
        dilate(labeledImage, labeledImage, cv::Mat(), cv::Point2i(-1, -1), 1);
        labeledImage = labeledImage & segmentedImage;
        cv::Scalar sumStateT = sum(labeledImage);
        int newSum = sumStateT[0];
        if(newSum == sumState) break;
        sumState = newSum;
        iterations++;
    }

    return 1;
}

int ManualLabeling::catheterFloodFillSegmentation(const cv::Mat& inputImage, cv::Mat &outputImg, cv::Point2i seedPt, cv::Rect regionIn) {
    cv::Mat inputImageHSV;
    outputImg = cv::Mat::zeros(inputImage.size(), CV_8UC1);
    cvtColor(inputImage, inputImageHSV, CV_BGR2HSV);


    int newMaskVal = 255;
    cv::Scalar newVal = cv::Scalar(120, 120, 120);

    int connectivity = 8;
    int flags = connectivity + (newMaskVal << 8 ) + cv::FLOODFILL_FIXED_RANGE + cv::FLOODFILL_MASK_ONLY;

    int lo = 10;
    int up = 10;

    cv::Mat mask2;
    mask2 = cv::Mat::zeros(inputImage.rows + 2, inputImage.cols + 2, CV_8UC1);
    floodFill(inputImage, mask2, seedPt, newVal, 0, cv::Scalar(lo, lo, lo), cv::Scalar(up, up, up), flags);
    cv::Mat mask = mask2(cv::Range( 1, mask2.rows - 1 ), cv::Range( 1, mask2.cols - 1 ) );

    cv::Mat elementOpen = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat elementClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));

    cv::Mat openMask,closeMask;

    morphologyEx(mask, openMask, cv::MORPH_OPEN, elementOpen);
    morphologyEx(openMask, closeMask, cv::MORPH_CLOSE, elementClose);

    outputImg = closeMask.clone();

    return 1;
}
