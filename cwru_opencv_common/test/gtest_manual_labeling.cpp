//
// Created by tipakorng on 6/6/16.
//

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <cwru_opencv_common/image_label.h>
#include <gtest/gtest.h>
#include "../include/cwru_opencv_common/manual_labeling.h"

struct ManualLabelingTest : testing::Test {

    ManualLabeling* manualLabeling;

    ros::NodeHandle nodeHandle;

    ManualLabelingTest() {
        manualLabeling = new ManualLabeling(nodeHandle);
    }

    ~ManualLabelingTest() {
        delete manualLabeling;
    }
};

int importRawImage(const char* imagePath, cv::Mat &outputImage){

    FILE* rawFile;
    int iImageSize = 1080*1920;

    if ((rawFile = fopen(imagePath, "rb")) == NULL)
    {
        //cout << "Error opening file: " << imagePath << endl;
        return -1;
    }

    fseek(rawFile, 0, SEEK_END);
    int fileSize = ftell(rawFile);
    rewind(rawFile);


    unsigned char* imageBuffer = (unsigned char*)  calloc(sizeof(unsigned char),fileSize);
    int imageSize = fread(imageBuffer, sizeof(unsigned char),fileSize, rawFile);

    cv::Mat inputRaw;

    if(imageSize == 3*iImageSize)
    {
        //ROS_INFO("image size is 3 x ");
        inputRaw = cv::Mat(1080,1920,CV_8UC3,imageBuffer);
        outputImage = inputRaw.clone();
    }
    else if(imageSize == iImageSize)
    {
        //ROS_INFO("image size is 1 x ");
        inputRaw = cv::Mat(1080,1920,CV_8UC1,imageBuffer);
        cvtColor(inputRaw, outputImage, CV_BayerBG2BGR);

    }
    else{
        if(feof)
            ROS_INFO("Error reading image: loadSize was %d",imageSize);
        if(feof(rawFile))
        {
            ROS_INFO("End of File reached");

        }
        if(ferror(rawFile))
        {
            ROS_INFO("A file io error occured"); //,explain_fread(g_srcBuffer,sizeof (char),iImageSize,rawFile));
            perror("What happened?");
        }
        fclose(rawFile);
        free(imageBuffer);
        return 0;
    }

    free(imageBuffer);
    fclose(rawFile);
    return 1;
}

TEST_F(ManualLabelingTest, testManualLabelingCallback) {
    // Import image
    std::string imagePathStr = ros::package::getPath("cwru_opencv_common") + "/test/test_images/test_4_comb_2016-02-01-135224-0000.raw";
    const char* imagePath = imagePathStr.c_str();
    cv::Mat imageCv;
    importRawImage(imagePath, imageCv);
    // Convert cv::Mat to sensor_msgs::Image
    sensor_msgs::Image image;
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageCv).toImageMsg(image);
    // Put image in service request
    cwru_opencv_common::image_label srv;
    srv.request.imageQuery = image;
    srv.request.requestedPoints = 10;
    // Call service
    manualLabeling->manualLabelingCallback(srv.request, srv.response);
    // Convert sensor_msgs::Mat back to cv::Mat
    cv_bridge::CvImagePtr cvImagePtr;
    cvImagePtr = cv_bridge::toCvCopy(srv.response.imageResp, sensor_msgs::image_encodings::BGR8);
    // Show image
    cv::namedWindow("Labeled image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Labeled image", cvImagePtr->image);
    cv::waitKey(0);

}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "gtest_manual_labeling");
    return RUN_ALL_TESTS();
}