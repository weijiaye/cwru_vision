#ifndef GRAB_CUT_ROS_H
#define GRAB_CUT_ROS_H


#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace cv_grab_cut
{


class GrabCutObject
{
public:
    enum{ NOT_SET = 0, IN_PROCESS = 1, SET = 2 };
    static const int radius = 2;
    static const int thickness = -1;

    void reset();
    void setImageAndWinName( const cv::Mat& _image, const std::string& _winName );
    void showImage() const;
    void mouseClick( int event, int x, int y, int flags, void* param );
    int nextIter();
    int getIterCount() const { return iterCount; }
    int getMask(cv::Mat & binMask) const;
private:
    void setRectInMask();
    void setLblsInMask( int flags, cv::Point p, bool isPr );

    const std::string* winName;
    const cv::Mat* image;
    cv::Mat mask;
    cv::Mat bgdModel, fgdModel;

    uchar rectState, lblsState, prLblsState;
    bool isInitialized;

    cv::Rect rect;
    std::vector<cv::Point> fgdPxls, bgdPxls, prFgdPxls, prBgdPxls;
    int iterCount;
};

void grabCutDemo(const cv::Mat &, cv::Mat & );

};



#endif
