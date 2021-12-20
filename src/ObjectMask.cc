#include "ObjectMask.h"
namespace ORB_SLAM2 {

ObjectMask::ObjectMask(int classId, std::string ClassName, cv::Rect box, cv::Mat& biMask, cv::Size frameSize, int maskSize, float score):
mClassID(classId), mClassName(ClassName), mBox(box), mObjectMask(biMask), mFrameSize(frameSize), mnMaskSize(maskSize), mfScore(score)
{}

void ObjectMask::convertToFullFrame(cv::Mat& fullMask) const {
    fullMask = cv::Mat(mFrameSize, CV_8U, cv::Scalar(255));
    cv::Mat matZeros = cv::Mat::zeros(mFrameSize, CV_8U);
    matZeros(mBox).copyTo(fullMask(mBox), mObjectMask); // if bMask, then copy 0
}

void ObjectMask::showMask() const { 
    cv::Mat fullMask;
    convertToFullFrame(fullMask);
    cv::namedWindow(mClassName, cv::WINDOW_AUTOSIZE);
    cv::Mat imS;
    cv::resize(fullMask, imS, cv::Size(fullMask.size[1]/5, fullMask.size[0]/5));
    cv::imshow(mClassName, imS);
    cv::waitKey(500);
    cv::destroyAllWindows();
}

bool ObjectMask::isInited() const {
    return mbisInited;
}



} //ORB_SLAM2