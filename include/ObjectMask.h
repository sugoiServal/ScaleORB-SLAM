#ifndef OBJECTMASK_H_
#define OBJECTMASK_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <string>

namespace ORB_SLAM2 {
class ObjectMask {
    public:
        ObjectMask(int classId, std::string ClassName, cv::Rect box, cv::Mat& biMask, cv::Size frameSize, int maskSize, float score);
        void convertToFullFrame(cv::Mat& fullMask) const;
        void showMask() const;
        bool isInited() const;
    
    
    public:    
    
        // class ID
        int mClassID;               // class ID of the object mask
        std::string mClassName;     // name of the class
        
        // location in frame
        cv::Rect mBox;              // bounding box   
        cv::Size mFrameSize;        // frame size of the original frame the mask extract from

        // mask
        cv::Mat mObjectMask;        // objectMask, size of the bounding box (CV_8U), 255 is occupied    
        int mnMaskSize;             // size of the object mask: number of occupied pixel        
        float mfScore;              // object detection score

        // used by objects initialization  
        bool mbisInited = false;     
};

} //namespace ORB_SLAM

#endif // OBJECTMASK_H_
