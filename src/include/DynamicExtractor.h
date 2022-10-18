
#ifndef DYNAMICEXTR_H_
#define DYNAMICEXTR_H_

#include "ObjectMask.h"
#include <opencv2/imgproc.hpp>
#include<opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>

namespace ORB_SLAM2
{

class DynamicExtractor {
    public:
        // constructor:
            // path to the model: 
                // user_arg => system(...,model_path) => Tracking ==> ORB_Extractor(private)      (old version)
                // user_arg => system(...,model_path) => DynanmicExtractor (new version)
            // confThreshold: bounding box confidence score threadhold
            // maskThreshold: threshold of possibility for a pixel to be categorize as occupied 
        DynamicExtractor(const std::string &strModelPath, 
                        float confThreshold = 0.9, float maskThreshold = 0.2 );  //confThreshold = 0.6, float maskThreshold = 0.3

        // compute dynamic mask for given frame
            // frame expects RGB frame
        int extractMask(const cv::Mat &frame, std::vector<std::shared_ptr<ObjectMask>>& mvpObjectMasks); 
        
        // from mvsClasses retrieve the class name(string) given a class id (int)
        void getClassName(int maskClassId, std::string& className) const; 

        void MaskCulling(std::vector<std::shared_ptr<ObjectMask>>& mvpObjectMasks);


    private:
        cv::dnn::Net mNet;  // mask-rcnn model

        // Extractor parameters
        float mfConfThreshold; // Confidence threshold
        float mfMaskThreshold; // Mask threshold

        int mnMinSize; // minimal mask size that filter out small object mask
        int mnMaxSize; // maximal mask size that filter out small object mask
        std::vector<std::string> prohibitObject; // objectClass that is not used in the system

        std::vector<std::string> mvsClasses; // classId --> className
        
};

} //namespace ORB_SLAM

#endif // DYNAMICEXTR_H_
