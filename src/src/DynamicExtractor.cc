#include "DynamicExtractor.h"
using namespace cv::dnn;
using namespace std;

bool comp(const cv::Mat &mask1, const cv::Mat &mask2){
    int size_mask1 = mask1.cols*mask1.rows-countNonZero(mask1);
    int size_mask2 = mask2.cols*mask2.rows-countNonZero(mask2);
    return size_mask2>size_mask1;
}

namespace ORB_SLAM2
{
    DynamicExtractor::DynamicExtractor(const std::string &strModelPath, 
                        float confThreshold, float maskThreshold) : mfConfThreshold(confThreshold), mfMaskThreshold(maskThreshold) {
        std::string modelConfig = strModelPath + "mask_rcnn_inception_v2_coco_2018_01_28.pbtxt";  // text graph file to OpenCV DNN support
        std::string modelWeights = strModelPath +  "frozen_inference_graph.pb";  // Tensorflow framework network weight
        std::string classesFile = strModelPath + "mscoco_labels.names";  // label id of objects that were used in training
        
        // Load names of classes
        std::ifstream ifs(classesFile.c_str());
        std::string line;
        while (getline(ifs, line)) mvsClasses.emplace_back(line);

        // Load the network
        mNet = cv::dnn::readNetFromTensorflow(modelWeights, modelConfig);

        // Set computation backend
        std::cout << "[DynamicExtractorINFO] setting preferable backend and target to CUDA..." << std::endl;
	    mNet.setPreferableBackend(DNN_BACKEND_CUDA);
        mNet.setPreferableTarget(DNN_TARGET_CUDA);
    }
    
    int DynamicExtractor::extractMask(const cv::Mat &frame, std::vector<std::shared_ptr<ObjectMask>>& mvpObjectMasks) {
        cv::Mat blob;  // convert frame into blob as DNN input

        // Create 4D blob from a frame as input
            // solved: swapRGB=true 
        cv::dnn::blobFromImage(frame, blob, 1.0, cv::Size(frame.cols, frame.rows), cv::Scalar(), true, false);  

        
        mNet.setInput(blob);

        // Runs the forward pass to get output from the output layers
        std::vector<std::string> outNames{"detection_out_final", "detection_masks"};
        std::vector<cv::Mat> outs;
        mNet.forward(outs, outNames);  // outs: OutputArrayOfArrays 
        cv::Mat outDetections = outs[0];
        cv::Mat outMasks = outs[1];

        // outDetections.dims = 4, temp.size = [1 X 1 X 100 X 7] outDetections.channels = 1
        // outMasks.dims = 4, temp.size = [100 X 90 X 15 X 15] outMasks.channels = 1 (100 top mark ROI, 90 objects classes, each mask is of size 15×15)
           
            // Output size of masks(outMasks) is NxCxHxW where
            // N - number of detected boxes
            // C - number of classes (excluding background)
            // HxW - segmentation shape

            // Output size of outDetections [1 X 1 X 100 X 7]: not batch(1*1), 100 ROI and 7 output params
            // 0: ??
            // 1: classId; 2:score; 3,4,5,6: bb left/top/right/bottom (percent);   

        // reshape to channel = 1, 2D mat, row for numDetections, cols = 7  
        const int numDetections = outDetections.size[2];
        outDetections = outDetections.reshape(1, outDetections.total() / 7);  // ==> mat (100 * 7)

        // for each detected box, if score > mfConfThreshold
        for (int i = 0; i < numDetections; ++i) {
            float score = outDetections.at<float>(i, 2);
            if (score > mfConfThreshold) {

                // Extract class id
                int classId = static_cast<int>(outDetections.at<float>(i, 1));
                
                // Extract bounding box
                int left = static_cast<int>(frame.cols * outDetections.at<float>(i, 3));
                int top = static_cast<int>(frame.rows * outDetections.at<float>(i, 4));
                int right = static_cast<int>(frame.cols * outDetections.at<float>(i, 5));
                int bottom = static_cast<int>(frame.rows * outDetections.at<float>(i, 6));

                left = max(0, min(left, frame.cols - 1));
                top = max(0, min(top, frame.rows - 1));
                right = max(0, min(right, frame.cols - 1));
                bottom = max(0, min(bottom, frame.rows - 1));
                cv::Rect box = cv::Rect(left, top, right - left + 1, bottom - top + 1); // x,y,w,h
                
                // Extract the mask for the object
                cv::Mat rawMask(outMasks.size[2], outMasks.size[3], CV_32F, outMasks.ptr<float>(i, classId)); // copy outMasks (15*15) at [i, classId, :, :]
                  
                // Resize the mask to size of box, threshold into binary 255/0 mask
                cv::resize(rawMask, rawMask, cv::Size(box.width, box.height));  // <opencv2/imgproc.hpp>
                cv::Mat biMask = (rawMask > mfMaskThreshold);
                biMask.convertTo(biMask, CV_8U);

                // get mask size, frame size and className
                int maskSize = cv::countNonZero(biMask);
                cv::Size frameSize(frame.rows, frame.cols); // (w, h)
                std::string ClassName;
                getClassName(classId, ClassName);
 
                // save ObjectMask
                auto pMask = std::shared_ptr<ObjectMask>(new ObjectMask(classId, ClassName, box, biMask, frameSize, maskSize, score));
                mvpObjectMasks.emplace_back(pMask);
            }
        }
        return 0;
    }
    
    void DynamicExtractor::getClassName(int maskClassId, std::string& className) const {
        className = mvsClasses[maskClassId];
    } 
    
    void DynamicExtractor::MaskCulling(std::vector<std::shared_ptr<ObjectMask>>& mvpObjectMasks) {   // delete mask that is too big ()ie. table

        mvpObjectMasks.erase(std::remove_if(mvpObjectMasks.begin(), mvpObjectMasks.end(),
        [] (const std::shared_ptr<ObjectMask> pMask) {
            return (!pMask->mClassName.compare("dining table") || !pMask->mClassName.compare("bench")|| pMask->mnMaskSize < 1100 || pMask->mnMaskSize > 120000);
        }),
        mvpObjectMasks.end());
    }
} //namespace ORB_SLAM
