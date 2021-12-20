#ifndef SCALE_ESTIMATOR_H_
#define SCALE_ESTIMATOR_H_
#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <memory>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include "Object.h"
#include "ObjectMask.h"
#include "DynamicExtractor.h"
#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "LocalMapping.h"
#include "Tracking.h"
#include "LoopClosing.h"
#include "algorithm"



enum ScalMode{
    BUILDER = 0,  // run with RGBD/Stereo to collect scale data
    MONO = 1,    // run in Monocular and use scale data to estimate scale
};
class LoopClosing;
class Tracking;
class LocalMapping;

namespace ORB_SLAM2 {
class ScaleEstimator {
    public:
        ScaleEstimator(const std::string &pathObjScaleLib, const std::string &strModelPath, bool bMonocular, double mask_thres, double conf_thres);


        // !! main functions: Object
        void estimateInitMapScaleMonocular();  // Estimate initial scale factor from initial map (Monocular) 
        void CreateInitialObjects(KeyFrame* pKFinit); 
        void ProcessNewKeyFrame();
        void ObjectCulling();  // culling highly overlapping object(many mapPoints point to two same object), by number of of points
        std::shared_ptr<Object> ObjectCreation(MapPoint* pMP, std::shared_ptr<ObjectMask> pMask);   // create object with 1 mapPoint            

        // !! main functions: Scale
        void EstimateObjectScales();
        void CalculateCorrection();
        void ExportLogMono(const std::string& strSequenceName, std::string KFTrjDir);
        void ExportLogBuilder(const std::string& strSequenceName, std::string KFTrjDir, std::string camTrjDir);

        // !! util
        std::map<MapPoint*, const cv::KeyPoint*> getMapPointKeyPointAssociation(KeyFrame* pKF, std::set<MapPoint*>& spObservedMPs) const;         // find all MapPoint associated 

        // !! visualization UI
        void showFrame(KeyFrame* pKF) const;
        void printObjects() const;

        // !! thread
        void setThreads(LoopClosing* pLoopCloser, Tracking* pTracker, LocalMapping* pLocalMapper);        // add threads
        bool CheckNewKeyFrames();
        void Run();
        void InsertKeyFrame(KeyFrame *pKF);
        bool CheckFinish();
        void RequestFinish();
        void SetFinish();
        bool isFinished();

        // !! Getter
        DynamicExtractor* getDynamicExtractor() const;      
        
        // debuging
        void sendPause();
        void sendRestart();

    protected: 
        ScalMode mMode;

        // KeyFrame
        std::list<KeyFrame*> mlNewKeyFrames;
        KeyFrame* mpCurrentKeyFrame;

        // store all objects 
        std::vector<std::shared_ptr<Object>> mvObjects;
        std::map<std::shared_ptr<Object>, size_t> mapObjectID;
        std::map<int, int> mapInstanceLog;   // store how many instance has been created for each Object class

        // path to the reference object scales
        std::string msPathObjScaleLib;

        // RCNN mask extractor
        DynamicExtractor* mpDynamicExtractor;

        // mutex 
        std::mutex mMutexNewKFs;
        std::mutex mMutexFinish;

        // bool
        bool mbFinishRequested;
        bool mbFinished;

        // other threads
        LoopClosing* mpLoopCloser;
        Tracking* mpTracker; 
        LocalMapping* mpLocalMapper;

    private:
        // store globalScaleFactor
        double mfCorrection;
        std::vector<double> mvfCorrectionLog;

        
};



} //namespace ORB_SLAM2



#endif // SCALE_ESTIMATOR_H_