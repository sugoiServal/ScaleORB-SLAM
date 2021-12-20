#include "ScaleEstimator.h"

namespace ORB_SLAM2
{

// TODO put GUI to Drawer thread

ScaleEstimator::ScaleEstimator(const std::string &strPathObjScaleLib, const std::string &strModelPath, bool bMonocular, double mask_thres, double conf_thres): msPathObjScaleLib(strPathObjScaleLib) {
    
    mbFinishRequested = false;
    mbFinished = false;
    
    std::cout<< "SCALE ESTIMATOR ON: " << endl;
    if (bMonocular) {  
        mMode = MONO;
        std::cout << "  --Mode: MONO" << std::endl;
    }
    else {
        mMode = BUILDER;
        std::cout << "  --Mode: BUILDER" << std::endl;
    }
    mpDynamicExtractor = new DynamicExtractor(strModelPath, conf_thres, mask_thres);
};

// !! Main functions
void ScaleEstimator::CreateInitialObjects(KeyFrame* pKFinit) {
// search in initial Map MapPoints that falls in object mask, and create Objects accordingly  
    std::srand(time(0));
    // 1-1 correspondences of keyPoint and MapPoint
    std::set<MapPoint*> spObservedMPs = pKFinit->GetMapPoints();     // KF observed MapPoint
    auto mapMapPoint_KeyPoint = getMapPointKeyPointAssociation(pKFinit, spObservedMPs);   // const pMP to const pKP

    
    // TODO take care of Points that just out of the contour of mask
    auto vpObjectMasks = pKFinit->getMasks();      
    std::map<const std::shared_ptr<ObjectMask>, std::shared_ptr<Object>> mapMask_Object;

    int nmp = 0;
    for (const auto& pMask : vpObjectMasks) {

        float x = pMask->mBox.x;
        float y = pMask->mBox.y;
        float width = pMask->mBox.width;
        float height = pMask->mBox.height;

        for (auto& pMapPoint : spObservedMPs) { 
            if(!pMapPoint->isBad()) { 
                
                // get keyPoint of the MapPoint  

                const cv::KeyPoint* pKeyPoint;        
                if (mapMapPoint_KeyPoint.count(pMapPoint) > 0) {
                    pKeyPoint= mapMapPoint_KeyPoint.at(pMapPoint);
                }
                else {continue;}
                float kp_x = pKeyPoint->pt.x;
                float kp_y = pKeyPoint->pt.y;

                // if KeyPoint 2d coordinate in Bounding box
                if (x <= kp_x && kp_x <= (x+width) && y <= kp_y && kp_y <= (y+height)) {

                    // if KeyPoint 2d coordinate in Mask

                    if (static_cast<int>(pMask->mObjectMask.at<unsigned char>(kp_x-x, kp_y-y)) == 255) {
                        nmp++;
                        if (!pMask->isInited()) {     // if Mask don't have corres Object now
                            
                            // new object with one MapPoint
                            std::shared_ptr<Object>  pObject = ObjectCreation(pMapPoint, pMask);                             
                            mapMask_Object.emplace(pMask, pObject);                           
                            pMapPoint->addObject(pObject);   // add MapPoint associta to Object (in MapPoint)
                            //cout  << ": create object " << pObject->mClassName << " with one MapPoint" << std::endl;
                        }
                        else {   // object has been created for the mask

                            auto pObject = mapMask_Object.at(pMask);
                            pObject->addMapPoint(pMapPoint);
                            pMapPoint->addObject(pObject);   // add MapPoint associta to Object (in MapPoint)     
                            //cout  << "   : add " <<  pObject->getNumMP() << " MapPoint to object " << pObject->mClassName << "(" << nmp <<"/" << spObservedMPs.size() << ")" << std::endl;
                        } 
                    }
                    else {continue;}  //!! not in Mask (maybe just out of the contour of mask)
                }
                else {continue;}  // not in 
            } // pMapPoint->isBad()
        }
    }
    for (auto& pObj : mvObjects) {
        cout  << "create object -" << pObj->mClassName << "(" << pObj->mInstanceNumber << ")" << " with " << pObj->getNumMP() << " MapPoints" << std::endl;
        cout  << "    --overlapping: " << pObj->getNumOverlapMP() << std::endl;
    }
    showFrame(pKFinit);
}

void ScaleEstimator::Run() {
    //mbFinished = false;
    int FrameId = 0;
    while(1) {
        if(CheckNewKeyFrames()) {
            FrameId += 1;  
            cout << endl << endl << endl;
            cout << "----------------------------------- process KeyFrame " << FrameId << " -----------------";      
            //ObjectCulling(); 
            ProcessNewKeyFrame();
            EstimateObjectScales();
            CalculateCorrection();   
        }

        // TODO::
            // scale lib（map<ClassID, scale>) in ScaleEstimator, initialize from &strPathObjScaleLib (txt file)
            // calculateCorrectionMono()
                // design the model to take all confident objects scales (in mono) and all queried objects scale, output an overall correction
                // consider weighted linear model 
        if(CheckFinish())
            break;
    }    
        // Export or correcting scale 
        // call python 
        SetFinish(); 
}

// TODO
void ScaleEstimator::CalculateCorrection() {
    cout << endl <<"CONSTRUCTION: CalculateCorrection()";
    mfCorrection = 0;
    mvfCorrectionLog.emplace_back(mfCorrection);
}
void ScaleEstimator::ObjectCulling() {
    // search for objects with highly overlapping points and merge those objects

    std::vector<std::shared_ptr<Object>> sus;
    float alpha = 0.5;
    float beta = 0.6;
        
    // find sus
    for (auto& pObj: mvObjects) {
        if (!pObj->isBad()) {
            if((pObj->getNumOverlapMP()/pObj->getNumMP()) >= alpha) {
                sus.emplace_back(pObj);
            }
        }
    }
    // sort sus by number of point
    while (!sus.empty()) {
        std::sort(sus.begin(), sus.end(), [](const std::shared_ptr<Object> lhs, const std::shared_ptr<Object> rhs){ return lhs->getNumMP() > rhs->getNumMP();} );   // from largest to smallest (top of stack)
        // process the smallest
        auto& pObj = sus.back();  // smallest object
        std::set<MapPoint*> psMPs = pObj->getMapPoints();

        // find the object most of current Object's points also at
        std::map<std::shared_ptr<Object>, int> refer_count;
        for (auto& pMP : psMPs) {
            std::set<std::shared_ptr<Object>> spObject = pMP->getObjects();
            for (auto& MPpObj : spObject) {
                if (!MPpObj->isBad()) {
                    if (MPpObj.get() != pObj.get()) {    // the object need to be a different one
                        if  (refer_count.count(MPpObj) <= 0) {
                            refer_count.emplace(MPpObj, 1);
                        }
                        else {

                            refer_count.at(MPpObj) += 1;
                        }
                    }
                }
            }
        }

        // get target Object
        std::shared_ptr<Object> targetObj;
        int max = -1;
        for (const auto& pair : refer_count) {
            if (pair.second > max) {
                targetObj = pair.first;
                max = pair.second;
            }
        }   
        // if most of cur_object MPs also in the target (beta percent) 
        if ((max/pObj->getNumMP() >= beta)) {
            // if (targetObj->mClassID==pObj->mClassID) {
                //merge cur Obj to Target      
                for (auto& pMP : psMPs) {
                    pMP->mergeToObj(targetObj);
                }
                // delete the current object from all objects
                std::cout << "------------------!!!!  Merging" << pObj->mClassName << "(" << pObj->mInstanceNumber << ") with" << targetObj->mClassName << "(" << targetObj->mInstanceNumber << ")"  << std::endl;
                pObj->setBadFlag();
            //}
        }     
        // find sus again
        sus.clear();
        for (auto& paObj: mvObjects) {
            if (!paObj->isBad()) {
                if((paObj->getNumOverlapMP()/paObj->getNumMP()) >= alpha) {
                    sus.emplace_back(paObj);
                }
            }
        }
    }     
    // second part, culling badly created object with less than 5 points for 3 keyframes, free its mapPoints


}

void ScaleEstimator::EstimateObjectScales() {
    // for each new keyFrame. we estimate all objects whose points > 20 once 
        // the estimated scales will be exported after the run
            // or will be used to update Map/track scale in Mono mode 
    for (auto& pObj : mvObjects) {
        if (!pObj->isBad()) {
            if (pObj->getNumMP() >= 20) {
                pObj->EstimateScale();
            }
        }
    }
}   

void ScaleEstimator::ProcessNewKeyFrame() {
    // object-mask association referenced by last KeyFrame
    // all new MapPoint must be observed by current KF
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }
    auto& pKFcur = mpCurrentKeyFrame;    
    std::set<MapPoint*> spObservedMPs = pKFcur->GetMapPoints();     // KF observed MapPoint
    auto mapMapPoint_KeyPoint = getMapPointKeyPointAssociation(pKFcur, spObservedMPs);   // const pMP to const pKP
    auto vpObjectMasks = pKFcur->getMasks();   

    // classify new MapPoints and old MapPoints   
    std::set<MapPoint*> spMPsObjs;
    std::set<MapPoint*> spMPsNoObjs;

    for (auto& pMapPoint : spObservedMPs) {
        if (pMapPoint->isBad()) {continue;}
        else {
            if (pMapPoint->isInObject()) {
                spMPsObjs.insert(pMapPoint);   // old mapPoints, already in object
            }
            else {
                spMPsNoObjs.insert(pMapPoint);  // new mapPoints, to be assigned
            }
        }
    }

    // I. try to build mask-object association with old mapPoints
    std::map<const std::shared_ptr<ObjectMask>, std::map<std::shared_ptr<Object>, int>> mapMask_Object_int;  // map: Object : ObjectMask : counter_of_mapPoints_match                        
        // find MapPoints in Masks, if found, create or add Object-Mask association
    for (auto& pMapPoint : spMPsObjs) {  // build association RANK list
        // keyPoint

        const cv::KeyPoint* pKeyPoint;
        if (mapMapPoint_KeyPoint.count(pMapPoint) > 0) {
            pKeyPoint = mapMapPoint_KeyPoint.at(pMapPoint);
        }
        else {continue;}
        float kp_x = pKeyPoint->pt.x;
        float kp_y = pKeyPoint->pt.y;

        // search keyPoint in masks
        for (const auto& pMask : vpObjectMasks) {

            float x = pMask->mBox.x;
            float y = pMask->mBox.y;
            float width = pMask->mBox.width;
            float height = pMask->mBox.height;

            // if KeyPoint 2d coordinate in Bounding box
            if (x <= kp_x && kp_x <= (x+width) && y <= kp_y && kp_y <= (y+height)) {
                // if KeyPoint 2d coordinate in Mask

                if (static_cast<int>(pMask->mObjectMask.at<unsigned char>(kp_x-x, kp_y-y)) == 255) {
                    
                    std::set<std::shared_ptr<Object>> spObjects = pMapPoint->getObjects();   
                    for (std::shared_ptr<Object> pObject : spObjects) {  
                        if (!pObject->isBad()) {
                            if (mapMask_Object_int.count(pMask) <= 0) {
                                std::map<std::shared_ptr<Object>, int> Obj_int = {{pObject, 1}};
                                mapMask_Object_int.emplace(pMask, Obj_int);
                            }
                            else { 

                                if (mapMask_Object_int.at(pMask).count(pObject) <= 0) {

                                    mapMask_Object_int.at(pMask).emplace(pObject, 1);
                                }
                                else {

                                    mapMask_Object_int.at(pMask).at(pObject) += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // find best mask-object association
    std::map<const std::shared_ptr<ObjectMask>, std::shared_ptr<Object>> mask_object_asso;
    for (auto& mask_obj : mapMask_Object_int) {
        // find best match
        int max = -1;
        std::shared_ptr<Object> targetObj;
        for (auto& obj_int : mask_obj.second) {
            if (obj_int.second > max) {
                max = obj_int.second;
                targetObj = obj_int.first;
            }
        }
        mask_object_asso.emplace(mask_obj.first, targetObj);
    }

    // II. assign new mapPoints through best mask-object association
    for (auto& pMP : spMPsNoObjs) {

            const cv::KeyPoint* pKeyPoint;
            if (mapMapPoint_KeyPoint.count(pMP) > 0) {
                pKeyPoint= mapMapPoint_KeyPoint.at(pMP);
            }                    
            else {continue;}


            float kp_x = pKeyPoint->pt.x;
            float kp_y = pKeyPoint->pt.y;

            for (const auto& pMask : vpObjectMasks) {
                float x = pMask->mBox.x;
                float y = pMask->mBox.y;
                float width = pMask->mBox.width;
                float height = pMask->mBox.height;

                // if KeyPoint 2d coordinate in Bounding box
                if (x <= kp_x && kp_x <= (x+width) && y <= kp_y && kp_y <= (y+height)) {

                    // if KeyPoint 2d coordinate in Mask
                    if (static_cast<int>(pMask->mObjectMask.at<unsigned char>(kp_x-x, kp_y-y)) == 255) {
                        if (mask_object_asso.count(pMask) <= 0) {     // if Mask don't have corres Object now                               
                            // new object with one MapPoint
                            std::shared_ptr<Object>  pObject = ObjectCreation(pMP, pMask);                                                      
                            pMP->addObject(pObject);   // add MapPoint associta to Object (in MapPoint)
                            cout  << ": create object " << pObject->mClassName << "(" << pObject->mInstanceNumber << ")" << " with one MapPoint" << std::endl;
                        }
                        else {   // object has been created for the mask
                                  
                            auto pObject = mask_object_asso.at(pMask);
                            pObject->addMapPoint(pMP);
                            pMP->addObject(pObject);   // add MapPoint associta to Object (in MapPoint)     
                            //cout  << "   : add " <<  pObject->getNumMP() << " MapPoint to object " << pObject->mClassName << "(" << nmp <<"/" << spObservedMPs.size() << ")" << std::endl;
                        } 
                    }
                    else {continue;}  //!! not in Mask (maybe just out of the contour of mask)
                }
                else {continue;}  // not in box
            }               
    }
    showFrame(pKFcur);
}

std::shared_ptr<Object> ScaleEstimator::ObjectCreation(MapPoint* pMP, std::shared_ptr<ObjectMask> pMask) {
        int Ninstance;
        if (mapInstanceLog.count(pMask->mClassID) <= 0) {
            mapInstanceLog.emplace(pMask->mClassID, 0);
            Ninstance = 0;
        }
        else {

            mapInstanceLog.at(pMask->mClassID) += 1;
            Ninstance = mapInstanceLog.at(pMask->mClassID);
        }
        auto pObject = std::shared_ptr<Object>(new Object(pMP, pMask, Ninstance));  
        mvObjects.emplace_back(pObject);  // add to Object vector 
        mapObjectID.emplace(pObject, mvObjects.size()-1); //add object index map
        return pObject;
    }

void ScaleEstimator::ExportLogMono(const std::string& strSequenceName, std::string KFTrjDir) {
    std::cout << std::endl << "finishing scale estimator " << strSequenceName << std::endl;
    std::string fileName;


    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y_%m_%d_%H_%M");
    auto tmstr = oss.str();
    std::string mode;
    if (mMode == BUILDER) {
        mode = "Builder\n";
        fileName = msPathObjScaleLib + "/Scale/Builder/" + strSequenceName + "_" + tmstr + ".txt";
    }
    else if (mMode == MONO) {
        mode = "Mono\n"; 
        fileName = msPathObjScaleLib + "/Scale/Mono/" + strSequenceName + "_" + tmstr + ".txt";
    }
    std::string Column = "ObjectClass InstanceID NumMP ScaleFactor(start to end) \n";

    std::ofstream outFile(fileName, ios_base::out);
    outFile << mode << Column << KFTrjDir << "\n";
    for (const auto& pObj : mvObjects) {
        if (!pObj->isBad()) {
            auto scaleFactors = pObj->getScalesFactors();
            if (scaleFactors.size() > 0) {
                outFile << pObj->mClassName << " " << pObj->mInstanceNumber << " " << pObj->getNumMP();
                for (const auto& scale : scaleFactors) {
                    outFile << " " << scale;
                }
                outFile << "\n";
            }
        }
    }
} 
void ScaleEstimator::ExportLogBuilder(const std::string& strSequenceName, std::string KFTrjDir, std::string camTrjDir) {
    std::cout << std::endl << "finishing scale estimator " << strSequenceName << std::endl;
    std::string fileName;


    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y_%m_%d_%H_%M");
    auto tmstr = oss.str();
    std::string mode;
    if (mMode == BUILDER) {
        mode = "Builder\n";
        fileName = msPathObjScaleLib + "/Scale/Builder/" + strSequenceName + "_" + tmstr + ".txt";
    }
    else if (mMode == MONO) {
        mode = "Mono\n"; 
        fileName = msPathObjScaleLib + "/Scale/Mono/" + strSequenceName + "_" + tmstr + ".txt";
    }
    std::string Column = "ObjectClass InstanceID NumMP ScaleFactor(start to end) \n";

    std::ofstream outFile(fileName, ios_base::out);
    outFile << mode << Column << KFTrjDir << "\n" << camTrjDir << "\n";
    for (const auto& pObj : mvObjects) {
        if (!pObj->isBad()) {
            auto scaleFactors = pObj->getScalesFactors();
            if (scaleFactors.size() > 0) {
                outFile << pObj->mClassName << " " << pObj->mInstanceNumber << " " << pObj->getNumMP();
                for (const auto& scale : scaleFactors) {
                    outFile << " " << scale;
                }
                outFile << "\n";
            }
        }
    }
} 

// !! visualization
void ScaleEstimator::showFrame(KeyFrame* pKF) const {
    // draw RGBD frame with mask and MapPoints ('s projection in the Frame)
    cv::Mat RGBframe = pKF->mpRGBframe->clone();  // image
    auto vpMasks = pKF->getMasks();  //mask
    cv::Scalar color(117,117,117); // mask color

    // print all objects
    printObjects();
    
    // draw masks
    for (const auto& pMask : vpMasks) {
        // from mask
        cv::Rect box = pMask->mBox;  // mask 
        cv::Mat ObjectMask = pMask->mObjectMask; // box

        // find contours
        std::vector<cv::Mat> contours;
        cv::Mat hierarchy;
        cv::findContours(ObjectMask, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

        // draw mask 
        cv::Mat coloredRoi = (0.4 * color + 0.7 * RGBframe(box));
        coloredRoi.convertTo(coloredRoi, CV_8UC3);
        cv::drawContours(coloredRoi, contours, -1, color, 5, cv::LINE_8, hierarchy, 100);
        coloredRoi.copyTo(RGBframe(box), ObjectMask);

    }

    // draw MapPoints
    std::set<MapPoint*> spObservedMPs = pKF->GetMapPoints();     // KF observed MapPoint
    auto mapMapPoint_KeyPoint = getMapPointKeyPointAssociation(pKF, spObservedMPs);  // const pMP to const pKP
    int npt = 0;
    for (const auto& pMapPoint : spObservedMPs) {
        if (!pMapPoint->isBad()) {

            const cv::KeyPoint* pKeyPoint;
            if (mapMapPoint_KeyPoint.count(pMapPoint) > 0) {
                    pKeyPoint= mapMapPoint_KeyPoint.at(pMapPoint);
            }
            else {continue;}     


            if (!pMapPoint->isInObject()) {
                cv::Scalar white(255, 255, 255);
                cv::circle(RGBframe, cv::Point(pKeyPoint->pt.x, pKeyPoint->pt.y), 1, white, -1);
                //cv::circle (InputOutputArray img, Point center, int radius, const Scalar &color, int thickness=1, int lineType=LINE_8, int shift=0)
            }
        }
    }
    // print different color for each object 
    for (const auto& pObject : mvObjects) {
        if (!pObject->isBad() && pObject->getNumMP() >= 4) {
            cv::Scalar color =  pObject->mColor;
            for (auto& pMapPoint : pObject->mvpMapPoints) {
                if (!pMapPoint->isBad()) {
                    const cv::KeyPoint* pKeyPoint;
 
                    if (mapMapPoint_KeyPoint.count(pMapPoint)>0) {
                        pKeyPoint= mapMapPoint_KeyPoint.at(pMapPoint);  
                        cv::circle(RGBframe, cv::Point(pKeyPoint->pt.x, pKeyPoint->pt.y), 3, color, -1);
                        npt+= pMapPoint -> getObjectNum();
                    }
                }
            }
        }
    }

    // print frame
    ////std::cout << npt << std::endl;
    cv::namedWindow("init");
    cv::imshow("init", RGBframe);
    cv::waitKey(1);
    //cv::destroyWindow("init");
}

void ScaleEstimator::printObjects() const{
    cout << "****************************" << endl;
    cout << "   current Objects:" << endl;
    for (const auto& pObj : mvObjects) {
        if (!pObj->isBad()) {
            if (pObj->getNumMP()>=5) {
                cout << "          -" << pObj->mClassName << "(" << pObj->mInstanceNumber << "):"  << "/"<<pObj->getNumMP()  << endl;
            }
        }
    }
}

// !! threading ScaleEstimator::run
bool ScaleEstimator::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void ScaleEstimator::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    //mbAbortBA=true;
}

std::map<MapPoint*, const cv::KeyPoint*> ScaleEstimator::getMapPointKeyPointAssociation(KeyFrame* pKF, std::set<MapPoint*>& spObservedMPs) const {
// search all MapPoints(in Map) associated KeyPoints in given KeyFrame

    std::map<MapPoint*, const cv::KeyPoint*> mapMapPoint_KeyPoint;    // container

    // get KeyPoints associate to those MapPoints
    for (auto& pMapPoint : spObservedMPs) {
        if(!pMapPoint->isBad()) {    // deal culled MapPoints
            std::map<KeyFrame*, size_t> mapKF_KeyPoint = pMapPoint->GetObservations();
            if (mapKF_KeyPoint.count(pKF) <= 0) {   // the keyframe does not observe the MapPoint
                continue;
            }
            else {

                size_t KPidx = mapKF_KeyPoint.at(pKF);

                const cv::KeyPoint* pKFKeyPoint = &(pKF->mvKeys[KPidx]);
                mapMapPoint_KeyPoint.emplace(pMapPoint, pKFKeyPoint);
            }
        }
        else {continue;}
    }
    return mapMapPoint_KeyPoint;
}

DynamicExtractor* ScaleEstimator::getDynamicExtractor() const {
    return mpDynamicExtractor;
}

bool ScaleEstimator::CheckFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void ScaleEstimator::RequestFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

void ScaleEstimator::SetFinish() {
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool ScaleEstimator::isFinished() {
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

// !! Scale SLAM debug, pausing whole system
void ScaleEstimator::setThreads(LoopClosing* pLoopCloser, Tracking* pTracker, LocalMapping* pLocalMapper) {
    mpLoopCloser = pLoopCloser;
    mpTracker = pTracker;
    mpLocalMapper = pLocalMapper;
}

void ScaleEstimator::sendPause() {  // pause whole system
    mpLoopCloser->waitHere();
    mpTracker->waitHere();
    mpLocalMapper->waitHere();
}   

void ScaleEstimator::sendRestart() {     // restart whole system
    mpLoopCloser->restartHere();
    mpTracker->restartHere();
    mpLocalMapper->restartHere();
    }

} //namespace ORB_SLAM