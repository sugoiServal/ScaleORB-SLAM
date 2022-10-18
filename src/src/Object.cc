#include "Object.h"
namespace ORB_SLAM2 {

// initialize Object with one MapPoint
Object::Object(MapPoint* pMapPoint, const std::shared_ptr<ObjectMask> pObjMask, int instanceNumber) : mClassID(pObjMask->mClassID), mClassName(pObjMask->mClassName), mInstanceNumber(instanceNumber) {    
    // add MapPoint
    mvpMapPoints.emplace_back(pMapPoint);
    // set Mask init state for initial Object Map
    pObjMask->mbisInited = true;

    // assign color
    mColor = cv::Scalar(
                    (double)std::rand() / RAND_MAX * 255,
                    (double)std::rand() / RAND_MAX * 255,
                    (double)std::rand() / RAND_MAX * 255);
}

void Object::addMapPoint(MapPoint* pMapPoint) {
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints.emplace_back(pMapPoint);
}

std::set<MapPoint*> Object::getMapPoints() {
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s; 
}
float Object::getNumMP() const{
    //unique_lock<mutex> lock(mMutexFeatures);
    int Nmp = 0;
    for (MapPoint* pMP : mvpMapPoints) {
        if (!pMP->isBad()) {Nmp++;}
    }
    return static_cast<float>(Nmp);
}
float Object::getNumOverlapMP() const{
    //unique_lock<mutex> lock(mMutexFeatures);
    int NOmp = 0;
    for (MapPoint* pMP : mvpMapPoints) {
        if (!pMP->isBad()) {
            if (pMP->getObjectNum() >= 2) {NOmp++;}        
        }
    }
    return static_cast<float>(NOmp);
}

std::vector<double> Object::getScalesFactors() {
    unique_lock<mutex> lock(mMutexFeatures);
    std::vector<double> scaleFactors = mvfScalesFactors;
    return scaleFactors;
}

bool Object::isBad() const {
    return isbad;
}
void Object::setBadFlag() {
    unique_lock<mutex> lock(mMutexFeatures);
    isbad = true;
}

void Object::EstimateScale() {
    cv::Mat Certroid = cv::Mat::zeros(3, 1, CV_32F);
    double numMP = 0;
    for (auto& pMP : mvpMapPoints) {
        if (!pMP->isBad()) {
            Certroid = Certroid + pMP->GetWorldPos();
            numMP += 1;
        }      
    }
    Certroid = Certroid/numMP;

    double averageDist = 0;
    numMP = 0;
    for (auto& pMP : mvpMapPoints) {
        if (!pMP->isBad()) {
            cv::Mat MPxyz = pMP->GetWorldPos();
            double dist = cv::norm(MPxyz-Certroid);
            averageDist += dist;
            numMP += 1;
        }      
    }
    mvfScalesFactors.emplace_back(averageDist/numMP);
}


} // ORB_SLAM2s