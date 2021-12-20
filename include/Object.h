
//counter(point): culling object
//vMapPoints (dynamic update according to world map)
#ifndef OBJECT_H_
#define OBJECT_H_

#include <opencv2/core/core.hpp>
#include <mutex>
#include <string>
#include <memory>
#include <set>
#include <cstdlib>
#include "MapPoint.h"
#include "ObjectMask.h"

namespace ORB_SLAM2 {
class Object {
    public:
        Object(MapPoint* pMapPoint, const std::shared_ptr<ObjectMask> pObjMask, int instanceNumber);  // create with 1 MapPoint and 1 KF
        void addMapPoint(MapPoint* pMapPoint);
        void eraseMapPoint(MapPoint* pMapPoint);  // !! only erase outliers
        std::set<MapPoint*> getMapPoints();
        float getNumMP() const;
        float getNumOverlapMP() const;
        std::vector<double> getScalesFactors();
        bool isBad() const;
        void setBadFlag();
        void EstimateScale();


    public:
    // object Identity
    int mClassID;    
    std::string mClassName;    
    int mInstanceNumber;

    // color 
    cv::Scalar mColor;
    
    // MapPoints
    std::vector<MapPoint*> mvpMapPoints;

    int mliveTime = 0;   // !! TODO how many keyFrame has passed since the object has been created
    
    protected:
    std::mutex mMutexFeatures;
    bool isbad = false;

    // Scale
    std::vector<double> mvfScalesFactors;
    double avgScalesFactor;
};


} // ORB_SLAM2


#endif // OBJECT_H_