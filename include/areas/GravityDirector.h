#pragma once

#include "al/LiveActor/LiveActor.h"
#include "al/area/AreaObj.h"
#include "al/util.hpp"
#include "container/seadPtrArray.h"
#include "heap/seadDisposer.h"

namespace galaxy {

class GravityDirector {
    SEAD_SINGLETON_DISPOSER(GravityDirector);
    GravityDirector();
    ~GravityDirector();
    public:
        void init();
        void getAllGravityAreas(const al::LiveActor*);
        void getInverseDistance(sead::Vector3f&, const al::LiveActor*, const al::AreaObj*);
        void calcTotalGravity(sead::Vector3f&, const al::LiveActor*);
        void calcPointGravity(sead::Vector3f&, const al::LiveActor*, const al::AreaObj*);
        void calcCubeGravity(sead::Vector3f&, const al::LiveActor*, const al::AreaObj*);
        void calcConeGravity(sead::Vector3f&, const al::LiveActor*, const al::AreaObj*);
        void calcDiskGravity(sead::Vector3f&, const al::LiveActor*, const al::AreaObj*);
        void calcDiskTorusGravity(sead::Vector3f&, const al::LiveActor*, const al::AreaObj*);
        void calcSegmentGravity(sead::Vector3f&, const al::LiveActor*, const al::AreaObj*);
        void calcParallelGravity(sead::Vector3f&, const al::LiveActor*, const al::AreaObj*);
    private:
        int mHighestPriority = -100;
        sead::PtrArray<al::AreaObj> mAreas;
        sead::PtrArray<sead::Vector3f> mGravities;

};

} // namespace galaxy