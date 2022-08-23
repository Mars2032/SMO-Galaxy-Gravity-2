#include "areas/GravityDirector.h"
#include "heap/seadDisposer.h"
#include "math/seadMatrixCalcCommon.h"
#include "math/seadVector.h"

namespace galaxy {

SEAD_SINGLETON_DISPOSER_IMPL(GravityDirector);

void GravityDirector::getAllGravityAreas(const al::LiveActor* actor) {
    al::AreaObjGroup* pointGroup = al::tryFindAreaObjGroup(actor, "GravityPointArea");
    al::AreaObjGroup* cubeGroup = al::tryFindAreaObjGroup(actor, "GravityCubeArea");
    al::AreaObjGroup* coneGroup = al::tryFindAreaObjGroup(actor, "GravityConeArea");
    al::AreaObjGroup* parallelGroup = al::tryFindAreaObjGroup(actor, "GravityParallelArea");
    al::AreaObjGroup* diskGroup = al::tryFindAreaObjGroup(actor, "GravityDiskArea");
    al::AreaObjGroup* diskTorusGroup = al::tryFindAreaObjGroup(actor, "GravityDiskTorusArea");
    al::AreaObjGroup* segmentGroup = al::tryFindAreaObjGroup(actor, "GravitySegmentGroup");
    al::AreaObjGroup* groups[7] = {pointGroup, cubeGroup, coneGroup, parallelGroup, diskGroup, diskTorusGroup, segmentGroup};
    sead::Vector3f actorTrans = al::getTrans(actor);
    for (int i = 0; i < 7; i++) {
        if (groups[i]) {
            for (int j = 0; j < groups[i]->mCurCount; j++) {
                al::AreaObj* curArea = groups[i]->getAreaObj(j);
                if (curArea->isInVolume(actorTrans)) { // test if the actor is in the area
                    if (curArea->mPriority > this->mHighestPriority) { // see if it has the highest priority
                        this->mHighestPriority = curArea->mPriority; // if it does, set the highest priority member
                        this->mAreas.clear(); // then, clear the array, as everything with lower priority does not matter
                        this->mAreas.pushFront(curArea); // add this area to the array
                    } else if (curArea->mPriority == this->mHighestPriority) { // if the priority is equal to the highest priority,
                        this->mAreas.pushFront(curArea); // it matters, so add it to the array.
                    }
                }
            }
        }
    }
}

void GravityDirector::calcTotalGravity(sead::Vector3f &gravity, const al::LiveActor *actor) {
    for (int i = 0; i < this->mAreas.size(); i++) {
        al::AreaObj* curArea = this->mAreas.at(i);
        const char* areaName = this->mAreas.at(i)->mName;
        if (al::isEqualString("GravityPointArea", areaName)) {
            calcPointGravity(gravity, actor, curArea);
        } else if (al::isEqualString("GravityCubeArea", areaName)) {
            calcCubeGravity(gravity, actor, curArea);
        } else if (al::isEqualString("GravityConeArea", areaName)) {
            calcConeGravity(gravity, actor, curArea);
        } else if (al::isEqualString("GravityDiskArea", areaName)) {
            calcDiskGravity(gravity, actor, curArea);    
        } else if (al::isEqualString("GravityDiskTorusArea", areaName)) {
            calcDiskTorusGravity(gravity, actor, curArea);
        } else if (al::isEqualString("GravityParallelArea", areaName)) {
            calcParallelGravity(gravity, actor, curArea);
        } else if (al::isEqualString("GravitySegmentArea", areaName)) {
            calcSegmentGravity(gravity, actor, curArea);
        }
        this->mGravities.pushFront(&gravity);
    }

    sead::Vector3f combinedGrav = *this->mGravities.at(0);
    combinedGrav.normalize();
    combinedGrav /= pow(this->mGravities.at(0)->length(), 2);
    if (this->mGravities.size() > 1) {
        for (int i = 1; i < this->mGravities.size(); i++) {
            sead::Vector3f curGravity = *this->mGravities.at(i);
            curGravity.normalize();
            curGravity /= pow(this->mGravities.at(i)->length(), 2);
            combinedGrav += curGravity;
        }
    }
    combinedGrav.normalize();
    gravity = combinedGrav;
}

void GravityDirector::getInverseDistance(sead::Vector3f& dist, const al::LiveActor* actor, const al::AreaObj* area) {
    sead::Matrix33f inverse;
    sead::Vector3f areaTrans;
    sead::Vector3f actorTrans = al::getTrans(actor);
    sead::Matrix33CalcCommon<float>::copy(inverse, area->mAreaTR);
    sead::Matrix33CalcCommon<float>::inverse(inverse, inverse);
    sead::Matrix34CalcCommon<float>::getTranslation(areaTrans, area->mAreaTR);
    dist = areaTrans - actorTrans;
}

void GravityDirector::calcParallelGravity(sead::Vector3f & gravity, const al::LiveActor *actor, const al::AreaObj *area) {
    sead::Vector3f dist;
    getInverseDistance(dist, actor, area);
    // rotate the cube to the direction wanted. default is down 
    // validAngleDegree should be -1 if you want the whole area to be valid. Works best on Cylinder areas
    float validAngleDegree;
    al::tryGetAreaObjArg(&validAngleDegree, area, "ValidAngleDeg");
    float distAngle = al::calcAngleDegree(sead::Vector3f(-dist.x,0,-dist.z),sead::Vector3f::ex);
    if (al::sign(dist.z) == -1) {
        distAngle = 360.0f - distAngle;
    }
    if (validAngleDegree > 0) {
        if  (distAngle <= validAngleDegree) {
            gravity = -sead::Vector3f::ey;
        } else {
            gravity = sead::Vector3f::zero;
        }
    } else {
        gravity = -sead::Vector3f::ey;
    }

    sead::Matrix33f mtx;
    sead::Matrix33CalcCommon<float>::copy(mtx, area->mAreaTR);
    sead::Vector3CalcCommon<float>::mul(gravity, mtx, gravity);
}

void GravityDirector::calcPointGravity(sead::Vector3f& gravity, const al::LiveActor* actor, const al::AreaObj* area) {
    sead::Vector3f actorTrans = al::getTrans(actor);
    sead::Vector3f areaTrans;
    sead::Matrix34CalcCommon<float>::getTranslation(areaTrans, area->mAreaTR);
    gravity = areaTrans - actorTrans;
}

void GravityDirector::calcCubeGravity(sead::Vector3f & gravity, const al::LiveActor *actor, const al::AreaObj *area) {
    sead::Vector3f dist;
    getInverseDistance(dist, actor, area);
    float radius;
    al::tryGetAreaObjArg(&radius, area, "Radius");                
    // gravity points towards nearest point on a cubes surface; if inside, pushes you out.
    if (abs(dist.x) < radius && abs(dist.y) < radius && abs(dist.z) < radius) {
        gravity = -dist;
    } else {
        sead::Vector3f closestPointInBox = {al::clamp(dist.x,-radius,radius),  al::clamp(dist.y,-radius,radius), al::clamp(dist.z,-radius,radius)};
        gravity = dist - closestPointInBox;
    }
    sead::Matrix33f mtx;
    sead::Matrix33CalcCommon<float>::copy(mtx, area->mAreaTR);
    sead::Vector3CalcCommon<float>::mul(gravity, mtx, gravity);
}

void GravityDirector::calcDiskGravity(sead::Vector3f & gravity, const al::LiveActor *actor, const al::AreaObj *area) {
    sead::Vector3f dist;
    getInverseDistance(dist, actor, area);
    float radius;
    al::tryGetAreaObjArg(&radius, area, "Radius");
    if (sqrtf(dist.x*dist.x+dist.z*dist.z) < radius) {
        gravity = {0, dist.y, 0};
    } else {
        float gravityPoint = sqrtf(radius*radius/(dist.x*dist.x + dist.z*dist.z));
        gravity = {dist.x*(1-gravityPoint), dist.y, dist.z*(1-gravityPoint)};
    }
    sead::Matrix33f mtx;
    sead::Matrix33CalcCommon<float>::copy(mtx, area->mAreaTR);
    sead::Vector3CalcCommon<float>::mul(gravity, mtx, gravity);
}

void GravityDirector::calcDiskTorusGravity(sead::Vector3f &gravity, const al::LiveActor *actor, const al::AreaObj *area) {
    sead::Vector3f dist;
    getInverseDistance(dist, actor, area);
    float radius;
    al::tryGetAreaObjArg(&radius, area, "Radius");
    float gravityPoint = sqrtf(radius*radius/(dist.x*dist.x + dist.z*dist.z));
    gravity = {dist.x*(1-gravityPoint), dist.y, dist.z*(1-gravityPoint)};
    sead::Matrix33f mtx;
    sead::Matrix33CalcCommon<float>::copy(mtx, area->mAreaTR);
    sead::Vector3CalcCommon<float>::mul(gravity, mtx, gravity);
}

void GravityDirector::calcConeGravity(sead::Vector3f &gravity, const al::LiveActor *actor, const al::AreaObj *area) {
    sead::Vector3f dist;
    getInverseDistance(dist, actor, area);
    float radius;
    float height;
    al::tryGetAreaObjArg(&radius, area, "Radius");
    al::tryGetAreaObjArg(&height, area, "Height");
    float distXZ = sqrtf(dist.x*dist.x + dist.z*dist.z);
    if (dist.y > 0) {
        gravity = sead::Vector3f::zero;
    } else if (abs(dist.y) > height && distXZ <= abs(radius/height*(height+dist.y))) {
        gravity = {dist.x, dist.y + height, dist.z};
    } else {
        gravity = {height*dist.x/distXZ, -radius, height*dist.z/distXZ};
    }
    sead::Matrix33f mtx;
    sead::Matrix33CalcCommon<float>::copy(mtx, area->mAreaTR);
    sead::Vector3CalcCommon<float>::mul(gravity, mtx, gravity);
}

void GravityDirector::calcSegmentGravity(sead::Vector3f &gravity, const al::LiveActor *actor, const al::AreaObj *area) {
    sead::Vector3f dist;
    getInverseDistance(dist, actor, area);
    float radius;
    al::tryGetAreaObjArg(&radius, area, "Radius");
    if (dist.y > radius) {
        gravity = {dist.x, dist.y + radius, dist.z};
    } else if (dist.y < -radius) {
        gravity = {dist.x, dist.y - radius, dist.z};
    } else {
        gravity = {dist.x, 0, dist.y};
    }
    sead::Matrix33f mtx;
    sead::Matrix33CalcCommon<float>::copy(mtx, area->mAreaTR);
    sead::Vector3CalcCommon<float>::mul(gravity, mtx, gravity);
}

} // namespace galaxy