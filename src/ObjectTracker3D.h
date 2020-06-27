#ifndef OBJECTTRACKER3D_H
#define OBJECTTRACKER3D_H

#include "FeatureTracker.h"
#include "dataStructures.h" // DataFrame, BoundingBox

struct BBoxPair
{
    const BoundingBox* prev;
    const BoundingBox* curr;
};

class ObjectTracker3D
{
public:

    ObjectTracker3D(FeatureTracker ft ): featureTracker_(ft)
    {
        
    }
    // methods
    BBoxPair ProcessImagePair(const DataFrame& lastFrame,
                              DataFrame& currentFrame);


    FeatureTracker featureTracker_;

private:

};


#endif /* OBJECTTRACKER3D_H */
