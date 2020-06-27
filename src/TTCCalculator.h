#ifndef TTCCALCULATOR_H
#define TTCCALCULATOR_H

#include "dataStructures.h" // BoundingBox, DataFrame


struct TTCResults
{
    double camera;
    double lidar;
};


class TTCCalculator
{
public:

    TTCResults ComputeTTC(const BoundingBox& prevBB,
                          const BoundingBox& _currBB,
                          const double sensorFrameRate,
                          const DataFrame& lastFrame,
                          const DataFrame& currentFrame);
};


#endif /* TTCCALCULATOR_H */
