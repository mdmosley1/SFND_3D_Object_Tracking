#ifndef TTCCALCULATOR_H
#define TTCCALCULATOR_H

#include "dataStructures.h" // BoundingBox, DataFrame


struct TTCResults
{
    TTCResults() : camera(0.0), lidar(0.0) {}
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
