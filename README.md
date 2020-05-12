# MidTerm Report:

This is the Final report for the camera/lidar based 3D object tracking
project. Below I describe the performance of the time to collision
(TTC) estimates for lidar and camera individually.

## Performance Evaluation 1: Lidar TTC
There are some instances where the TTC increases from one frame to the
next, even though this is probably innaccurate. One contributing
factor is that we are using a constant velocity model for the vehicle,
despite the fact that the vehicle is gradually slowing down throughout
the duration of the dataset. Another factor is that the median lidar
point does not always capture the same physical location on the
vehicle every frame.

## Performance Evaluation 2: Camera TTC
Based on experimentation, using a combination of the ORB feature
detector and the BRISK feature descriptor yields the most accurate
results. There are, however, a few cases where the camera TTC is
drastically different from the lidar TTC. I attribute this to
occasional bad feature matches across frames. For the most part, these
can be remedied by taking the median distance ratio. But there are a
few bad frames where even this does not help.
