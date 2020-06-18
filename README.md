# MidTerm Report:

This is the Final report for the camera/lidar based 3D object tracking
project. Below I describe the performance of the time to collision
(TTC) estimates for lidar and camera individually.

## FP.1 Match 3D Objects   
3D-bounding boxes are matched between frames based on the number of point correspondences. This was implemented [here](https://github.com/mdmosley1/SFND_3D_Object_Tracking/blob/85dd3622030d08074ffcc4e7cf4fd66dd1becc61/src/camFusion_Student.cpp#L306-L352)

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
