#include "ObjectTracker3D.h"
#include <iostream>
#include "constants.h" // sensorFrameRate


using namespace std;


BoundingBox const* GetBoxWithID(const DataFrame& frame, int id)
{
    for (auto& box : frame.boundingBoxes)
        if (box.boxID == id) // check whether current match partner corresponds to this BB
            return &box;
    return nullptr;
}

// return index of first bounding box that contains the keypoint
int FindBboxId(const cv::KeyPoint& currKpt, const std::vector<BoundingBox>& bboxes)
{
    // check which bounding box contains the given keypoint
    for (int i = 0; i < bboxes.size(); ++i)
    {
        if (bboxes[i].roi.contains(currKpt.pt))
            return bboxes[i].boxID;
    }
    return -1;
}


// Function tp find the Entry with largest Value in a Map 
int FindBoxIdWithMostMatches( std::map<int, int> sampleMap) 
{ 
    // Reference variable to help find 
    // the entry with the highest value 
    std::pair<int, int> entryWithMaxValue = std::make_pair(-1, 0);
  
    // Iterate in the map to find the required entry 
    std::map<int, int>::iterator currentEntry; 
    for (currentEntry = sampleMap.begin(); currentEntry != sampleMap.end(); ++currentEntry)
        if (currentEntry->second > entryWithMaxValue.second)
            entryWithMaxValue = std::make_pair( currentEntry->first, currentEntry->second); 
  
    return entryWithMaxValue.first; 
}

// match bounding boxes between current dataframe and last dataframe
// based on keypoint correspondences
std::map<int, int> matchBoundingBoxes(const DataFrame& prevFrame,
                                      const DataFrame& currFrame)
{
    std::map<int,int> bboxBestMatches;
    std::multimap<int, int> bboxMatches;
    // for each match in keypoint match vector of current frame
    for (auto match :  currFrame.kptMatches)
    {
        auto currKpt = currFrame.keypoints[match.trainIdx];
        auto prevKpt = prevFrame.keypoints[match.queryIdx];
        
        // find index of enclosing bbox (if any) in current frame for current frame keypoint
        int bboxIdCurr = FindBboxId(currKpt, currFrame.boundingBoxes);

        // find index of enclosing bbox (if any) in prev frame for prev frame keypoint
        int bboxIdPrev = FindBboxId(prevKpt, prevFrame.boundingBoxes);

        // if either keypoint is not enclosed by bounding box, then skip 
        if (bboxIdCurr == -1 || bboxIdPrev == -1)
            continue;

        // increment number of matches for currentBbox -> prevBbox
        bboxMatches.emplace(bboxIdCurr, bboxIdPrev);
    }

    // process the multimap for each bounding box in currFrame
    cout << "Printing out bbox matches:" << "\n";
    // iterate over each bounding box from current frame and find the best match to bounding box in previous frame
    //for (int currBoxId = 0; currBoxId < currFrame.boundingBoxes.size(); ++currBoxIdx)
    for (auto& bbox : currFrame.boundingBoxes)
    {
        // create new std::map
        std::map<int,int> prevBboxIdToNumMatches;
        // for each entry in bboxMatches multimap
        for (auto match : bboxMatches)
        {
            //  increment number of matches for prevBboxIdx
            if (match.first == bbox.boxID)
                prevBboxIdToNumMatches[match.second]++;
        }
        int bestPrevBboxId = FindBoxIdWithMostMatches(prevBboxIdToNumMatches);
        bboxBestMatches.emplace(bestPrevBboxId, bbox.boxID);

        cout << "BBox from current frame: " << bbox.boxID << "\n";
        cout << "BBox from prev frame: " << bestPrevBboxId << "\n";
        cout << "\n";
    }

    cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << endl;
    // VisualizeBboxMatches(prevFrame, "Previous image bboxes");
    // VisualizeBboxMatches(currFrame, "Current image bboxes");

    return bboxBestMatches;
}

std::tuple<const BoundingBox*,const BoundingBox*> FindEgoLaneBoundingBoxes(const DataFrame& lastFrame,
                                                                           const DataFrame& newFrame)
{
    for (auto it1 = newFrame.bbMatches.begin(); it1 != newFrame.bbMatches.end(); ++it1)
    {
        // find bounding boxes associates with current match
        const BoundingBox* currBB = GetBoxWithID(newFrame, it1->second);
        const BoundingBox* prevBB = GetBoxWithID(lastFrame, it1->first);
        
        if (currBB == nullptr || prevBB == nullptr)
            continue;

        // We are only interested in the bounding boxes which have
        // lidar points, since these correspond to the vehicle in the
        // ego lane
        if( currBB->lidarPoints.size() > 0 && prevBB->lidarPoints.size() > 0 ) 
        {
            return std::make_tuple(currBB, prevBB);
        }
    }

    return std::make_tuple(nullptr, nullptr);
}

BBoxPair ObjectTracker3D::ProcessImagePair(const DataFrame& lastFrame,
                                           DataFrame& currentFrame)
{
    currentFrame.kptMatches = featureTracker_.TrackFeatures(lastFrame,
                                                            currentFrame);

    // TRACK 3D OBJECT BOUNDING BOXES
    // associate bounding boxes between current and previous frame using keypoint matches
    map<int, int> bbBestMatches = matchBoundingBoxes(lastFrame, currentFrame); 

    // store matches in current data frame
    currentFrame.bbMatches = bbBestMatches;

    const BoundingBox* currBB = nullptr;
    const BoundingBox* prevBB = nullptr;
    // Find the bounding boxes for the ego lane vehicle for
    // current image and last image
    std::tie(currBB, prevBB) = FindEgoLaneBoundingBoxes(lastFrame,
                                                        currentFrame);
    const BBoxPair bboxPair = {prevBB, currBB};

    return bboxPair;    
}
