////include
//SLAM
#include "DataAssociation.h"
#include "../Engine/EKFSLAMEngine.h"
//std
#include <limits>

using namespace SLAM;
using namespace std;

double SLAM::BasicDataAssociation_constants::DistanceThreshold = 1.0;
std::vector<LandmarkAssociation> BasicDataAssociation(const std::vector<Observation>& observations, const EKFSLAMEngine& se) {
    //the return value
    vector<LandmarkAssociation> ret(observations.size());
    //check if landmark has been associated
    bool* associated = new bool[se.GetTrackedLandmarksSize()]{false};
    
    for(int ii = 0; ii < observations.size(); ++ii) {
        double min_distance = std::numeric_limits<double>::max();
        int min_index = -1;
        
        for(int jj = 0; jj < se.GetTrackedLandmarksSize(); ++jj) {
            if(associated[jj]) continue;
            
            const Landmark& l(se.GetLandmark(jj));
            
            double d = DefaultDistance(observations[ii].Z, l.Model.H(se.GetStateEstimation())).norm();
            
            if(d < BasicDataAssociation_constants::DistanceThreshold && d < min_distance) {
                min_distance = d;
                min_index = jj;
            }
        }
        
        if(min_index != -1) {
            //this observation is associated with a known landmark
            associated[min_index] = true;
        }
        ret[ii] = LandmarkAssociation(ii, min_index);
    }
    
    delete[] associated;
    return ret;
}