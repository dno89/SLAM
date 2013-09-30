////include
//SLAM
#include "../Base/core.cpp"
#include "../Base/DMDebug.h"
#include "DataAssociation.h"
#include "../Engine/EKFSLAMEngine.h"
//std
#include <limits>
#include <chrono>

using namespace SLAM;
using namespace std;

IMPORT_DEBUG_LOG()

//BASIC DATA ASSOCIATION
double SLAM::BasicDataAssociation_constants::DistanceThreshold = 2.0;

std::vector<LandmarkAssociation> SLAM::BasicDataAssociation(const std::vector<Observation>& observations, const EKFSLAMEngine& se) {
    DOPEN_CONTEXT("BasicDataAssociation")
#ifndef NDEBUG
    auto t_start = chrono::high_resolution_clock::now();
#endif  //NDEBUG
    
    //the return value
    vector<LandmarkAssociation> ret(observations.size());
    //check if landmark has been associated
    ///FIXME: this works the best
//     bool* associated = new bool[se.GetTrackedLandmarksSize()]{(false)};
    bool* associated = new bool[se.GetTrackedLandmarksSize()];
    for(int kk = 0; kk < se.GetTrackedLandmarksSize(); ++kk) {
        ///FIXME: with uninitialized vector it work better.
        ///FIXME: with no association, it work best
        associated[kk] = false;
    }
    
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
            
            DPRINT("Association found between Z = (" << observations[ii].Z.transpose() << ") and landmark " << min_index << ": Xm = (" << se.GetLandmark(min_index).Xm.transpose() << "), Zm = (" << se.GetLandmark(min_index).Model.H(se.GetStateEstimation()).transpose())
        } else {
            DPRINT("Z = (" << observations[ii].Z.transpose() << ") not associated")
        }
        ret[ii] = LandmarkAssociation(ii, min_index);
    }
    
#ifndef NDEBUG
    auto dt = chrono::high_resolution_clock::now() - t_start;
    DINFO("The whole association took " << chrono::duration_cast<chrono::microseconds>(dt).count() << " us")
#endif  //NDEBUG
    
    DCLOSE_CONTEXT("BasicDataAssociation")
    delete[] associated;
    return ret;
}