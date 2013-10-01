////include
//SLAM
#include "../Base/core.cpp"
#include "../Base/DMDebug.h"
#include "DataAssociation.h"
#include "../Engine/EKFSLAMEngine.h"
#include "SequentialAssociator.hpp"
//std
#include <limits>
#include <chrono>

using namespace SLAM;
using namespace std;

IMPORT_DEBUG_LOG()

//BASIC DATA ASSOCIATION
double SLAM::BasicDataAssociationConstants::DistanceThreshold = 2.0;

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
            
            if(d < BasicDataAssociationConstants::DistanceThreshold && d < min_distance) {
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

////SEQUENTIAL DATA ASSOCIATION
vector<LandmarkAssociation> SLAM::SequentialDataAssociation(const vector<Observation>& observations, const EKFSLAMEngine& se) {
	////typedef
	typedef VectorType ValueType;
	typedef SequentialAssociator<ValueType, ValueType, ScalarType> SAType;
	
	//the returned vector of landmark associations
	vector<LandmarkAssociation> ret;
	
	//first divide the observations into homogeneous group wrt the landmark model
	map<LandmarkModel, vector<ValueType>> observation_groups;
	map<LandmarkModel, vector<int>> observation_original_indexes;
	for(int ii = 0; ii < observations.size(); ++ii) {
		///TODO: check this is OK
// 		if(!observation_groups.count(observations[ii].LM)) {
// 			//If not already present, insert it
// 			observation_groups.insert(make_pair(observations[ii].LM, vector<ValueType>()));
// 			observation_original_indexes.insert(make_pair(observations[ii].LM, vector<int>()));
// 		}
		//add this element
		observation_groups[observations[ii].LM].push_back(observations[ii].Z);
		//keep track of its original index
		observation_original_indexes[observations[ii].LM].push_back(ii);
	}
	
	//divide the traked landmarks into homogeneous group wrt the landmark model
	//these groups store the perception associated to the landmark state, not the state itself
	map<LandmarkModel, vector<ValueType>> landmark_groups;
	map<LandmarkModel, vector<int>> landmark_original_indexes;
	const vector<Landmark>& lms = se.GetLandmarks();
	for(int ii = 0; ii < lms.size(); ++ii) {
		///TODO: check this is OK
// 		if(!landmark_groups.count(lms[ii].Model)) {
// 			//If not already present, insert it
// 			landmark_groups.insert(make_pair(lms[ii].Model.GetModel(), vector<ValueType>()));
// 			landmark_original_indexes.insert(make_pair(lms[ii].Model.GetModel(), vector<ValueType>()));
// 		}
		//add the perception associated to this element
		//note Model is a RestrainedModel
		landmark_groups[lms[ii].Model].push_back(lms[ii].Model.H(se.GetStateEstimation()));
		//keep track of its original index in the tracked landmarks
		landmark_original_indexes[observations[ii].LM].push_back(ii);
	}
	
	//now iterate over each observation group
	for(auto it = observation_groups.begin(); it != observation_groups.end(); ++it) {
		const LandmarkModel& lm = it->first;
		//lambdas to wrap the return type of the distance function and take the norm
		SAType se([&lm](const SAType::Value1& v1, const SAType::Value2& v2){ return lm.Distance(v1, v2).norm();});
		
		if(landmark_groups.count(lm)) {
			//there are tracked landmarks of this type
		}
	}
}
