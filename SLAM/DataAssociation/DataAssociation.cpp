/**
 * \file DataAssociation.cpp
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

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
#include <algorithm>
#include <iterator>

using namespace SLAM;
using namespace std;

IMPORT_DEBUG_LOG()

//BASIC DATA ASSOCIATION
ScalarType SLAM::BasicDataAssociationConstants::DistanceThreshold = 2.0;
std::vector<LandmarkAssociation> SLAM::BasicDataAssociation(const std::vector<Observation>& observations, const EKFSLAMEngine& se) {
    DOPEN_CONTEXT("BasicDataAssociation")
#ifndef NDEBUG
    auto t_start = chrono::high_resolution_clock::now();
#endif  //NDEBUG
    
    //the return value
    vector<LandmarkAssociation> ret(observations.size());
    //check if landmark has been associated
//     bool* associated = new bool[se.GetTrackedLandmarksSize()]{(false)};
    bool* associated = new bool[se.GetTrackedLandmarksSize()];
    for(int kk = 0; kk < se.GetTrackedLandmarksSize(); ++kk) {
        associated[kk] = false;
    }
    
    for(int ii = 0; ii < observations.size(); ++ii) {
        double min_distance = numeric_limits<double>::max();
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
///TODO: DEBUG THIS!!! FIXME FIXME
ScalarType SequentialDataAssociationContants::DistanceThreshold = 2.0;
vector<LandmarkAssociation> SLAM::SequentialDataAssociation(const vector<Observation>& observations, const EKFSLAMEngine& se) {
	////typedef
// 	typedef VectorType ValueType;
	typedef SequentialAssociator<VectorType, VectorType, ScalarType> SAType;
    typedef map<LandmarkPerceptionModel, vector<pair<VectorType, int>>> MapType;
    //MapType: for each model type, store a vector of Observations(landmark observations) with the relative original index in the initial sequence
	
	//the returned vector of landmark associations
	vector<LandmarkAssociation> ret;
	
	//first divide the observations into homogeneous group wrt the landmark model
	MapType observation_groups;
	for(int ii = 0; ii < observations.size(); ++ii) {
		//add this element
		observation_groups[observations[ii].LM].push_back(make_pair(observations[ii].Z, ii));
	}
	
	//divide the traked landmarks into homogeneous group wrt the landmark model
	//these groups store the perception associated to the landmark state, not the state itself
	MapType landmark_groups;
	const vector<Landmark>& lms = se.GetLandmarks();
	for(int ii = 0; ii < lms.size(); ++ii) {
		//add the perception associated to this element
		//note Model is a RestrainedModel
		landmark_groups[lms[ii].Model].push_back(make_pair(lms[ii].Model.H(se.GetStateEstimation()), ii));
	}
	
	//observations group and landmarks group must be sorted (Required by sequential associator)
	//sort observation
	for(auto it = observation_groups.begin(); it != observation_groups.end(); ++it) {
        const LandmarkPerceptionModel& lm = it->first;
        
        //sort
        sort(it->second.begin(), it->second.end(), [&lm](const MapType::mapped_type::value_type& v1, const MapType::mapped_type::value_type& v2){return lm.Sort(v1.first, v2.first);});
    }
    //sort landmark observation
    for(auto it = landmark_groups.begin(); it != landmark_groups.end(); ++it) {
        const LandmarkPerceptionModel& lm = it->first;
        
        //sort
        sort(it->second.begin(), it->second.end(), [&lm](const MapType::mapped_type::value_type& v1, const MapType::mapped_type::value_type& v2){return lm.Sort(v1.first, v2.first);});
    }
	
	//now iterate over each observation group
	for(auto it = observation_groups.begin(); it != observation_groups.end(); ++it) {
		const LandmarkPerceptionModel& lm = it->first;
        
		if(!landmark_groups.count(lm)) {
			//there are not tracked landmarks of this type
            continue;
		}
		
		//lambdas to wrap the return type of the distance function and take the norm
		SAType se([&lm](const SAType::Value1& v1, const SAType::Value2& v2){ return lm.Distance(v1, v2).norm();});
		
        //create the observation subgroup
        vector<VectorType> observations_Z;
        for(auto p : observation_groups[lm]) {
            observations_Z.push_back(p.first);
        }
		//create the landmark subgroup
		vector<VectorType> landmarks_Z;
        for(auto p : landmark_groups[lm]) {
            landmarks_Z.push_back(p.first);
        }
		
		//make the association
		SAType::AssociationVector av;
		se.associate(observations_Z, landmarks_Z, av);
        
        vector<LandmarkAssociation> sub_ret(observation_groups[lm].size());
        //set the correct observation index in the sub return
        for(int ii = 0; ii < sub_ret.size(); ++ii) {
            sub_ret[ii].ObservationIndex = observation_groups[lm][ii].second;
        }
        
        for(auto ip : av) {
            int tmp_ob_index = ip.first;
            int tmp_lm_index = ip.second;
            
            //distance threshold
            if(lm.Distance(observations_Z[tmp_ob_index], landmarks_Z[tmp_lm_index]).norm() < SequentialDataAssociationContants::DistanceThreshold) {
                //accept this association
                int real_lm_index = landmark_groups[lm][tmp_lm_index].second;
                sub_ret[tmp_ob_index].LandmarkIndex = real_lm_index;
            }
        }
        
        //add the sub_ret to the total return
        copy(sub_ret.begin(), sub_ret.end(), back_inserter(ret));
	}
	
	/**
	 * TODO: i due vettori (osservazioni e percezioni di landmark) devono essere ordinati per forzare un ordine di associazione.
	 * Bisogna inserire nel LandmarkModel un puntatore ad una funzione di ordinamento per le percezioni.
	 * Usando quella funzione bisogna ordinare i vector dei vari sottogruppi, ordinando allo stesso modo anche i vector che mantengono gli indici originali. Per fare le 2 cose insieme si potrebbero usare mappe di vector di coppie <VectorType, int> in modo da spostare tutti gli elementi insieme
	 */
    
    return ret;
}