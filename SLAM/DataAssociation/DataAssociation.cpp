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
extern "C" {
#include "hungarian.h"
}
//std
#include <limits>
#include <chrono>
#include <algorithm>
#include <iterator>

using namespace SLAM;
using namespace SLAM::Models;
using namespace SLAM::Association;
using namespace std;

///FIXME
// IMPORT_DEBUG_LOG()

////GREEDY DATA ASSOCIATION
ScalarType SLAM::Association::GreedyDataAssociationParams::DistanceThreshold = 2.0;
vector<LandmarkAssociation> SLAM::Association::GreedyDataAssociation(const vector<Observation>& observations, const EKFSLAMEngine& se) {
    DOPEN_CONTEXT("GreedyDataAssociation")
    
    if(observations.empty()) {
        DPRINT("Empty observation vector")
        DCLOSE_CONTEXT("GreedyDataAssociation")
        return vector<LandmarkAssociation>();
    }
    
    ////typedef
//  typedef VectorType ValueType;
    typedef map<LandmarkPerceptionModel, vector<pair<VectorType, int>>> MapType;
    //MapType: for each model type, store a vector of Observations(landmark observations) with the relative original index in the initial sequence
    
    //the returned vector of landmark associations
    vector<LandmarkAssociation> ret;
    
    //first divide the observations into homogeneous group wrt the landmark model
    MapType observation_groups;
    for(int ii = 0; ii < observations.size(); ++ii) {
        //add this element
        observation_groups[observations[ii].LM].push_back(make_pair(observations[ii].LM.LPM.Normalize(observations[ii].Z), ii));
    }
    //divide the traked landmarks into homogeneous group wrt the landmark model
    //these groups store the perception associated to the landmark state, not the state itself
    MapType landmark_groups;
    const vector<Landmark>& lms = se.GetLandmarks();
    for(int ii = 0; ii < lms.size(); ++ii) {
        //add the perception associated to this element
        //note Model is a RestrainedModel
        landmark_groups[lms[ii].Model].push_back(make_pair(lms[ii].Model.Normalize(lms[ii].Model.H(se.GetStateEstimation())), ii));
    }
    
    //now iterate over each observation group
    for(auto it = observation_groups.begin(); it != observation_groups.end(); ++it) {
        const LandmarkPerceptionModel& lm = it->first;
        
        DPRINT("GROUP")
        
        DPRINT("Observations: ")
#ifndef NDEBUG
        for(int kk = 0; kk < observation_groups[lm].size(); ++kk) {
            DPRINT(kk << ": (" << observation_groups[lm][kk].first.transpose() << ") - " << observation_groups[lm][kk].second)
        }
#endif
        DPRINT("Landmarks: ")
#ifndef NDEBUG
        for(int kk = 0; kk < landmark_groups[lm].size(); ++kk) {
            DPRINT(kk << ": (" << landmark_groups[lm][kk].first.transpose() << ") - " << landmark_groups[lm][kk].second)
        }
#endif
        
        //the association for this subgroup
        vector<LandmarkAssociation> sub_ret(observation_groups[lm].size());
        //set the correct observation index in the sub return
        for(int ii = 0; ii < sub_ret.size(); ++ii) {
            sub_ret[ii].ObservationIndex = observation_groups[lm][ii].second;
        }
        
        if(!landmark_groups[lm].empty()) {
            vector<bool> associated(landmark_groups[lm].size());
            for(auto a : associated) {
                a = false;
            }
            vector<pair<int, int>> av;
            
            for(int ii = 0; ii < observation_groups[lm].size(); ++ii) {
                //for each observation
                ScalarType min_distance = numeric_limits<ScalarType>::max();
                int min_index = -1;
                
                for(int jj = 0; jj < landmark_groups[lm].size(); ++jj) {
                    if(associated[jj])
                        continue;
                    
                    //check every landmark
                    ScalarType distance = lm.Distance(observation_groups[lm][ii].first, landmark_groups[lm][jj].first);
                    
                    if(distance < min_distance) {
                        min_distance = distance;
                        min_index = jj;
                    }
                }
                
                if(min_index >= 0) { 
                    associated[min_index] = true;
                    av.push_back(make_pair(ii, min_index));
                }
            }
            
            DPRINT("The group associations are:")
            for(auto ip : av) {
                int tmp_ob_index = ip.first;
                int tmp_lm_index = ip.second;
                
                DPRINT(ip.first << " - " << ip.second)
                
                //distance threshold
                if(lm.Distance(observation_groups[lm][tmp_ob_index].first, landmark_groups[lm][tmp_lm_index].first) < SequentialDataAssociationParams::DistanceThreshold) {
                    //accept this association
                    int real_lm_index = landmark_groups[lm][tmp_lm_index].second;
                    sub_ret[tmp_ob_index].LandmarkIndex = real_lm_index;
                } else {
                    DWARNING("Association discarded because the distance (" << lm.Distance(observation_groups[lm][tmp_ob_index].first, landmark_groups[lm][tmp_lm_index].first) << ") is greater than threshold")
                }
            }
        }
        
#ifndef NDEBUG
        DPRINT("The final associations are: ")
        for(int ii = 0; ii < sub_ret.size(); ++ii) {
            DLOG() << "(" << sub_ret[ii].ObservationIndex << ", " << sub_ret[ii].LandmarkIndex << "), ";
        }
        DLOG() << endl;
#endif
        
        //add the sub_ret to the total return
        copy(sub_ret.begin(), sub_ret.end(), back_inserter(ret));
    }
    
    DCLOSE_CONTEXT("GreedyDataAssociation")
    
    return ret;
}

////SEQUENTIAL DATA ASSOCIATION
///TODO: FIXME BUG: a volta l'associazione è palesemente sbagliata, pur avendo il giusto ordine
ScalarType SLAM::Association::SequentialDataAssociationParams::DistanceThreshold = 2.0;
vector<LandmarkAssociation> SLAM::Association::SequentialDataAssociation(const vector<Observation>& observations, const EKFSLAMEngine& se) {
	DOPEN_CONTEXT("SequentialDataAssociation")
    
    if(observations.empty()) {
        DPRINT("Empty observation vector")
        DCLOSE_CONTEXT("SequentialDataAssociation")
        return vector<LandmarkAssociation>();
    }
	
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
        DPRINT("Raw observation: " << observations[ii].Z.transpose())
        DPRINT("Normalized observation: " << observations[ii].LM.LPM.Normalize(observations[ii].Z).transpose())
		observation_groups[observations[ii].LM].push_back(make_pair(observations[ii].LM.LPM.Normalize(observations[ii].Z), ii));
	}
	DPRINT("")
	//divide the traked landmarks into homogeneous group wrt the landmark model
	//these groups store the perception associated to the landmark state, not the state itself
	MapType landmark_groups;
	const vector<Landmark>& lms = se.GetLandmarks();
	for(int ii = 0; ii < lms.size(); ++ii) {
		//add the perception associated to this element
		//note Model is a RestrainedModel
        DPRINT("Raw landmark: " << lms[ii].Model.H(se.GetStateEstimation()).transpose())
        DPRINT("Normalized landmark: " << lms[ii].Model.Normalize(lms[ii].Model.H(se.GetStateEstimation())).transpose())
        landmark_groups[lms[ii].Model].push_back(make_pair(lms[ii].Model.Normalize(lms[ii].Model.H(se.GetStateEstimation())), ii));
	}
	
	//observations group and landmarks group must be sorted (Required by sequential associator)
	//sort observation
	DPRINT("Sorting of observations group")
	for(auto it = observation_groups.begin(); it != observation_groups.end(); ++it) {
        const LandmarkPerceptionModel& lm = it->first;
        
// #ifndef NDEBUG
// 		DPRINT("Before Sorting")
// 		for(int kk = 0; kk < observation_groups[lm].size(); ++kk) {
// 			DPRINT(kk << ": (" << observation_groups[lm][kk].first.transpose() << ") - " << observation_groups[lm][kk].second)
// 		}
// #endif
		
        //sort
        sort(it->second.begin(), it->second.end(), [&lm](const MapType::mapped_type::value_type& v1, const MapType::mapped_type::value_type& v2){return lm.Sort(v1.first, v2.first);});
		
#ifndef NDEBUG
// 		DPRINT("After Sorting")
		for(int kk = 0; kk < observation_groups[lm].size(); ++kk) {
			DPRINT(kk << ": (" << observation_groups[lm][kk].first.transpose() << ") - " << observation_groups[lm][kk].second)
		}
#endif
    }
    
    //sort landmark observation
    DPRINT("Landmark estimation sorting")
    for(auto it = landmark_groups.begin(); it != landmark_groups.end(); ++it) {
        const LandmarkPerceptionModel& lm = it->first;
        
// #ifndef NDEBUG
// 		DPRINT("Before Sorting")
// 		for(int kk = 0; kk < landmark_groups[lm].size(); ++kk) {
// 			DPRINT(kk << ": (" << landmark_groups[lm][kk].first.transpose() << ") - " << landmark_groups[lm][kk].second )
// 		}
// #endif
        //sort
        sort(it->second.begin(), it->second.end(), [&lm](const MapType::mapped_type::value_type& v1, const MapType::mapped_type::value_type& v2){return lm.Sort(v1.first, v2.first);});
		
#ifndef NDEBUG
// 		DPRINT("After sorting")
		for(int kk = 0; kk < landmark_groups[lm].size(); ++kk) {
			DPRINT(kk << ": (" << landmark_groups[lm][kk].first.transpose() << ") - " << landmark_groups[lm][kk].second)
		}
#endif
    }
	
	//now iterate over each observation group
	for(auto it = observation_groups.begin(); it != observation_groups.end(); ++it) {
		const LandmarkPerceptionModel& lm = it->first;
		
		//the association for this subgroup
		vector<LandmarkAssociation> sub_ret(observation_groups[lm].size());
		//set the correct observation index in the sub return
		for(int ii = 0; ii < sub_ret.size(); ++ii) {
			sub_ret[ii].ObservationIndex = observation_groups[lm][ii].second;
		}
        
		if(landmark_groups.count(lm)) {
		
			//lambdas to wrap the return type of the distance function and take the norm
// 			SAType sa([&lm](const SAType::Value1& v1, const SAType::Value2& v2){ 
//                 DINFO("The distance between v1: (" << v1.transpose() << ") and v2: (" << v2.transpose() << ") is " << lm.Distance(v1, v2).norm())
//                 return lm.Distance(v1, v2).norm();
//             });
            SAType sa([&lm](const SAType::Value1& v1, const SAType::Value2& v2){return lm.Distance(v1, v2);}, false);
			
// 			DPRINT("The observation are: ")
			//create the observation subgroup
			vector<VectorType> observations_Z;
			for(auto p : observation_groups[lm]) {
				observations_Z.push_back(p.first);
// 				DPRINT(observations_Z.back().transpose())
			}
			
// 			DPRINT("The landmarks are: ")
			//create the landmark subgroup
			vector<VectorType> landmarks_Z;
			for(auto p : landmark_groups[lm]) {
				landmarks_Z.push_back(p.first);
// 				DPRINT(landmarks_Z.back().transpose())
			}
		
			//make the association
			SAType::AssociationVector av;
			sa.associate(observations_Z, landmarks_Z, av);
			
			DTRACE_L(sa.getDistanceMatrix())
			
			DPRINT("The sequential association found are:")
			for(auto ip : av) {
				int tmp_ob_index = ip.first;
				int tmp_lm_index = ip.second;
				
				DPRINT(ip.first << " - " << ip.second)
				
				//distance threshold
				if(lm.Distance(observations_Z[tmp_ob_index], landmarks_Z[tmp_lm_index]) < SequentialDataAssociationParams::DistanceThreshold) {
					//accept this association
					int real_lm_index = landmark_groups[lm][tmp_lm_index].second;
					sub_ret[tmp_ob_index].LandmarkIndex = real_lm_index;
				} else {
					DWARNING("Association discarded because the distance (" << lm.Distance(observations_Z[tmp_ob_index], landmarks_Z[tmp_lm_index]) << ") is greater than threshold")
				}
			}
		}
		
#ifndef NDEBUG
		DPRINT("The final associations are: ")
		for(int ii = 0; ii < sub_ret.size(); ++ii) {
			DLOG() << "(" << sub_ret[ii].ObservationIndex << ", " << sub_ret[ii].LandmarkIndex << "), ";
		}
		DLOG() << endl;
#endif
        
        //add the sub_ret to the total return
        copy(sub_ret.begin(), sub_ret.end(), back_inserter(ret));
	}
	
	/**
	 * TODO: i due vettori (osservazioni e percezioni di landmark) devono essere ordinati per forzare un ordine di associazione.
	 * Bisogna inserire nel LandmarkModel un puntatore ad una funzione di ordinamento per le percezioni.
	 * Usando quella funzione bisogna ordinare i vector dei vari sottogruppi, ordinando allo stesso modo anche i vector che mantengono gli indici originali. Per fare le 2 cose insieme si potrebbero usare mappe di vector di coppie <VectorType, int> in modo da spostare tutti gli elementi insieme
	 */
	
	DCLOSE_CONTEXT("SequentialDataAssociation")
    
    return ret;
}

vector< LandmarkAssociation > HungarianDataAssociation ( const vector< Observation >& observations, const EKFSLAMEngine& se) {
    DOPEN_CONTEXT("HungarianDataAssociation")
    
    if(observations.empty()) {
        DPRINT("Empty observation vector")
        DCLOSE_CONTEXT("HungarianDataAssociation")
        return vector<LandmarkAssociation>();
    }
    
    ////typedef
//  typedef VectorType ValueType;
    typedef map<LandmarkPerceptionModel, vector<pair<VectorType, int>>> MapType;
    //MapType: for each model type, store a vector of Observations(landmark observations) with the relative original index in the initial sequence
    
    //the returned vector of landmark associations
    vector<LandmarkAssociation> ret;
    
    //first divide the observations into homogeneous group wrt the landmark model
    MapType observation_groups;
    for(int ii = 0; ii < observations.size(); ++ii) {
        //add this element
        observation_groups[observations[ii].LM].push_back(make_pair(observations[ii].LM.LPM.Normalize(observations[ii].Z), ii));
    }
    //divide the traked landmarks into homogeneous group wrt the landmark model
    //these groups store the perception associated to the landmark state, not the state itself
    MapType landmark_groups;
    const vector<Landmark>& lms = se.GetLandmarks();
    for(int ii = 0; ii < lms.size(); ++ii) {
        //add the perception associated to this element
        //note Model is a RestrainedModel
        landmark_groups[lms[ii].Model].push_back(make_pair(lms[ii].Model.Normalize(lms[ii].Model.H(se.GetStateEstimation())), ii));
    }
    
    //now iterate over each observation group
    for(auto it = observation_groups.begin(); it != observation_groups.end(); ++it) {
        const LandmarkPerceptionModel& lm = it->first;
        
        DPRINT("GROUP")
        
        DPRINT("Observations: ")
#ifndef NDEBUG
        for(int kk = 0; kk < observation_groups[lm].size(); ++kk) {
            DPRINT(kk << ": (" << observation_groups[lm][kk].first.transpose() << ") - " << observation_groups[lm][kk].second)
        }
#endif
        DPRINT("Landmarks: ")
#ifndef NDEBUG
        for(int kk = 0; kk < landmark_groups[lm].size(); ++kk) {
            DPRINT(kk << ": (" << landmark_groups[lm][kk].first.transpose() << ") - " << landmark_groups[lm][kk].second)
        }
#endif
        
        //the association for this subgroup
        vector<LandmarkAssociation> sub_ret(observation_groups[lm].size());
        //set the correct observation index in the sub return
        for(int ii = 0; ii < sub_ret.size(); ++ii) {
            sub_ret[ii].ObservationIndex = observation_groups[lm][ii].second;
        }
        
        if(!landmark_groups[lm].empty()) {
            vector<bool> associated(landmark_groups[lm].size());
            for(auto a : associated) {
                a = false;
            }
            vector<pair<int, int>> av;
            
            for(int ii = 0; ii < observation_groups[lm].size(); ++ii) {
                //for each observation
                ScalarType min_distance = numeric_limits<ScalarType>::max();
                int min_index = -1;
                
                for(int jj = 0; jj < landmark_groups[lm].size(); ++jj) {
                    if(associated[jj])
                        continue;
                    
                    //check every landmark
                    ScalarType distance = lm.Distance(observation_groups[lm][ii].first, landmark_groups[lm][jj].first);
                    
                    if(distance < min_distance) {
                        min_distance = distance;
                        min_index = jj;
                    }
                }
                
                if(min_index >= 0) { 
                    associated[min_index] = true;
                    av.push_back(make_pair(ii, min_index));
                }
            }
            
            DPRINT("The group associations are:")
            for(auto ip : av) {
                int tmp_ob_index = ip.first;
                int tmp_lm_index = ip.second;
                
                DPRINT(ip.first << " - " << ip.second)
                
                //distance threshold
                if(lm.Distance(observation_groups[lm][tmp_ob_index].first, landmark_groups[lm][tmp_lm_index].first) < SequentialDataAssociationParams::DistanceThreshold) {
                    //accept this association
                    int real_lm_index = landmark_groups[lm][tmp_lm_index].second;
                    sub_ret[tmp_ob_index].LandmarkIndex = real_lm_index;
                } else {
                    DWARNING("Association discarded because the distance (" << lm.Distance(observation_groups[lm][tmp_ob_index].first, landmark_groups[lm][tmp_lm_index].first) << ") is greater than threshold")
                }
            }
        }
        
#ifndef NDEBUG
        DPRINT("The final associations are: ")
        for(int ii = 0; ii < sub_ret.size(); ++ii) {
            DLOG() << "(" << sub_ret[ii].ObservationIndex << ", " << sub_ret[ii].LandmarkIndex << "), ";
        }
        DLOG() << endl;
#endif
        
        //add the sub_ret to the total return
        copy(sub_ret.begin(), sub_ret.end(), back_inserter(ret));
    }
    
    DCLOSE_CONTEXT("HungarianDataAssociation")
    
    return ret;
}
