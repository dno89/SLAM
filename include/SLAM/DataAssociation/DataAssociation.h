/**
 * \file DataAssociation.h
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

#pragma once

////include
//SLAM
#include "../Base/core.h"
#include "../Base/types.h"
//std
#include <vector>

namespace SLAM {
    ////TYPE FORWARDING
    class EKFSLAMEngine;
	
	namespace Association {
    
    ////PROTOTYPES
    namespace GreedyDataAssociationParams {
//         extern ScalarType DistanceThreshold;
    }
    /**
     * @brief a sample function that make the association of feature perceived and perception associated with the estimated landmark position
     * @p observations a vector of observation according to the @struct Observation
     * @p se a reference to the current SLAMEngine
     * @return a vector containg pair of indexes. The indexes refere to the position of observation in @p observations and the index of the landmark in @p se
     */
    std::vector<LandmarkAssociation> GreedyDataAssociation(const std::vector<Observation>& observations, const EKFSLAMEngine& se);
	
    namespace SequentialDataAssociationParams {
//         extern ScalarType DistanceThreshold;
    }
    ///FIXME: BUGGED
	std::vector<LandmarkAssociation> SequentialDataAssociation(const std::vector<Observation>&, const EKFSLAMEngine&);
	
    namespace HungarianDataAssociationParams {
//         extern ScalarType DistanceThreshold;
        extern ScalarType ToIntFactor;
    }
    std::vector<LandmarkAssociation> HungarianDataAssociation(const std::vector<Observation>&, const EKFSLAMEngine&);
    
	}
}