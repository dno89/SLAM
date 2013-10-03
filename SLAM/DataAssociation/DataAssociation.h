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
    namespace BasicDataAssociationParams {
        extern ScalarType DistanceThreshold;
    }
    std::vector<LandmarkAssociation> BasicDataAssociation(const std::vector<Observation>&, const EKFSLAMEngine&);
	
    namespace SequentialDataAssociationParams {
        extern ScalarType DistanceThreshold;
    }
	std::vector<LandmarkAssociation> SequentialDataAssociation(const std::vector<Observation>&, const EKFSLAMEngine&);
	
	}
}