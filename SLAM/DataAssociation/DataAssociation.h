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
    
    ////PROTOTYPES
    namespace BasicDataAssociationConstants {
        extern double DistanceThreshold;
    }
    std::vector<LandmarkAssociation> BasicDataAssociation(const std::vector<Observation>&, const EKFSLAMEngine&);
	
	std::vector<LandmarkAssociation> SequentialDataAssociation(const std::vector<Observation>&, const EKFSLAMEngine&);
}