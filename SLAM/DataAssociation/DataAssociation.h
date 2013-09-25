#pragma once

////include
//SLAM
#include "../Base/core.h"
#include "../Engine/EKFSLAMEngine.h"
//std
#include <vector>

namespace SLAM {
    ////TYPE FORWARDING
    class EKFSLAMEngine;
    
    ////PROTOTYPES
    std::vector<AssociatedPerception> BasicDataAssociation(const std::vector<Observation>&, const EKFSLAMEngine&);
}