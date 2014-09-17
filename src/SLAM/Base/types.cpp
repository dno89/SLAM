/**
 * \file types.cpp
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

////include
//SLAM
#include <SLAM/Base/types.h>

using namespace SLAM;

VehicleModel::VehicleModel(VehicleFunction f, VehicleJacobian df_dxv) : m_F(f), m_dF_dXv(df_dxv) {
	if(!(m_F && m_dF_dXv)) {
		throw std::runtime_error("VehicleModel::VehicleModel(VehicleFunction,VehicleJacobian) ERROR: f and df_dxv must be non-NULL!\n");
	}
}
VehicleModel::VehicleModel() {}
VehicleModel::operator bool() const {
	return m_F && m_dF_dXv;
}

VectorType VehicleModel::F(const VectorType& Xv, const VectorType& U) const {
	if(!(m_F && m_dF_dXv)) {
		throw std::runtime_error("VehicleModel::F ERROR: structure not initialized\n");
	}

	return (*m_F)(Xv, U);
}
MatrixType VehicleModel::dF_dXv(const VectorType& Xv, const VectorType& U) const {
	if(!(m_F && m_dF_dXv)) {
		throw std::runtime_error("VehicleModel::F ERROR: structure not initialized\n");
	}

	return (*m_dF_dXv)(Xv, U);
}


LandmarkPerceptionModel::LandmarkPerceptionModel( LandmarkPerceptionModel::ObservationFunction h, LandmarkPerceptionModel::ObservationJacobian dh_dxv, LandmarkPerceptionModel::ObservationJacobian dh_dxm, LandmarkPerceptionModel::DifferenceFunction difference, LandmarkPerceptionModel::DistanceFunction distance, ScalarType distanceThreshold, LandmarkPerceptionModel::SortFunction sort, LandmarkPerceptionModel::NormalizeFunction normalize ) : m_H(h), m_dH_dXv(dh_dxv), m_dH_dXm(dh_dxm), m_difference(difference), m_distance(distance), AssociationDistanceThreshold(distanceThreshold), m_sort(sort), m_normalize(normalize) {
	check("LandmarkModel::LandmarkModel(ObservationFunction,ObservationJacobian,ObservationJacobian) ERROR: h, dh_dxv and df_dxm must be non-NULL!\n");
}
LandmarkPerceptionModel::LandmarkPerceptionModel() {}
bool LandmarkPerceptionModel::operator==(const LandmarkPerceptionModel& m) {
	return  m_H == m.m_H &&
	        m_dH_dXm == m.m_dH_dXm &&
	        m_dH_dXv == m.m_dH_dXv;
}
LandmarkPerceptionModel::operator bool() const {
	return m_H && m_dH_dXm && m_dH_dXv && m_distance && m_difference && m_sort && m_normalize;
}
LandmarkPerceptionModel::ObservationType LandmarkPerceptionModel::H(const VehicleStateType& Xv, const LandmarkStateType& Xm) const {
	check("LandmarkModel::H ERROR: functions not initialized\n");
	return (*m_H)(Xv, Xm);
}
MatrixType LandmarkPerceptionModel::dH_dXv(const VehicleStateType& Xv, const LandmarkStateType& Xm) const {
	check("LandmarkModel::dH_dXv ERROR: functions not initialized\n");
	return (*m_dH_dXv)(Xv, Xm);
}
MatrixType LandmarkPerceptionModel::dH_dXm(const VehicleStateType& Xv, const LandmarkStateType& Xm) const {
	check("LandmarkModel::dH_dXm ERROR: functions not initialized\n");
	return (*m_dH_dXm)(Xv, Xm);
}
VectorType LandmarkPerceptionModel::Difference(const ObservationType& v1, const ObservationType& v2) const {
	check("LandmarkModel::Difference ERROR: functions not initialized\n");
	return (*m_difference)(v1, v2);
}
ScalarType LandmarkPerceptionModel::Distance(const ObservationType& v1, const ObservationType& v2) const {
	check("LandmarkModel::Distance ERROR: functions not initialized\n");
	return (*m_distance)(v1, v2);
}
bool LandmarkPerceptionModel::Sort(const ObservationType& v1, const ObservationType& v2) const {
	check("LandmarkModel::Sort ERROR: functions not initialized\n");
	return (*m_sort)(v1, v2);
}
VectorType LandmarkPerceptionModel::Normalize(const ObservationType& v) const {
	if(!(m_H && m_dH_dXm && m_dH_dXv && m_distance && m_difference && m_sort && m_normalize)) {
		throw std::runtime_error("LandmarkModel::Distance ERROR: functions not initialized\n");
	}

	return (*m_normalize)(v);
}
inline void LandmarkPerceptionModel::check(const char* str) const {
	if(!(m_H && m_dH_dXm && m_dH_dXv && m_distance && m_difference && m_sort && m_normalize)) {
		throw std::runtime_error(str);
	}
}

LandmarkInitializationModel::LandmarkInitializationModel(InitializationFunction g, InitializationJacobian dg_dxv, InitializationJacobian dg_dz) : m_G(g), m_dG_dXv(dg_dxv), m_dG_dZ(dg_dz) {
	if(!(m_G && m_dG_dZ && m_dG_dXv)) {
		throw std::runtime_error("LandmarkInitializationModel::LandmarkInitializationModel(InitializationFunction,InitializationJacobian,InitializationJacobian) ERROR: g, dg_dxv and dg_dz must be non-NULL!\n");
	}
}
LandmarkInitializationModel::LandmarkInitializationModel() {}
LandmarkInitializationModel::operator bool() const {
	return m_G && m_dG_dZ && m_dG_dXv;
}

LandmarkInitializationModel::LandmarkStateType LandmarkInitializationModel::G(const VehicleStateType& Xv, const ObservationType& Z) const {
	if(!(m_G && m_dG_dZ && m_dG_dXv)) {
		throw std::runtime_error("LandmarkInitializationModel::H ERROR: functions not initialized\n");
	}

	return (*m_G)(Xv, Z);
}
MatrixType LandmarkInitializationModel::dG_dXv(const VehicleStateType& Xv, const ObservationType& Z) const {
	if(!(m_G && m_dG_dZ && m_dG_dXv)) {
		throw std::runtime_error("LandmarkInitializationModel::dH_dXv ERROR: functions not initialized\n");
	}

	return (*m_dG_dXv)(Xv, Z);
}
MatrixType LandmarkInitializationModel::dG_dZ(const VehicleStateType& Xv, const ObservationType& Z) const {
	if(!(m_G && m_dG_dZ && m_dG_dXv)) {
		throw std::runtime_error("LandmarkInitializationModel::dH_dZ ERROR: functions not initialized\n");
	}

	return (*m_dG_dZ)(Xv, Z);
}

LandmarkModel::LandmarkModel() {}
LandmarkModel::LandmarkModel(const LandmarkPerceptionModel& lpm, const LandmarkInitializationModel& lim) :
	LPM(lpm), LIM(lim) {}

// RestrainedLandmarkModel::RestrainedLandmarkModel(const LandmarkPerceptionModel& landmark_model, const VectorType& landmark_state) :
// 	m_lm(landmark_model), m_ls(landmark_state) {}
// 
// RestrainedLandmarkModel::ObservationType RestrainedLandmarkModel::H(const VehicleStateType& Xv) const {
// 	return m_lm.H(Xv, m_ls);
// }
// MatrixType RestrainedLandmarkModel::dH_dXv(const VehicleStateType& Xv) const {
// 	return m_lm.dH_dXv(Xv, m_ls);
// }
// MatrixType RestrainedLandmarkModel::dH_dXm(const VehicleStateType& Xv) const {
// 	return m_lm.dH_dXm(Xv, m_ls);
// }
// ScalarType RestrainedLandmarkModel::Distance(const ObservationType& v1, const ObservationType& v2) const {
// 	return m_lm.Distance(v1, v2);
// }
// VectorType RestrainedLandmarkModel::Difference(const ObservationType& v1, const ObservationType& v2) const {
// 	return m_lm.Difference(v1, v2);
// }
// VectorType RestrainedLandmarkModel::Normalize(const ObservationType& v) const {
// 	return m_lm.Normalize(v);
// }
// const LandmarkPerceptionModel& RestrainedLandmarkModel::GetModel() const {
// 	return m_lm;
// }
// RestrainedLandmarkModel::operator const LandmarkPerceptionModel&() const {
// 	return m_lm;
// }

Landmark::Landmark(const VectorType& state, const LandmarkPerceptionModel& model) :
	AccumulatedSize(0), Xm(state), Model(model) {}

Landmark::Landmark(const Landmark& l) : AccumulatedSize(l.AccumulatedSize), Xm(l.Xm), Model(l.Model)
{}

Observation::Observation() {}
Observation::Observation(const VectorType& z, const MatrixType& pz, const LandmarkModel& lm) : Z(z), Pz(pz), LM(lm)
{}
Observation::Observation(const VectorType& z, const MatrixType& pz, const LandmarkPerceptionModel& lpm, const LandmarkInitializationModel& lim) : Z(z), Pz(pz), LM(lpm, lim)
{}

AssociatedPerception::AssociatedPerception(const VectorType& observation, int associated_index) :
	Z(observation), AssociatedIndex(associated_index), AccumulatedSize(0) {}
LandmarkAssociation::LandmarkAssociation(int observation_index, int landmark_index) :
	ObservationIndex(observation_index), LandmarkIndex(landmark_index)
{}

ProprioceptiveModel::ProprioceptiveModel(ObservationFunction h, ObservationJacobian dh_dxv, DifferenceFunction difference) : m_H(h), m_dH_dXv(dh_dxv), m_difference(difference) {
	check("ProprioceptiveModel::ProprioceptiveModel(ObservationFunction,ObservationJacobian,DifferenceFunction) ERROR: h and dh_dxv must be non-NULL!\n");
}
ProprioceptiveModel::ProprioceptiveModel() {}
bool ProprioceptiveModel::operator==(const ProprioceptiveModel& m) {
	return  m_H == m.m_H &&
	        m_dH_dXv == m.m_dH_dXv;
}
ProprioceptiveModel::operator bool() const {
	return m_H && m_dH_dXv && m_difference;
}
ProprioceptiveModel::ObservationType ProprioceptiveModel::H(const VehicleStateType& Xv) const {
	check("ProprioceptiveModel::H ERROR: functions not initialized\n");
	return (*m_H)(Xv);
}
MatrixType ProprioceptiveModel::dH_dXv(const VehicleStateType& Xv) const {
	check("ProprioceptiveModel::dH_dXv ERROR: functions not initialized\n");
	return (*m_dH_dXv)(Xv);
}
VectorType ProprioceptiveModel::Difference(const ObservationType& v1, const ObservationType& v2) const {
	check("ProprioceptiveModel::Difference ERROR: functions not initialized\n");
	return (*m_difference)(v1, v2);
}
inline void ProprioceptiveModel::check(const char* str) const {
	if(!(m_H && m_dH_dXv && m_difference)) {
		throw std::runtime_error(str);
	}
}

ProprioceptiveObservation::ProprioceptiveObservation() {}
ProprioceptiveObservation::ProprioceptiveObservation(const VectorType& z, const MatrixType& pz, const ProprioceptiveModel& pm) : Z(z), Pz(pz), PM(pm)
{}