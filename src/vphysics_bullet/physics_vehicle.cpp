// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_vehicle.h"

CPhysicsVehicleController::CPhysicsVehicleController(const vehicleparams_t &params) :
		m_VehicleParameters(params) {
	memset(&m_OperatingParameters, 0, sizeof(m_OperatingParameters));
}

const vehicle_operatingparams_t &CPhysicsVehicleController::GetOperatingParams() {
	return m_OperatingParameters;
}

const vehicleparams_t &CPhysicsVehicleController::GetVehicleParams() {
	return m_VehicleParameters;
}

vehicleparams_t &CPhysicsVehicleController::GetVehicleParamsForChange() {
	return m_VehicleParameters;
}

/* DUMMY */ bool CPhysicsVehicleController::GetWheelContactPoint(int index, Vector *pContactPoint, int *pSurfaceProps) {
	if (pContactPoint != nullptr) {
		pContactPoint->Zero();
	}
	if (pSurfaceProps != nullptr) {
		*pSurfaceProps = 0;
	}
	return false;
}
