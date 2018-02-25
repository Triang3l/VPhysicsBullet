// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_VEHICLE_H
#define PHYSICS_VEHICLE_H

#include "physics_internal.h"
#include "vphysics/vehicles.h"

class CPhysicsVehicleController : public IPhysicsVehicleController {
public:
	BT_DECLARE_ALIGNED_ALLOCATOR()

	CPhysicsVehicleController(const vehicleparams_t &params);

	/* DUMMY */ virtual void Update(float dt, vehicle_controlparams_t &controls) {}
	virtual const vehicle_operatingparams_t &GetOperatingParams();
	virtual const vehicleparams_t &GetVehicleParams();
	virtual vehicleparams_t &GetVehicleParamsForChange();
	/* DUMMY */ virtual float UpdateBooster(float dt) { return 0.0f; }
	/* DUMMY */ virtual int GetWheelCount() { return 0; }
	/* DUMMY */ virtual IPhysicsObject *GetWheel(int index) { return nullptr; }
	/* DUMMY */ virtual bool GetWheelContactPoint(int index, Vector *pContactPoint, int *pSurfaceProps);
	/* DUMMY */ virtual void SetSpringLength(int wheelIndex, float length) {}
	/* DUMMY */ virtual void SetWheelFriction(int wheelIndex, float friction) {}
	/* DUMMY */ virtual void OnVehicleEnter() {}
	/* DUMMY */ virtual void OnVehicleExit() {}
	/* DUMMY */ virtual void SetEngineDisabled(bool bDisable) {}
	/* DUMMY */ virtual bool IsEngineDisabled() { return false; }
	/* DUMMY */ virtual void GetCarSystemDebugData(vehicle_debugcarsystem_t &debugCarSystem) {
		memset(&debugCarSystem, 0, sizeof(debugCarSystem));
	}
	/* DUMMY */ virtual void VehicleDataReload() {}

private:
	vehicleparams_t m_VehicleParameters;
	vehicle_operatingparams_t m_OperatingParameters;
};

#endif
