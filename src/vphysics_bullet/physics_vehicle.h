// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_VEHICLE_H
#define PHYSICS_VEHICLE_H

#include "physics_internal.h"
#include "vphysics/vehicles.h"

class CPhysicsVehicleController : public IPhysicsVehicleController {
public:
	// IPhysicsVehicleController methods.

	/* DUMMY */ virtual void Update(float dt, vehicle_controlparams_t &controls) {}
	virtual const vehicle_operatingparams_t &GetOperatingParams();
	virtual const vehicleparams_t &GetVehicleParams();
	virtual vehicleparams_t &GetVehicleParamsForChange();
	/* DUMMY */ virtual float UpdateBooster(float dt) { return 0.0f; }
	virtual int GetWheelCount();
	virtual IPhysicsObject *GetWheel(int index);
	/* DUMMY */ virtual bool GetWheelContactPoint(int index, Vector *pContactPoint, int *pSurfaceProps);
	/* DUMMY */ virtual void SetSpringLength(int wheelIndex, float length) {}
	/* DUMMY */ virtual void SetWheelFriction(int wheelIndex, float friction) {}
	virtual void OnVehicleEnter();
	virtual void OnVehicleExit();
	/* DUMMY */ virtual void SetEngineDisabled(bool bDisable) {}
	/* DUMMY */ virtual bool IsEngineDisabled() { return false; }
	/* DUMMY */ virtual void GetCarSystemDebugData(vehicle_debugcarsystem_t &debugCarSystem) {
		memset(&debugCarSystem, 0, sizeof(debugCarSystem));
	}
	/* DUMMY */ virtual void VehicleDataReload() {}

	// Internal methods.

	void SetBodyObject(IPhysicsObject *bodyObject);

	void ShiftWheelTransforms(const btTransform &offset);

	virtual void ModifyGravity(btVector3 &gravity);
	virtual void Simulate(btScalar timeStep) = 0;

	virtual void Release() = 0; // Subclasses must call SetBodyObject(nullptr)!

protected:
	CPhysicsVehicleController(IPhysicsObject *bodyObject, const vehicleparams_t &params);

	IPhysicsObject *m_BodyObject;

	vehicleparams_t m_VehicleParameters;

	struct Wheel {
		IPhysicsObject *m_Object;
		class CPhysicsConstraint_Suspension *m_Constraint;
	};
	Wheel m_Wheels[VEHICLE_MAX_WHEEL_COUNT];

	bool m_Occupied;
	vehicle_operatingparams_t m_OperatingParameters;

private:
	void CreateWheels();
	void DestroyWheels();
};

class CPhysicsVehicleController_WheeledCar : public CPhysicsVehicleController {
public:
	CPhysicsVehicleController_WheeledCar(IPhysicsObject *bodyObject,
			const vehicleparams_t &params);

	virtual void Simulate(btScalar timeStep);

	virtual void Release();
};

class CPhysicsVehicleController_Airboat : public CPhysicsVehicleController {
public:
	CPhysicsVehicleController_Airboat(IPhysicsObject *bodyObject,
			const vehicleparams_t &params, IPhysicsGameTrace *gameTrace);

	virtual void ModifyGravity(btVector3 &gravity);
	virtual void Simulate(btScalar timeStep);

	virtual void Release();

private:
	IPhysicsGameTrace *m_GameTrace;
};

#endif
