// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_vehicle.h"
#include "physics_constraint.h"
#include "physics_environment.h"
#include "physics_object.h"

CPhysicsVehicleController::CPhysicsVehicleController(
		IPhysicsObject *bodyObject, const vehicleparams_t &params) :
		m_BodyObject(nullptr), m_VehicleParameters(params) {
	btSetMin(m_VehicleParameters.axleCount, VEHICLE_MAX_AXLE_COUNT);
	btSetMin(m_VehicleParameters.wheelsPerAxle, VEHICLE_MAX_WHEEL_COUNT / VEHICLE_MAX_AXLE_COUNT);

	memset(&m_Wheels, 0, sizeof(m_Wheels));

	memset(&m_OperatingParameters, 0, sizeof(m_OperatingParameters));

	SetBodyObject(bodyObject);
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

int CPhysicsVehicleController::GetWheelCount() {
	return m_VehicleParameters.axleCount * m_VehicleParameters.wheelsPerAxle;
}

IPhysicsObject *CPhysicsVehicleController::GetWheel(int index) {
	if (index < 0 || index >= GetWheelCount()) {
		return nullptr;
	}
	return m_Wheels[index].m_Object;
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

void CPhysicsVehicleController::SetBodyObject(IPhysicsObject *bodyObject) {
	if (m_BodyObject != nullptr) {
		static_cast<CPhysicsObject *>(m_BodyObject)->NotifyAttachedToVehicleController(nullptr, false);
		DestroyWheels();
	}
	m_BodyObject = bodyObject;
	if (bodyObject != nullptr) {
		CreateWheels();
		static_cast<CPhysicsObject *>(bodyObject)->NotifyAttachedToVehicleController(this, false);
	}
}

void CPhysicsVehicleController::CreateWheels() {
	CPhysicsObject *bodyObject = static_cast<CPhysicsObject *>(m_BodyObject);
	CPhysicsEnvironment *environment = static_cast<CPhysicsEnvironment *>(bodyObject->GetEnvironment());

	bodyObject->Wake();

	Vector bodyPosition;
	QAngle bodyAngles;
	matrix3x4_t bodyTransform;
	bodyObject->GetPositionAtPSI(&bodyPosition, &bodyAngles);
	AngleMatrix(bodyAngles, bodyPosition, bodyTransform);

	objectparams_t wheelObjectParams;
	memset(&wheelObjectParams, 0, sizeof(wheelObjectParams));
	wheelObjectParams.pName = "VehicleWheel";
	wheelObjectParams.pGameData = bodyObject->GetGameData();
	wheelObjectParams.enableCollisions = true;

	int wheelIndex = 0;
	for (int axleIndex = 0; axleIndex < m_VehicleParameters.axleCount; ++axleIndex) {
		const vehicle_axleparams_t &axle = m_VehicleParameters.axles[axleIndex];

		wheelObjectParams.mass = axle.wheels.mass;
		wheelObjectParams.inertia = axle.wheels.inertia;
		wheelObjectParams.damping = axle.wheels.damping;
		wheelObjectParams.rotdamping = axle.wheels.rotdamping;
		wheelObjectParams.volume = ((4.0f / 3.0f) * M_PI_F) *
				axle.wheels.radius * axle.wheels.radius * axle.wheels.radius;

		for (int axleWheelIndex = 0; axleWheelIndex < m_VehicleParameters.wheelsPerAxle;
				++axleWheelIndex, ++wheelIndex) {
			Wheel &wheel = m_Wheels[wheelIndex];
			if (wheel.m_Object != nullptr) {
				// The wheel was reloaded during save load.
				// TODO: Check if this actually happens - either handle properly, or prevent.
				continue;
			}
			Vector wheelOffset = axle.offset + ((axleWheelIndex & 1 ? 1.0f : -1.0f) * axle.wheelOffset);
			Vector wheelPosition;
			VectorTransform(wheelOffset, bodyTransform, wheelPosition);
			CPhysicsObject *wheelObject = static_cast<CPhysicsObject *>(environment->CreateSphereObject(
					axle.wheels.radius, axle.wheels.materialIndex, wheelPosition, bodyAngles,
					&wheelObjectParams, false));
			wheel.m_Object = wheelObject;
			wheelObject->NotifyAttachedToVehicleController(this, true);
			wheelObject->SetCallbackFlags(wheelObject->GetCallbackFlags() | CALLBACK_IS_VEHICLE_WHEEL);
			wheel.m_Constraint = static_cast<CPhysicsConstraint_Suspension *>(
					environment->CreateSuspensionConstraint(bodyObject, wheelObject,
					nullptr, wheelOffset, axle.suspension));
			wheelObject->Wake();
		}
	}
}

void CPhysicsVehicleController::ShiftWheelTransforms(const btTransform &offset) {
	int wheelCount = GetWheelCount();
	for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
		IPhysicsObject *wheelObject = m_Wheels[wheelIndex].m_Object;
		if (wheelObject == nullptr) {
			continue;
		}
		CPhysicsObject *wheelPhysicsObject = static_cast<CPhysicsObject *>(wheelObject);
		wheelPhysicsObject->ProceedToTransform(offset * wheelPhysicsObject->GetRigidBody()->getWorldTransform());
	}
}

void CPhysicsVehicleController::DestroyWheels() {
	IPhysicsEnvironment *environment = static_cast<CPhysicsObject *>(m_BodyObject)->GetEnvironment();
	Assert(environment != nullptr);

	int wheelCount = GetWheelCount();
	for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
		Wheel &wheel = m_Wheels[wheelIndex];
		if (wheel.m_Constraint != nullptr) {
			environment->DestroyConstraint(wheel.m_Constraint);
			wheel.m_Constraint = nullptr;
		}
		if (wheel.m_Object != nullptr) {
			environment->DestroyObject(wheel.m_Object);
			wheel.m_Object = nullptr;
		}
	}
}

void CPhysicsVehicleController::ModifyGravity(btVector3 &gravity) {
	gravity += m_VehicleParameters.body.addGravity * gravity;
}

/***************************
 * Car with physical wheels
 ***************************/

CPhysicsVehicleController_WheeledCar::CPhysicsVehicleController_WheeledCar(
		IPhysicsObject *bodyObject, const vehicleparams_t &params) :
		CPhysicsVehicleController(bodyObject, params) {}

void CPhysicsVehicleController_WheeledCar::Release() {
	SetBodyObject(nullptr);
	VPhysicsDelete(CPhysicsVehicleController_WheeledCar, this);
}

/******************
 * Raycast airboat
 ******************/

CPhysicsVehicleController_Airboat::CPhysicsVehicleController_Airboat(
		IPhysicsObject *bodyObject, const vehicleparams_t &params,
		IPhysicsGameTrace *gameTrace) :
		CPhysicsVehicleController(bodyObject, params),
		m_GameTrace(gameTrace) {}

void CPhysicsVehicleController_Airboat::ModifyGravity(btVector3 &gravity) {
	gravity.setValue(0.0f, gravity.getY() <= 0.0f ? -9.81f : 9.81f, 0.0f);
}

void CPhysicsVehicleController_Airboat::Release() {
	SetBodyObject(nullptr);
	VPhysicsDelete(CPhysicsVehicleController_Airboat, this);
}
