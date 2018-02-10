// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_shadow.h"
#include "physics_object.h"

void ComputeVPhysicsController(btVector3 &currentSpeed, const btVector3 &delta,
		btScalar maxSpeed, btScalar maxDampSpeed, btScalar scaleDelta, btScalar damping,
		btVector3 *outImpulse) {
	if (currentSpeed.length2() < 1e-6f) {
		currentSpeed.setZero();
	}

	btVector3 addVelocity = delta * scaleDelta;
	btScalar addSpeed2 = addVelocity.length2();
	if (addSpeed2 > maxSpeed * maxSpeed) {
		if (maxSpeed > 0.0f) {
			addVelocity *= maxSpeed / btSqrt(addSpeed2);
		} else {
			addVelocity.setZero();
		}
	}

	btVector3 dampVelocity = currentSpeed * -damping;
	btScalar dampSpeed2 = dampVelocity.length2();
	if (dampSpeed2 > maxDampSpeed * maxDampSpeed) {
		if (maxDampSpeed > 0.0f) {
			dampVelocity *= maxDampSpeed / btSqrt(dampSpeed2);
		} else {
			dampVelocity.setZero();
		}
	}

	currentSpeed += addVelocity + dampVelocity;

	if (outImpulse != nullptr) {
		*outImpulse = addVelocity;
	}
}

CPhysicsShadowController::CPhysicsShadowController(IPhysicsObject *object,
			bool allowTranslation, bool allowRotation) :
		m_Object(object),
		m_SecondsToArrival(0.0f),
		m_Enable(false),
		m_AllowPhysicsMovement(allowTranslation),
		m_AllowPhysicsRotation(allowRotation),
		m_UseShadowMaterial(true) {
	memset(&m_Shadow, 0, sizeof(m_Shadow));
	m_Shadow.m_DampFactor = 1.0f;
	static_cast<CPhysicsObject *>(m_Object)->NotifyAttachedToShadowController(this);
}

CPhysicsShadowController::~CPhysicsShadowController() {
	static_cast<CPhysicsObject *>(m_Object)->NotifyAttachedToShadowController(nullptr);
}

void CPhysicsShadowController::Update(const Vector &position, const QAngle &angles, float timeOffset) {
	btTransform oldTransform = m_Shadow.m_TargetObjectTransform;
	btVector3 &origin = m_Shadow.m_TargetObjectTransform.getOrigin();
	btMatrix3x3 &basis = m_Shadow.m_TargetObjectTransform.getBasis();
	ConvertPositionToBullet(position, origin);
	ConvertRotationToBullet(angles, basis);
	m_SecondsToArrival = btMax(btScalar(timeOffset), btScalar(0.0f));
	m_Enable = true;
	// Don't wake up if calling with exactly the same data repeatedly.
	if ((origin - oldTransform.getOrigin()).length2() < 1e-6f) {
		btVector3 basisDelta = (basis[0] - oldTransform.getBasis()[0]).absolute() +
				(basis[1] - oldTransform.getBasis()[1]).absolute() +
				(basis[2] - oldTransform.getBasis()[2]).absolute();
		if (basisDelta.length2() < 1e-6f) {
			return;
		}
	}
	m_Object->Wake();
}

void CPhysicsShadowController::MaxSpeed(float maxSpeed, float maxAngularSpeed) {
	m_Shadow.m_MaxSpeed = m_Shadow.m_MaxDampSpeed = HL2BULLET(maxSpeed);
	m_Shadow.m_MaxAngular = m_Shadow.m_MaxDampAngular = DEG2RAD(maxAngularSpeed);
}

void CPhysicsShadowController::StepUp(float height) {
	static_cast<CPhysicsObject *>(m_Object)->StepUp(HL2BULLET(height));
}

void CPhysicsShadowController::SetTeleportDistance(float teleportDistance) {
	m_Shadow.m_TeleportDistance = HL2BULLET(teleportDistance);
}

float CPhysicsShadowController::GetTargetPosition(Vector *pPositionOut, QAngle *pAnglesOut) {
	if (pPositionOut != nullptr) {
		ConvertPositionToHL(m_Shadow.m_TargetObjectTransform.getOrigin(), *pPositionOut);
	}
	if (pAnglesOut != nullptr) {
		ConvertRotationToHL(m_Shadow.m_TargetObjectTransform.getBasis(), *pAnglesOut);
	}
	return (float) m_SecondsToArrival;
}

float CPhysicsShadowController::GetTeleportDistance() {
	return BULLET2HL(m_Shadow.m_TeleportDistance);
}

void CPhysicsShadowController::GetMaxSpeed(float *pMaxSpeedOut, float *pMaxAngularSpeedOut) {
	if (pMaxSpeedOut != nullptr) {
		*pMaxSpeedOut = BULLET2HL(m_Shadow.m_MaxSpeed);
	}
	if (pMaxAngularSpeedOut != nullptr) {
		*pMaxAngularSpeedOut = RAD2DEG(m_Shadow.m_MaxAngular);
	}
}

bool CPhysicsShadowController::AllowsTranslation() {
	return m_AllowPhysicsMovement;
}

bool CPhysicsShadowController::AllowsRotation() {
	return m_AllowPhysicsRotation;
}

void CPhysicsShadowController::GetLastImpulse(Vector *pOut) {
	ConvertPositionToHL(m_Shadow.m_LastImpulse, *pOut);
}

void CPhysicsShadowController::UseShadowMaterial(bool bUseShadowMaterial) {
	m_UseShadowMaterial = bUseShadowMaterial;
}

void CPhysicsShadowController::ObjectMaterialChanged(int materialIndex) {
	// No need to do anything as the object handles the shadow material.
}

void CPhysicsShadowController::Simulate(btScalar timeStep) {
	if (!m_Enable) {
		m_Shadow.m_LastObjectPosition.setZero();
		return;
	}
	CPhysicsObject *object = static_cast<CPhysicsObject *>(m_Object);
	object->ComputeBulletShadowControl(m_Shadow, m_SecondsToArrival, timeStep);
	// If we have time left, subtract it off.
	m_SecondsToArrival = btMax(m_SecondsToArrival - timeStep, btScalar(0.0f));
	// TODO: For objects with physics movement allowed, do something with friction snapshots.
}

CPhysicsPlayerController::CPhysicsPlayerController(IPhysicsObject *object) :
		m_Object(object),
		m_PushInvMassLimit(1.0f / 50000.0f),
		m_PushSpeedLimit(HL2BULLET(10000.0f)) {}

void CPhysicsPlayerController::SetObject(IPhysicsObject *pObject) {
	Assert(pObject != nullptr);
	if (m_Object == pObject) {
		return;
	}
	static_cast<CPhysicsObject *>(m_Object)->NotifyAttachedToPlayerController(nullptr);
	m_Object = pObject;
	static_cast<CPhysicsObject *>(m_Object)->NotifyAttachedToPlayerController(this);
}

void CPhysicsPlayerController::StepUp(float height) {
	static_cast<CPhysicsObject *>(m_Object)->StepUp(HL2BULLET(height));
}

void CPhysicsPlayerController::Jump() {
	// Not implemented in IVP VPhysics.
}

IPhysicsObject *CPhysicsPlayerController::GetObject() {
	return m_Object;
}

void CPhysicsPlayerController::SetPushMassLimit(float maxPushMass) {
	if (maxPushMass > 0.0f) {
		m_PushInvMassLimit = 1.0f / maxPushMass;
	} else {
		m_PushInvMassLimit = BT_LARGE_FLOAT;
	}
}

void CPhysicsPlayerController::SetPushSpeedLimit(float maxPushSpeed) {
	m_PushSpeedLimit = HL2BULLET(maxPushSpeed);
}

float CPhysicsPlayerController::GetPushMassLimit() {
	if (m_PushInvMassLimit >= BT_LARGE_FLOAT) {
		return 0.0f;
	}
	return (float) (1.0f / m_PushInvMassLimit);
}

float CPhysicsPlayerController::GetPushSpeedLimit() {
	return BULLET2HL(m_PushSpeedLimit);
}
