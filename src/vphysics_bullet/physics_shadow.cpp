// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_shadow.h"
#include "physics_object.h"

static void ComputeController(btVector3 &currentSpeed, const btVector3 &delta,
		btScalar maxSpeed, btScalar maxDampSpeed,
		btScalar scaleDelta, btScalar damping,
		btVector3 *outAcceleration) {
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

	if (outAcceleration != nullptr) {
		*outAcceleration = addVelocity;
	}
}

CPhysicsShadowController::CPhysicsShadowController(IPhysicsObject *object,
			bool allowTranslation, bool allowRotation) :
		m_Object(object),
		m_AllowPhysicsMovement(allowTranslation),
		m_AllowPhysicsRotation(allowRotation),
		m_UseShadowMaterial(true) {
	static_cast<CPhysicsObject *>(m_Object)->NotifyAttachedToShadowController(this);
}

CPhysicsShadowController::~CPhysicsShadowController() {
	static_cast<CPhysicsObject *>(m_Object)->NotifyAttachedToShadowController(nullptr);
}

void CPhysicsShadowController::StepUp(float height) {
	static_cast<CPhysicsObject *>(m_Object)->StepUp(HL2BULLET(height));
}

bool CPhysicsShadowController::AllowsTranslation() {
	return m_AllowPhysicsMovement;
}

bool CPhysicsShadowController::AllowsRotation() {
	return m_AllowPhysicsRotation;
}

void CPhysicsShadowController::UseShadowMaterial(bool bUseShadowMaterial) {
	m_UseShadowMaterial = bUseShadowMaterial;
}

void CPhysicsShadowController::ObjectMaterialChanged(int materialIndex) {
	// No need to do anything as the object handles the shadow material.
}
