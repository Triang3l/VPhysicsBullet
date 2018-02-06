// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_SHADOW_H
#define PHYSICS_SHADOW_H

#include "physics_internal.h"

struct ShadowControlBulletParameters_t {
	// Positions are in object coordinates, not mass center coordinates.

	btTransform m_TargetObjectTransform;
	btScalar m_MaxAngular;
	btScalar m_MaxDampAngular;
	btScalar m_MaxSpeed;
	btScalar m_MaxDampSpeed;
	btScalar m_DampFactor;
	btScalar m_TeleportDistance;

	btVector3 m_LastObjectPosition;
	btVector3 m_LastImpulse;

	ShadowControlBulletParameters_t() :
			m_TargetObjectTransform(btTransform::getIdentity()),
			m_MaxAngular(0.0f), m_MaxDampAngular(0.0f),
			m_MaxSpeed(0.0f), m_MaxDampSpeed(0.0f),
			m_DampFactor(0.0f), m_TeleportDistance(0.0f),
			m_LastObjectPosition(0.0f, 0.0f, 0.0f),
			m_LastImpulse(0.0f, 0.0f, 0.0f) {}

	ShadowControlBulletParameters_t(const hlshadowcontrol_params_t &params) :
			m_LastObjectPosition(0.0f, 0.0f, 0.0f),
			m_LastImpulse(0.0f, 0.0f, 0.0f) {
		ConvertFromHL(params);
	}

	void ConvertFromHL(const hlshadowcontrol_params_t &params) {
		ConvertPositionToBullet(params.targetPosition, m_TargetObjectTransform.getOrigin());
		ConvertRotationToBullet(params.targetRotation, m_TargetObjectTransform.getBasis());
		m_MaxAngular = DEG2RAD(params.maxAngular);
		m_MaxDampAngular = DEG2RAD(params.maxDampAngular);
		m_MaxSpeed = HL2BULLET(params.maxSpeed);
		m_MaxDampSpeed = HL2BULLET(params.maxDampSpeed);
		m_DampFactor = params.dampFactor;
		m_TeleportDistance = HL2BULLET(params.teleportDistance);
	}

	void ConvertToHL(hlshadowcontrol_params_t &params) const {
		ConvertPositionToHL(m_TargetObjectTransform.getOrigin(), params.targetPosition);
		ConvertRotationToHL(m_TargetObjectTransform.getBasis(), params.targetRotation);
		params.maxAngular = RAD2DEG(m_MaxAngular);
		params.maxDampAngular = RAD2DEG(m_MaxDampAngular);
		params.maxSpeed = BULLET2HL(m_MaxSpeed);
		params.maxDampSpeed = BULLET2HL(m_MaxDampSpeed);
		params.dampFactor = m_DampFactor;
		params.teleportDistance = BULLET2HL(m_TeleportDistance);
	}
};

void ComputeVPhysicsController(btVector3 &currentSpeed, const btVector3 &delta,
		btScalar maxSpeed, btScalar maxDampSpeed, btScalar scaleDelta, btScalar damping,
		btVector3 *outImpulse);

class CPhysicsShadowController : public IPhysicsShadowController {
public:
	CPhysicsShadowController(IPhysicsObject *object,
			bool allowTranslation, bool allowRotation);
	virtual ~CPhysicsShadowController();

	// IPhysicsShadowController methods.

	virtual void MaxSpeed(float maxSpeed, float maxAngularSpeed);
	virtual void StepUp(float height);
	virtual void SetTeleportDistance(float teleportDistance);
	virtual bool AllowsTranslation();
	virtual bool AllowsRotation();
	virtual void GetLastImpulse(Vector *pOut);
	virtual void UseShadowMaterial(bool bUseShadowMaterial);
	virtual void ObjectMaterialChanged(int materialIndex);
	virtual float GetTeleportDistance();
	virtual void GetMaxSpeed(float *pMaxSpeedOut, float *pMaxAngularSpeedOut);

	// Internal methods.

	void Simulate(btScalar timeStep);
	FORCEINLINE bool IsUsingShadowMaterial() const { return m_UseShadowMaterial; }

private:
	IPhysicsObject *m_Object;
	ShadowControlBulletParameters_t m_Shadow;
	btScalar m_SecondsToArrival;
	bool m_Enable;
	bool m_AllowPhysicsMovement, m_AllowPhysicsRotation;
	bool m_UseShadowMaterial;
};

#endif
