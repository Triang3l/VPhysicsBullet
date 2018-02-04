// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_SHADOW_H
#define PHYSICS_SHADOW_H

#include "physics_internal.h"

struct ShadowControlBulletParameters_t {
	// Positions are in object coordinates, not mass center coordinates.

	btTransform targetObjectTransform;
	btScalar maxAngular;
	btScalar maxDampAngular;
	btScalar maxSpeed;
	btScalar maxDampSpeed;
	btScalar dampFactor;
	btScalar teleportDistance;

	btVector3 lastObjectPosition;
	btVector3 lastImpulse;
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

	virtual void StepUp(float height);
	virtual void SetTeleportDistance(float teleportDistance);
	virtual bool AllowsTranslation();
	virtual bool AllowsRotation();
	virtual void GetLastImpulse(Vector *pOut);
	virtual void UseShadowMaterial(bool bUseShadowMaterial);
	virtual void ObjectMaterialChanged(int materialIndex);
	virtual float GetTeleportDistance();

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
