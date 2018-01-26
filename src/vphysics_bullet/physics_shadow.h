// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_SHADOW_H
#define PHYSICS_SHADOW_H

#include "physics_internal.h"

class CPhysicsShadowController : public IPhysicsShadowController {
public:
	CPhysicsShadowController(IPhysicsObject *object,
			bool allowTranslation, bool allowRotation);

	// IPhysicsShadowController methods.

	virtual void StepUp(float height);
	virtual bool AllowsTranslation();
	virtual bool AllowsRotation();
	virtual void UseShadowMaterial(bool bUseShadowMaterial);
	virtual void ObjectMaterialChanged(int materialIndex);

	// Internal methods.

	FORCEINLINE bool IsUsingShadowMaterial() const { return m_UseShadowMaterial; }

private:
	IPhysicsObject *m_Object;

	bool m_AllowPhysicsMovement, m_AllowPhysicsRotation;

	bool m_UseShadowMaterial;
};

#endif
