// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_SHADOW_H
#define PHYSICS_SHADOW_H

#include "physics_internal.h"

class CPhysicsShadowController : public IPhysicsShadowController {
public:
	CPhysicsShadowController();

	// IPhysicsShadowController methods.

	virtual void UseShadowMaterial(bool bUseShadowMaterial);

	// Internal methods.

	FORCEINLINE bool IsUsingShadowMaterial() const { return m_UseShadowMaterial; }

private:
	bool m_UseShadowMaterial;
};

#endif
