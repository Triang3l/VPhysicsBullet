// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_shadow.h"

CPhysicsShadowController::CPhysicsShadowController() :
		m_UseShadowMaterial(true) {}

void CPhysicsShadowController::UseShadowMaterial(bool bUseShadowMaterial) {
	m_UseShadowMaterial = bUseShadowMaterial;
}
