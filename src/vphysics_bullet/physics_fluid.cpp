// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_fluid.h"
#include "physics_material.h"

// memdbgon must be the last include file in a .cpp file!!!
// #include "tier0/memdbgon.h"

CPhysicsFluidController::CPhysicsFluidController(IPhysicsObject *fluidObject, const fluidparams_t *params) :
		m_Contents(params->contents),
		m_GameData(params->pGameData) {
	ConvertDirectionToBullet(params->surfacePlane.AsVector3D(), m_SurfacePlane);
	m_SurfacePlane.setW(-HL2BULLET(params->surfacePlane.w));
	g_pPhysSurfaceProps->GetPhysicsProperties(fluidObject->GetMaterialIndex(),
			&m_Density, nullptr, nullptr, nullptr);
}

void CPhysicsFluidController::SetGameData(void *pGameData) {
	m_GameData = pGameData;
}

void *CPhysicsFluidController::GetGameData() const {
	return m_GameData;
}

void CPhysicsFluidController::GetSurfacePlane(Vector *pNormal, float *pDist) const {
	if (pNormal != nullptr) {
		ConvertDirectionToHL(m_SurfacePlane, *pNormal);
	}
	if (pDist != nullptr) {
		*pDist = -BULLET2HL(m_SurfacePlane.getW());
	}
}

float CPhysicsFluidController::GetDensity() const {
	return m_Density;
}

int CPhysicsFluidController::GetContents() const {
	return m_Contents;
}
