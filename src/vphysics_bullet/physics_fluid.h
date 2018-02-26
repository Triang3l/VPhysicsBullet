// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_FLUID_H
#define PHYSICS_FLUID_H

#include "physics_internal.h"

class CPhysicsFluidController : public IPhysicsFluidController {
public:
	CPhysicsFluidController(IPhysicsObject *fluidObject, const fluidparams_t *params);

	virtual void SetGameData(void *pGameData);
	virtual void *GetGameData() const;
	virtual void GetSurfacePlane(Vector *pNormal, float *pDist) const;
	virtual float GetDensity() const;
	/* DUMMY */ virtual void WakeAllSleepingObjects() {}
	virtual int GetContents() const;

private:
	btVector4 m_SurfacePlane;
	float m_Density;
	int m_Contents;
	void *m_GameData;
};

#endif
