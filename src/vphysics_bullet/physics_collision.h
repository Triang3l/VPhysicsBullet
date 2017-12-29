// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_COLLISION_H
#define PHYSICS_COLLISION_H

#include "physics_internal.h"

class CPhysicsCollision : public IPhysicsCollision {
public:
	virtual CPhysConvex *ConvexFromVerts(Vector **pVerts, int vertCount);

	virtual void SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData);

	virtual void ConvexFree(CPhysConvex *pConvex);

	virtual CPhysConvex *ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron);
};

#endif
