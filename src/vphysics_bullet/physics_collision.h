// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_COLLISION_H
#define PHYSICS_COLLISION_H

#include "physics_internal.h"
#include "tier1/utlvector.h"

class CPhysicsCollision : public IPhysicsCollision {
public:

	// IPhysicsCollision methods.

	virtual CPhysConvex *ConvexFromVerts(Vector **pVerts, int vertCount);
	virtual void SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData);
	virtual void ConvexFree(CPhysConvex *pConvex);
	virtual CPhysConvex *BBoxToConvex(const Vector &mins, const Vector &maxs);
	virtual CPhysConvex *ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron);
	virtual int CollideIndex(const CPhysCollide *pCollide);
	virtual CPhysCollide *BBoxToCollide(const Vector &mins, const Vector &maxs);

	// Internal methods.

	void SetCollideIndex(CPhysCollide *pCollide, int index);

private:
	// BBoxes need to be offset when added to compound collides.
	// User data of bboxes points to this.
	struct BBoxCache_t {
		Vector halfExtents;
		Vector origin;
		btBoxShape *boxShape; // Returned from BBoxToConvex - zero origin.
		btCompoundShape *compoundShape; // Returned from BBoxToCollide - correct origin.
	};

	CUtlVector<BBoxCache_t> m_BBoxCache;

	BBoxCache_t *CreateBBox(const Vector &mins, const Vector &maxs);
	bool IsCollideCachedBBox(const CPhysCollide *pCollide) const;
};

#endif
