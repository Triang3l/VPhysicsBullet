// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_collision.h"
#include "mathlib/polyhedron.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// CPhysConvex user data:
// * btConvexHullShape: likely hull created for ICollisionQuery.
// * btBoxShape: BBoxConvexCache_t pointer.
// CPhysConvex user index is set externally and can be queried.
// CPhysCollide user index is the VCollide solid index.

// TODO: Cleanup the bbox cache when shutting down.
// Not sure what to do with it in thread contexts though.

/*******************
 * Convex polyhedra
 *******************/

CPhysConvex *CPhysicsCollision::ConvexFromVerts(Vector **pVerts, int vertCount) {
	BEGIN_BULLET_ALLOCATION();
	btAlignedObjectArray<btVector3> points;
	points.resizeNoInitialize(vertCount);
	for (int vertIndex = 0; vertIndex < vertCount; ++vertIndex) {
		ConvertPositionToBullet(*pVerts[vertIndex], points[vertIndex]);
	}
	btConvexHullShape *shape = new btConvexHullShape(&points[0][0], vertCount);
	END_BULLET_ALLOCATION();
	return reinterpret_cast<CPhysConvex *>(shape);
}

CPhysConvex *CPhysicsCollision::ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron) {
	const Vector *verts = ConvexPolyhedron.pVertices;
	int vertCount = ConvexPolyhedron.iVertexCount;
	BEGIN_BULLET_ALLOCATION();
	btAlignedObjectArray<btVector3> points;
	points.resizeNoInitialize(vertCount);
	for (int vertIndex = 0; vertIndex < vertCount; ++vertIndex) {
		ConvertPositionToBullet(verts[vertIndex], points[vertIndex]);
	}
	btConvexHullShape *shape = new btConvexHullShape(&points[0][0], vertCount);
	END_BULLET_ALLOCATION();
	return reinterpret_cast<CPhysConvex *>(shape);
}

/*****************
 * Bounding boxes
 *****************/

CPhysicsCollision::BBoxCache_t *CPhysicsCollision::CreateBBox(const Vector &mins, const Vector &maxs) {
	Vector halfExtents = maxs - mins;
	halfExtents.x = fabsf(halfExtents.x);
	halfExtents.y = fabsf(halfExtents.y);
	halfExtents.z = fabsf(halfExtents.z);
	Vector origin = (mins + maxs) * 0.5f;

	BBoxCache_t *bbox;

	int bboxIndex, bboxCount = m_BBoxCache.Count();
	for (bboxIndex = 0; bboxIndex < bboxCount; ++bboxIndex) {
		bbox = &m_BBoxCache[bboxIndex];
		for (int component = 0; component < 3; ++component) {
			if (fabsf(bbox->halfExtents[component] - halfExtents[component]) > 0.1f ||
					fabsf(bbox->origin[component] - origin[component]) > 0.1f) {
				bbox = nullptr;
				break;
			}
		}
		if (bbox != nullptr) {
			return bbox;
		}
	}

	btVector3 bulletHalfExtents, bulletOrigin;
	ConvertPositionToBullet(halfExtents, bulletHalfExtents);
	bulletHalfExtents = bulletHalfExtents.absolute(); // Because conversion changes signs.
	ConvertPositionToBullet(origin, bulletOrigin);

	bbox = &m_BBoxCache[m_BBoxCache.AddToTail()];
	bbox->halfExtents = halfExtents;
	bbox->origin = origin;
	BEGIN_BULLET_ALLOCATION();
	bbox->boxShape = new btBoxShape(bulletHalfExtents);
	bbox->boxShape->setUserPointer(bbox);
	bbox->compoundShape = new btCompoundShape(false);
	bbox->compoundShape->addChildShape(
			btTransform(btMatrix3x3::getIdentity(), bulletOrigin), bbox->boxShape);
	END_BULLET_ALLOCATION();
	return bbox;
}

CPhysConvex *CPhysicsCollision::BBoxToConvex(const Vector &mins, const Vector &maxs) {
	const BBoxCache_t *bbox = CreateBBox(mins, maxs);
	if (bbox == nullptr) {
		return nullptr;
	}
	return reinterpret_cast<CPhysConvex *>(bbox->boxShape);
}

CPhysCollide *CPhysicsCollision::BBoxToCollide(const Vector &mins, const Vector &maxs) {
	const BBoxCache_t *bbox = CreateBBox(mins, maxs);
	if (bbox == nullptr) {
		return nullptr;
	}
	return reinterpret_cast<CPhysCollide *>(bbox->compoundShape);
}

bool CPhysicsCollision::IsCollideCachedBBox(const CPhysCollide *pCollide) const {
	const btCompoundShape *compoundShape = reinterpret_cast<const btCompoundShape *>(pCollide);
	if (compoundShape->getNumChildShapes() != 1) {
		return false;
	}
	const btCollisionShape *childShape = compoundShape->getChildShape(0);
	if (childShape->getShapeType() != BOX_SHAPE_PROXYTYPE) {
		return false;
	}
	void *userData = childShape->getUserPointer();
	if (userData == nullptr) {
		return false;
	}
	return reinterpret_cast<const BBoxCache_t *>(userData)->compoundShape == compoundShape;
}

/************
 * User data
 ************/

void CPhysicsCollision::SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData) {
	reinterpret_cast<btCollisionShape *>(pConvex)->setUserIndex((int) gameData);
}

int CPhysicsCollision::CollideIndex(const CPhysCollide *pCollide) {
	Assert(!IsCollideCachedBBox(pCollide));
	return reinterpret_cast<const btCollisionShape *>(pCollide)->getUserIndex();
}

void CPhysicsCollision::SetCollideIndex(CPhysCollide *pCollide, int index) {
	reinterpret_cast<btCollisionShape *>(pCollide)->setUserIndex(index);
}

/**************
 * Destruction
 **************/

void CPhysicsCollision::ConvexFree(CPhysConvex *pConvex) {
	btCollisionShape *shape = reinterpret_cast<btCollisionShape *>(pConvex);
	if (shape->getShapeType() == BOX_SHAPE_PROXYTYPE && shape->getUserPointer() != nullptr) {
		// All bboxes are cached, but may be shutting down, in this case it's nullptr.
		return;
	}
	delete shape;
}
