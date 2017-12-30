// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_collision.h"
#include "mathlib/polyhedron.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/* CPhysConvex can be one of the following:
 * - btConvexHullShape - its user data is its hull for ICollisionQuery if ever requested.
 * - btBoxShape - always from the bbox cache, its user data is BBoxConvexCache_t.
 * User index (contents) of CPhysConvex is set with SetConvexGameData.
 *
 * CPhysCollide is either:
 * - btCompoundShape - user index is VCollide solid index.
 * - btBvhTriangleMeshShape - virtual mesh (displacement) or polysoup.
 * User index of CPhysCollide is the solid index in VCollide.
 */

// TODO: Cleanup the bbox cache when shutting down.
// Not sure what to do with it in thread contexts though.

#define CPHYSCONVEX_HULL_FAILED ((void *) -1)

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
	shape->setUserIndex(0);
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
	shape->setUserIndex(0);
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
	bbox->boxShape->setUserIndex(0);
	bbox->boxShape->setUserPointer(bbox);
	bbox->compoundShape = new btCompoundShape(false);
	bbox->compoundShape->setUserIndex(0);
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
	void *userPointer = childShape->getUserPointer();
	if (userPointer == nullptr) {
		return false;
	}
	return reinterpret_cast<const BBoxCache_t *>(userPointer)->compoundShape == compoundShape;
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

/***************
 * Tool queries
 ***************/

HullResult *CPhysicsCollision::GetConvexHull(btConvexHullShape *shape) {
	void *userPointer = shape->getUserPointer();
	if (userPointer == CPHYSCONVEX_HULL_FAILED) {
		return nullptr;
	}
	if (userPointer != nullptr) {
		return reinterpret_cast<HullResult *>(userPointer);
	}

	BEGIN_BULLET_ALLOCATION();
	HullResult *hullResult = new HullResult;
	HullError hullError = m_HullLibrary.CreateConvexHull(
			HullDesc(QF_TRIANGLES, shape->getNumPoints(), shape->getPoints()), *hullResult);
	END_BULLET_ALLOCATION();

	if (hullError != QE_OK) {
		delete hullResult;
		shape->setUserPointer(CPHYSCONVEX_HULL_FAILED);
		return nullptr;
	}

	shape->setUserPointer(hullResult);
	return hullResult;
}

float CPhysicsCollision::ConvexVolume(CPhysConvex *pConvex) {
	btScalar bulletVolume = 0.0f;

	btCollisionShape *shape = reinterpret_cast<btCollisionShape *>(pConvex);
	int shapeType = shape->getShapeType();
	if (shapeType == CONVEX_HULL_SHAPE_PROXYTYPE) {
		const HullResult *hull = GetConvexHull(static_cast<btConvexHullShape *>(shape));
		if (hull != nullptr && hull->mNumOutputVertices > 0) {
			// Tetrahedronalize this hull and compute its volume.
			const btVector3 *vertices = &hull->m_OutputVertices[0];
			const btVector3 &v0 = vertices[0];
			const unsigned int *indices = &hull->m_Indices[0];
			unsigned int indexCount = hull->mNumIndices;
			for (int indexIndex = 0; indexIndex < indexCount; indexCount += 3) {
				btVector3 a = vertices[indices[indexIndex]] - v0;
				btVector3 b = vertices[indices[indexIndex + 1]] - v0;
				btVector3 c = vertices[indices[indexIndex + 2]] - v0;
				bulletVolume += btFabs(a.dot(b.cross(c)));
			}
			bulletVolume *= 1.0f / 6.0f;
		}
	} else if (shapeType == BOX_SHAPE_PROXYTYPE) {
		const btVector3 &halfExtents =
				static_cast<const btBoxShape *>(shape)->getHalfExtentsWithoutMargin();
		bulletVolume = 8.0f * halfExtents.getX() * halfExtents.getY() * halfExtents.getZ();
	}

	return (float) bulletVolume * (BULLET2HL_FACTOR * BULLET2HL_FACTOR * BULLET2HL_FACTOR);
}

float CPhysicsCollision::ConvexSurfaceArea(CPhysConvex *pConvex) {
	btScalar bulletArea = 0.0f;

	btCollisionShape *shape = reinterpret_cast<btCollisionShape *>(pConvex);
	int shapeType = shape->getShapeType();
	if (shapeType == CONVEX_HULL_SHAPE_PROXYTYPE) {
		const HullResult *hull = GetConvexHull(static_cast<btConvexHullShape *>(shape));
		if (hull != nullptr && hull->mNumOutputVertices > 0) {
			const btVector3 *vertices = &hull->m_OutputVertices[0];
			const unsigned int *indices = &hull->m_Indices[0];
			unsigned int indexCount = hull->mNumIndices;
			for (int indexIndex = 0; indexIndex < indexCount; indexCount += 3) {
				btVector3 v0 = vertices[indices[indexIndex]];
				btVector3 e0 = vertices[indices[indexIndex + 1]] - v0;
				btVector3 e1 = vertices[indices[indexIndex + 2]] - v0;
				bulletArea += e0.cross(e1).length();
			}
			bulletArea *= 0.5f;
		}
	} else if (shapeType == BOX_SHAPE_PROXYTYPE) {
		const btVector3 &halfExtents =
				static_cast<const btBoxShape *>(shape)->getHalfExtentsWithoutMargin();
		bulletArea = 8.0f * halfExtents.getX() * halfExtents.getY() +
				4.0 * halfExtents.getZ() * (halfExtents.getX() + halfExtents.getY());
	}

	return (float) bulletArea * (BULLET2HL_FACTOR * BULLET2HL_FACTOR);
}

/**************
 * Destruction
 **************/

void CPhysicsCollision::ConvexFree(CPhysConvex *pConvex) {
	btCollisionShape *shape = reinterpret_cast<btCollisionShape *>(pConvex);
	int shapeType = shape->getShapeType();
	void *userPointer = shape->getUserPointer();

	// All bboxes are cached and must not be freed except for when shutting down.
	// In this case, the cache pointer is null.
	if (shapeType == BOX_SHAPE_PROXYTYPE && userPointer != nullptr) {
		return;
	}

	// Remove the convex hull.
	if (shapeType == CONVEX_HULL_SHAPE_PROXYTYPE &&
			userPointer != nullptr && userPointer != CPHYSCONVEX_HULL_FAILED) {
		delete reinterpret_cast<HullResult *>(userPointer);
	}

	delete shape;
}
