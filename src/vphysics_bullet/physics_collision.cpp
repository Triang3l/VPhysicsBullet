// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_collision.h"
#include "physics_object.h"
#include "mathlib/polyhedron.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/* CPhysConvex can be one of the following:
 * - btConvexHullShape - user data is ConvexHullData_t.
 * - btBoxShape - always from the bbox cache, its user data is BBoxConvexCache_t.
 * User index (contents) of CPhysConvex is set with SetConvexGameData.
 *
 * CPhysCollide is either:
 * - btCompoundShape - user index is VCollide solid index, may be a cached bbox.
 * - btBvhTriangleMeshShape - virtual mesh (displacement) or polysoup.
 * - btSphereShape - for spherical physics objects.
 * User index of CPhysCollide is the solid index in VCollide.
 * User pointer is one of the objects using this collide for mass center updates.
 */

// TODO: Cleanup the bbox cache when shutting down.
// Not sure what to do with it in thread contexts though.

static CPhysicsCollision s_PhysCollision;
CPhysicsCollision *g_pPhysCollision = &s_PhysCollision;

/*******************
 * Convex polyhedra
 *******************/

btConvexHullShape *CPhysicsCollision::ConvexFromBulletPoints(
		const btVector3 *points, unsigned int pointCount) {
	if (pointCount == 0) {
		return nullptr;
	}
	ConvexHullData_t *hullData = new ConvexHullData_t;
	HullResult &hull = hullData->m_Hull;
	BEGIN_BULLET_ALLOCATION();
	HullError hullError = m_HullLibrary.CreateConvexHull(
			HullDesc(QF_TRIANGLES, pointCount, points), hull);
	END_BULLET_ALLOCATION();
	if (hullError != QE_OK || hull.mNumOutputVertices == 0) {
		AssertMsg(false, "Convex hull creation failed");
		delete hullData;
		return nullptr;
	}

	// Find the area-weighted average of triangle centroids for center of mass calculation.
	btScalar surfaceArea = 0.0f;
	btVector3 areaWeightedAverage(0.0f, 0.0f, 0.0f);
	const btVector3 *vertices = &hull.m_OutputVertices[0];
	unsigned int vertexCount = hull.mNumOutputVertices;
	const unsigned int *indices = &hull.m_Indices[0];
	unsigned int indexCount = hull.mNumIndices;
	for (unsigned int indexIndex = 0; indexIndex < indexCount; indexIndex += 3) {
		const btVector3 &v0 = vertices[indices[indexIndex]];
		const btVector3 &v1 = vertices[indices[indexIndex + 1]];
		const btVector3 &v2 = vertices[indices[indexIndex + 2]];
		btScalar triangleArea = (v1 - v0).cross(v2 - v0).length() * 0.5f;
		surfaceArea += triangleArea;
		areaWeightedAverage += (v0 + v1 + v2) * (1.0f / 3.0f) * triangleArea;
	}
	hullData->m_SurfaceArea = surfaceArea;
	hullData->m_AreaWeightedAverage = areaWeightedAverage;

	// Create the mesh.
	BEGIN_BULLET_ALLOCATION();
	btConvexHullShape *shape = new btConvexHullShape(&vertices[0][0], (int) vertexCount);
	END_BULLET_ALLOCATION();
	shape->setUserIndex(0);
	shape->setUserPointer(hullData);
	return shape;
}

CPhysConvex *CPhysicsCollision::ConvexFromVerts(Vector **pVerts, int vertCount) {
	BEGIN_BULLET_ALLOCATION();
	btAlignedObjectArray<btVector3> points;
	points.resizeNoInitialize(vertCount);
	END_BULLET_ALLOCATION();
	for (int vertIndex = 0; vertIndex < vertCount; ++vertIndex) {
		ConvertPositionToBullet(*pVerts[vertIndex], points[vertIndex]);
	}
	btConvexHullShape *shape = ConvexFromBulletPoints(&points[0], points.size());
	if (shape == nullptr) {
		return nullptr;
	}
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
	btConvexHullShape *shape = ConvexFromBulletPoints(&points[0], points.size());
	if (shape == nullptr) {
		return nullptr;
	}
	return reinterpret_cast<CPhysConvex *>(shape);
}

/*****************
 * Bounding boxes
 *****************/

CPhysicsCollision::BBoxCache_t *CPhysicsCollision::CreateBBox(const Vector &mins, const Vector &maxs) {
	Vector halfExtents = maxs - mins;
	VectorAbs(halfExtents, halfExtents);
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
	bbox->compoundShape = new btCompoundShape(false, 1);
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

bool CPhysicsCollision::IsCollideCachedBBox(const btCollisionShape *shape) const {
	if (shape->getShapeType() != COMPOUND_SHAPE_PROXYTYPE) {
		return false;
	}
	const btCompoundShape *compoundShape = static_cast<const btCompoundShape *>(shape);
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

/******************
 * Compound shapes
 ******************/

CPhysCollide *CPhysicsCollision::ConvertConvexToCollide(CPhysConvex **pConvex, int convexCount) {
	if (convexCount == 0 || pConvex == nullptr) {
		return nullptr;
	}

	// Calculate the center of mass.
	btVector3 areaWeightedAverage(0.0f, 0.0f, 0.0f);
	btScalar area = 0.0f;
	for (int convexIndex = 0; convexIndex < convexCount; ++convexIndex) {
		const btCollisionShape *shape =
				reinterpret_cast<const btCollisionShape *>(pConvex[convexIndex]);
		btVector3 convexAreaWeightedAverage;
		area += ConvexSurfaceAreaAndWeightedAverage(shape, convexAreaWeightedAverage);
		areaWeightedAverage += convexAreaWeightedAverage;
	}

	btVector3 massCenter;
	if (area > 1e-4) {
		massCenter = areaWeightedAverage / area;
	} else {
		// For very small collides, use the AABB center as the center of mass.
		btVector3 aabbMin(FLT_MAX, FLT_MAX, FLT_MAX), aabbMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		const btTransform &identity = btTransform::getIdentity();
		for (int convexIndex = 0; convexIndex < convexCount; ++convexIndex) {
			const btCollisionShape *convexShape =
					reinterpret_cast<const btCollisionShape *>(pConvex[convexIndex]);
			btVector3 convexAabbMin, convexAabbMax;
			convexShape->getAabb(identity, convexAabbMin, convexAabbMax);
			if (convexShape->getShapeType() == BOX_SHAPE_PROXYTYPE) {
				btVector3 origin;
				ConvertPositionToBullet(reinterpret_cast<const BBoxCache_t *>(
						convexShape->getUserPointer())->origin, origin);
				convexAabbMin += origin;
				convexAabbMax += origin;
			}
			aabbMin.setMin(convexAabbMin);
			aabbMax.setMax(convexAabbMax);
		}
		massCenter = (aabbMin + aabbMax) * 0.5f;
	}
	btTransform childTransform(btMatrix3x3::getIdentity(), -massCenter);

	BEGIN_BULLET_ALLOCATION();
	btCompoundShape *compoundShape = new btCompoundShape(convexCount > 1, convexCount);
	compoundShape->setUserIndex(0);
	for (int convexIndex = 0; convexIndex < convexCount; ++convexIndex) {
		// This takes the ownership of the children!
		compoundShape->addChildShape(childTransform,
				reinterpret_cast<btCollisionShape *>(pConvex[convexIndex]));
	}
	END_BULLET_ALLOCATION();
	return reinterpret_cast<CPhysCollide *>(compoundShape);
}

btVector3 CPhysicsCollision::CollideGetBulletMassCenter(const btCollisionShape *shape) {
	if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) {
		return -(static_cast<const btCompoundShape *>(shape)->getChildTransform(0).getOrigin());
	}
	return btVector3(0.0f, 0.0f, 0.0f);
}

void CPhysicsCollision::CollideGetMassCenter(CPhysCollide *pCollide, Vector *pOutMassCenter) {
	ConvertPositionToHL(CollideGetBulletMassCenter(pCollide), *pOutMassCenter);
}

void CPhysicsCollision::CollideSetMassCenter(CPhysCollide *pCollide, const Vector &massCenter) {
	btCollisionShape *shape = reinterpret_cast<btCollisionShape *>(pCollide);
	if (shape->getShapeType() != COMPOUND_SHAPE_PROXYTYPE || IsCollideCachedBBox(shape)) {
		AssertMsg(false, "Can only override mass center of compound shapes.");
		return;
	}

	btVector3 bulletOldMassCenter = CollideGetBulletMassCenter(shape), bulletMassCenter;
	ConvertPositionToBullet(massCenter, bulletMassCenter);
	btTransform transform(btMatrix3x3::getIdentity(), -bulletMassCenter);

	btCompoundShape *compoundShape = static_cast<btCompoundShape *>(shape);
	int childCount = compoundShape->getNumChildShapes();
	for (int childIndex = 0; childIndex < childCount; ++childIndex) {
		compoundShape->updateChildTransform(childIndex, transform, false);
	}
	compoundShape->recalculateLocalAabb();

	void *objectPointer = shape->getUserPointer();
	if (objectPointer != nullptr) {
		CPhysicsObject *startObject = static_cast<CPhysicsObject *>(
				reinterpret_cast<IPhysicsObject *>(objectPointer));
		CPhysicsObject *object = startObject;
		do {
			object->NotifyMassCenterChanged(bulletOldMassCenter);
			object = object->GetNextCollideObject();
		} while (object != startObject);
	}
}

/************
 * User data
 ************/

void CPhysicsCollision::SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData) {
	reinterpret_cast<btCollisionShape *>(pConvex)->setUserIndex((int) gameData);
}

int CPhysicsCollision::CollideIndex(const CPhysCollide *pCollide) {
	return reinterpret_cast<const btCollisionShape *>(pCollide)->getUserIndex();
}

void CPhysicsCollision::SetCollideIndex(CPhysCollide *pCollide, int index) {
	reinterpret_cast<btCollisionShape *>(pCollide)->setUserIndex(index);
}

/**********
 * Queries
 **********/

float CPhysicsCollision::ConvexVolume(CPhysConvex *pConvex) {
	btScalar bulletVolume = 0.0f;

	btCollisionShape *shape = reinterpret_cast<btCollisionShape *>(pConvex);
	int shapeType = shape->getShapeType();
	if (shapeType == CONVEX_HULL_SHAPE_PROXYTYPE) {
		// Tetrahedronalize the hull and compute its volume.
		const HullResult &hull = reinterpret_cast<const ConvexHullData_t *>(
				shape->getUserPointer())->m_Hull;
		const btVector3 *vertices = &hull.m_OutputVertices[0];
		const btVector3 &v0 = vertices[0];
		const unsigned int *indices = &hull.m_Indices[0];
		unsigned int indexCount = hull.mNumIndices;
		for (int indexIndex = 0; indexIndex < indexCount; indexCount += 3) {
			btVector3 a = vertices[indices[indexIndex]] - v0;
			btVector3 b = vertices[indices[indexIndex + 1]] - v0;
			btVector3 c = vertices[indices[indexIndex + 2]] - v0;
			bulletVolume += btFabs(a.dot(b.cross(c)));
		}
		bulletVolume *= 1.0f / 6.0f;
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
		bulletArea = reinterpret_cast<const ConvexHullData_t *>(
				shape->getUserPointer())->m_SurfaceArea;
	} else if (shapeType == BOX_SHAPE_PROXYTYPE) {
		const btVector3 &halfExtents =
				static_cast<const btBoxShape *>(shape)->getHalfExtentsWithoutMargin();
		bulletArea = 8.0f * halfExtents.getX() * halfExtents.getY() +
				4.0 * halfExtents.getZ() * (halfExtents.getX() + halfExtents.getY());
	}

	return (float) bulletArea * (BULLET2HL_FACTOR * BULLET2HL_FACTOR);
}

btScalar CPhysicsCollision::ConvexSurfaceAreaAndWeightedAverage(
		const btCollisionShape *shape, btVector3 &areaWeightedAverage) {
	int shapeType = shape->getShapeType();
	void *userPointer = shape->getUserPointer();
	if (shapeType == CONVEX_HULL_SHAPE_PROXYTYPE) {
		const ConvexHullData_t &hullData =
				*reinterpret_cast<const ConvexHullData_t *>(userPointer);
		areaWeightedAverage = hullData.m_AreaWeightedAverage;
		return hullData.m_SurfaceArea;
	}
	if (shapeType == BOX_SHAPE_PROXYTYPE) {
		const BBoxCache_t &bboxCache =
				*reinterpret_cast<const BBoxCache_t *>(userPointer);
		const btVector3 &halfExtents =
				static_cast<const btBoxShape *>(shape)->getHalfExtentsWithoutMargin();
		btScalar area = 8.0f * halfExtents.getX() * halfExtents.getY() +
				4.0 * halfExtents.getZ() * (halfExtents.getX() + halfExtents.getY());
		btVector3 origin;
		ConvertPositionToBullet(bboxCache.origin, origin);
		areaWeightedAverage = origin * area;
		return area;
	}
	areaWeightedAverage.setZero();
	return 0.0f;
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
	if (shapeType == CONVEX_HULL_SHAPE_PROXYTYPE) {
		delete reinterpret_cast<ConvexHullData_t *>(userPointer);
	}

	delete shape;
}
