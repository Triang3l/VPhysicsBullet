// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_collision.h"
#include "physics_object.h"
#include "mathlib/polyhedron.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// TODO: Cleanup the bbox cache when shutting down.
// Not sure what to do with it in thread contexts though.

static CPhysicsCollision s_PhysCollision;
CPhysicsCollision *g_pPhysCollision = &s_PhysCollision;

/***********
 * Convexes
 ***********/

float CPhysicsCollision::ConvexVolume(CPhysConvex *pConvex) {
	return (float) pConvex->GetVolume() * (BULLET2HL_FACTOR * BULLET2HL_FACTOR * BULLET2HL_FACTOR);
}

float CPhysicsCollision::ConvexSurfaceArea(CPhysConvex *pConvex) {
	return (float) pConvex->GetSurfaceArea() * (BULLET2HL_FACTOR * BULLET2HL_FACTOR);
}

void CPhysicsCollision::SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData) {
	pConvex->GetShape()->setUserIndex((int) gameData);
}

void CPhysicsCollision::ConvexFree(CPhysConvex *pConvex) {
	if (pConvex->GetOwner() == CPhysConvex::OWNER_GAME) {
		delete pConvex;
	}
}

/***************
 * Convex hulls
 ***************/

CPhysConvex_Hull::CPhysConvex_Hull(HullResult *hull) :
		m_Shape(&hull->m_OutputVertices[0][0], hull->mNumOutputVertices),
		m_Hull(hull) {
	Initialize();

	// Find the area-weighted average of triangle centroids for center of mass calculation.
	btScalar surfaceArea = 0.0f;
	btVector3 areaWeightedAverage(0.0f, 0.0f, 0.0f);
	const btVector3 *vertices = &m_Hull->m_OutputVertices[0];
	unsigned int vertexCount = m_Hull->mNumOutputVertices;
	const unsigned int *indices = &m_Hull->m_Indices[0];
	unsigned int indexCount = m_Hull->mNumIndices;
	for (unsigned int indexIndex = 0; indexIndex < indexCount; indexIndex += 3) {
		const btVector3 &v0 = vertices[indices[indexIndex]];
		const btVector3 &v1 = vertices[indices[indexIndex + 1]];
		const btVector3 &v2 = vertices[indices[indexIndex + 2]];
		btScalar triangleArea = (v1 - v0).cross(v2 - v0).length() * 0.5f;
		surfaceArea += triangleArea;
		areaWeightedAverage += (v0 + v1 + v2) * (1.0f / 3.0f) * triangleArea;
	}
	m_SurfaceArea = surfaceArea;
	if (surfaceArea > 1e-4) {
		m_MassCenter = areaWeightedAverage / m_SurfaceArea;
	} else {
		// Use the geometric average as the mass center.
		btVector3 aabbMins(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
		btVector3 aabbMaxs(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
		for (unsigned int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
			const btVector3 &vertex = vertices[vertexIndex];
			aabbMins.setMin(vertex);
			aabbMaxs.setMax(vertex);
		}
		m_MassCenter = (aabbMins + aabbMaxs) * 0.5f;
	}
}

CPhysConvex_Hull *CPhysConvex_Hull::CreateFromBulletPoints(
		HullLibrary &hullLibrary, const btVector3 *points, int pointCount) {
	if (pointCount == 0) {
		return nullptr;
	}
	HullResult *hull = new HullResult;
	HullError hullError = hullLibrary.CreateConvexHull(
			HullDesc(QF_TRIANGLES, pointCount, points), *hull);
	if (hullError != QE_OK || hull->mNumOutputVertices == 0) {
		AssertMsg(false, "Convex hull creation failed");
		delete hull;
		return nullptr;
	}
	return new CPhysConvex_Hull(hull);
}

btScalar CPhysConvex_Hull::GetVolume() const {
	// Tetrahedronalize the hull and compute its volume.
	btScalar volume = 0.0f;
	const btVector3 *vertices = &m_Hull->m_OutputVertices[0];
	const btVector3 &v0 = vertices[0];
	const unsigned int *indices = &m_Hull->m_Indices[0];
	unsigned int indexCount = m_Hull->mNumIndices;
	for (int indexIndex = 0; indexIndex < indexCount; indexCount += 3) {
		btVector3 a = vertices[indices[indexIndex]] - v0;
		btVector3 b = vertices[indices[indexIndex + 1]] - v0;
		btVector3 c = vertices[indices[indexIndex + 2]] - v0;
		volume += btFabs(a.dot(b.cross(c)));
	}
	volume *= 1.0f / 6.0f;
}

CPhysConvex *CPhysicsCollision::ConvexFromVerts(Vector **pVerts, int vertCount) {
	BEGIN_BULLET_ALLOCATION();
	btAlignedObjectArray<btVector3> points;
	points.resizeNoInitialize(vertCount);
	END_BULLET_ALLOCATION();
	for (int vertIndex = 0; vertIndex < vertCount; ++vertIndex) {
		ConvertPositionToBullet(*pVerts[vertIndex], points[vertIndex]);
	}
	BEGIN_BULLET_ALLOCATION();
	CPhysConvex_Hull *convex = CPhysConvex_Hull::CreateFromBulletPoints(
			m_HullLibrary, &points[0], points.size());
	END_BULLET_ALLOCATION();
	return convex;
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
	BEGIN_BULLET_ALLOCATION();
	CPhysConvex_Hull *convex = CPhysConvex_Hull::CreateFromBulletPoints(
			m_HullLibrary, &points[0], points.size());
	END_BULLET_ALLOCATION();
	return convex;
}

/***********************************************
 * Bounding boxes (both convex and collideable)
 ***********************************************/

btScalar CPhysConvex_Box::GetVolume() const {
	const btVector3 &halfExtents = m_Shape.getHalfExtentsWithoutMargin();
	return 8.0f * halfExtents.getX() * halfExtents.getY() * halfExtents.getZ();
}

btScalar CPhysConvex_Box::GetSurfaceArea() const {
	const btVector3 &halfExtents = m_Shape.getHalfExtentsWithoutMargin();
	return 8.0f * halfExtents.getX() * halfExtents.getY() +
			4.0 * halfExtents.getZ() * (halfExtents.getX() + halfExtents.getY());
}

CPhysCollide_Compound *CPhysicsCollision::CreateBBox(const Vector &mins, const Vector &maxs) {
	if (mins == maxs) {
		return nullptr;
	}

	btVector3 bulletMins, bulletMaxs;
	ConvertPositionToBullet(mins, bulletMins);
	ConvertPositionToBullet(maxs, bulletMaxs);
	btVector3 halfExtents = (bulletMaxs - bulletMins).absolute() * 0.5f;
	btVector3 origin = (bulletMins + bulletMaxs) * 0.5f;

	const btScalar threshold = HL2BULLET(0.1f);

	for (int bboxIndex = m_BBoxCache.Count() - 1; bboxIndex >= 0; --bboxIndex) {
		CPhysCollide_Compound *cacheCompound = m_BBoxCache[bboxIndex];
		const CPhysConvex_Box *cacheBox = static_cast<const CPhysConvex_Box *>(
				reinterpret_cast<const CPhysConvex *>(
						cacheCompound->GetCompoundShape()->getChildShape(0)->getUserPointer()));
		const btVector3 &cacheHalfExtents = cacheBox->GetBoxShape()->getHalfExtentsWithoutMargin();
		const btVector3 &cacheOrigin = cacheBox->GetOriginInCompound();
		for (int component = 0; component < 3; ++component) {
			if (btFabs(cacheHalfExtents[component] - halfExtents[component]) > threshold ||
					btFabs(cacheOrigin[component] - origin[component]) > threshold) {
				cacheCompound = nullptr;
				break;
			}
			if (cacheCompound != nullptr) {
				return cacheCompound;
			}
		}
	}

	BEGIN_BULLET_ALLOCATION();
	CPhysConvex *box = new CPhysConvex_Box(halfExtents, origin);
	box->SetOwner(CPhysConvex::OWNER_INTERNAL);
	CPhysCollide_Compound *compound = new CPhysCollide_Compound(&box, 1);
	compound->SetOwner(CPhysCollide::OWNER_INTERNAL);
	END_BULLET_ALLOCATION();
	m_BBoxCache.AddToTail(compound);
	return compound;
}

CPhysConvex *CPhysicsCollision::BBoxToConvex(const Vector &mins, const Vector &maxs) {
	CPhysCollide_Compound *compound = CreateBBox(mins, maxs);
	if (compound == nullptr) {
		return nullptr;
	}
	return reinterpret_cast<CPhysConvex *>(
			compound->GetCompoundShape()->getChildShape(0)->getUserPointer());
}

CPhysCollide *CPhysicsCollision::BBoxToCollide(const Vector &mins, const Vector &maxs) {
	return CreateBBox(mins, maxs);
}

/***************
 * Collideables
 ***************/

void CPhysCollide::RemoveObjectReference(IPhysicsObject *object) {
	if (m_ObjectReferenceList == object) {
		m_ObjectReferenceList = static_cast<CPhysicsObject *>(object)->GetNextCollideObject();
		if (m_ObjectReferenceList == object) {
			m_ObjectReferenceList = nullptr;
		}
	}
}

float CPhysicsCollision::CollideVolume(CPhysCollide *pCollide) {
	return (float) pCollide->GetVolume() * (BULLET2HL_FACTOR * BULLET2HL_FACTOR * BULLET2HL_FACTOR);
}

float CPhysicsCollision::CollideSurfaceArea(CPhysCollide *pCollide) {
	return (float) pCollide->GetSurfaceArea() * (BULLET2HL_FACTOR * BULLET2HL_FACTOR);
}

void CPhysicsCollision::CollideGetMassCenter(CPhysCollide *pCollide, Vector *pOutMassCenter) {
	ConvertPositionToHL(pCollide->GetMassCenter(), *pOutMassCenter);
}

void CPhysicsCollision::CollideSetMassCenter(CPhysCollide *pCollide, const Vector &massCenter) {
	btVector3 bulletMassCenter;
	ConvertPositionToBullet(massCenter, bulletMassCenter);
	pCollide->SetMassCenter(bulletMassCenter);
}

void CPhysCollide::NotifyObjectsOfMassCenterChange(const btVector3 &oldMassCenter) {
	IPhysicsObject *firstObject = GetObjectReferenceList();
	if (firstObject != nullptr) {
		CPhysicsObject *object = static_cast<CPhysicsObject *>(firstObject);
		do {
			object->NotifyMassCenterChanged(oldMassCenter);
			object = object->GetNextCollideObject();
		} while (object != firstObject);
	}
}

int CPhysicsCollision::CollideIndex(const CPhysCollide *pCollide) {
	return pCollide->GetShape()->getUserIndex();
}

void CPhysicsCollision::SetCollideIndex(CPhysCollide *pCollide, int index) {
	pCollide->GetShape()->setUserIndex(index);
}

/******************
 * Compound shapes
 ******************/

CPhysCollide_Compound::CPhysCollide_Compound(CPhysConvex **pConvex, int convexCount) :
		m_Shape(convexCount > 1, convexCount) {
	Assert(convexCount > 0);

	Initialize();

	// Calculate the center of mass.
	if (convexCount > 1) {
		btVector3 areaWeightedAverage(0.0f, 0.0f, 0.0f);
		btScalar area = 0.0f;
		for (int convexIndex = 0; convexIndex < convexCount; ++convexIndex) {
			const CPhysConvex *convex = pConvex[convexIndex];
			btScalar convexArea = convex->GetSurfaceArea();
			area += convexArea;
			areaWeightedAverage += (convex->GetOriginInCompound() + convex->GetMassCenter()) * convexArea;
		}
		// TODO: Do something if area is near 0 (center of AABB?) if needed.
		m_MassCenter = areaWeightedAverage / area;
	} else {
		m_MassCenter = pConvex[0]->GetMassCenter() + pConvex[0]->GetOriginInCompound();
	}

	btTransform transform(btMatrix3x3::getIdentity());
	for (int convexIndex = 0; convexIndex < convexCount; ++convexIndex) {
		CPhysConvex *convex = pConvex[convexIndex];
		if (convex->GetOwner() == CPhysConvex::OWNER_GAME) {
			convex->SetOwner(CPhysConvex::OWNER_COMPOUND);
		}
		transform.setOrigin(convex->GetOriginInCompound() - m_MassCenter);
		m_Shape.addChildShape(transform, convex->GetShape());
	}
}

CPhysCollide *CPhysicsCollision::ConvertConvexToCollide(CPhysConvex **pConvex, int convexCount) {
	if (convexCount == 0 || pConvex == nullptr) {
		return nullptr;
	}
	BEGIN_BULLET_ALLOCATION();
	CPhysCollide_Compound *compound = new CPhysCollide_Compound(pConvex, convexCount);
	END_BULLET_ALLOCATION();
	return compound;
}

btScalar CPhysCollide_Compound::GetVolume() const {
	btScalar volume = 0.0f;
	int childCount = m_Shape.getNumChildShapes();
	for (int childIndex = 0; childIndex < childCount; ++childIndex) {
		volume += reinterpret_cast<const CPhysConvex *>(
				m_Shape.getChildShape(childIndex)->getUserPointer())->GetVolume();
	}
	return volume;
}

btScalar CPhysCollide_Compound::GetSurfaceArea() const {
	btScalar area = 0.0f;
	int childCount = m_Shape.getNumChildShapes();
	for (int childIndex = 0; childIndex < childCount; ++childIndex) {
		area += reinterpret_cast<const CPhysConvex *>(
				m_Shape.getChildShape(childIndex)->getUserPointer())->GetSurfaceArea();
	}
	return area;
}

void CPhysCollide_Compound::SetMassCenter(const btVector3 &massCenter) {
	btVector3 oldMassCenter = m_MassCenter;
	m_MassCenter = massCenter;
	int childCount = m_Shape.getNumChildShapes();
	for (int childIndex = 0; childIndex < childCount; ++childIndex) {
		btTransform childTransform = m_Shape.getChildTransform(childIndex);
		childTransform.setOrigin(reinterpret_cast<const CPhysConvex *>(
				m_Shape.getChildShape(childIndex)->getUserPointer())->GetOriginInCompound() - massCenter);
		m_Shape.updateChildTransform(childIndex, childTransform, false);
	}
	m_Shape.recalculateLocalAabb();
	NotifyObjectsOfMassCenterChange(oldMassCenter);
}

/**********
 * Spheres
 **********/

btScalar CPhysCollide_Sphere::GetVolume() const {
	btScalar radius = m_Shape.getRadius();
	return ((4.0f / 3.0f) * SIMD_PI) * radius * radius * radius;
}

btScalar CPhysCollide_Sphere::GetSurfaceArea() const {
	btScalar radius = m_Shape.getRadius();
	return (4.0f * SIMD_PI) * radius * radius;
}
