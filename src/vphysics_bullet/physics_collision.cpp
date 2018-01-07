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

/****************************************
 * Utility for convexes and collideables
 ****************************************/

btVector3 CPhysicsCollision::BoxInertia(const btVector3 &extents) {
	btVector3 l2 = extents * extents;
	return (1.0f / 12.0f) * btVector3(l2.y() + l2.z(), l2.x() + l2.z(), l2.x() + l2.y());
}

btVector3 CPhysicsCollision::OffsetInertia(
		const btVector3 &inertia, const btVector3 &origin, bool absolute) {
	btScalar o2 = origin.length2();
	btVector3 newInertia = inertia + btVector3(o2, o2, o2) - (origin * origin);
	return absolute ? newInertia.absolute() : newInertia;
}

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

	// Based on btConvexTriangleMeshShape::calculatePrincipalAxisTransform, but without rotation.

	const btVector3 *vertices = &hull->m_OutputVertices[0];
	unsigned int vertexCount = hull->mNumOutputVertices;
	const unsigned int *indices = &hull->m_Indices[0];
	unsigned int indexCount = hull->mNumIndices;

	// Volume and center of mass.
	const btVector3 &ref = vertices[indices[0]];
	btVector3 massCenterSum(0.0f, 0.0f, 0.0f);
	btScalar sixVolume = 0.0f;
	for (unsigned int indexIndex = 3; indexIndex < indexCount; indexIndex += 3) {
		const btVector3 &v0 = vertices[indices[indexIndex]];
		const btVector3 &v1 = vertices[indices[indexIndex + 1]];
		const btVector3 &v2 = vertices[indices[indexIndex + 2]];
		btScalar tetrahedronSixVolume = btFabs((v0 - ref).triple(v1 - ref, v2 - ref));
		massCenterSum += (0.25f * tetrahedronSixVolume) * (v0 + v1 + v2 + ref);
		sixVolume += tetrahedronSixVolume;
	}
	m_Volume = (1.0f / 6.0f) * sixVolume;
	if (m_Volume > 0.0f) {
		m_MassCenter = massCenterSum / sixVolume;
		m_Inertia.setZero();
		for (unsigned int indexIndex = 0; indexIndex < indexCount; indexIndex += 3) {
			btVector3 a = vertices[indices[indexIndex]] - m_MassCenter;
			btVector3 b = vertices[indices[indexIndex + 1]] - m_MassCenter;
			btVector3 c = vertices[indices[indexIndex + 2]] - m_MassCenter;
			btVector3 i = btFabs(a.triple(b, c)) * (0.1f / 6.0f) *
					(a * a + b * b + c * c + a * b + a * c + b * c);
			m_Inertia[0] += i[1] + i[2];
			m_Inertia[1] += i[2] + i[0];
			m_Inertia[2] += i[0] + i[1];
		}
		m_Inertia = (m_Inertia / m_Volume).absolute();
	} else {
		// Use a box approximation.
		btVector3 aabbMin(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
		btVector3 aabbMax(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
		m_Shape.getAabb(btTransform(btMatrix3x3::getIdentity()), aabbMin, aabbMax);
		m_MassCenter = (aabbMin + aabbMax) * 0.5f;
		m_Inertia = CPhysicsCollision::OffsetInertia(
				CPhysicsCollision::BoxInertia(aabbMax - aabbMin),
				(aabbMin + aabbMax) * 0.5f);
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
	if (hullError != QE_OK || hull->mNumIndices < 3) {
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
	for (int indexIndex = 3; indexIndex < indexCount; indexCount += 3) {
		btVector3 a = vertices[indices[indexIndex]] - v0;
		btVector3 b = vertices[indices[indexIndex + 1]] - v0;
		btVector3 c = vertices[indices[indexIndex + 2]] - v0;
		volume += btFabs(a.dot(b.cross(c)));
	}
	volume *= 1.0f / 6.0f;
}

btScalar CPhysConvex_Hull::GetSurfaceArea() const {
	const btVector3 *vertices = &m_Hull->m_OutputVertices[0];
	unsigned int vertexCount = m_Hull->mNumOutputVertices;
	const unsigned int *indices = &m_Hull->m_Indices[0];
	unsigned int indexCount = m_Hull->mNumIndices;
	btScalar area = 0.0f;
	for (unsigned int indexIndex = 0; indexIndex < indexCount; indexIndex += 3) {
		const btVector3 &v0 = vertices[indices[indexIndex]];
		const btVector3 &v1 = vertices[indices[indexIndex + 1]];
		const btVector3 &v2 = vertices[indices[indexIndex + 2]];
		area += (v1 - v0).cross(v2 - v0).length();
	}
	return 0.5f * area;
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

btVector3 CPhysConvex_Box::GetInertia() const {
	return CPhysicsCollision::BoxInertia(2.0f * m_Shape.getHalfExtentsWithoutMargin());
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

	// Calculate volume and center of mass.
	m_Volume = 0.0f;
	m_MassCenter.setZero();
	for (int convexIndex = 0; convexIndex < convexCount; ++convexIndex) {
		const CPhysConvex *convex = pConvex[convexIndex];
		btScalar convexVolume = convex->GetVolume();
		m_Volume += convexVolume;
		m_MassCenter += (convex->GetOriginInCompound() + convex->GetMassCenter()) * convexVolume;
	}
	if (m_Volume > 0.0f) {
		m_MassCenter /= m_Volume;
	} else {
		btVector3 aabbMin(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
		btVector3 aabbMax(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
		btTransform convexAabbTransform(btMatrix3x3::getIdentity());
		for (int convexIndex = 0; convexIndex < convexCount; ++convexIndex) {
			const CPhysConvex *convex = pConvex[convexIndex];
			convexAabbTransform.setOrigin(convex->GetOriginInCompound());
			btVector3 convexAabbMin, convexAabbMax;
			convex->GetShape()->getAabb(convexAabbTransform, convexAabbMin, convexAabbMax);
			aabbMin.setMin(convexAabbMin);
			aabbMax.setMax(convexAabbMax);
		}
		m_MassCenter = (aabbMin + aabbMax) * 0.5f;
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

	CalculateInertia();
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
	CalculateInertia();
	NotifyObjectsOfMassCenterChange(oldMassCenter);
}

void CPhysCollide_Compound::CalculateInertia() {
	if (m_Volume >= 0) {
		m_Inertia.setZero();
		int childCount = m_Shape.getNumChildShapes();
		for (int childIndex = 0; childIndex < childCount; ++childIndex) {
			const CPhysConvex *convex = reinterpret_cast<const CPhysConvex *>(
					m_Shape.getChildShape(childIndex)->getUserPointer());
			const btVector3 &origin = m_Shape.getChildTransform(childIndex).getOrigin();
			m_Inertia += convex->GetVolume() * CPhysicsCollision::OffsetInertia(
					convex->GetInertia(), m_Shape.getChildTransform(childIndex).getOrigin(), false);
		}
		m_Inertia = (m_Inertia / m_Volume).absolute();
	} else {
		btVector3 aabbMin, aabbMax;
		m_Shape.getAabb(btTransform(btMatrix3x3::getIdentity()), aabbMin, aabbMax);
		m_Inertia = CPhysicsCollision::OffsetInertia(
				CPhysicsCollision::BoxInertia(aabbMax - aabbMin),
				(aabbMin + aabbMax) * 0.5f);
	}
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

btVector3 CPhysCollide_Sphere::GetInertia() const {
	btScalar elem = m_Shape.getRadius();
	elem *= elem * 0.4;
	return btVector3(elem, elem, elem);
}
