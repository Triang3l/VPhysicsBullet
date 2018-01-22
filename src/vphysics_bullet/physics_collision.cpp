// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_collision.h"
#include "physics_object.h"
#include <LinearMath/btGeometryUtil.h>
#include "mathlib/polyhedron.h"
#include "tier0/dbg.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// TODO: Cleanup the bbox cache when shutting down.
// Not sure what to do with it in thread contexts though.

static CPhysicsCollision s_PhysCollision;
CPhysicsCollision *g_pPhysCollision = &s_PhysCollision;

/***************************
 * Serialization structures
 ***************************/

#define VCOLLIDE_VPHYSICS_ID MAKEID('V', 'P', 'H', 'Y')
#define VCOLLIDE_IVP_COMPACT_SURFACE_ID MAKEID('I', 'V', 'P', 'S')

#define VCOLLIDE_VERSION_IVP 0x100
#define VCOLLIDE_MODEL_TYPE_IVP_COMPACT_SURFACE 0

// Triang3l's Bullet interface.
// To completely prevent loading Bullet collideables in IVP and
// other VPhysics implementations, and also to version separately.
#define VCOLLIDE_VERSION_BULLET 0x3b00

struct VCollide_SurfaceHeader {
	DECLARE_BYTESWAP_DATADESC()
	int vphysicsID;
	short version;
	short modelType;
	int surfaceSize;
	Vector dragAxisAreas;
	int axisMapSize;
};

#ifdef _X360
#pragma bitfield_order(push, lsb_to_msb)
#endif

struct VCollide_IVP_U_Float_Point {
	DECLARE_BYTESWAP_DATADESC()
	float k[3];
	float hesse_val;
};

struct VCollide_IVP_Compact_Edge {
	DECLARE_BYTESWAP_DATADESC()
	BEGIN_BITFIELD(bf)
	unsigned int start_point_index : 16;
	signed int opposite_index : 15;
	unsigned int is_virtual : 1;
	END_BITFIELD()
};

struct VCollide_IVP_Compact_Triangle {
	DECLARE_BYTESWAP_DATADESC()
	BEGIN_BITFIELD(bf)
	unsigned int tri_index : 12;
	unsigned int pierce_index : 12;
	unsigned int material_index : 7;
	unsigned int is_virtual : 1;
	END_BITFIELD()
	VCollide_IVP_Compact_Edge c_three_edges[3];
};

struct VCollide_IVP_Compact_Ledge {
	DECLARE_BYTESWAP_DATADESC()
	int c_point_offset;
	union {
		int ledgetree_node_offset;
		int client_data;
	};
	BEGIN_BITFIELD(bf)
	unsigned int has_children_flag : 2;
	unsigned int is_compact_flag : 2;
	unsigned int dummy : 4;
	unsigned int size_div_16 : 24;
	END_BITFIELD()
	short n_triangles;
	short for_future_use;

	FORCEINLINE int get_n_points() const {
		return size_div_16 - n_triangles - 1;
	}
};

struct VCollide_IVP_Compact_Ledgetree_Node {
	DECLARE_BYTESWAP_DATADESC()
	int offset_right_node;
	int offset_compact_ledge;
	float center[3];
	float radius;
	unsigned char box_sizes[3];
	unsigned char free_0;
};

struct VCollide_IVP_Compact_Surface {
	DECLARE_BYTESWAP_DATADESC()
	float mass_center[3];
	float rotation_inertia[3];
	float upper_limit_radius;
	BEGIN_BITFIELD(bf)
	unsigned int max_factor_surface_deviation : 8;
	int byte_size : 24;
	END_BITFIELD()
	int offset_ledgetree_root;
	int dummy[3];
};

#ifdef _X360
#pragma bitfield_order(pop)
#endif

BEGIN_BYTESWAP_DATADESC(VCollide_SurfaceHeader)
	DEFINE_FIELD(vphysicsID, FIELD_INTEGER),
	DEFINE_FIELD(version, FIELD_SHORT),
	DEFINE_FIELD(modelType, FIELD_SHORT),
	DEFINE_FIELD(surfaceSize, FIELD_INTEGER),
	DEFINE_FIELD(dragAxisAreas, FIELD_VECTOR),
	DEFINE_FIELD(axisMapSize, FIELD_INTEGER)
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(VCollide_IVP_U_Float_Point)
	DEFINE_ARRAY(k, FIELD_FLOAT, 3),
	DEFINE_FIELD(hesse_val, FIELD_FLOAT),
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(VCollide_IVP_Compact_Edge)
	DEFINE_BITFIELD(bf, FIELD_INTEGER, 32),
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(VCollide_IVP_Compact_Triangle)
	DEFINE_BITFIELD(bf, FIELD_INTEGER, 32),
	DEFINE_EMBEDDED_ARRAY(c_three_edges, 3),
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(VCollide_IVP_Compact_Ledge)
	DEFINE_FIELD(c_point_offset, FIELD_INTEGER),
	DEFINE_FIELD(ledgetree_node_offset, FIELD_INTEGER),
	DEFINE_BITFIELD(bf, FIELD_INTEGER, 32),
	DEFINE_FIELD(n_triangles, FIELD_SHORT),
	DEFINE_FIELD(for_future_use, FIELD_SHORT),
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(VCollide_IVP_Compact_Ledgetree_Node)
	DEFINE_FIELD(offset_right_node, FIELD_INTEGER),
	DEFINE_FIELD(offset_compact_ledge, FIELD_INTEGER),
	DEFINE_ARRAY(center, FIELD_FLOAT, 3),
	DEFINE_FIELD(radius, FIELD_FLOAT),
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(VCollide_IVP_Compact_Surface)
	DEFINE_ARRAY(mass_center, FIELD_FLOAT, 3),
	DEFINE_ARRAY(rotation_inertia, FIELD_FLOAT, 3),
	DEFINE_FIELD(upper_limit_radius, FIELD_FLOAT),
	DEFINE_BITFIELD(bf, FIELD_INTEGER, 32),
	DEFINE_FIELD(offset_ledgetree_root, FIELD_INTEGER),
	DEFINE_ARRAY(dummy, FIELD_INTEGER, 3),
END_BYTESWAP_DATADESC()

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

void CPhysConvex::Initialize() {
	btCollisionShape *shape = GetShape();
	shape->setUserPointer(this);
	shape->setUserIndex(0);
}

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

void CPhysConvex_Hull::Initialize() {
	CPhysConvex::Initialize();
	m_Volume = -1.0;
	m_Shape.setMargin(VPHYSICS_CONVEX_DISTANCE_MARGIN);
}

CPhysConvex_Hull::CPhysConvex_Hull(const btVector3 *points, int pointCount,
		const unsigned int *indices, int triangleCount) :
		m_Shape(&points[0][0], pointCount) {
	Initialize();
	int indexCount = triangleCount * 3;
	m_TriangleIndices.resizeNoInitialize(indexCount);
	memcpy(&m_TriangleIndices[0], indices, indexCount * sizeof(indices[0]));
}

CPhysConvex_Hull::CPhysConvex_Hull(const VCollide_IVP_Compact_Ledge *ledge, CByteswap &byteswap,
		const btVector3 *ledgePoints, int ledgePointCount) :
		m_Shape(&ledgePoints[0][0], ledgePointCount) {
	Initialize();
	m_Shape.setUserIndex(ledge->client_data);
	VCollide_IVP_Compact_Ledge swappedLedge;
	byteswap.SwapBufferToTargetEndian(&swappedLedge, const_cast<VCollide_IVP_Compact_Ledge *>(ledge));
	const VCollide_IVP_Compact_Triangle *triangles =
			reinterpret_cast<const VCollide_IVP_Compact_Triangle *>(ledge + 1);
	int triangleCount = swappedLedge.n_triangles;
	m_TriangleIndices.resizeNoInitialize(triangleCount * 3);
	unsigned int *indices = &m_TriangleIndices[0];
	for (int triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
		VCollide_IVP_Compact_Triangle swappedTriangle;
		byteswap.SwapBufferToTargetEndian(&swappedTriangle,
				const_cast<VCollide_IVP_Compact_Triangle *>(&triangles[triangleIndex]));
		int indexIndex = triangleIndex * 3;
		indices[indexIndex] = swappedTriangle.c_three_edges[0].start_point_index;
		indices[indexIndex + 1] = swappedTriangle.c_three_edges[1].start_point_index;
		indices[indexIndex + 2] = swappedTriangle.c_three_edges[2].start_point_index;
		if (swappedTriangle.material_index > 0) {
			if (m_TriangleMaterials.size() == 0) {
				m_TriangleMaterials.resizeNoInitialize(triangleCount);
				memset(&m_TriangleMaterials[0], 0, triangleCount * sizeof(m_TriangleMaterials[0]));
			}
			m_TriangleMaterials[triangleIndex] = swappedTriangle.material_index;
		}
	}
	if (m_TriangleMaterials.size() != 0) {
		CalculateTrianglePlanes();
	}
}

CPhysConvex_Hull *CPhysConvex_Hull::CreateFromBulletPoints(
		HullLibrary &hullLibrary, const btVector3 *points, int pointCount) {
	if (pointCount < 3) {
		return nullptr;
	}
	HullResult hull;
	HullError hullError = hullLibrary.CreateConvexHull(
			HullDesc(QF_TRIANGLES, pointCount, points), hull);
	if (hullError != QE_OK || hull.mNumFaces == 0) {
		AssertMsg(false, "Convex hull creation failed");
		return nullptr;
	}
	return new CPhysConvex_Hull(&hull.m_OutputVertices[0], hull.mNumOutputVertices,
			&hull.m_Indices[0], hull.mNumFaces);
}

CPhysConvex_Hull *CPhysConvex_Hull::CreateFromIVPCompactLedge(
		const VCollide_IVP_Compact_Ledge *ledge, CByteswap &byteswap) {
	VCollide_IVP_Compact_Ledge swappedLedge;
	byteswap.SwapBufferToTargetEndian(&swappedLedge, const_cast<VCollide_IVP_Compact_Ledge *>(ledge));
	const VCollide_IVP_U_Float_Point *ivpPoints = reinterpret_cast<const VCollide_IVP_U_Float_Point *>(
			reinterpret_cast<const byte *>(ledge) + swappedLedge.c_point_offset);
	btAlignedObjectArray<btVector3> pointArray;
	int pointCount = swappedLedge.get_n_points();
	if (pointCount < 3 || swappedLedge.n_triangles <= 0) {
		return nullptr;
	}
	pointArray.resizeNoInitialize(pointCount);
	btVector3 *points = &pointArray[0];
	for (int pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
		VCollide_IVP_U_Float_Point swappedPoint;
		byteswap.SwapBufferToTargetEndian(&swappedPoint, const_cast<VCollide_IVP_U_Float_Point *>(&ivpPoints[pointIndex]));
		points[pointIndex].setValue(swappedPoint.k[0], -swappedPoint.k[1], -swappedPoint.k[2]);
	}
	return new CPhysConvex_Hull(ledge, byteswap, points, pointCount);
}

void CPhysConvex_Hull::CalculateVolumeProperties() {
	if (m_Volume >= 0.0f) {
		return;
	}
	// Based on btConvexTriangleMeshShape::calculatePrincipalAxisTransform, but without rotation.
	const btVector3 *points = &m_Shape.getPoints[0][0];
	const unsigned int *indices = &m_TriangleIndices[0];
	const btVector3 &ref = points[indices[0]];
	int indexCount = m_TriangleIndices.size();
	btScalar sixVolume = 0.0f;
	btVector3 massCenterSum(0.0f, 0.0f, 0.0f);
	for (int indexIndex = 3; indexIndex < indexCount; indexIndex += 3) {
		const btVector3 &p0 = points[indices[indexIndex]];
		const btVector3 &p1 = points[indices[indexIndex + 1]];
		const btVector3 &p2 = points[indices[indexIndex + 2]];
		btScalar tetrahedronSixVolume = btFabs((p0 - ref).triple(p1 - ref, p2 - ref));
		sixVolume += tetrahedronSixVolume;
		massCenterSum += (0.25f * tetrahedronSixVolume) * (p0 + p1 + p2 + ref);
	}
	m_Volume = (1.0f / 6.0f) * sixVolume;
	if (m_Volume > 0.0f) {
		m_MassCenter = massCenterSum / sixVolume;
		m_Inertia.setZero();
		for (int indexIndex = 0; indexIndex < indexCount; indexIndex += 3) {
			btVector3 a = points[indices[indexIndex]] - m_MassCenter;
			btVector3 b = points[indices[indexIndex + 1]] - m_MassCenter;
			btVector3 c = points[indices[indexIndex + 2]] - m_MassCenter;
			btVector3 i = btFabs(a.triple(b, c)) * (0.1f / 6.0f) *
					(a * a + b * b + c * c + a * b + a * c + b * c);
			m_Inertia[0] += i[1] + i[2];
			m_Inertia[1] += i[2] + i[0];
			m_Inertia[2] += i[0] + i[1];
		}
		m_Inertia = (m_Inertia / m_Volume).absolute();
	} else {
		// Use a box approximation.
		btVector3 aabbMin, aabbMax;
		m_Shape.getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
		m_MassCenter = (aabbMin + aabbMax) * 0.5f;
		m_Inertia = CPhysicsCollision::OffsetInertia(
				CPhysicsCollision::BoxInertia(aabbMax - aabbMin), m_MassCenter);
	}
}

btScalar CPhysConvex_Hull::GetVolume() const {
	const_cast<CPhysConvex_Hull *>(this)->CalculateVolumeProperties();
	return m_Volume;
}

btScalar CPhysConvex_Hull::GetSurfaceArea() const {
	const btVector3 *points = m_Shape.getPoints();
	const unsigned int *indices = &m_TriangleIndices[0];
	int indexCount = m_TriangleIndices.size();
	btScalar area = 0.0f;
	for (int indexIndex = 0; indexIndex < indexCount; indexIndex += 3) {
		const btVector3 &p0 = points[indices[indexIndex]];
		const btVector3 &p1 = points[indices[indexIndex + 1]];
		const btVector3 &p2 = points[indices[indexIndex + 2]];
		area += (p1 - p0).cross(p2 - p0).length();
	}
	return 0.5f * area;
}

btVector3 CPhysConvex_Hull::GetMassCenter() const {
	const_cast<CPhysConvex_Hull *>(this)->CalculateVolumeProperties();
	return m_MassCenter;
}

btVector3 CPhysConvex_Hull::GetInertia() const {
	const_cast<CPhysConvex_Hull *>(this)->CalculateVolumeProperties();
	return m_Inertia;
}

// This is a hack, gives per-plane surface index, not per-triangle,
// as Bullet doesn't do per-triangle collision detection for convexes.
// However, per-triangle materials are used only by world brushes,
// which can't have coplanar triangles with different materials.
int CPhysConvex_Hull::GetTriangleMaterialIndex(const btVector3 &point) const {
	int triangleCount = m_TriangleMaterials.size();
	if (triangleCount == 0) {
		return 0;
	}

	btVector3 aabbMin, aabbMax;
	m_Shape.getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
	btVector3 center = (aabbMin + aabbMax) * 0.5f;
	btVector3 pointCenterRelative = point - center; // Doesn't have to be normalized.
	btScalar margin = m_Shape.getMargin();

	// Project the point onto each plane that isn't opposite to the contact direction,
	// then choose the plane where the projected point is the closest to the center.
	// The best projection should be on the shape, while other ones should be outside.
	const btVector4 *planes = &m_TrianglePlanes[0];
	int closestTriangle = 0; // Fall back to a random triangle within the brush in case of failure.
	btScalar closestProjectionDistance2 = BT_LARGE_FLOAT;
	for (int triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
		const btVector4 &plane = planes[triangleIndex];
		// Without this check, the opposite side of the convex would be treated as forward.
		if (plane.dot(pointCenterRelative) < 0.0000001f) {
			continue;
		}
		btScalar projectionDistance2 = center.distance2(
				point - (plane.dot(point) - (plane.getW() - margin)) * plane);
		if (projectionDistance2 < closestProjectionDistance2) {
			closestProjectionDistance2 = projectionDistance2;
			closestTriangle = triangleIndex;
		}
	}
	return m_TriangleMaterials[closestTriangle];
}

void CPhysConvex_Hull::CalculateTrianglePlanes() {
	if (m_TrianglePlanes.size() != 0) {
		return;
	}
	int triangleCount = m_TriangleIndices.size() / 3;
	m_TrianglePlanes.resizeNoInitialize(triangleCount);
	btVector4 *planes = &m_TrianglePlanes[0];
	const btVector3 *points = &m_Shape.getPoints[0][0];
	const unsigned int *indices = &m_TriangleIndices[0];
	for (int triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
		int indexIndex = triangleIndex * 3;
		const btVector3 &v1 = points[indices[indexIndex]];
		btVector3 normal = points[indices[indexIndex + 1]].cross(points[indices[indexIndex + 2]]);
		normal.normalize();
		planes[triangleIndex].setValue(normal.getX(), normal.getY(), normal.getZ(), v1.dot(normal));
	}
}

CPhysConvex *CPhysicsCollision::ConvexFromVerts(Vector **pVerts, int vertCount) {
	btAlignedObjectArray<btVector3> pointArray;
	pointArray.resizeNoInitialize(vertCount);
	btVector3 *points = &pointArray[0];
	for (int vertIndex = 0; vertIndex < vertCount; ++vertIndex) {
		ConvertPositionToBullet(*pVerts[vertIndex], points[vertIndex]);
	}
	return CPhysConvex_Hull::CreateFromBulletPoints(m_HullLibrary, points, vertCount);
}

CPhysConvex *CPhysicsCollision::ConvexFromPlanes(float *pPlanes, int planeCount, float mergeDistance) {
	btAlignedObjectArray<btVector3> planeArray;
	planeArray.resizeNoInitialize(planeCount);
	btVector4 *bulletPlanes = static_cast<btVector4 *>(&planeArray[0]);
	for (int planeIndex = 0; planeIndex < planeCount; ++planeIndex) {
		const float *listPlane = &pPlanes[planeIndex * 4];
		btVector4 &bulletPlane = bulletPlanes[planeIndex];
		ConvertDirectionToBullet(Vector(listPlane[0], listPlane[1], listPlane[2]), bulletPlane);
		bulletPlane.setW(-HL2BULLET(listPlane[3]));
	}
	btAlignedObjectArray<btVector3> pointArray;
	btGeometryUtil::getVerticesFromPlaneEquations(planeArray, pointArray);
	return CPhysConvex_Hull::CreateFromBulletPoints(m_HullLibrary, &pointArray[0], pointArray.size());
}

CPhysConvex *CPhysicsCollision::ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron) {
	const Vector *verts = ConvexPolyhedron.pVertices;
	int vertCount = ConvexPolyhedron.iVertexCount;
	btAlignedObjectArray<btVector3> pointArray;
	pointArray.resizeNoInitialize(vertCount);
	btVector3 *points = &pointArray[0];
	for (int vertIndex = 0; vertIndex < vertCount; ++vertIndex) {
		ConvertPositionToBullet(verts[vertIndex], points[vertIndex]);
	}
	return CPhysConvex_Hull::CreateFromBulletPoints(m_HullLibrary, points, vertCount);
}

/***********************************************
 * Bounding boxes (both convex and collideable)
 ***********************************************/

CPhysConvex_Box::CPhysConvex_Box(const btVector3 &halfExtents, const btVector3 &origin) :
		m_Shape(halfExtents), m_Origin(origin) {
	Initialize();
	m_Shape.setMargin(VPHYSICS_CONVEX_DISTANCE_MARGIN);
}

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

	CPhysConvex *box = new CPhysConvex_Box(halfExtents, origin);
	box->SetOwner(CPhysConvex::OWNER_INTERNAL);
	CPhysCollide_Compound *compound = new CPhysCollide_Compound(&box, 1);
	compound->SetOwner(CPhysCollide::OWNER_INTERNAL);
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

Vector CPhysicsCollision::CollideGetExtent(const CPhysCollide *pCollide,
		const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction) {
	btVector3 bulletOrigin, bulletDirection;
	btMatrix3x3 bulletRotation;
	ConvertPositionToBullet(collideOrigin, bulletOrigin);
	ConvertRotationToBullet(collideAngles, bulletRotation);
	ConvertDirectionToBullet(direction, bulletDirection);
	Vector extent;
	ConvertPositionToHL(pCollide->GetExtent(bulletOrigin, bulletRotation, bulletDirection), extent);
	return extent;
}

void CPhysicsCollision::CollideGetAABB(Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide,
		const Vector &collideOrigin, const QAngle &collideAngles) {
	btTransform transform;
	ConvertRotationToBullet(collideAngles, transform.getBasis());
	ConvertPositionToBullet(collideOrigin, transform.getOrigin());
	transform.getOrigin() += transform.getBasis() * pCollide->GetMassCenter();
	btVector3 aabbMin, aabbMax;
	pCollide->GetShape()->getAabb(transform, aabbMin, aabbMax);
	Vector hlAabbMin, hlAabbMax;
	ConvertPositionToHL(aabbMin, hlAabbMin);
	ConvertPositionToHL(aabbMax, hlAabbMax);
	VectorMin(hlAabbMin, hlAabbMax, *pMins);
	VectorMax(hlAabbMin, hlAabbMax, *pMaxs);
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

void CPhysCollide::SetOrthographicAreas(const btVector3 &areas) {
	m_OrthographicAreas = areas;
	IPhysicsObject *firstObject = GetObjectReferenceList();
	if (firstObject != nullptr) {
		CPhysicsObject *object = static_cast<CPhysicsObject *>(firstObject);
		do {
			object->NotifyOrthographicAreasChanged();
			object = object->GetNextCollideObject();
		} while (object != firstObject);
	}
}

Vector CPhysicsCollision::CollideGetOrthographicAreas(const CPhysCollide *pCollide) {
	Vector areas;
	ConvertAbsoluteDirectionToHL(pCollide->GetOrthographicAreas(), areas);
	return areas;
}

void CPhysicsCollision::CollideSetOrthographicAreas(CPhysCollide *pCollide, const Vector &areas) {
	btVector3 bulletAreas;
	ConvertAbsoluteDirectionToBullet(areas, bulletAreas);
	pCollide->SetOrthographicAreas(bulletAreas);
}

int CPhysicsCollision::CollideIndex(const CPhysCollide *pCollide) {
	return pCollide->GetShape()->getUserIndex();
}

int CPhysicsCollision::GetConvexesUsedInCollideable(const CPhysCollide *pCollideable,
		CPhysConvex **pOutputArray, int iOutputArrayLimit) {
	int convexCount = pCollideable->GetConvexes(pOutputArray, iOutputArrayLimit);
	return MIN(convexCount, iOutputArrayLimit);
}

unsigned int CPhysicsCollision::ReadStat(int statID) {
	return 0; // Not implemented in v31 IVP VPhysics, increments commented out in v29.
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
		const btTransform &convexAabbTransform = btTransform::getIdentity();
		for (int convexIndex = 0; convexIndex < convexCount; ++convexIndex) {
			const CPhysConvex *convex = pConvex[convexIndex];
			btVector3 convexAabbMin, convexAabbMax;
			convex->GetShape()->getAabb(convexAabbTransform, convexAabbMin, convexAabbMax);
			const btVector3 &convexOrigin = convex->GetOriginInCompound();
			aabbMin.setMin(convexAabbMin + convexOrigin);
			aabbMax.setMax(convexAabbMax + convexOrigin);
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

CPhysCollide_Compound::CPhysCollide_Compound(
		const VCollide_IVP_Compact_Ledgetree_Node *root, CByteswap &byteswap,
		const btVector3 &massCenter, const btVector3 &inertia,
		const btVector3 &orthographicAreas) :
		CPhysCollide(orthographicAreas),
		m_Shape(root->offset_right_node != 0 /* Swap not required */),
		m_Volume(-1.0f), m_MassCenter(massCenter), m_Inertia(inertia) {
	AddIVPCompactLedgetreeNode(root, byteswap);
}

void CPhysCollide_Compound::AddIVPCompactLedgetreeNode(
		const VCollide_IVP_Compact_Ledgetree_Node *node, CByteswap &byteswap) {
	VCollide_IVP_Compact_Ledgetree_Node swappedNode;
	byteswap.SwapBufferToTargetEndian(&swappedNode, const_cast<VCollide_IVP_Compact_Ledgetree_Node *>(node));
	if (swappedNode.offset_right_node == 0) {
		CPhysConvex_Hull *convex = CPhysConvex_Hull::CreateFromIVPCompactLedge(
				reinterpret_cast<const VCollide_IVP_Compact_Ledge *>(
						reinterpret_cast<const byte *>(node) + swappedNode.offset_compact_ledge), byteswap);
		convex->SetOwner(CPhysConvex::OWNER_COMPOUND);
		m_Shape.addChildShape(btTransform(btMatrix3x3::getIdentity(),
				convex->GetOriginInCompound() - m_MassCenter), convex->GetShape());
	} else {
		AddIVPCompactLedgetreeNode(node + 1, byteswap);
		AddIVPCompactLedgetreeNode(reinterpret_cast<const VCollide_IVP_Compact_Ledgetree_Node *>(
				reinterpret_cast<const byte *>(node) + swappedNode.offset_right_node), byteswap);
	}
}

CPhysCollide *CPhysicsCollision::ConvertConvexToCollide(CPhysConvex **pConvex, int convexCount) {
	if (convexCount == 0 || pConvex == nullptr) {
		return nullptr;
	}
	return new CPhysCollide_Compound(pConvex, convexCount);
}

btScalar CPhysCollide_Compound::GetVolume() const {
	if (m_Volume < 0.0f) {
		btScalar &volume = const_cast<CPhysCollide_Compound *>(this)->m_Volume;
		volume = 0.0f;
		int childCount = m_Shape.getNumChildShapes();
		for (int childIndex = 0; childIndex < childCount; ++childIndex) {
			volume += reinterpret_cast<const CPhysConvex *>(
					m_Shape.getChildShape(childIndex)->getUserPointer())->GetVolume();
		}
	}
	return m_Volume;
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

btVector3 CPhysCollide_Compound::GetExtent(const btVector3 &origin, const btMatrix3x3 &rotation,
		const btVector3 &direction) const {
	btVector3 localDirection = direction * rotation;
	btVector3 extent = origin;
	btScalar maxDot = -BT_LARGE_FLOAT;
	int childCount = m_Shape.getNumChildShapes();
	for (int childIndex = 0; childIndex < childCount; ++childIndex) {
		const btConvexShape *shape = static_cast<const btConvexShape *>(
				m_Shape.getChildShape(childIndex));
		btVector3 localSupportingVertex = shape->localGetSupportingVertex(localDirection);
		btVector3 worldSupportingVertex = origin + (rotation * (localSupportingVertex +
				reinterpret_cast<CPhysConvex *>(shape->getUserPointer())->GetOriginInCompound()));
		btScalar dot = worldSupportingVertex.dot(direction);
		if (dot > maxDot) {
			maxDot = dot;
			extent = worldSupportingVertex;
		}
	}
	return extent;
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
	btScalar volume = GetVolume(); // Not necessarily precalculated.
	if (volume >= 0) {
		m_Inertia.setZero();
		int childCount = m_Shape.getNumChildShapes();
		for (int childIndex = 0; childIndex < childCount; ++childIndex) {
			const CPhysConvex *convex = reinterpret_cast<const CPhysConvex *>(
					m_Shape.getChildShape(childIndex)->getUserPointer());
			m_Inertia += convex->GetVolume() * CPhysicsCollision::OffsetInertia(
					convex->GetInertia(), m_Shape.getChildTransform(childIndex).getOrigin(), false);
		}
		m_Inertia = (m_Inertia / volume).absolute();
	} else {
		btVector3 aabbMin, aabbMax;
		m_Shape.getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
		m_Inertia = CPhysicsCollision::OffsetInertia(
				CPhysicsCollision::BoxInertia(aabbMax - aabbMin),
				(aabbMin + aabbMax) * 0.5f);
	}
}

int CPhysCollide_Compound::GetConvexes(CPhysConvex **output, int limit) const {
	int childCount = m_Shape.getNumChildShapes();
	if (childCount < limit) {
		limit = childCount;
	}
	for (int childIndex = 0; childIndex < limit; ++childIndex) {
		output[childIndex] = reinterpret_cast<CPhysConvex *>(
				m_Shape.getChildShape(childIndex)->getUserPointer());
	}
	return childCount;
}

/**********
 * Spheres
 **********/

btScalar CPhysCollide_Sphere::GetVolume() const {
	btScalar radius = GetRadius();
	return ((4.0f / 3.0f) * SIMD_PI) * radius * radius * radius;
}

btScalar CPhysCollide_Sphere::GetSurfaceArea() const {
	btScalar radius = GetRadius();
	return (4.0f * SIMD_PI) * radius * radius;
}

btVector3 CPhysCollide_Sphere::GetExtent(const btVector3 &origin, const btMatrix3x3 &rotation,
		const btVector3 &direction) const {
	return origin + (m_Shape.getRadius() * direction);
}

btVector3 CPhysCollide_Sphere::GetInertia() const {
	btScalar elem = GetRadius();
	elem *= elem * 0.4;
	return btVector3(elem, elem, elem);
}

CPhysCollide_Sphere *CPhysicsCollision::CreateSphereCollide(btScalar radius) {
	return new CPhysCollide_Sphere(radius);
}

/************************
 * Collide serialization
 ************************/

CPhysCollide *CPhysicsCollision::UnserializeIVPCompactSurface(
		const VCollide_IVP_Compact_Surface *surface, CByteswap &byteswap,
		const btVector3 &orthographicAreas) {
	VCollide_IVP_Compact_Surface swappedSurface;
	byteswap.SwapBufferToTargetEndian(&swappedSurface, const_cast<VCollide_IVP_Compact_Surface *>(surface));
	if (swappedSurface.dummy[2] != VCOLLIDE_IVP_COMPACT_SURFACE_ID) {
		return nullptr;
	}
	return new CPhysCollide_Compound(
			reinterpret_cast<const VCollide_IVP_Compact_Ledgetree_Node *>(
					reinterpret_cast<const byte *>(surface) + swappedSurface.offset_ledgetree_root),
			byteswap,
			btVector3(swappedSurface.mass_center[0], -swappedSurface.mass_center[1], -swappedSurface.mass_center[2]),
			btVector3(swappedSurface.rotation_inertia[0], swappedSurface.rotation_inertia[1], swappedSurface.rotation_inertia[2]),
			orthographicAreas);
}

CPhysCollide *CPhysicsCollision::UnserializeCollide(const char *pBuffer, int size, int index, bool swap) {
	CByteswap byteswap;
	byteswap.ActivateByteSwapping(swap);
	VCollide_SurfaceHeader swappedHeader;
	byteswap.SwapBufferToTargetEndian(&swappedHeader, const_cast<VCollide_SurfaceHeader *>(
			reinterpret_cast<const VCollide_SurfaceHeader *>(pBuffer)));
	CPhysCollide *collide = nullptr;
	if (swappedHeader.vphysicsID == VCOLLIDE_VPHYSICS_ID) {
		if (swappedHeader.version != VCOLLIDE_VERSION_IVP &&
				swappedHeader.version != VCOLLIDE_VERSION_BULLET) {
			return nullptr;
		}
		btVector3 orthographicAreas;
		ConvertAbsoluteDirectionToBullet(swappedHeader.dragAxisAreas, orthographicAreas);
		const char *collideBuffer = pBuffer + sizeof(VCollide_SurfaceHeader);
		switch (swappedHeader.modelType) {
		case VCOLLIDE_MODEL_TYPE_IVP_COMPACT_SURFACE:
			collide = UnserializeIVPCompactSurface(
					reinterpret_cast<const VCollide_IVP_Compact_Surface *>(collideBuffer),
					byteswap, orthographicAreas);
			break;
		}
	} else {
		DevMsg("Old format .PHY file loaded!!!\n");
		collide = UnserializeIVPCompactSurface(
				reinterpret_cast<const VCollide_IVP_Compact_Surface *>(pBuffer),
				byteswap, btVector3(1.0f, 1.0f, 1.0f));
	}
	if (collide != nullptr) {
		collide->GetShape()->setUserIndex(index);
	} else {
		DevMsg("Null physics model\n");
	}
	return collide;
}

CPhysCollide *CPhysicsCollision::UnserializeCollide(char *pBuffer, int size, int index) {
	return UnserializeCollide(pBuffer, size, index, false);
}

void CPhysicsCollision::VCollideLoad(vcollide_t *pOutput,
			int solidCount, const char *pBuffer, int size, bool swap) {
	memset(pOutput, 0, sizeof(*pOutput));
	pOutput->solidCount = solidCount;
	pOutput->solids = new CPhysCollide *[solidCount];
	int position = 0;
	for (int solidIndex = 0; solidIndex < solidCount; ++solidIndex) {
		union {
			int solidSize;
			char solidSizeBytes[sizeof(int)];
		};
		if (swap) {
			for (int sizeByteIndex = 0; sizeByteIndex < sizeof(int); ++sizeByteIndex) {
				solidSizeBytes[sizeof(int) - 1 - sizeByteIndex] = pBuffer[position + sizeByteIndex];
			}
		} else {
			memcpy(&solidSize, pBuffer + position, sizeof(int));
		}
		position += sizeof(int);
		pOutput->solids[solidIndex] = UnserializeCollide(pBuffer + position, solidSize, solidIndex, swap);
		position += solidSize;
	}
	int keySize = size - position;
	pOutput->pKeyValues = new char[keySize];
	memcpy(pOutput->pKeyValues, pBuffer + position, keySize);
}
