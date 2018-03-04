// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_collide.h"
#include "physics_parse.h"
#include "physics_object.h"
#include <LinearMath/btGeometryUtil.h>
#include "mathlib/polyhedron.h"
#include "mathlib/vplane.h"
#include "tier0/dbg.h"

static CPhysicsCollision s_PhysCollision;
CPhysicsCollision *g_pPhysCollision = &s_PhysCollision;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(CPhysicsCollision, IPhysicsCollision,
		VPHYSICS_COLLISION_INTERFACE_VERSION, s_PhysCollision);

CPhysicsCollision::CPhysicsCollision() :
		m_InContactTest(false),
		m_TraceBoxShape(btVector3(1.0f, 1.0f, 1.0f)),
		m_TracePointShape(VPHYSICS_CONVEX_DISTANCE_MARGIN),
		m_TraceConeShape(1.0f, 1.0f) {
	m_TraceBoxShape.setMargin(VPHYSICS_CONVEX_DISTANCE_MARGIN);
	m_TraceConeShape.setMargin(VPHYSICS_CONVEX_DISTANCE_MARGIN);

	m_ContactTestCollisionConfiguration = VPhysicsNew(btDefaultCollisionConfiguration);
	m_ContactTestDispatcher = VPhysicsNew(btCollisionDispatcher, m_ContactTestCollisionConfiguration);
	m_ContactTestBroadphase = VPhysicsNew(btSimpleBroadphase, 2); // 0 is dangerous as it's array size.
	m_ContactTestCollisionWorld = VPhysicsNew(btCollisionWorld,
			m_ContactTestDispatcher, m_ContactTestBroadphase, m_ContactTestCollisionConfiguration);
}

CPhysicsCollision::~CPhysicsCollision() {
	VPhysicsDelete(btCollisionWorld, m_ContactTestCollisionWorld);
	VPhysicsDelete(btSimpleBroadphase, m_ContactTestBroadphase);
	VPhysicsDelete(btCollisionDispatcher, m_ContactTestDispatcher);
	VPhysicsDelete(btDefaultCollisionConfiguration, m_ContactTestCollisionConfiguration);

	int sphereCount = m_SphereCache.Count();
	for (int sphereIndex = 0; sphereIndex < sphereCount; ++sphereIndex) {
		CPhysCollide_Sphere *sphere = m_SphereCache[sphereIndex];
		Assert(sphere->GetObjectReferenceList() == nullptr);
		if (sphere->GetObjectReferenceList() != nullptr) {
			DevMsg("Freed sphere collision model while in use!!!\n");
			continue;
		}
		sphere->Release();
	}

	int bboxCount = m_BBoxCache.Count();
	for (int bboxIndex = 0; bboxIndex < bboxCount; ++bboxIndex) {
		CPhysCollide_Compound *bboxCompound = m_BBoxCache[bboxIndex];
		Assert(bboxCompound->GetObjectReferenceList() == nullptr);
		if (bboxCompound->GetObjectReferenceList() != nullptr) {
			DevMsg("Freed bounding box collision model while in use!!!\n");
			continue;
		}
		// A bbox may be a part of other compounds, but there's no way to check that.
		CPhysConvex *bboxConvex = reinterpret_cast<CPhysConvex *>(
				bboxCompound->GetCompoundShape()->getChildShape(0)->getUserPointer());
		bboxCompound->Release();
		bboxConvex->Release();
	}
}

IPhysicsCollision *CPhysicsCollision::ThreadContextCreate() {
	// IVP VPhysics v29 used to create a new CPhysicsCollision, but v31 returns this.
	// Due to g_pPhysCollision references, and because object reference lists are thread-unsafe,
	// this VPhysics implementation has to be single-threaded.
	return this;
}

void CPhysicsCollision::ThreadContextDestroy(IPhysicsCollision *pThreadContext) {}

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

BEGIN_BYTESWAP_DATADESC(VCollide_SurfaceHeader)
	DEFINE_FIELD(vphysicsID, FIELD_INTEGER),
	DEFINE_FIELD(version, FIELD_SHORT),
	DEFINE_FIELD(modelType, FIELD_SHORT),
	DEFINE_FIELD(surfaceSize, FIELD_INTEGER),
	DEFINE_FIELD(dragAxisAreas, FIELD_VECTOR),
	DEFINE_FIELD(axisMapSize, FIELD_INTEGER)
END_BYTESWAP_DATADESC();

BEGIN_BYTESWAP_DATADESC(VCollide_IVP_U_Float_Point)
	DEFINE_ARRAY(k, FIELD_FLOAT, 3),
	DEFINE_FIELD(hesse_val, FIELD_FLOAT),
END_BYTESWAP_DATADESC();

BEGIN_BYTESWAP_DATADESC(VCollide_IVP_Compact_Edge)
	DEFINE_BITFIELD(bf, FIELD_INTEGER, 32),
END_BYTESWAP_DATADESC();

BEGIN_BYTESWAP_DATADESC(VCollide_IVP_Compact_Triangle)
	DEFINE_BITFIELD(bf, FIELD_INTEGER, 32),
	DEFINE_EMBEDDED_ARRAY(c_three_edges, 3),
END_BYTESWAP_DATADESC();

BEGIN_BYTESWAP_DATADESC(VCollide_IVP_Compact_Ledge)
	DEFINE_FIELD(c_point_offset, FIELD_INTEGER),
	DEFINE_FIELD(ledgetree_node_offset, FIELD_INTEGER),
	DEFINE_BITFIELD(bf, FIELD_INTEGER, 32),
	DEFINE_FIELD(n_triangles, FIELD_SHORT),
	DEFINE_FIELD(for_future_use, FIELD_SHORT),
END_BYTESWAP_DATADESC();

BEGIN_BYTESWAP_DATADESC(VCollide_IVP_Compact_Ledgetree_Node)
	DEFINE_FIELD(offset_right_node, FIELD_INTEGER),
	DEFINE_FIELD(offset_compact_ledge, FIELD_INTEGER),
	DEFINE_ARRAY(center, FIELD_FLOAT, 3),
	DEFINE_FIELD(radius, FIELD_FLOAT),
END_BYTESWAP_DATADESC();

BEGIN_BYTESWAP_DATADESC(VCollide_IVP_Compact_Surface)
	DEFINE_ARRAY(mass_center, FIELD_FLOAT, 3),
	DEFINE_ARRAY(rotation_inertia, FIELD_FLOAT, 3),
	DEFINE_FIELD(upper_limit_radius, FIELD_FLOAT),
	DEFINE_BITFIELD(bf, FIELD_INTEGER, 32),
	DEFINE_FIELD(offset_ledgetree_root, FIELD_INTEGER),
	DEFINE_ARRAY(dummy, FIELD_INTEGER, 3),
END_BYTESWAP_DATADESC();

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
		pConvex->Release();
	}
}

/***************
 * Convex hulls
 ***************/

void CPhysConvex_Hull::Initialize() {
	CPhysConvex::Initialize();
	m_Volume = -1.0;
	// Prevent leaking uninitialized data during serialization.
	m_MassCenter.setZero();
	m_Inertia.setValue(1.0f, 1.0f, 1.0f);
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

CPhysConvex_Hull::CPhysConvex_Hull(const btVector3 *points, int pointCount, const CPolyhedron &polyhedron) :
		m_Shape(&points[0][0], pointCount) {
	Initialize();

	const Polyhedron_IndexedLine_t *lines = polyhedron.pLines;
	const Polyhedron_IndexedLineReference_t *lineIndices = polyhedron.pIndices;
	const Polyhedron_IndexedPolygon_t *polygons = polyhedron.pPolygons;
	int polygonCount = polyhedron.iPolygonCount;

	int triangleIndexCount = 0;
	for (int polygonIndex = 0; polygonIndex < polygonCount; ++polygonIndex) {
		int polygonTriangleCount = polygons[polygonIndex].iIndexCount - 2;
		if (polygonTriangleCount <= 0) {
			continue;
		}
		triangleIndexCount += polygonTriangleCount * 3;
	}
	m_TriangleIndices.resizeNoInitialize(triangleIndexCount);

	unsigned int *triangleIndices = &m_TriangleIndices[0];
	triangleIndexCount = 0;
	for (int polygonIndex = 0; polygonIndex < polygonCount; ++polygonIndex) {
		const Polyhedron_IndexedPolygon_t &polygon = polygons[polygonIndex];
		int polygonTriangleCount = polygon.iIndexCount - 2;
		if (polygonTriangleCount <= 0) {
			continue;
		}
		const Polyhedron_IndexedLineReference_t *lineReference = &lineIndices[polygon.iFirstIndex];
		unsigned int polygonStartIndex =
				lines[lineReference->iLineIndex].iPointIndices[1 - lineReference->iEndPointIndex];
		for (int polygonTriangleIndex = 0; polygonTriangleIndex < polygonTriangleCount; ++polygonTriangleIndex) {
			triangleIndices[triangleIndexCount++] = polygonStartIndex;
			triangleIndices[triangleIndexCount++] =
					lines[lineReference->iLineIndex].iPointIndices[lineReference->iEndPointIndex];
			++lineReference;
			triangleIndices[triangleIndexCount++] =
					lines[lineReference->iLineIndex].iPointIndices[lineReference->iEndPointIndex];
		}
	}
}

CPhysConvex_Hull::CPhysConvex_Hull(
		const VCollide_IVP_Compact_Triangle *swappedAndRemappedTriangles, int triangleCount,
		const btVector3 *ledgePoints, int ledgePointCount, int userIndex) :
		m_Shape(&ledgePoints[0][0], ledgePointCount) {
	Initialize();
	m_Shape.setUserIndex(userIndex);
	m_TriangleIndices.resizeNoInitialize(triangleCount * 3);
	unsigned int *indices = &m_TriangleIndices[0];
	for (int triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
		const VCollide_IVP_Compact_Triangle &triangle = swappedAndRemappedTriangles[triangleIndex];
		int indexIndex = triangleIndex * 3;
		indices[indexIndex] = triangle.c_three_edges[0].start_point_index;
		indices[indexIndex + 1] = triangle.c_three_edges[1].start_point_index;
		indices[indexIndex + 2] = triangle.c_three_edges[2].start_point_index;
		if (triangle.material_index > 0) {
			if (m_TriangleMaterials.size() == 0) {
				m_TriangleMaterials.resizeNoInitialize(triangleCount);
				memset(&m_TriangleMaterials[0], 0, triangleCount * sizeof(m_TriangleMaterials[0]));
			}
			m_TriangleMaterials[triangleIndex] = triangle.material_index;
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
	return VPhysicsNew(CPhysConvex_Hull, &hull.m_OutputVertices[0], hull.mNumOutputVertices,
			&hull.m_Indices[0], hull.mNumFaces);
}

CPhysConvex_Hull *CPhysicsCollision::CreateConvexHullFromIVPCompactLedge(
		const VCollide_IVP_Compact_Ledge *ledge, CByteswap &byteswap) {
	// IVP surfaces have a common array of points for all ledges, need to include only points referenced by triangles.

	// Byte swapping triangles.
	VCollide_IVP_Compact_Ledge swappedLedge;
	byteswap.SwapBufferToTargetEndian(&swappedLedge, const_cast<VCollide_IVP_Compact_Ledge *>(ledge));
	int triangleCount = swappedLedge.n_triangles;
	if (triangleCount <= 0) {
		return nullptr;
	}
	const VCollide_IVP_Compact_Triangle *triangles =
			reinterpret_cast<const VCollide_IVP_Compact_Triangle *>(ledge + 1);
	m_SwappedAndRemappedIVPTriangles.EnsureCount(triangleCount);
	byteswap.SwapBufferToTargetEndian(&m_SwappedAndRemappedIVPTriangles[0],
			const_cast<VCollide_IVP_Compact_Triangle *>(triangles), triangleCount);

	// Finding the first and the last points (for map size).
	int pointFirst = INT_MAX, pointLast = 0;
	for (int triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
		const VCollide_IVP_Compact_Triangle &triangle = m_SwappedAndRemappedIVPTriangles[triangleIndex];
		for (int vertexIndex = 0; vertexIndex < 3; ++vertexIndex) {
			int pointIndex = (int) triangle.c_three_edges[vertexIndex].start_point_index;
			pointFirst = MIN(pointIndex, pointFirst);
			pointLast = MAX(pointIndex, pointLast);
		}
	}
	m_IVPPointMap.EnsureCount(pointLast - pointFirst + 1);
	memset(&m_IVPPointMap[0], 0xff, m_IVPPointMap.Count() * sizeof(m_IVPPointMap[0]));

	// Remapping the points that are actually used.
	const VCollide_IVP_U_Float_Point *ivpPoints = reinterpret_cast<const VCollide_IVP_U_Float_Point *>(
			reinterpret_cast<const byte *>(ledge) + swappedLedge.c_point_offset);
	btAlignedObjectArray<btVector3> &points = GetHullCreationPointArray();
	points.resizeNoInitialize(0);
	points.reserve(swappedLedge.get_n_points());
	for (int triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
		VCollide_IVP_Compact_Triangle &triangle = m_SwappedAndRemappedIVPTriangles[triangleIndex];
		for (int vertexIndex = 0; vertexIndex < 3; ++vertexIndex) {
			VCollide_IVP_Compact_Edge &edge = triangle.c_three_edges[vertexIndex];
			int pointIndexInMap = edge.start_point_index - pointFirst;
			int pointRemappedIndex = m_IVPPointMap[pointIndexInMap];
			if (pointRemappedIndex < 0) {
				VCollide_IVP_U_Float_Point swappedPoint;
				byteswap.SwapBufferToTargetEndian(&swappedPoint, const_cast<VCollide_IVP_U_Float_Point *>(&ivpPoints[edge.start_point_index]));
				pointRemappedIndex = points.size();
				points.push_back(btVector3(swappedPoint.k[0], -swappedPoint.k[1], -swappedPoint.k[2]));
				m_IVPPointMap[pointIndexInMap] = pointRemappedIndex;
			}
			edge.start_point_index = pointRemappedIndex;
		}
	}

	m_IVPPointMap.RemoveAll();

	CPhysConvex_Hull *hull = VPhysicsNew(CPhysConvex_Hull, &m_SwappedAndRemappedIVPTriangles[0], triangleCount,
			&points[0], points.size(), swappedLedge.client_data);
	m_SwappedAndRemappedIVPTriangles.RemoveAll();
	return hull;
}

void CPhysConvex_Hull::CalculateVolumeProperties() {
	if (m_Volume >= 0.0f) {
		return;
	}
	// Based on btConvexTriangleMeshShape::calculatePrincipalAxisTransform, but without rotation.
	const btVector3 *points = &m_Shape.getPoints()[0];
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
		massCenterSum += tetrahedronSixVolume * (p0 + p1 + p2 + ref);
	}
	m_Volume = (1.0f / 6.0f) * sixVolume;
	if (m_Volume > 0.0f) {
		m_MassCenter = massCenterSum / (4.0f * sixVolume);
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

bool CPhysConvex_Hull::GetConvexTriangleMeshSubmergedVolume(
		const btVector3 &origin, const btVector3 *points, int pointCount,
		const unsigned int *indices, int indexCount,
		const btVector4 &plane, btScalar &volume, btVector3 &volumeWeightedBuoyancyCenter) {
	const btScalar onThreshold = HL2BULLET(VP_EPSILON);

	// Finding the origin of tetrahedron volume calculations,
	// either on the line between the highest and the deepest points
	// or any point on the plane if there's one.
	// Also handling the cases when the mesh is fully submerged or dry.
	int highestPointIndex = -1, deepestPointIndex = -1, pointOnPlaneIndex = -1;
	btScalar highestPointDistance = -BT_LARGE_FLOAT, deepestPointDistance = BT_LARGE_FLOAT;
	for (int pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
		btScalar pointDistance = plane.dot(points[pointIndex] + origin) + plane.getW();
		if (btFabs(pointDistance) < onThreshold) {
			pointOnPlaneIndex = pointIndex;
			continue;
		}
		if (pointDistance > 0.0f) {
			if (pointDistance > highestPointDistance) {
				highestPointDistance = pointDistance;
				highestPointIndex = pointIndex;
			}
		} else {
			if (pointDistance < deepestPointDistance) {
				deepestPointDistance = pointDistance;
				deepestPointIndex = pointIndex;
			}
		}
	}
	if (deepestPointIndex < 0) {
		// Fully above water.
		volume = 0.0f;
		volumeWeightedBuoyancyCenter.setZero();
		return true;
	}
	if (highestPointIndex < 0) {
		// Fully submerged - use the mass center.
		return false;
	}
	btVector3 ref;
	if (pointOnPlaneIndex >= 0) {
		ref = points[pointOnPlaneIndex] + origin;
	} else {
		btVector3 deepestToHighest = points[highestPointIndex] - points[deepestPointIndex];
		ref = points[deepestPointIndex] + origin -
				((deepestPointDistance / plane.dot(deepestToHighest)) * deepestToHighest);
	}

	// Finding the volumes and mass centers of submerged tetrahedra.
	for (int indexIndex = 0; indexIndex < indexCount; indexIndex += 3) {
		btVector3 p[3];
		int above[3], below[3], countAbove = 0, countBelow = 0;
		for (int pointIndex = 0; pointIndex < 3; ++pointIndex) {
			p[pointIndex] = points[indices[indexIndex + pointIndex]] + origin;
			btScalar pointDistance = plane.dot(p[pointIndex]) + plane.getW();
			if (btFabs(pointDistance) < onThreshold) {
				continue;
			}
			if (pointDistance > 0.0f) {
				above[countAbove++] = pointIndex;
			} else {
				below[countBelow++] = pointIndex;
			}
		}

		if (countBelow == 0) { // Fully above water - skip.
			continue;
		}

		btScalar tetrahedronSixVolume;

		if (countAbove == 0) { // Fully submerged.
			tetrahedronSixVolume = btFabs((p[0] - ref).triple(p[1] - ref, p[2] - ref));
			volume += tetrahedronSixVolume;
			volumeWeightedBuoyancyCenter += tetrahedronSixVolume * (p[0] + p[1] + p[2] + ref);
			continue;
		}

		if (countBelow == 1) { // The submerged part is a triangle.
			const btVector3 &pb = p[below[0]];
			btScalar pbDistance = plane.dot(pb) + plane.getW();

			// One edge is always clipped, since there's at least 1 point above.
			btVector3 pbToPa = p[above[0]] - pb;
			btVector3 po1 = pb - ((pbDistance / plane.dot(pbToPa)) * pbToPa);

			// Another point is either on an edge or on the plane.
			btVector3 po2;
			if (countAbove >= 2) {
				pbToPa = p[above[1]] - pb;
				po2 = pb - ((pbDistance / plane.dot(pbToPa)) * pbToPa);
			} else {
				int po2Index = above[0] + 1;
				if (po2Index >= 3) {
					po2Index -= 3;
				}
				if (po2Index == below[0]) {
					++po2Index;
					if (po2Index >= 3) {
						po2Index -= 3;
					}
				}
				po2 = p[po2Index];
			}

			tetrahedronSixVolume = btFabs((pb - ref).triple(po1 - ref, po2 - ref));
			volume += tetrahedronSixVolume;
			volumeWeightedBuoyancyCenter += tetrahedronSixVolume * (pb + po1 + po2 + ref);
		} else { // The submerged part is a quadrilateral.
			const btVector3 &pa = p[above[0]], &pb1 = p[below[0]], &pb2 = p[below[1]];
			btScalar paDistance = plane.dot(pa) + plane.getW();

			btVector3 paToPb = pa - pb1;
			btVector3 po1 = pa - ((paDistance / plane.dot(paToPb)) * paToPb);

			paToPb = pa - pb2;
			btVector3 po2 = pa - ((paDistance / plane.dot(paToPb)) * paToPb);

			tetrahedronSixVolume = btFabs((pb1 - ref).triple(po1 - ref, po2 - ref));
			volume += tetrahedronSixVolume;
			volumeWeightedBuoyancyCenter += tetrahedronSixVolume * (pb1 + po1 + po2 + ref);

			tetrahedronSixVolume = btFabs((pb1 - ref).triple(po2 - ref, pb2 - ref));
			volume += tetrahedronSixVolume;
			volumeWeightedBuoyancyCenter += tetrahedronSixVolume * (pb1 + po2 + pb2 + ref);
		}
	}

	volume *= 1.0f / 6.0f;
	volumeWeightedBuoyancyCenter *= 1.0f / 24.0f;

	return true;
}

btScalar CPhysConvex_Hull::GetSubmergedVolume(const btVector4 &plane, btVector3 &volumeWeightedBuoyancyCenter) const {
	btScalar volume;
	const btVector3 &origin = GetOriginInCompound();
	if (!GetConvexTriangleMeshSubmergedVolume(origin, m_Shape.getPoints(), m_Shape.getNumPoints(),
			&m_TriangleIndices[0], m_TriangleIndices.size(), plane, volume, volumeWeightedBuoyancyCenter)) {
		volume = GetVolume();
		volumeWeightedBuoyancyCenter = (origin + GetMassCenter()) * volume;
	}
	return volume;
}

int CPhysConvex_Hull::GetTriangleCount() const {
	return m_TriangleIndices.size() / 3;
}

void CPhysConvex_Hull::GetTriangleVertices(int triangleIndex, btVector3 vertices[3]) const {
	const btVector3 *points = m_Shape.getPoints();
	const unsigned int *indices = &m_TriangleIndices[0];
	int indexIndex = triangleIndex * 3;
	vertices[0] = points[indices[indexIndex]];
	vertices[1] = points[indices[indexIndex + 1]];
	vertices[2] = points[indices[indexIndex + 2]];
}

int CPhysConvex_Hull::GetTriangleMaterialIndex(int triangleIndex) const {
	if (m_TriangleMaterials.size() == 0) {
		return 0;
	}
	return m_TriangleMaterials[triangleIndex];
}

// This is a hack, gives per-plane surface index, not per-triangle,
// as Bullet doesn't do per-triangle collision detection for convexes.
// However, per-triangle materials are used only by world brushes,
// which can't have coplanar triangles with different materials.
int CPhysConvex_Hull::GetTriangleMaterialIndexAtPoint(const btVector3 &point) const {
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
		btScalar planeDot = plane.dot(pointCenterRelative);
		// Without this check, the opposite side of the convex would be treated as forward.
		if (planeDot < 0.0000001f) {
			continue;
		}
		btScalar projectionDistance2 = center.distance2(pointCenterRelative -
				(planeDot + plane.getW() + margin) * plane);
		if (projectionDistance2 < closestProjectionDistance2) {
			closestProjectionDistance2 = projectionDistance2;
			closestTriangle = triangleIndex;
		}
	}
	return m_TriangleMaterials[closestTriangle];
}

void CPhysConvex_Hull::SetTriangleMaterialIndex(int triangleIndex, int index7bits) {
	if (m_TriangleMaterials.size() == 0) {
		if (index7bits == 0) {
			return;
		}
		m_TriangleMaterials.resizeNoInitialize(m_TriangleIndices.size() / 3);
		memset(&m_TriangleMaterials[0], 0, m_TriangleMaterials.size() * sizeof(m_TriangleMaterials[0]));
	}
	CalculateTrianglePlanes();
	m_TriangleMaterials[triangleIndex] = index7bits;
}

void CPhysConvex_Hull::CalculateTrianglePlanes() {
	if (m_TrianglePlanes.size() != 0) {
		return;
	}
	btVector3 aabbMin, aabbMax;
	m_Shape.getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
	btVector3 center = (aabbMin + aabbMax) * 0.5f;
	int triangleCount = m_TriangleIndices.size() / 3;
	m_TrianglePlanes.resizeNoInitialize(triangleCount);
	btVector4 *planes = &m_TrianglePlanes[0];
	const btVector3 *points = &m_Shape.getPoints()[0];
	const unsigned int *indices = &m_TriangleIndices[0];
	for (int triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
		int indexIndex = triangleIndex * 3;
		const btVector3 &v1 = points[indices[indexIndex]];
		const btVector3 &v2 = points[indices[indexIndex + 1]];
		const btVector3 &v3 = points[indices[indexIndex + 2]];
		btVector3 normal = (v2 - v1).cross(v3 - v1);
		normal.normalize();
		// TODO: Check the case when the AABB center is on a triangle.
		// Maybe ensure the windings from all sources are correct:
		// IVP surfaces, HullLibrary, polyhedra and convex polygons.
		btScalar dist = (v1 - center).dot(normal);
		/* if (dist < 0.0f) {
			normal = -normal;
			dist = -dist;
		} */
		planes[triangleIndex].setValue(normal.getX(), normal.getY(), normal.getZ(), -dist);
	}
}

void CPhysConvex_Hull::Release() {
	VPhysicsDelete(CPhysConvex_Hull, this);
}

CPhysConvex *CPhysicsCollision::ConvexFromVerts(Vector **pVerts, int vertCount) {
	btAlignedObjectArray<btVector3> &pointArray = g_pPhysCollision->GetHullCreationPointArray();
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
	btAlignedObjectArray<btVector3> &pointArray = g_pPhysCollision->GetHullCreationPointArray();
	pointArray.resizeNoInitialize(0);
	btGeometryUtil::getVerticesFromPlaneEquations(planeArray, pointArray);
	return CPhysConvex_Hull::CreateFromBulletPoints(m_HullLibrary, &pointArray[0], pointArray.size());
}

CPhysConvex *CPhysicsCollision::ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron) {
	const Vector *verts = ConvexPolyhedron.pVertices;
	int vertCount = ConvexPolyhedron.iVertexCount;
	if (vertCount < 3) {
		return nullptr;
	}
	btAlignedObjectArray<btVector3> &pointArray = g_pPhysCollision->GetHullCreationPointArray();
	pointArray.resizeNoInitialize(vertCount);
	btVector3 *points = &pointArray[0];
	for (int vertIndex = 0; vertIndex < vertCount; ++vertIndex) {
		ConvertPositionToBullet(verts[vertIndex], points[vertIndex]);
	}
	return VPhysicsNew(CPhysConvex_Hull, points, vertCount, ConvexPolyhedron);
}

/***********************************************
 * Bounding boxes (both convex and collideable)
 ***********************************************/

CPhysConvex_Box::CPhysConvex_Box(const btVector3 &halfExtents, const btVector3 &origin) :
		m_Shape(halfExtents), m_Origin(origin) {
	Initialize();
	m_Shape.setMargin(VPHYSICS_CONVEX_DISTANCE_MARGIN);
	// The constructor subtracts the default margin.
	// Assume the margin is outside, just like for convex hulls.
	m_Shape.setImplicitShapeDimensions(halfExtents);
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

btScalar CPhysConvex_Box::GetSubmergedVolume(const btVector4 &plane, btVector3 &volumeWeightedBuoyancyCenter) const {
	btScalar volume;
	const btVector3 &origin = GetOriginInCompound();
	const btVector3 &halfExtents = m_Shape.getHalfExtentsWithoutMargin();
	btVector3 points[8];
	for (int pointIndex = 0; pointIndex < 8; ++pointIndex) {
		points[pointIndex].setValue(
				halfExtents.getX() * ((pointIndex & 4) ? 1.0f : -1.0f),
				halfExtents.getY() * ((pointIndex & 2) ? 1.0f : -1.0f),
 				halfExtents.getZ() * ((pointIndex & 1) ? 1.0f : -1.0f));
	}
	if (!CPhysConvex_Hull::GetConvexTriangleMeshSubmergedVolume(origin, points, 8,
			s_BoxTriangleIndices, 36, plane, volume, volumeWeightedBuoyancyCenter)) {
		volume = GetVolume();
		volumeWeightedBuoyancyCenter = origin * volume;
	}
	return volume;
}

int CPhysConvex_Box::GetTriangleCount() const {
	return 36;
}

void CPhysConvex_Box::GetTriangleVertices(int triangleIndex, btVector3 vertices[3]) const {
	const btVector3 &halfExtents = m_Shape.getHalfExtentsWithoutMargin();
	const btVector3 &origin = GetOriginInCompound();
	int indexIndex = triangleIndex * 3;
	for (int vertexIndex = 0; vertexIndex < 3; ++vertexIndex) {
		unsigned int index = s_BoxTriangleIndices[indexIndex + vertexIndex];
		vertices[vertexIndex].setValue(
				origin.getX() + (halfExtents.getX() * ((index & 4) ? 1.0f : -1.0f)),
				origin.getY() + (halfExtents.getY() * ((index & 2) ? 1.0f : -1.0f)),
 				origin.getZ() + (halfExtents.getZ() * ((index & 1) ? 1.0f : -1.0f)));
	}
}

void CPhysConvex_Box::Release() {
	VPhysicsDelete(CPhysConvex_Box, this);
}

const unsigned int CPhysConvex_Box::s_BoxTriangleIndices[36] = {
	0, 1, 3,
	0, 3, 2,
	4, 5, 1,
	4, 1, 0,
	2, 3, 7,
	2, 7, 6,
	1, 5, 7,
	1, 7, 3,
	4, 0, 2,
	4, 2, 6,
	5, 4, 6,
	5, 6, 7
};

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
		}
		if (cacheCompound != nullptr) {
			return cacheCompound;
		}
	}

	CPhysConvex *box = VPhysicsNew(CPhysConvex_Box, halfExtents, origin);
	box->SetOwner(CPhysConvex::OWNER_INTERNAL);
	CPhysCollide_Compound *compound = VPhysicsNew(CPhysCollide_Compound, &box, 1);
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

void CPhysCollide::ComputeOrthographicAreas(btScalar axisEpsilon) {
	btCollisionShape *shape = GetShape();
	btCollisionObject *collisionObject = g_pPhysCollision->GetTraceCollisionObject();
	collisionObject->setCollisionShape(shape);

	// Fire rays in the following pattern (centers of the sides are always checked):
	//  _________
	// |         |
	// | *  *  * |
	// |         |
	// |         |
	// | *  *  * |
	// |         |
	// |         |
	// | *  *  * |
	// |_________|
	btVector3 aabbMin, aabbMax;
	shape->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
	btVector3 halfRayCounts = ((aabbMax - aabbMin) * 0.5f) / axisEpsilon;
	halfRayCounts[0] = floor(halfRayCounts[0]);
	halfRayCounts[1] = floor(halfRayCounts[1]);
	halfRayCounts[2] = floor(halfRayCounts[2]);
	btVector3 rayOrigins = ((aabbMin + aabbMax) * 0.5f) - (halfRayCounts * axisEpsilon);
	int rayCounts[] = {
		(int) halfRayCounts.getX() * 2 + 1,
		(int) halfRayCounts.getY() * 2 + 1,
		(int) halfRayCounts.getZ() * 2 + 1
	};
	btTransform rayFrom = btTransform::getIdentity(), rayTo = btTransform::getIdentity();
	struct OrthographicAreasResultCallback : public btCollisionWorld::RayResultCallback {
		void ResetOrthographicAreasResult() {
			m_collisionObject = nullptr;
			m_closestHitFraction = 1.0f;
		}
		virtual	btScalar addSingleResult(
				btCollisionWorld::LocalRayResult &rayResult, bool normalInWorldSpace) {
			m_collisionObject = rayResult.m_collisionObject;
			m_closestHitFraction = 0.0f; // Stop tracing once a hit is detected.
			return 0.0f;
		}
	};
	OrthographicAreasResultCallback result;
	int hitCount;
	btVector3 areas;

	rayFrom.getOrigin()[0] = aabbMin.getX() - VPHYSICS_CONVEX_DISTANCE_MARGIN;
	rayTo.getOrigin()[0] = aabbMax.getX() + VPHYSICS_CONVEX_DISTANCE_MARGIN;
	hitCount = 0;
	for (int y = 0; y < rayCounts[1]; ++y) {
		rayFrom.getOrigin()[1] = rayTo.getOrigin()[1] = rayOrigins.getY() + btScalar(y) * axisEpsilon;
		for (int z = 0; z < rayCounts[2]; ++z) {
			rayFrom.getOrigin()[2] = rayTo.getOrigin()[2] = rayOrigins.getZ() + btScalar(z) * axisEpsilon;
			result.ResetOrthographicAreasResult();
			btCollisionWorld::rayTestSingle(rayFrom, rayTo, collisionObject, shape,
					btTransform::getIdentity(), result);
			if (result.m_collisionObject != nullptr) {
				++hitCount;
			}
		}
	}
	areas.setX(btScalar(hitCount) / btScalar(rayCounts[1] * rayCounts[2]));

	rayFrom.getOrigin()[1] = aabbMin.getY() - VPHYSICS_CONVEX_DISTANCE_MARGIN;
	rayTo.getOrigin()[1] = aabbMax.getY() + VPHYSICS_CONVEX_DISTANCE_MARGIN;
	hitCount = 0;
	for (int x = 0; x < rayCounts[0]; ++x) {
		rayFrom.getOrigin()[0] = rayTo.getOrigin()[0] = rayOrigins.getX() + btScalar(x) * axisEpsilon;
		for (int z = 0; z < rayCounts[2]; ++z) {
			rayFrom.getOrigin()[2] = rayTo.getOrigin()[2] = rayOrigins.getZ() + btScalar(z) * axisEpsilon;
			result.ResetOrthographicAreasResult();
			btCollisionWorld::rayTestSingle(rayFrom, rayTo, collisionObject, shape,
					btTransform::getIdentity(), result);
			if (result.m_collisionObject != nullptr) {
				++hitCount;
			}
		}
	}
	areas.setY(btScalar(hitCount) / btScalar(rayCounts[0] * rayCounts[2]));

	rayFrom.getOrigin()[2] = aabbMin.getZ() - VPHYSICS_CONVEX_DISTANCE_MARGIN;
	rayTo.getOrigin()[2] = aabbMax.getZ() + VPHYSICS_CONVEX_DISTANCE_MARGIN;
	hitCount = 0;
	for (int x = 0; x < rayCounts[0]; ++x) {
		rayFrom.getOrigin()[0] = rayTo.getOrigin()[0] = rayOrigins.getX() + btScalar(x) * axisEpsilon;
		for (int y = 0; y < rayCounts[1]; ++y) {
			rayFrom.getOrigin()[1] = rayTo.getOrigin()[1] = rayOrigins.getY() + btScalar(y) * axisEpsilon;
			result.ResetOrthographicAreasResult();
			btCollisionWorld::rayTestSingle(rayFrom, rayTo, collisionObject, shape,
					btTransform::getIdentity(), result);
			if (result.m_collisionObject != nullptr) {
				++hitCount;
			}
		}
	}
	areas.setZ(btScalar(hitCount) / btScalar(rayCounts[0] * rayCounts[1]));

	SetOrthographicAreas(areas);
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

void CPhysicsCollision::DestroyCollide(CPhysCollide *pCollide) {
	if (pCollide->GetOwner() != CPhysCollide::OWNER_GAME) {
		return;
	}
	Assert(pCollide->GetObjectReferenceList() == nullptr);
	if (pCollide->GetObjectReferenceList() != nullptr) {
		DevMsg("Freed collision model while in use!!!\n");
		return;
	}
	pCollide->Release();
	CleanupCompoundConvexDeleteQueue();
}

/*********
 * Traces
 *********/

void CPhysicsCollision::ClearTrace(trace_t *trace) {
	memset(trace, 0, sizeof(*trace));
	trace->fraction = 1.0f;
	trace->surface.name = "**empty**";
}

void CPhysicsCollision::TraceBox(const Ray_t &ray, unsigned int contentsMask,
		IConvexInfo *pConvexInfo, const CPhysCollide *pCollide,
		const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	ClearTrace(ptr);

	// Test shape.
	if (!ray.m_IsRay) {
		btVector3 halfExtents;
		ConvertPositionToBullet(ray.m_Extents, halfExtents);
		m_TraceBoxShape.setImplicitShapeDimensions(halfExtents.absolute());
	}

	// Target shape.
	const btCollisionShape *colObjShape = pCollide->GetShape();
	m_TraceCollisionObject.setCollisionShape(const_cast<btCollisionShape *>(colObjShape));
	btTransform colObjWorldTransform;
	ConvertRotationToBullet(collideAngles, colObjWorldTransform.getBasis());
	ConvertPositionToBullet(collideOrigin - ray.m_Start, colObjWorldTransform.getOrigin());
	colObjWorldTransform.getOrigin() += colObjWorldTransform.getBasis() * pCollide->GetMassCenter();
	TraceContentsFilter contentsFilter(pConvexInfo, contentsMask, colObjShape);

	// Ray (for simplicity and precision, starting at zero).
	btTransform rayToTransform;
	ConvertPositionToBullet(ray.m_Delta, rayToTransform.getOrigin()); // Basis not needed yet.
	btScalar rayLength2 = rayToTransform.getOrigin().length2();
	bool isSwept = (rayLength2 > 1e-6f);

	// Some defaults, unknown hit normal will be handled later.
	btVector3 hitNormal(0.0f, 0.0f, 0.0f);
	btVector3 hitPoint = rayToTransform.getOrigin();

	// First, try contact test because ray and convex tests don't report starting in a solid.
	if (ray.m_IsRay) {
		m_ContactTestCollisionObject.setCollisionShape(&m_TracePointShape);
	} else {
		m_ContactTestCollisionObject.setCollisionShape(&m_TraceBoxShape);
	}
	m_ContactTestCollisionObject.setWorldTransform(btTransform::getIdentity());
	m_TraceCollisionObject.setWorldTransform(colObjWorldTransform);
	ContactTestResultCallback contactTestResult(&m_ContactTestCollisionObject, &contentsFilter);
	m_InContactTest = true;
	m_ContactTestCollisionWorld->contactPairTest(&m_ContactTestCollisionObject, &m_TraceCollisionObject, contactTestResult);
	m_InContactTest = false;

	if (contactTestResult.m_Hit) {
		ptr->fraction = 0.0f;
		ptr->startsolid = ptr->allsolid = true;
		if (ray.m_IsRay) {
			hitPoint.setZero();
		} else {
			hitNormal = contactTestResult.m_ShallowestHitNormal;
			hitPoint = contactTestResult.m_ShallowestHitPoint;
		}
		ptr->contents = contactTestResult.m_ShallowestHitContents;
	} else if (isSwept) {
		// Not starting in a solid and need to cast a ray/convex.
		rayToTransform.getBasis().setIdentity();
		if (ray.m_IsRay) {
			RayTestResultCallback rayTestResult(&contentsFilter, colObjWorldTransform.getBasis());
			btCollisionWorld::rayTestSingle(btTransform::getIdentity(), rayToTransform,
					&m_TraceCollisionObject, colObjShape, colObjWorldTransform, rayTestResult);
			if (rayTestResult.m_collisionObject != nullptr) {
				ptr->fraction = rayTestResult.m_closestHitFraction;
				hitNormal = rayTestResult.m_ClosestHitNormal;
				hitPoint = rayToTransform.getOrigin() * rayTestResult.m_closestHitFraction;
				ptr->contents = rayTestResult.m_ClosestHitContents;
			}
		} else {
			ConvexTestResultCallback convexTestResult(&contentsFilter, colObjWorldTransform.getBasis());
			btCollisionWorld::objectQuerySingle(
					&m_TraceBoxShape, btTransform::getIdentity(), rayToTransform,
					&m_TraceCollisionObject, colObjShape, colObjWorldTransform, convexTestResult,
					2.0f * VPHYSICS_CONVEX_DISTANCE_MARGIN);
			if (convexTestResult.m_HitCollisionObject != nullptr) {
				ptr->fraction = convexTestResult.m_closestHitFraction;
				hitNormal = convexTestResult.m_ClosestHitNormal;
				hitPoint = convexTestResult.m_ClosestHitPoint;
				ptr->contents = convexTestResult.m_ClosestHitContents;
			}
		}
	}

	// If nothing was hit or a ray trace started in a solid, fake the normal.
	if (hitNormal.isZero()) {
		if (isSwept) {
			hitNormal = rayToTransform.getOrigin() / -btSqrt(rayLength2);
		} else {
			hitNormal.setValue(-1.0f, 0.0f, 0.0f);
		}
	}

	// Write the common data.
	VectorAdd(ray.m_Start, ray.m_StartOffset, ptr->startpos);
	VectorMA(ptr->startpos, ptr->fraction, ray.m_Delta, ptr->endpos);
	ConvertDirectionToHL(hitNormal, ptr->plane.normal);
	Vector hitPointHL;
	ConvertPositionToHL(hitPoint, hitPointHL);
	ptr->plane.dist = DotProduct(hitPointHL + ray.m_Start, ptr->plane.normal);
}

void CPhysicsCollision::TraceBox(const Vector &start, const Vector &end,
		const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide,
		const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	Ray_t ray;
	ray.Init(start, end, mins, maxs);
	TraceBox(ray, MASK_ALL, nullptr, pCollide, collideOrigin, collideAngles, ptr);
}

void CPhysicsCollision::TraceBox(const Ray_t &ray, const CPhysCollide *pCollide,
		const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	TraceBox(ray, MASK_ALL, nullptr, pCollide, collideOrigin, collideAngles, ptr);
}

void CPhysicsCollision::TraceCollide(const Vector &start, const Vector &end,
		const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide,
		const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	ClearTrace(ptr);
	const btCollisionShape *testShape = pSweepCollide->GetShape();
	Assert(testShape->isCompound() || testShape->isConvex());
	if (!testShape->isCompound() && !testShape->isConvex()) {
		return;
	}

	// Target shape.
	const btCollisionShape *colObjShape = pCollide->GetShape();
	m_TraceCollisionObject.setCollisionShape(const_cast<btCollisionShape *>(colObjShape));
	btTransform colObjWorldTransform;
	ConvertRotationToBullet(collideAngles, colObjWorldTransform.getBasis());
	ConvertPositionToBullet(collideOrigin - start, colObjWorldTransform.getOrigin());
	colObjWorldTransform.getOrigin() += colObjWorldTransform.getBasis() * pCollide->GetMassCenter();

	// Ray (for simplicity and precision, starting at zero).
	btTransform rayFromTransform;
	ConvertRotationToBullet(sweepAngles, rayFromTransform.getBasis()); // Origin not needed yet.
	btVector3 rayMassCenterOffset = rayFromTransform.getBasis() * pSweepCollide->GetMassCenter();
	btVector3 rayDelta;
	ConvertPositionToBullet(end - start, rayDelta);
	btScalar rayLength2 = rayDelta.length2();
	bool isSwept = (rayLength2 > 1e-6f);

	// Some defaults.
	btVector3 hitNormal(0.0f, 0.0f, 0.0f);
	btVector3 hitPoint = rayMassCenterOffset + rayDelta;

	// Contact test.
	m_ContactTestCollisionObject.setCollisionShape(const_cast<btCollisionShape *>(testShape));
	rayFromTransform.setOrigin(rayMassCenterOffset);
	m_ContactTestCollisionObject.setWorldTransform(rayFromTransform);
	m_TraceCollisionObject.setWorldTransform(colObjWorldTransform);
	ContactTestResultCallback contactTestResult(&m_ContactTestCollisionObject, nullptr);
	m_InContactTest = true;
	m_ContactTestCollisionWorld->contactPairTest(&m_ContactTestCollisionObject, &m_TraceCollisionObject, contactTestResult);
	m_InContactTest = false;

	if (contactTestResult.m_Hit) {
		ptr->fraction = 0.0f;
		ptr->startsolid = ptr->allsolid = true;
		hitNormal = contactTestResult.m_ShallowestHitNormal;
		hitPoint = contactTestResult.m_ShallowestHitPoint;
		ptr->contents = CONTENTS_SOLID;
	} else if (isSwept) {
		// Not starting in a solid, need to sweep.
		ConvexTestResultCallback convexTestResult(nullptr, colObjWorldTransform.getBasis());
		btTransform rayToTransform;
		rayToTransform.setBasis(rayFromTransform.getBasis());
		if (testShape->isCompound()) {
			btVector3 childRayDelta = rayDelta;
			const btCompoundShape *testCompoundShape = static_cast<const btCompoundShape *>(testShape);
			int childCount = testCompoundShape->getNumChildShapes();
			for (int childIndex = 0; childIndex < childCount; ++childIndex) {
				rayFromTransform.setOrigin(rayMassCenterOffset + (rayFromTransform.getBasis() *
						testCompoundShape->getChildTransform(childIndex).getOrigin()));
				rayToTransform.setOrigin(rayFromTransform.getOrigin() + childRayDelta);
				convexTestResult.Reset();
				btCollisionWorld::objectQuerySingle(
						static_cast<const btConvexShape *>(testCompoundShape->getChildShape(childIndex)),
						rayFromTransform, rayToTransform,
						&m_TraceCollisionObject, colObjShape, colObjWorldTransform,
						convexTestResult, 2.0f * VPHYSICS_CONVEX_DISTANCE_MARGIN);
				if (convexTestResult.m_HitCollisionObject != nullptr) {
					ptr->fraction *= convexTestResult.m_closestHitFraction;
					childRayDelta *= convexTestResult.m_closestHitFraction;
					hitNormal = convexTestResult.m_ClosestHitNormal;
					hitPoint = convexTestResult.m_ClosestHitPoint;
					ptr->contents = CONTENTS_SOLID;
				}
			}
		} else {
			// rayFromTransform configured by contact test already.
			rayToTransform.setOrigin(rayFromTransform.getOrigin() + rayDelta);
			btCollisionWorld::objectQuerySingle(
					static_cast<const btConvexShape *>(testShape), rayFromTransform, rayToTransform,
					&m_TraceCollisionObject, colObjShape, colObjWorldTransform,
					convexTestResult, 2.0f * VPHYSICS_CONVEX_DISTANCE_MARGIN);
			if (convexTestResult.m_HitCollisionObject != nullptr) {
				ptr->fraction = convexTestResult.m_closestHitFraction;
				hitNormal = convexTestResult.m_ClosestHitNormal;
				hitPoint = convexTestResult.m_ClosestHitPoint;
				ptr->contents = CONTENTS_SOLID;
			}
		}
	}

	// If nothing was hit or a ray trace started in a solid, fake the normal.
	if (hitNormal.isZero()) {
		if (isSwept) {
			hitNormal = rayDelta / -btSqrt(rayLength2);
		} else {
			hitNormal.setValue(-1.0f, 0.0f, 0.0f);
		}
	}

	// Write the data.
	ptr->startpos = start;
	VectorMA(ptr->startpos, ptr->fraction, end - start, ptr->endpos);
	ConvertDirectionToHL(hitNormal, ptr->plane.normal);
	Vector hitPointHL;
	ConvertPositionToHL(hitPoint, hitPointHL);
	ptr->plane.dist = DotProduct(hitPointHL + start, ptr->plane.normal);
}

bool CPhysicsCollision::IsBoxIntersectingCone(
		const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone) {
	btVector3 boxHalfExtents;
	ConvertPositionToBullet((boxAbsMaxs - boxAbsMins) * 0.5f, boxHalfExtents);
	m_TraceBoxShape.setImplicitShapeDimensions(boxHalfExtents.absolute());
	m_ContactTestCollisionObject.setCollisionShape(&m_TraceBoxShape);
	m_ContactTestCollisionObject.setWorldTransform(btTransform::getIdentity());

	// TODO: Verify the origin and the direction of the cone.
	// Assuming the origin is the apex and the normal is pointing to the base.
	// In Bullet, the origin of a cone is its center (at mid-height) and the apex is positive.
	matrix3x4_t coneMatrix;
	VectorMatrix(-cone.normal, coneMatrix);
	MatrixSetColumn(cone.origin - ((boxAbsMins + boxAbsMaxs) * 0.5f) + (cone.normal * (cone.h * 0.5f)), 3, coneMatrix);
	btTransform coneTransform;
	ConvertMatrixToBullet(coneMatrix, coneTransform);
	btScalar coneHeight = HL2BULLET(cone.h);
	m_TraceConeShape.setHeight(coneHeight);
	m_TraceConeShape.setRadius(coneHeight * btTan(DEG2RAD(cone.theta)));
	m_TraceCollisionObject.setCollisionShape(&m_TraceConeShape);
	m_TraceCollisionObject.setWorldTransform(coneTransform);

	struct ConeContactTestResultCallback : public btCollisionWorld::ContactResultCallback {
		bool m_Hit;
		ConeContactTestResultCallback() : m_Hit(false) {}
		btScalar addSingleResult(btManifoldPoint &cp,
				const btCollisionObjectWrapper *colObj0Wrap, int partId0, int index0,
				const btCollisionObjectWrapper *colObj1Wrap, int partId1, int index1) {
			if (cp.getDistance() < -2.0f * VPHYSICS_CONVEX_DISTANCE_MARGIN) {
				m_Hit = true;
			}
			return 0.0f;
		}
	};
	ConeContactTestResultCallback contactTestResult;
	m_InContactTest = true;
	m_ContactTestCollisionWorld->contactPairTest(&m_ContactTestCollisionObject, &m_TraceCollisionObject, contactTestResult);
	m_InContactTest = false;
	return contactTestResult.m_Hit;
}

/******************
 * Compound shapes
 ******************/

CPhysCollide_Compound::CPhysCollide_Compound(CPhysConvex **pConvex, int convexCount) :
		m_Shape(false, convexCount) {
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
	if (convexCount > 1) {
		m_Shape.createAabbTreeFromChildren();
	}

	CalculateInertia();
}

CPhysCollide_Compound::CPhysCollide_Compound(
		const VCollide_IVP_Compact_Ledgetree_Node *root, CByteswap &byteswap,
		const btVector3 &massCenter, const btVector3 &inertia,
		const btVector3 &orthographicAreas) :
		CPhysCollide(orthographicAreas),
		m_Shape(false),
		m_Volume(-1.0f), m_MassCenter(massCenter), m_Inertia(inertia) {
	Initialize();
	g_pPhysCollision->PushIVPNode(root);
	const VCollide_IVP_Compact_Ledgetree_Node *node;
	while ((node = g_pPhysCollision->PopIVPNode()) != nullptr) {
		VCollide_IVP_Compact_Ledgetree_Node swappedNode;
		byteswap.SwapBufferToTargetEndian(&swappedNode, const_cast<VCollide_IVP_Compact_Ledgetree_Node *>(node));
		if (swappedNode.offset_right_node == 0) {
			CPhysConvex_Hull *convex = g_pPhysCollision->CreateConvexHullFromIVPCompactLedge(
					reinterpret_cast<const VCollide_IVP_Compact_Ledge *>(
							reinterpret_cast<const byte *>(node) + swappedNode.offset_compact_ledge), byteswap);
			convex->SetOwner(CPhysConvex::OWNER_COMPOUND);
			m_Shape.addChildShape(btTransform(btMatrix3x3::getIdentity(),
					convex->GetOriginInCompound() - m_MassCenter), convex->GetShape());
		} else {
			g_pPhysCollision->PushIVPNode(node + 1);
			g_pPhysCollision->PushIVPNode(reinterpret_cast<const VCollide_IVP_Compact_Ledgetree_Node *>(
					reinterpret_cast<const byte *>(node) + swappedNode.offset_right_node));
		}
	}
	if (m_Shape.getNumChildShapes() > 1) {
		m_Shape.createAabbTreeFromChildren();
	}
}

CPhysCollide *CPhysicsCollision::ConvertConvexToCollide(CPhysConvex **pConvex, int convexCount) {
	convertconvexparams_t convertParams;
	convertParams.Defaults();
	return ConvertConvexToCollideParams(pConvex, convexCount, convertParams);
}

CPhysCollide *CPhysicsCollision::ConvertConvexToCollideParams(CPhysConvex **pConvex, int convexCount,
		const convertconvexparams_t &convertParams) {
	if (convexCount == 0 || pConvex == nullptr) {
		return nullptr;
	}
	CPhysCollide_Compound *collide = VPhysicsNew(CPhysCollide_Compound, pConvex, convexCount);
	if (convertParams.buildDragAxisAreas) {
		collide->ComputeOrthographicAreas(HL2BULLET(sqrtf(MAX(convertParams.dragAreaEpsilon, 0.25f))));
	}
	return collide;
}

CPhysPolysoup *CPhysicsCollision::PolysoupCreate() {
	return VPhysicsNew(CPhysPolysoup);
}

CPhysPolysoup::~CPhysPolysoup() {
	int convexCount = m_Convexes.Count();
	for (int convexIndex = 0; convexIndex < convexCount; ++convexIndex) {
		g_pPhysCollision->ConvexFree(m_Convexes[convexIndex]);
	}
}

void CPhysicsCollision::PolysoupDestroy(CPhysPolysoup *pSoup) {
	VPhysicsDelete(CPhysPolysoup, pSoup);
}

void CPhysPolysoup::AddTriangle(HullLibrary &hullLibrary,
		const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits) {
	btVector3 points[3];
	ConvertPositionToBullet(a, points[0]);
	ConvertPositionToBullet(b, points[1]);
	ConvertPositionToBullet(c, points[2]);
	CPhysConvex_Hull *convex = CPhysConvex_Hull::CreateFromBulletPoints(hullLibrary, points, 3);
	if (convex == nullptr) {
		Warning("Degenerate Triangle\n(%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f)\n",
				a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
		return;
	}
	convex->SetTriangleMaterialIndex(0, materialIndex7bits);
	m_Convexes.AddToTail(convex);
}

void CPhysicsCollision::PolysoupAddTriangle(CPhysPolysoup *pSoup,
		const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits) {
	pSoup->AddTriangle(m_HullLibrary, a, b, c, materialIndex7bits);
}

CPhysCollide *CPhysPolysoup::ConvertToCollide() {
	int convexCount = m_Convexes.Count();
	if (convexCount == 0) {
		return nullptr;
	}
	CPhysCollide *collide = VPhysicsNew(CPhysCollide_Compound, &m_Convexes[0], convexCount);
	m_Convexes.RemoveAll();
	return collide;
}

CPhysCollide *CPhysicsCollision::ConvertPolysoupToCollide(CPhysPolysoup *pSoup, bool useMOPP) {
	return pSoup->ConvertToCollide();
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
	if (GetObjectReferenceList() != nullptr) {
		DevMsg("Changed collide mass center while in use!!!\n");
		return;
	}
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

btScalar CPhysCollide_Compound::GetSubmergedVolume(const btVector4 &plane, btVector3 &buoyancyCenter) const {
	btScalar volume = 0.0f;
	btVector3 volumeWeightedBuoyancyCenter;
	buoyancyCenter.setZero();
	int childCount = m_Shape.getNumChildShapes();
	for (int childIndex = 0; childIndex < childCount; ++childIndex) {
		volume += reinterpret_cast<const CPhysConvex *>(
				m_Shape.getChildShape(childIndex)->getUserPointer())->GetSubmergedVolume(
						plane, volumeWeightedBuoyancyCenter);
		buoyancyCenter += volumeWeightedBuoyancyCenter;
	}
	if (volume > 0.0f) {
		buoyancyCenter /= volume;
	}
	return volume;
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

CCollisionQuery::CCollisionQuery(CPhysCollide *collide) {
	if (CPhysCollide_Compound::IsCompound(collide)) {
		m_CompoundShape = static_cast<CPhysCollide_Compound *>(collide)->GetCompoundShape();
	} else {
		m_CompoundShape = nullptr;
	}
}

int CCollisionQuery::ConvexCount() {
	if (m_CompoundShape == nullptr) {
		return 0;
	}
	return m_CompoundShape->getNumChildShapes();
}

int CCollisionQuery::TriangleCount(int convexIndex) {
	const CPhysConvex *convex = GetConvex(convexIndex);
	if (convex == nullptr) {
		return 0;
	}
	return convex->GetTriangleCount();
}

unsigned int CCollisionQuery::GetGameData(int convexIndex) {
	if (convexIndex < 0 || convexIndex >= ConvexCount()) {
		return 0;
	}
	return (unsigned int) m_CompoundShape->getChildShape(convexIndex)->getUserIndex();
}

void CCollisionQuery::GetTriangleVerts(int convexIndex, int triangleIndex, Vector *verts) {
	const CPhysConvex *convex = GetConvex(convexIndex);
	if (convex == nullptr || triangleIndex < 0 || triangleIndex >= convex->GetTriangleCount()) {
		verts[0].Zero();
		verts[1].Zero();
		verts[2].Zero();
		return;
	}
	btVector3 vertices[3];
	convex->GetTriangleVertices(triangleIndex, vertices);
	const btVector3 &origin = convex->GetOriginInCompound();
	ConvertPositionToHL(vertices[0] + origin, verts[0]);
	ConvertPositionToHL(vertices[1] + origin, verts[1]);
	ConvertPositionToHL(vertices[2] + origin, verts[2]);
}

void CCollisionQuery::SetTriangleVerts(int convexIndex, int triangleIndex, const Vector *verts) {
	// Not implemented in IVP VPhysics.
}

int CCollisionQuery::GetTriangleMaterialIndex(int convexIndex, int triangleIndex) {
	const CPhysConvex *convex = GetConvex(convexIndex);
	if (convex == nullptr || triangleIndex < 0 || triangleIndex >= convex->GetTriangleCount()) {
		return 0;
	}
	return convex->GetTriangleMaterialIndex(triangleIndex);
}

void CCollisionQuery::SetTriangleMaterialIndex(int convexIndex, int triangleIndex, int index7bits) {
	CPhysConvex *convex = GetConvex(convexIndex);
	if (convex == nullptr || triangleIndex < 0 || triangleIndex >= convex->GetTriangleCount()) {
		return;
	}
	convex->SetTriangleMaterialIndex(triangleIndex, index7bits);
}

ICollisionQuery *CPhysicsCollision::CreateQueryModel(CPhysCollide *pCollide) {
	return VPhysicsNew(CCollisionQuery, pCollide);
}

void CPhysicsCollision::DestroyQueryModel(ICollisionQuery *pQuery) {
	VPhysicsDelete(CCollisionQuery, pQuery);
}

CPhysCollide_Compound::~CPhysCollide_Compound() {
	int childCount = m_Shape.getNumChildShapes();
	for (int childIndex = 0; childIndex < childCount; ++childIndex) {
		g_pPhysCollision->AddCompoundConvexToDeleteQueue(
				reinterpret_cast<CPhysConvex *>(m_Shape.getChildShape(childIndex)->getUserPointer()));
	}
}

void CPhysCollide_Compound::Release() {
	VPhysicsDelete(CPhysCollide_Compound, this);
}

void CPhysicsCollision::AddCompoundConvexToDeleteQueue(CPhysConvex *convex) {
	if (convex->GetOwner() == CPhysConvex::OWNER_COMPOUND) {
		m_CompoundConvexDeleteQueue.AddToTail(convex);
	}
}

void CPhysicsCollision::CleanupCompoundConvexDeleteQueue() {
	for (int convexIndex = m_CompoundConvexDeleteQueue.Size() - 1; convexIndex >= 0; --convexIndex) {
		m_CompoundConvexDeleteQueue[convexIndex]->Release();
		m_CompoundConvexDeleteQueue.FastRemove(convexIndex);
	}
}

/**********
 * Spheres
 **********/

CPhysCollide_Sphere *CPhysicsCollision::CreateCachedSphereCollide(btScalar radius) {
	const btScalar radiusThreshold = HL2BULLET(0.1f);
	CPhysCollide_Sphere *collide = nullptr, *freeCollide = nullptr;
	for (int cacheIndex = m_SphereCache.Count() - 1; cacheIndex >= 0; --cacheIndex) {
		CPhysCollide_Sphere *cacheCollide = m_SphereCache[cacheIndex];
		if (btFabs(cacheCollide->GetRadius() - radius) < radiusThreshold) {
			collide = cacheCollide;
			break;
		}
		if (freeCollide == nullptr && cacheCollide->GetObjectReferenceList() == nullptr) {
			freeCollide = cacheCollide;
		}
	}
	if (collide == nullptr) {
		if (freeCollide != nullptr) {
			freeCollide->SetRadius(radius);
			collide = freeCollide;
		} else {
			collide = VPhysicsNew(CPhysCollide_Sphere, radius);
			collide->SetOwner(CPhysCollide::OWNER_INTERNAL);
			m_SphereCache.AddToTail(collide);
		}
	}
	return collide;
}

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

btScalar CPhysCollide_Sphere::GetSubmergedVolume(const btVector4 &plane, btVector3 &buoyancyCenter) const {
	btScalar radius = GetRadius();
	btScalar distance = -plane.getW();
	btScalar capHeight = radius - btFabs(distance);
	if (capHeight < HL2BULLET(VP_EPSILON)) {
		buoyancyCenter.setZero();
		if (distance > 0.0f) {
			return GetVolume();
		}
		return 0.0f;
	}
	if (distance > 0.0f) {
		capHeight += radius;
	}
	btScalar twoRadiiMinusHeight = 2.0f * radius - capHeight;
	btScalar threeRadiiMinusHeight = 3.0f * radius - capHeight;
	buoyancyCenter = (-0.75f * (twoRadiiMinusHeight * twoRadiiMinusHeight /
			threeRadiiMinusHeight)) * static_cast<const btVector3 &>(plane);
	return (SIMD_PI / 3.0f) * capHeight * capHeight * threeRadiiMinusHeight;
}

void CPhysCollide_Sphere::ComputeOrthographicAreas(btScalar axisEpsilon) {
	SetOrthographicAreas(btVector3(0.25f * SIMD_PI, 0.25f * SIMD_PI, 0.25f * SIMD_PI));
}

void CPhysCollide_Sphere::Release() {
	VPhysicsDelete(CPhysCollide_Sphere, this);
}

/**************************
 * Concave triangle meshes
 **************************/

CPhysCollide_TriangleMesh::CPhysCollide_TriangleMesh(const virtualmeshlist_t &virtualMesh) :
		m_MeshInterface(virtualMesh), m_Shape(&m_MeshInterface, true),
		m_SurfacePropsIndex(virtualMesh.surfacePropsIndex) {
	Initialize();
	m_Shape.setMargin(VPHYSICS_CONVEX_DISTANCE_MARGIN);
}

CPhysCollide_TriangleMesh::MeshInterface::MeshInterface(const virtualmeshlist_t &virtualMesh) {
	m_Vertices.resizeNoInitialize(virtualMesh.vertexCount);
	btVector3 *bulletVertices = &m_Vertices[0];
	for (int vertexIndex = 0; vertexIndex < virtualMesh.vertexCount; ++vertexIndex) {
		ConvertPositionToBullet(virtualMesh.pVerts[vertexIndex], bulletVertices[vertexIndex]);
	}
	static_assert(sizeof(virtualMesh.indices[0]) == sizeof(m_Indices[0]),
			"Virtual mesh indices are stored in a different type.");
	m_Indices.resizeNoInitialize(virtualMesh.indexCount);
	memcpy(&m_Indices[0], virtualMesh.indices, virtualMesh.indexCount * sizeof(virtualMesh.indices[0]));
}

void CPhysCollide_TriangleMesh::MeshInterface::getLockedReadOnlyVertexIndexBase(
		const unsigned char **vertexbase, int &numverts, PHY_ScalarType &type, int &stride,
		const unsigned char **indexbase, int &indexstride, int &numfaces, PHY_ScalarType &indicestype, int subpart) const {
	Assert(subpart == 0);
	*vertexbase = reinterpret_cast<const unsigned char *>(&m_Vertices[0][0]);
	numverts = m_Vertices.size();
#ifdef BT_USE_DOUBLE_PRECISION
	type = PHY_DOUBLE;
#else
	type = PHY_FLOAT;
#endif
	stride = sizeof(btVector3);
	*indexbase = reinterpret_cast<const unsigned char *>(&m_Indices[0]);
	indexstride = 3 * sizeof(unsigned short);
	numfaces = m_Indices.size() / 3;
	indicestype = PHY_SHORT;
}

btScalar CPhysCollide_TriangleMesh::GetSurfaceArea() const {
	const btVector3 *points = &m_MeshInterface.m_Vertices[0];
	const unsigned short *indices = &m_MeshInterface.m_Indices[0];
	int indexCount = m_MeshInterface.m_Indices.size();
	btScalar area = 0.0f;
	for (int indexIndex = 0; indexIndex < indexCount; indexIndex += 3) {
		const btVector3 &p0 = points[indices[indexIndex]];
		const btVector3 &p1 = points[indices[indexIndex + 1]];
		const btVector3 &p2 = points[indices[indexIndex + 2]];
		area += (p1 - p0).cross(p2 - p0).length();
	}
	return 0.5f * area;
}

void CPhysCollide_TriangleMesh::Release() {
	VPhysicsDelete(CPhysCollide_TriangleMesh, this);
}

CPhysCollide *CPhysicsCollision::CreateVirtualMesh(const virtualmeshparams_t &params) {
	if (params.pMeshEventHandler == nullptr) {
		return nullptr;
	}
	virtualmeshlist_t virtualMesh;
	params.pMeshEventHandler->GetVirtualMesh(params.userData, &virtualMesh);
	return VPhysicsNew(CPhysCollide_TriangleMesh, virtualMesh);
}

bool CPhysicsCollision::SupportsVirtualMesh() {
	return true;
}

/****************************
 * Collideable serialization
 ****************************/

CPhysCollide *CPhysicsCollision::UnserializeIVPCompactSurface(
		const VCollide_IVP_Compact_Surface *surface, CByteswap &byteswap,
		const btVector3 &orthographicAreas) {
	VCollide_IVP_Compact_Surface swappedSurface;
	byteswap.SwapBufferToTargetEndian(&swappedSurface, const_cast<VCollide_IVP_Compact_Surface *>(surface));
	if (swappedSurface.dummy[2] != VCOLLIDE_IVP_COMPACT_SURFACE_ID) {
		return nullptr;
	}
	return VPhysicsNew(CPhysCollide_Compound,
			reinterpret_cast<const VCollide_IVP_Compact_Ledgetree_Node *>(
					reinterpret_cast<const byte *>(surface) + swappedSurface.offset_ledgetree_root),
			byteswap,
			btVector3(swappedSurface.mass_center[0], -swappedSurface.mass_center[1], -swappedSurface.mass_center[2]),
			btVector3(swappedSurface.rotation_inertia[0], swappedSurface.rotation_inertia[1], swappedSurface.rotation_inertia[2]),
			orthographicAreas);
}

CPhysCollide *CPhysicsCollision::UnserializeCollideFromBuffer(
		const char *pBuffer, int size, int index, bool swap) {
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
	return UnserializeCollideFromBuffer(pBuffer, size, index, false);
}

void CPhysicsCollision::VCollideLoad(vcollide_t *pOutput,
			int solidCount, const char *pBuffer, int size, bool swap) {
	memset(pOutput, 0, sizeof(*pOutput));
	pOutput->solidCount = solidCount;
	pOutput->solids = new CPhysCollide *[solidCount]; // Safe.
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
		pOutput->solids[solidIndex] = UnserializeCollideFromBuffer(
				pBuffer + position, solidSize, solidIndex, swap);
		position += solidSize;
	}
	int keySize = size - position;
	pOutput->pKeyValues = new char[keySize]; // Safe.
	memcpy(pOutput->pKeyValues, pBuffer + position, keySize);
}

void CPhysicsCollision::VCollideUnload(vcollide_t *pVCollide) {
	for (int solidIndex = 0; solidIndex < pVCollide->solidCount; ++solidIndex) {
		CPhysCollide *solid = pVCollide->solids[solidIndex];
		if (solid == nullptr) {
			continue;
		}
		Assert(solid->GetOwner() == CPhysCollide::OWNER_GAME);
		Assert(solid->GetObjectReferenceList() == nullptr);
		if (solid->GetObjectReferenceList() != nullptr) {
			DevMsg("Freed collision model while in use!!!\n");
			return;
		}
	}
	for (int solidIndex = 0; solidIndex < pVCollide->solidCount; ++solidIndex) {
		CPhysCollide *solid = pVCollide->solids[solidIndex];
		if (solid != nullptr) {
			solid->Release();
		}
		CleanupCompoundConvexDeleteQueue();
	}
	delete[] pVCollide->solids; // Safe.
	delete[] pVCollide->pKeyValues; // Safe.
	memset(pVCollide, 0, sizeof(*pVCollide));
}

IVPhysicsKeyParser *CPhysicsCollision::VPhysicsKeyParserCreate(const char *pKeyData) {
	return VPhysicsNew(CVPhysicsKeyParser, pKeyData);
}

void CPhysicsCollision::VPhysicsKeyParserDestroy(IVPhysicsKeyParser *pParser) {
	VPhysicsDelete(CVPhysicsKeyParser, pParser);
}
