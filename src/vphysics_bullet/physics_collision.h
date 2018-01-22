// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_COLLISION_H
#define PHYSICS_COLLISION_H

#include "physics_internal.h"
#include <LinearMath/btConvexHull.h>
#include "tier1/byteswap.h"
#include "tier1/utlvector.h"

#define VPHYSICS_CONVEX_DISTANCE_MARGIN HL2BULLET(0.25f)

/************************
 * Convex shape wrappers
 ************************/

class CPhysConvex {
public:
	enum Owner {
		OWNER_GAME, // Created and by the game, not added to a compound collideable yet.
		OWNER_COMPOUND, // Part of a compound created by the game, destroyed with the compound.
		OWNER_INTERNAL // Managed internally by physics.
	};

	FORCEINLINE Owner GetOwner() const { return m_Owner; }
	FORCEINLINE void SetOwner(Owner owner) { m_Owner = owner; }

	virtual btCollisionShape *GetShape() = 0;
	virtual const btCollisionShape *GetShape() const = 0;

	virtual btScalar GetVolume() const { return 0.0f; }
	virtual btScalar GetSurfaceArea() const { return 0.0f; }
	virtual btVector3 GetMassCenter() const { return btVector3(0.0f, 0.0f, 0.0f); }
	virtual btVector3 GetInertia() const { return btVector3(1.0f, 1.0f, 1.0f); }

	virtual btVector3 GetOriginInCompound() const { return btVector3(0.0f, 0.0f, 0.0f); }

protected:
	CPhysConvex() : m_Owner(OWNER_GAME) {}

	virtual void Initialize();

private:
	Owner m_Owner;
};

class CPhysConvex_Hull : public CPhysConvex {
public:
	CPhysConvex_Hull(const btVector3 *points, int pointCount,
			const unsigned int *indices, int triangleCount);
	static CPhysConvex_Hull *CreateFromBulletPoints(
			HullLibrary &hullLibrary, const btVector3 *points, int pointCount);
	static CPhysConvex_Hull *CreateFromIVPCompactLedge(
			const struct VCollide_IVP_Compact_Ledge *ledge, CByteswap &byteswap);

	btCollisionShape *GetShape() { return &m_Shape; }
	const btCollisionShape *GetShape() const { return &m_Shape; }
	FORCEINLINE btConvexHullShape *GetConvexHullShape() { return &m_Shape; }
	FORCEINLINE const btConvexHullShape *GetConvexHullShape() const { return &m_Shape; }
	inline static bool IsHull(const CPhysConvex *convex) {
		return convex->GetShape()->getShapeType() == CONVEX_HULL_SHAPE_PROXYTYPE;
	}

	// For IVP ledges, first calls to these will calculate the values.
	virtual btScalar GetVolume() const;
	virtual btScalar GetSurfaceArea() const; // Slow, only needed for tools.
	virtual btVector3 GetMassCenter() const;
	virtual btVector3 GetInertia() const;

	FORCEINLINE bool HasPerTriangleMaterials() const {
		return m_TriangleMaterials.size() > 0;
	}
	// Returns an unremapped surface index, or 0 if no triangle-specific material.
	// Doesn't remap just in case the non-zero indices are mapped to 0.
	int GetTriangleMaterialIndex(const btVector3 &point) const;

protected:
	virtual void Initialize();

private:
	CPhysConvex_Hull(const struct VCollide_IVP_Compact_Ledge *ledge, CByteswap &byteswap,
			const btVector3 *ledgePoints, int ledgePointCount);

	btConvexHullShape m_Shape;

	btAlignedObjectArray<unsigned int> m_TriangleIndices;

	void CalculateTrianglePlanes();
	// For per-triangle materials.
	btAlignedObjectArray<btVector4> m_TrianglePlanes;
	// These are not remapped, as material table may be loaded after the collide.
	btAlignedObjectArray<unsigned char> m_TriangleMaterials;

	void CalculateVolumeProperties();
	btScalar m_Volume;
	btVector3 m_MassCenter;
	btVector3 m_Inertia;
};

class CPhysConvex_Box : public CPhysConvex {
public:
	CPhysConvex_Box(const btVector3 &halfExtents, const btVector3 &origin);

	btCollisionShape *GetShape() { return &m_Shape; }
	const btCollisionShape *GetShape() const { return &m_Shape; }
	FORCEINLINE btBoxShape *GetBoxShape() { return &m_Shape; }
	FORCEINLINE const btBoxShape *GetBoxShape() const { return &m_Shape; }
	inline static bool IsBox(const CPhysConvex *convex) {
		return convex->GetShape()->getShapeType() == BOX_SHAPE_PROXYTYPE;
	}

	virtual btScalar GetVolume() const;
	virtual btScalar GetSurfaceArea() const;
	virtual btVector3 GetInertia() const;

	virtual btVector3 GetOriginInCompound() const { return m_Origin; }

private:
	btBoxShape m_Shape;
	btVector3 m_Origin;
};

/***************
 * Collideables
 ***************/

class CPhysCollide {
public:
	enum Owner {
		OWNER_GAME, // Created and to be destroyed by the game.
		OWNER_INTERNAL // Managed internally by physics.
	};

	FORCEINLINE Owner GetOwner() const { return m_Owner; }
	FORCEINLINE void SetOwner(Owner owner) { m_Owner = owner; }

	virtual btCollisionShape *GetShape() = 0;
	virtual const btCollisionShape *GetShape() const = 0;

	virtual btScalar GetVolume() const { return 0.0f; }
	virtual btScalar GetSurfaceArea() const { return 0.0f; }
	virtual btVector3 GetExtent(const btVector3 &origin, const btMatrix3x3 &rotation,
			const btVector3 &direction) const { return origin; }

	virtual btVector3 GetMassCenter() const { return btVector3(0.0f, 0.0f, 0.0f); }
	virtual void SetMassCenter(const btVector3 &massCenter) {}
	virtual btVector3 GetInertia() const { return btVector3(1.0f, 1.0f, 1.0f); }

	FORCEINLINE const btVector3 &GetOrthographicAreas() const { return m_OrthographicAreas; }
	void SetOrthographicAreas(const btVector3 &areas);

	// Returns the true number of convexes, not clamped, for possibility of multiple calls.
	virtual int GetConvexes(CPhysConvex **output, int limit) const { return 0; }

	FORCEINLINE IPhysicsObject *GetObjectReferenceList() const {
		return m_ObjectReferenceList;
	}
	// For internal use in CPhysicsObject::AddReferenceToCollide!
	inline IPhysicsObject *AddObjectReference(IPhysicsObject *object) {
		IPhysicsObject *next = m_ObjectReferenceList;
		m_ObjectReferenceList = object;
		return next;
	}
	// For internal use in CPhysicsObject::RemoveReferenceToCollide!
	void RemoveObjectReference(IPhysicsObject *object);

protected:
	CPhysCollide(const btVector3 &orthographicAreas = btVector3(1.0f, 1.0f, 1.0f)) :
			m_Owner(OWNER_GAME), m_OrthographicAreas(orthographicAreas) {}

	void Initialize() {
		btCollisionShape *shape = GetShape();
		shape->setUserPointer(this);
		shape->setUserIndex(0);
	}

	void NotifyObjectsOfMassCenterChange(const btVector3 &oldMassCenter);

private:
	Owner m_Owner;

	btVector3 m_OrthographicAreas;

	IPhysicsObject *m_ObjectReferenceList;
};

class CPhysCollide_Compound : public CPhysCollide {
public:
	CPhysCollide_Compound(CPhysConvex **pConvex, int convexCount);
	CPhysCollide_Compound(
			const struct VCollide_IVP_Compact_Ledgetree_Node *root, CByteswap &byteswap,
			const btVector3 &massCenter, const btVector3 &inertia,
			const btVector3 &orthographicAreas);
	btCollisionShape *GetShape() { return &m_Shape; }
	const btCollisionShape *GetShape() const { return &m_Shape; }
	FORCEINLINE btCompoundShape *GetCompoundShape() { return &m_Shape; }
	FORCEINLINE const btCompoundShape *GetCompoundShape() const { return &m_Shape; }
	inline static bool IsCompound(const CPhysCollide *collide) {
		return collide->GetShape()->getShapeType() == COMPOUND_SHAPE_PROXYTYPE;
	}

	virtual btScalar GetVolume() const;
	virtual btScalar GetSurfaceArea() const;
	virtual btVector3 GetExtent(const btVector3 &origin, const btMatrix3x3 &rotation,
			const btVector3 &direction) const;

	virtual btVector3 GetMassCenter() const { return m_MassCenter; }
	virtual void SetMassCenter(const btVector3 &massCenter);

	virtual int GetConvexes(CPhysConvex **output, int limit) const;

private:
	btCompoundShape m_Shape;

	void AddIVPCompactLedgetreeNode(
			const struct VCollide_IVP_Compact_Ledgetree_Node *node, CByteswap &byteswap);

	void CalculateInertia();
	btScalar m_Volume;
	btVector3 m_MassCenter;
	btVector3 m_Inertia;
};

class CPhysCollide_Sphere : public CPhysCollide {
public:
	// The ortographic area fraction should be pi/4, but let's assume the engine assumes 1.
	CPhysCollide_Sphere(btScalar radius) : m_Shape(radius + VPHYSICS_CONVEX_DISTANCE_MARGIN) {
		Initialize();
	}
	btCollisionShape *GetShape() { return &m_Shape; }
	const btCollisionShape *GetShape() const { return &m_Shape; }
	FORCEINLINE btSphereShape *GetSphereShape() { return &m_Shape; }
	FORCEINLINE const btSphereShape *GetSphereShape() const { return &m_Shape; }
	inline static bool IsSphere(const CPhysCollide *collide) {
		return collide->GetShape()->getShapeType() == SPHERE_SHAPE_PROXYTYPE;
	}

	FORCEINLINE btScalar GetRadius() const {
		return m_Shape.getRadius() - VPHYSICS_CONVEX_DISTANCE_MARGIN;
	}

	virtual btScalar GetVolume() const;
	virtual btScalar GetSurfaceArea() const;
	virtual btVector3 GetExtent(const btVector3 &origin, const btMatrix3x3 &rotation,
			const btVector3 &direction) const;

	virtual btVector3 GetInertia() const;

private:
	btSphereShape m_Shape;
};

/************
 * Interface
 ************/

class CPhysicsCollision : public IPhysicsCollision {
public:
	// IPhysicsCollision methods.

	virtual CPhysConvex *ConvexFromVerts(Vector **pVerts, int vertCount);
	virtual CPhysConvex *ConvexFromPlanes(float *pPlanes, int planeCount, float mergeDistance);
	virtual float ConvexVolume(CPhysConvex *pConvex);
	virtual float ConvexSurfaceArea(CPhysConvex *pConvex);
	virtual CPhysCollide *UnserializeCollide(char *pBuffer, int size, int index);
	virtual float CollideVolume(CPhysCollide *pCollide);
	virtual float CollideSurfaceArea(CPhysCollide *pCollide);
	virtual Vector CollideGetExtent(const CPhysCollide *pCollide,
			const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction);
	virtual void CollideGetAABB(Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide,
			const Vector &collideOrigin, const QAngle &collideAngles);
	virtual void CollideGetMassCenter(CPhysCollide *pCollide, Vector *pOutMassCenter);
	virtual void CollideSetMassCenter(CPhysCollide *pCollide, const Vector &massCenter);
	virtual Vector CollideGetOrthographicAreas(const CPhysCollide *pCollide);
	virtual void CollideSetOrthographicAreas(CPhysCollide *pCollide, const Vector &areas);
	virtual void SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData);
	virtual void ConvexFree(CPhysConvex *pConvex);
	virtual CPhysConvex *BBoxToConvex(const Vector &mins, const Vector &maxs);
	virtual CPhysConvex *ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron);
	virtual CPhysCollide *ConvertConvexToCollide(CPhysConvex **pConvex, int convexCount);
	virtual int CollideIndex(const CPhysCollide *pCollide);
	virtual CPhysCollide *BBoxToCollide(const Vector &mins, const Vector &maxs);
	virtual int GetConvexesUsedInCollideable(const CPhysCollide *pCollideable,
			CPhysConvex **pOutputArray, int iOutputArrayLimit);
	virtual void VCollideLoad(vcollide_t *pOutput,
			int solidCount, const char *pBuffer, int size, bool swap);
	virtual unsigned int ReadStat(int statID);

	// Internal methods.

	CPhysCollide *UnserializeCollide(const char *pBuffer, int size, int index, bool swap);

	static btVector3 BoxInertia(const btVector3 &extents);
	static btVector3 OffsetInertia(
			const btVector3 &inertia, const btVector3 &origin, bool absolute = true);

	CPhysCollide_Sphere *CreateSphereCollide(btScalar radius);

private:
	HullLibrary m_HullLibrary;

	CPhysCollide_Compound *CreateBBox(const Vector &mins, const Vector &maxs);
	CUtlVector<CPhysCollide_Compound *> m_BBoxCache;

	CPhysCollide *UnserializeIVPCompactSurface(
			const struct VCollide_IVP_Compact_Surface *surface, CByteswap &byteswap,
			const btVector3 &orthographicAreas);
};

extern CPhysicsCollision *g_pPhysCollision;

#endif
