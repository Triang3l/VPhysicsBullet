// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_COLLISION_H
#define PHYSICS_COLLISION_H

#include "physics_internal.h"
#include <LinearMath/btConvexHull.h>
#include "tier1/utlvector.h"

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

	void Initialize() {
		btCollisionShape *shape = GetShape();
		shape->setUserPointer(this);
		shape->setUserIndex(0);
	}

private:
	Owner m_Owner;
};

class CPhysConvex_TriangleMesh : public CPhysConvex {
public:
	// Takes ownership of the hull.
	CPhysConvex_TriangleMesh(HullResult *hull);
	static CPhysConvex_TriangleMesh *CreateFromBulletPoints(
			HullLibrary &hullLibrary, const btVector3 *points, int pointCount);

	btCollisionShape *GetShape() { return &m_Shape; }
	const btCollisionShape *GetShape() const { return &m_Shape; }
	FORCEINLINE btConvexTriangleMeshShape *GetTriangleMeshShape() { return &m_Shape; }
	FORCEINLINE const btConvexTriangleMeshShape *GetTriangleMeshShape() const { return &m_Shape; }
	inline static bool IsTriangleMesh(const CPhysConvex *convex) {
		return convex->GetShape()->getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
	}

	virtual btScalar GetVolume() const;
	virtual btScalar GetSurfaceArea() const; // Slow.
	virtual btVector3 GetMassCenter() const { return m_MassCenter; }
	virtual btVector3 GetInertia() const { return m_Inertia; }

private:
	class StridingMeshInterface : public btStridingMeshInterface {
	public:
		StridingMeshInterface(const HullResult *hull) : m_Hull(hull) {}
		virtual void getLockedVertexIndexBase(
				unsigned char **vertexbase, int &numverts, PHY_ScalarType &type, int &stride,
				unsigned char **indexbase, int &indexstride, int &numfaces, PHY_ScalarType &indicestype,
				int subpart) { Assert(false); }
		virtual void getLockedReadOnlyVertexIndexBase(
				const unsigned char **vertexbase, int &numverts, PHY_ScalarType &type, int &stride,
				const unsigned char **indexbase, int &indexstride, int &numfaces, PHY_ScalarType &indicestype,
				int subpart) const;
		virtual void unLockVertexBase(int subpart) { Assert(false); }
		virtual void unLockReadOnlyVertexBase(int subpart) const { Assert(subpart == 0); }
		virtual int getNumSubParts() const { return 1; }
		virtual void preallocateVertices(int numverts) {}
		virtual void preallocateIndices(int numindices) {}
		const HullResult *GetHull() const { return m_Hull; }
	private:
		const HullResult *m_Hull;
	};

	StridingMeshInterface m_StridingMeshInterface; // Must precede m_Shape.
	btConvexTriangleMeshShape m_Shape;
	btScalar m_Volume;
	btVector3 m_MassCenter;
	btVector3 m_Inertia;
};

class CPhysConvex_Box : public CPhysConvex {
public:
	CPhysConvex_Box(const btVector3 &halfExtents, const btVector3 &origin) :
			m_Shape(halfExtents), m_Origin(origin) {}

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

	virtual btVector3 GetMassCenter() const { return btVector3(0.0f, 0.0f, 0.0f); }
	virtual void SetMassCenter(const btVector3 &massCenter) {}
	virtual btVector3 GetInertia() const { return btVector3(1.0f, 1.0f, 1.0f); }

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
	CPhysCollide() : m_Owner(OWNER_GAME) {}

	void Initialize() {
		btCollisionShape *shape = GetShape();
		shape->setUserPointer(this);
		shape->setUserIndex(0);
	}

	void NotifyObjectsOfMassCenterChange(const btVector3 &oldMassCenter);

private:
	Owner m_Owner;

	IPhysicsObject *m_ObjectReferenceList;
};

class CPhysCollide_Compound : public CPhysCollide {
public:
	CPhysCollide_Compound(CPhysConvex **pConvex, int convexCount);
	btCollisionShape *GetShape() { return &m_Shape; }
	const btCollisionShape *GetShape() const { return &m_Shape; }
	FORCEINLINE btCompoundShape *GetCompoundShape() { return &m_Shape; }
	FORCEINLINE const btCompoundShape *GetCompoundShape() const { return &m_Shape; }
	inline static bool IsCompound(const CPhysCollide *collide) {
		return collide->GetShape()->getShapeType() == COMPOUND_SHAPE_PROXYTYPE;
	}

	virtual btScalar GetVolume() const { return m_Volume; }
	virtual btScalar GetSurfaceArea() const;

	virtual btVector3 GetMassCenter() const { return m_MassCenter; }
	virtual void SetMassCenter(const btVector3 &massCenter);

private:
	btCompoundShape m_Shape;
	btScalar m_Volume;
	btVector3 m_MassCenter;
	btVector3 m_Inertia;

	void CalculateInertia();
};

class CPhysCollide_Sphere : public CPhysCollide {
public:
	CPhysCollide_Sphere(btScalar radius) : m_Shape(radius) {
		Initialize();
	}
	btCollisionShape *GetShape() { return &m_Shape; }
	const btCollisionShape *GetShape() const { return &m_Shape; }
	FORCEINLINE btSphereShape *GetSphereShape() { return &m_Shape; }
	FORCEINLINE const btSphereShape *GetSphereShape() const { return &m_Shape; }
	inline static bool IsSphere(const CPhysCollide *collide) {
		return collide->GetShape()->getShapeType() == SPHERE_SHAPE_PROXYTYPE;
	}

	virtual btScalar GetVolume() const;
	virtual btScalar GetSurfaceArea() const;

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
	virtual float ConvexVolume(CPhysConvex *pConvex);
	virtual float ConvexSurfaceArea(CPhysConvex *pConvex);
	virtual float CollideVolume(CPhysCollide *pCollide);
	virtual float CollideSurfaceArea(CPhysCollide *pCollide);
	virtual void CollideGetMassCenter(CPhysCollide *pCollide, Vector *pOutMassCenter);
	virtual void CollideSetMassCenter(CPhysCollide *pCollide, const Vector &massCenter);
	virtual void SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData);
	virtual void ConvexFree(CPhysConvex *pConvex);
	virtual CPhysConvex *BBoxToConvex(const Vector &mins, const Vector &maxs);
	virtual CPhysConvex *ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron);
	virtual CPhysCollide *ConvertConvexToCollide(CPhysConvex **pConvex, int convexCount);
	virtual int CollideIndex(const CPhysCollide *pCollide);
	virtual CPhysCollide *BBoxToCollide(const Vector &mins, const Vector &maxs);

	// Internal methods.

	static btVector3 BoxInertia(const btVector3 &extents);
	static btVector3 OffsetInertia(
			const btVector3 &inertia, const btVector3 &origin, bool absolute = true);

	void SetCollideIndex(CPhysCollide *pCollide, int index);

private:
	HullLibrary m_HullLibrary;

	CPhysCollide_Compound *CreateBBox(const Vector &mins, const Vector &maxs);
	CUtlVector<CPhysCollide_Compound *> m_BBoxCache;
};

extern CPhysicsCollision *g_pPhysCollision;

#endif
