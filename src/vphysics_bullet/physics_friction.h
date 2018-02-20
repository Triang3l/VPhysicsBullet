// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_FRICTION_H
#define PHYSICS_FRICTION_H

#include "physics_internal.h"
#include "vphysics/friction.h"

class CPhysicsFrictionSnapshot : public IPhysicsFrictionSnapshot {
public:
	CPhysicsFrictionSnapshot();
	CPhysicsFrictionSnapshot(IPhysicsObject *object);

	// IPhysicsFrictionSnapshot methods.

	virtual bool IsValid();
	virtual IPhysicsObject *GetObject(int index);
	virtual int GetMaterial(int index);
	virtual void GetContactPoint(Vector &out);
	virtual void GetSurfaceNormal(Vector &out);
	virtual float GetNormalForce();
	/* DUMMY */ virtual float GetEnergyAbsorbed() { return 0.0f; }
	/* DUMMY */ virtual void RecomputeFriction() {}
	/* DUMMY */ virtual void ClearFrictionForce() {}
	/* DUMMY */ virtual void MarkContactForDelete() {}
	/* DUMMY */ virtual void DeleteAllMarkedContacts(bool wakeObjects) {}
	virtual void NextFrictionData();
	virtual float GetFrictionCoefficient();

	// Internal methods.

	void Reset(IPhysicsObject *object);

private:
	IPhysicsObject *m_Object;
	btCollisionDispatcher *m_Dispatcher;
	int m_ManifoldIndex;
	bool m_ObjectIsB;
	int m_ContactIndex;

	inline btManifoldPoint &GetCurrentContact() const {
		return m_Dispatcher->getManifoldByIndexInternal(m_ManifoldIndex)->getContactPoint(m_ContactIndex);
	}
};

#endif
