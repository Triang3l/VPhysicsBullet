// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_FRICTION_H
#define PHYSICS_FRICTION_H

#include "physics_internal.h"
#include "vphysics/friction.h"

class CPhysicsFrictionSnapshot : public IPhysicsFrictionSnapshot {
public:
	CPhysicsFrictionSnapshot(IPhysicsObject *object) {}

	/* DUMMY */ virtual bool IsValid() { return false; }
	/* DUMMY */ virtual IPhysicsObject *GetObject(int index) { return nullptr; }
	/* DUMMY */ virtual int GetMaterial(int index) { return 0; }
	/* DUMMY */ virtual void GetContactPoint(Vector &out) { out.Zero(); }
	/* DUMMY */ virtual void GetSurfaceNormal(Vector &out) { out.Init(1.0f, 0.0f, 0.0f); }
	/* DUMMY */ virtual float GetNormalForce() { return 0.0f; }
	/* DUMMY */ virtual float GetEnergyAbsorbed() { return 0.0f; }
	/* DUMMY */ virtual void RecomputeFriction() {}
	/* DUMMY */ virtual void ClearFrictionForce() {}
	/* DUMMY */ virtual void MarkContactForDelete() {}
	/* DUMMY */ virtual void DeleteAllMarkedContacts(bool wakeObjects) {}
	/* DUMMY */ virtual void NextFrictionData() {}
	/* DUMMY */ virtual float GetFrictionCoefficient() { return 0.0f; }
};

#endif
