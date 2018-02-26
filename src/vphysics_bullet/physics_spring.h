// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_SPRING_H
#define PHYSICS_SPRING_H

#include "physics_internal.h"

class CPhysicsSpring : public IPhysicsSpring {
public:
	CPhysicsSpring(IPhysicsObject *objectStart, IPhysicsObject *objectEnd,
			const springparams_t *params) :
			m_ObjectStart(objectStart), m_ObjectEnd(objectEnd) {}

	/* DUMMY */ virtual void GetEndpoints(Vector *worldPositionStart, Vector *worldPositionEnd) {
		if (worldPositionStart != nullptr) {
			worldPositionStart->Zero();
		}
		if (worldPositionEnd != nullptr) {
			worldPositionEnd->Zero();
		}
	}
	/* DUMMY */ virtual void SetSpringConstant(float flSpringContant) {}
	/* DUMMY */ virtual void SetSpringDamping( float flSpringDamping) {}
	/* DUMMY */ virtual void SetSpringLength(float flSpringLength) {}
	/* DUMMY */ virtual IPhysicsObject *GetStartObject() { return m_ObjectStart; }
	/* DUMMY */ virtual IPhysicsObject *GetEndObject() { return m_ObjectEnd; }

private:
	IPhysicsObject *m_ObjectStart, *m_ObjectEnd;
};

#endif
