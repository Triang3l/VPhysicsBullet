// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_ENVIRONMENT_H
#define PHYSICS_ENVIRONMENT_H

#include "physics_internal.h"

class CPhysicsEnvironment : public IPhysicsEnvironment {
public:
	CPhysicsEnvironment();
	virtual ~CPhysicsEnvironment();

	// IPhysicsEnvironment methods.

	virtual void SetGravity(const Vector &gravityVector);
	virtual void GetGravity(Vector *pGravityVector) const;

private:
	btDefaultCollisionConfiguration *m_CollisionConfiguration;
	btCollisionDispatcher *m_Dispatcher;
	btBroadphaseInterface *m_Broadphase;
	btSequentialImpulseConstraintSolver *m_Solver;
	btDiscreteDynamicsWorld *m_DynamicsWorld;
};

#endif
