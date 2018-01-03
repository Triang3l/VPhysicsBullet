// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_ENVIRONMENT_H
#define PHYSICS_ENVIRONMENT_H

#include "physics_internal.h"
#include "vphysics/performance.h"
#include "tier1/utlvector.h"

class CPhysicsEnvironment : public IPhysicsEnvironment {
public:
	CPhysicsEnvironment();
	virtual ~CPhysicsEnvironment();

	// IPhysicsEnvironment methods.

	virtual void SetGravity(const Vector &gravityVector);
	virtual void GetGravity(Vector *pGravityVector) const;

	virtual void GetPerformanceSettings(physics_performanceparams_t *pOutput) const;
	virtual void SetPerformanceSettings(const physics_performanceparams_t *pSettings);

	// Internal methods.

	FORCEINLINE btDiscreteDynamicsWorld *GetDynamicsWorld() {
		return m_DynamicsWorld;
	}
	FORCEINLINE const btDiscreteDynamicsWorld *GetDynamicsWorld() const {
		return m_DynamicsWorld;
	}

	FORCEINLINE btScalar GetMaxSpeed() const {
		return HL2BULLET(m_PerformanceSettings.maxVelocity);
	}
	FORCEINLINE btScalar GetMaxAngularSpeed() const {
		return DEG2RAD(m_PerformanceSettings.maxAngularVelocity);
	}

private:
	btDefaultCollisionConfiguration *m_CollisionConfiguration;
	btCollisionDispatcher *m_Dispatcher;
	btBroadphaseInterface *m_Broadphase;
	btSequentialImpulseConstraintSolver *m_Solver;
	btDiscreteDynamicsWorld *m_DynamicsWorld;

	CUtlVector<IPhysicsObject *> m_NonStaticObjects;

	physics_performanceparams_t m_PerformanceSettings;

	static void PreTickCallback(btDynamicsWorld *world, btScalar timeStep);
};

#endif
