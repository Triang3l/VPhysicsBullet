// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#include "physics_environment.h"
#include "physics_object.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

CPhysicsEnvironment::CPhysicsEnvironment() {
	m_PerformanceSettings.Defaults();

	BEGIN_BULLET_ALLOCATION();
	m_CollisionConfiguration = new btDefaultCollisionConfiguration();
	m_Dispatcher = new btCollisionDispatcher(m_CollisionConfiguration);
	m_Broadphase = new btDbvtBroadphase();
	m_Solver = new btSequentialImpulseConstraintSolver();
	m_DynamicsWorld = new btDiscreteDynamicsWorld(m_Dispatcher, m_Broadphase, m_Solver, m_CollisionConfiguration);
	END_BULLET_ALLOCATION();

	m_DynamicsWorld->setInternalTickCallback(PreTickCallback, this, true);
}

CPhysicsEnvironment::~CPhysicsEnvironment() {
	delete m_DynamicsWorld;
	delete m_Solver;
	delete m_Broadphase;
	delete m_Dispatcher;
	delete m_CollisionConfiguration;
}

void CPhysicsEnvironment::SetGravity(const Vector &gravityVector) {
	btVector3 gravity;
	ConvertPositionToBullet(gravityVector, gravity);
	m_DynamicsWorld->setGravity(gravity);
}

void CPhysicsEnvironment::GetGravity(Vector *pGravityVector) const {
	ConvertPositionToHL(m_DynamicsWorld->getGravity(), *pGravityVector);
}

void CPhysicsEnvironment::GetPerformanceSettings(physics_performanceparams_t *pOutput) const {
	*pOutput = m_PerformanceSettings;
}

void CPhysicsEnvironment::SetPerformanceSettings(const physics_performanceparams_t *pSettings) {
	m_PerformanceSettings = *pSettings;
}

void CPhysicsEnvironment::PreTickCallback(btDynamicsWorld *world, btScalar timeStep) {
	CPhysicsEnvironment *environment = reinterpret_cast<CPhysicsEnvironment *>(world->getWorldUserInfo());
	IPhysicsObject * const *objects = environment->m_NonStaticObjects.Base();
	int objectCount = environment->m_NonStaticObjects.Count();
	for (int objectIndex = 0; objectIndex < objectCount; ++objectIndex) {
		CPhysicsObject *object = static_cast<CPhysicsObject *>(objects[objectIndex]);
		object->ApplyDamping((float) timeStep);
		object->ApplyForcesAndSpeedLimit();
	}
}
