// Copyright Valve Corporation, All rights reserved.
// Bullet integration by Triang3l, derivative work, in public domain if detached from Valve's work.

#ifndef PHYSICS_ENVIRONMENT_H
#define PHYSICS_ENVIRONMENT_H

#include "physics_internal.h"
#include "vphysics/performance.h"
#include "tier1/utlrbtree.h"
#include "tier1/utlvector.h"

class CPhysicsEnvironment : public IPhysicsEnvironment {
public:
	CPhysicsEnvironment();
	virtual ~CPhysicsEnvironment();

	// IPhysicsEnvironment methods.

	virtual void SetGravity(const Vector &gravityVector);
	virtual void GetGravity(Vector *pGravityVector) const;

	virtual void SetAirDensity(float density);
	virtual float GetAirDensity() const;

	virtual IPhysicsObject *CreatePolyObject(
			const CPhysCollide *pCollisionModel, int materialIndex,
			const Vector &position, const QAngle &angles, objectparams_t *pParams);
	virtual IPhysicsObject *CreatePolyObjectStatic(
			const CPhysCollide *pCollisionModel, int materialIndex,
			const Vector &position, const QAngle &angles, objectparams_t *pParams);
	virtual IPhysicsObject *CreateSphereObject(float radius, int materialIndex,
			const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic);

	virtual IPhysicsMotionController *CreateMotionController(IMotionEvent *pHandler);
	virtual void DestroyMotionController(IPhysicsMotionController *pController);

	virtual float GetSimulationTimestep() const;
	virtual void SetSimulationTimestep(float timestep);

	virtual void SetCollisionEventHandler(IPhysicsCollisionEvent *pCollisionEvents);
	virtual void SetObjectEventHandler(IPhysicsObjectEvent *pObjectEvents);

	virtual int GetActiveObjectCount() const;
	virtual void GetActiveObjects(IPhysicsObject **pOutputObjectList) const;
	virtual const IPhysicsObject **GetObjectList(int *pOutputObjectCount) const;

	virtual bool IsCollisionModelUsed(CPhysCollide *pCollide) const;

	virtual void TraceRay(const Ray_t &ray, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace);
	virtual void SweepCollideable(const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd,
			const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace);

	virtual void GetPerformanceSettings(physics_performanceparams_t *pOutput) const;
	virtual void SetPerformanceSettings(const physics_performanceparams_t *pSettings);

	// Internal methods.

	FORCEINLINE btDiscreteDynamicsWorld *GetDynamicsWorld() {
		return m_DynamicsWorld;
	}
	FORCEINLINE const btDiscreteDynamicsWorld *GetDynamicsWorld() const {
		return m_DynamicsWorld;
	}

	FORCEINLINE const btVector3 &GetBulletGravity() const {
		return m_Gravity;
	}

	void NotifyObjectRemoving(IPhysicsObject *object);

	void NotifyTriggerRemoved(IPhysicsObject *trigger);

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

	btVector3 m_Gravity;

	float m_AirDensity;

	CUtlVector<CPhysCollide *> m_SphereCache;

	void AddObject(IPhysicsObject *object);
	void UpdateActiveObjects();
	CUtlVector<IPhysicsObject *> m_Objects;
	CUtlVector<IPhysicsObject *> m_NonStaticObjects;
	CUtlVector<IPhysicsObject *> m_ActiveNonStaticObjects;
	IPhysicsObjectEvent *m_ObjectEvents;

	CUtlVector<IPhysicsMotionController *> m_MotionControllers;
	void SimulateMotionControllers(
			IPhysicsMotionController::priority_t priority, btScalar timeStep);

	btScalar m_SimulationTimeStep;
	int m_MaxSimulationSubSteps;
	static void PreTickCallback(btDynamicsWorld *world, btScalar timeStep);
	static void TickCallback(btDynamicsWorld *world, btScalar timeStep);
	class TickActionInterface : public btActionInterface {
		virtual void updateAction(btCollisionWorld *collisionWorld, btScalar deltaTimeStep);
		virtual void debugDraw(btIDebugDraw *debugDrawer) {}
	};
	TickActionInterface m_TickAction;

	IPhysicsCollisionEvent *m_CollisionEvents;

	struct TriggerTouch_t {
		IPhysicsObject *m_Trigger;
		IPhysicsObject *m_Object;
		// Temporary flag indicating there's still a touch between this pair.
		// Touches with this flag being false after checking all contacts are removed.
		bool m_TouchingThisTick;

		TriggerTouch_t(IPhysicsObject *trigger, IPhysicsObject *object) :
				m_Trigger(trigger), m_Object(object), m_TouchingThisTick(true) {}
	};
	static bool TriggerTouchLessFunc(const TriggerTouch_t &lhs, const TriggerTouch_t &rhs) {
		if (lhs.m_Trigger != rhs.m_Trigger) {
			return (lhs.m_Trigger < rhs.m_Trigger);
		}
		return (lhs.m_Object < rhs.m_Object);
	}
	CUtlRBTree<TriggerTouch_t> m_TriggerTouches;
	void CheckTriggerTouches();

	physics_performanceparams_t m_PerformanceSettings;
};

#endif
